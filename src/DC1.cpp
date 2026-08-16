
#include "DC1.h"
#include "Rtc.h"
#include "Util.h"
#ifdef USE_HOMEKIT
#include "HomeKit.h"
#endif

#pragma region 继承

void DC1::init()
{
    Led::init(LED_PIN, LOW);
    cat9554 = new CAT9554(CAT9554_SDA_PIN, CAT9554_SCL_PIN);
    cat9554->setIrqPin(CAT9554_IRQ_PIN);
    cat9554->setup();

    cse7766 = new CSE7766(CSE7766_RX_PIN, CSE7766_BAUDRATE);

    // 按键
    pinMode(KEY_0_PIN, INPUT_PULLDOWN_16);
    cat9554->pinMode(KEY_1_PIN, INPUT);
    cat9554->pinMode(KEY_2_PIN, INPUT);
    cat9554->pinMode(KEY_3_PIN, INPUT);

    // 继电器
    cat9554->pinMode(REL_0_PIN, OUTPUT);
    cat9554->pinMode(REL_1_PIN, OUTPUT);
    cat9554->pinMode(REL_2_PIN, OUTPUT);
    cat9554->pinMode(REL_3_PIN, OUTPUT);

    pinMode(LOGO_LED_PIN, OUTPUT);
    logoLed();

    strcpy(powerStatTopic, Mqtt::getStatTopic(F("power1")).c_str());
    strcpy(energyTeleTopic, Mqtt::getTeleTopic(F("energy")).c_str());

    channels = 4;
    for (uint8_t ch = 0; ch < channels; ch++)
    {
        // 0:开关通电时断开  1 : 开关通电时闭合  2 : 开关通电时状态与断电前相反  3 : 开关通电时保持断电前状态
        if (config.power_on_state == 2)
        {
            switchRelay(ch, !bitRead(config.last_state, ch), false); // 开关通电时状态与断电前相反
        }
        else if (config.power_on_state == 3)
        {
            switchRelay(ch, bitRead(config.last_state, ch), false); // 开关通电时保持断电前状态
        }
        else
        {
            switchRelay(ch, config.power_on_state == 1, false); // 开关通电时闭合
        }
        // 总开关关时跳过其他
        if (ch == 0 && !bitRead(lastState, 0) && config.sub_kinkage != 0)
        {
            break;
        }
    }
    energyInit();
}

bool DC1::moduleLed()
{
    if (WiFi.status() == WL_CONNECTED && (globalConfig.mqtt.port ==0 || Mqtt::mqttClient.connected()))
    {
        if (config.wifi_led == 0)
        {
            Led::on();
            return true;
        }
        else if (config.wifi_led == 1)
        {
            Led::off();
            return true;
        }
    }
    return false;
}

void DC1::loop()
{
    cse7766->loop();
    for (size_t ch = 0; ch < channels; ch++)
    {
        checkButton(ch);
    }

    if (bitRead(operationFlag, 0))
    {
        bitClear(operationFlag, 0);
        energyUpdate();
    }

    if (bitRead(operationFlag, 2))
    {
        bitClear(operationFlag, 2);
        timerCheck();
    }

    if (bitRead(operationFlag, 3))
    {
        bitClear(operationFlag, 3);
        scheduleCheck();
    }
}

void DC1::perSecondDo()
{
    bitSet(operationFlag, 0);
    bitSet(operationFlag, 2);
    bitSet(operationFlag, 3);
}
#pragma endregion

#pragma region 配置

void DC1::readConfig()
{
    Config::moduleReadConfig(MODULE_CFG_VERSION, sizeof(DC1ConfigMessage), DC1ConfigMessage_fields, &config);
}

void DC1::resetConfig()
{
    Debug::AddInfo(PSTR("moduleResetConfig . . . OK"));
    memset(&config, 0, sizeof(DC1ConfigMessage));

    config.power_on_state = 3;
    config.power_mode = 0;
    config.logo_led = 0;
    config.wifi_led = 0;
    config.sub_kinkage = 2;
    config.energy_power_delta = 10;
    config.report_interval = 60;
    config.energy_max_power = 2300;
}

void DC1::saveConfig(bool isEverySecond)
{
    if (bitRead(operationFlag, 1) || !isEverySecond)
    {
        bitClear(operationFlag, 1);
        energySync();
    }
    Config::moduleSaveConfig(MODULE_CFG_VERSION, DC1ConfigMessage_size, DC1ConfigMessage_fields, &config);
}
#pragma endregion

#pragma region MQTT

void DC1::mqttCallback(char *topic, char *payload, char *cmnd)
{
    if (strlen(cmnd) == 6 && strncmp(cmnd, "power", 5) == 0) // strlen("power1") = 6
    {
        uint8_t ch = cmnd[5] - 49;
        if (ch < channels)
        {
            switchRelay(ch, (strcmp(payload, "on") == 0 ? true : (strcmp(payload, "off") == 0 ? false : !bitRead(lastState, ch))), true);
            return;
        }
    }
    else if (strcmp(cmnd, "report") == 0)
    {
        reportPower();
        reportEnergy();
    }
    else if (strcmp(cmnd, "clear") == 0)
    {
        energyClear();
    }
    else if (strcmp(cmnd, "timer") == 0)
    {
        timerCmd(payload);
    }
    else if (strcmp(cmnd, "schedule") == 0)
    {
        scheduleCmd(payload);
    }
}

#pragma region 倒计时

void DC1::timerCheck()
{
    bool anyActive = false;
    for (uint8_t ch = 0; ch < channels; ch++)
    {
        if (ch >= config.timer_remaining_count)
        {
            break;
        }
        if (config.timer_remaining[ch] > 0)
        {
            anyActive = true;
            config.timer_remaining[ch]--;
            if (config.timer_remaining[ch] == 0)
            {
                Debug::AddInfo(PSTR("Timer ch%d expired -> %s"), ch + 1, config.timer_target[ch] ? "ON" : "OFF");
                switchRelay(ch, config.timer_target[ch] == 1, true);
                reportTimer();
                Config::delaySaveConfig(5);
            }
        }
    }
    // 倒计时激活期间每60秒持久化一次剩余秒数，断电最多丢60秒进度
    if (anyActive)
    {
        timerPersistCounter++;
        if (timerPersistCounter >= 60)
        {
            timerPersistCounter = 0;
            Config::delaySaveConfig(2);
        }
    }
}

void DC1::timerSet(uint8_t ch, uint32_t seconds, bool isOn)
{
    if (ch >= channels)
    {
        return;
    }
    config.timer_remaining[ch] = seconds;
    config.timer_target[ch] = isOn ? 1 : 0;
    if (config.timer_remaining_count < ch + 1)
    {
        config.timer_remaining_count = ch + 1;
    }
    if (config.timer_target_count < ch + 1)
    {
        config.timer_target_count = ch + 1;
    }
    Debug::AddInfo(PSTR("Timer ch%d set %lus -> %s"), ch + 1, (unsigned long)seconds, isOn ? "ON" : "OFF");
    Config::saveConfig();
    reportTimer();
}

void DC1::timerCancel(uint8_t ch)
{
    if (ch >= channels)
    {
        return;
    }
    config.timer_remaining[ch] = 0;
    Debug::AddInfo(PSTR("Timer ch%d cancelled"), ch + 1);
    Config::saveConfig();
    reportTimer();
}

void DC1::reportTimer()
{
    char msg[200];
    char *p = msg;
    p += sprintf(p, "{");
    for (uint8_t ch = 0; ch < channels; ch++)
    {
        uint32_t rem = (ch < config.timer_remaining_count) ? config.timer_remaining[ch] : 0;
        uint8_t tgt = (ch < config.timer_target_count) ? config.timer_target[ch] : 0;
        p += sprintf(p, "%s\"%d\":{\"remaining\":%lu,\"target\":\"%s\"}", ch ? "," : "", ch + 1, (unsigned long)rem, tgt ? "on" : "off");
    }
    p += sprintf(p, "}");
    Mqtt::publish(Mqtt::getStatTopic(F("timer")), msg);
}

void DC1::timerCmd(char *payload)
{
    if (strcmp(payload, "?") == 0)
    {
        reportTimer();
        return;
    }
    uint8_t ch = 0;
    unsigned long sec = 0;
    char act[4] = {0};
    int n = sscanf(payload, "%hhu %lu %3s", &ch, &sec, act);
    if (n < 1 || ch < 1 || ch > channels)
    {
        reportTimer();
        return;
    }
    if (n >= 3 && sec > 0)
    {
        timerSet(ch - 1, sec, strcmp(act, "on") == 0);
    }
    else if (n >= 2 && sec == 0)
    {
        timerCancel(ch - 1);
    }
    else
    {
        reportTimer();
    }
}
#pragma endregion

#pragma region 定时任务

#define SCHED_DISABLED 0xFFFFFFFF

void DC1::scheduleCheck()
{
    if (!Rtc::rtcTime.valid)
    {
        return; // 时间未同步不触发
    }
    uint16_t nowMin = Rtc::rtcTime.hour * 60 + Rtc::rtcTime.minute;
    for (uint8_t ch = 0; ch < channels; ch++)
    {
        if (ch >= config.sched_on_count)
        {
            break;
        }
        if (nowMin < schedLastMinute[ch])
        {
            schedLastMinute[ch] = -1; // 跨天重置
        }
        if (nowMin == schedLastMinute[ch])
        {
            continue; // 本分钟已处理
        }
        uint32_t on = config.sched_on[ch];
        uint32_t off = (ch < config.sched_off_count) ? config.sched_off[ch] : SCHED_DISABLED;
        if (on != SCHED_DISABLED && nowMin == (uint16_t)on)
        {
            schedLastMinute[ch] = nowMin;
            Debug::AddInfo(PSTR("Schedule ch%d ON"), ch + 1);
            switchRelay(ch, true, true);
            reportSchedule();
        }
        else if (off != SCHED_DISABLED && nowMin == (uint16_t)off)
        {
            schedLastMinute[ch] = nowMin;
            Debug::AddInfo(PSTR("Schedule ch%d OFF"), ch + 1);
            switchRelay(ch, false, true);
            reportSchedule();
        }
    }
}

void DC1::scheduleSet(uint8_t ch, int32_t onMin, int32_t offMin)
{
    if (ch >= channels)
    {
        return;
    }
    config.sched_on[ch] = (onMin >= 0) ? (uint32_t)onMin : SCHED_DISABLED;
    config.sched_off[ch] = (offMin >= 0) ? (uint32_t)offMin : SCHED_DISABLED;
    if (config.sched_on_count < ch + 1)
    {
        config.sched_on_count = ch + 1;
    }
    if (config.sched_off_count < ch + 1)
    {
        config.sched_off_count = ch + 1;
    }
    schedLastMinute[ch] = -1;
    Debug::AddInfo(PSTR("Schedule ch%d set on=%d off=%d"), ch + 1, onMin, offMin);
    Config::saveConfig();
    reportSchedule();
}

void DC1::scheduleClear(uint8_t ch)
{
    if (ch >= channels)
    {
        return;
    }
    config.sched_on[ch] = SCHED_DISABLED;
    config.sched_off[ch] = SCHED_DISABLED;
    schedLastMinute[ch] = -1;
    Debug::AddInfo(PSTR("Schedule ch%d cleared"), ch + 1);
    Config::saveConfig();
    reportSchedule();
}

void DC1::reportSchedule()
{
    char msg[200];
    char *p = msg;
    p += sprintf(p, "{");
    for (uint8_t ch = 0; ch < channels; ch++)
    {
        uint32_t on = (ch < config.sched_on_count) ? config.sched_on[ch] : SCHED_DISABLED;
        uint32_t off = (ch < config.sched_off_count) ? config.sched_off[ch] : SCHED_DISABLED;
        if (on != SCHED_DISABLED)
        {
            p += sprintf(p, "%s\"%d\":{\"on\":\"%02d:%02d\",\"off\":", ch ? "," : "", ch + 1, on / 60, on % 60);
        }
        else
        {
            p += sprintf(p, "%s\"%d\":{\"on\":\"--\",\"off\":", ch ? "," : "", ch + 1);
        }
        if (off != SCHED_DISABLED)
        {
            p += sprintf(p, "\"%02d:%02d\"}", off / 60, off % 60);
        }
        else
        {
            p += sprintf(p, "\"--\"}");
        }
    }
    p += sprintf(p, "}");
    Mqtt::publish(Mqtt::getStatTopic(F("schedule")), msg);
}

void DC1::scheduleCmd(char *payload)
{
    if (strcmp(payload, "?") == 0)
    {
        reportSchedule();
        return;
    }
    uint8_t ch = 0;
    int onM = -1, offM = -1;
    int n = sscanf(payload, "%hhu %d %d", &ch, &onM, &offM);
    if (n < 1 || ch < 1 || ch > channels)
    {
        reportSchedule();
        return;
    }
    if (n == 1)
    {
        reportSchedule(); // 只给通道号 = 查询
        return;
    }
    if (n >= 3)
    {
        if (onM < 0 && offM < 0)
        {
            scheduleClear(ch - 1);
        }
        else
        {
            scheduleSet(ch - 1, onM, offM);
        }
    }
    else
    {
        scheduleSet(ch - 1, onM, -1); // 只设置开启时间
    }
}
#pragma endregion

void DC1::mqttConnected()
{
    strcpy(powerStatTopic, Mqtt::getStatTopic(F("power1")).c_str());
    strcpy(energyTeleTopic, Mqtt::getTeleTopic(F("energy")).c_str());
    if (globalConfig.mqtt.discovery)
    {
        mqttDiscovery(true);
    }

    reportPower();
    reportEnergy();
}

void DC1::mqttDiscovery(bool isEnable)
{
    char topic[50];
    char message[500];

    String availability = Mqtt::getTeleTopic(F("availability"));
    char cmndTopic[100];
    strcpy(cmndTopic, Mqtt::getCmndTopic(F("power1")).c_str());
    for (size_t ch = 0; ch < channels; ch++)
    {
        sprintf(topic, PSTR("%s/switch/%s_%d/config"), globalConfig.mqtt.discovery_prefix, UID, (ch + 1));
        if (isEnable)
        {
            cmndTopic[strlen(cmndTopic) - 1] = ch + 49;           // 48 + 1 + ch
            powerStatTopic[strlen(powerStatTopic) - 1] = ch + 49; // 48 + 1 + ch
            sprintf(message,
                    PSTR("{\"name\":\"%s_%d\","
                         "\"uniq_id\":\"%s_%d\","
                         "\"cmd_t\":\"%s\","
                         "\"stat_t\":\"%s\","
                         "\"pl_off\":\"off\","
                         "\"pl_on\":\"on\","
                         "\"avty_t\":\"%s\","
                         "\"pl_avail\":\"online\","
                         "\"pl_not_avail\":\"offline\","
                         "\"device\":{\"identifiers\":[\"%s\"],\"name\":\"%s\",\"manufacturer\":\"Phicomm\",\"model\":\"DC1\",\"sw_version\":\"%s\"}}"),
                    UID, (ch + 1),
                    UID, (ch + 1),
                    cmndTopic,
                    powerStatTopic,
                    availability.c_str(),
                    UID,
                    module->getModuleCNName().c_str(),
                    module->getModuleVersion().c_str());
            Mqtt::publish(topic, message, true);
            //Debug::AddInfo(PSTR("discovery: %s - %s"), topic, message);
        }
        else
        {
            Mqtt::publish(topic, "", true);
        }
    }

    String tims[] = {F("voltage"), F("current"), F("power"), F("apparent_power"), F("reactive_power"), F("factor"), F("total"), F("yesterday"), F("today"), F("starttime")};
    String tims2[] = {F("V"), F("A"), F("W"), F("VA"), F("VAr"), F(""), F("kWh"), F("kWh"), F("kWh"), F("")};
    String energy = Mqtt::getTeleTopic(F("energy"));
    for (size_t i = 0; i < 10; i++)
    {
        sprintf(topic, PSTR("%s/sensor/%s_%s/config"), globalConfig.mqtt.discovery_prefix, UID, tims[i].c_str());
        if (isEnable)
        {
            if (tims2[i].length() == 0)
            {
                sprintf(message,
                        PSTR("{\"name\":\"%s_%s\","
                             "\"stat_t\":\"%s\","
                             "\"uniq_id\":\"%s_%s\","
                             "\"val_tpl\":\"{{value_json.%s}}\","
                             "\"device\":{\"identifiers\":[\"%s\"],\"name\":\"%s\",\"manufacturer\":\"Phicomm\",\"model\":\"DC1\",\"sw_version\":\"%s\"}}"),
                        UID, tims[i].c_str(),
                        energy.c_str(),
                        UID, tims[i].c_str(),
                        tims[i].c_str(),
                        UID,
                        module->getModuleCNName().c_str(),
                        module->getModuleVersion().c_str());
            }
            else
            {
                sprintf(message,
                        PSTR("{\"name\":\"%s_%s\","
                             "\"stat_t\":\"%s\","
                             "\"uniq_id\":\"%s_%s\","
                             "\"val_tpl\":\"{{value_json.%s}}\","
                             "\"unit_of_meas\":\"%s\","
                             "\"device\":{\"identifiers\":[\"%s\"],\"name\":\"%s\",\"manufacturer\":\"Phicomm\",\"model\":\"DC1\",\"sw_version\":\"%s\"}}"),
                        UID, tims[i].c_str(),
                        energy.c_str(),
                        UID, tims[i].c_str(),
                        tims[i].c_str(), tims2[i].c_str(),
                        UID,
                        module->getModuleCNName().c_str(),
                        module->getModuleVersion().c_str());
            }
            Mqtt::publish(topic, message, true);
            //Debug::AddInfo(PSTR("discovery: %s - %s"), topic, message);
        }
        else
        {
            Mqtt::publish(topic, "", true);
        }
    }
    if (isEnable)
    {
        Mqtt::availability();
        reportPower();
        reportEnergy();
    }
}
#pragma endregion

#pragma region Http

void DC1::httpAdd(ESP8266WebServer *server)
{
    server->on(F("/dc1_do"), std::bind(&DC1::httpDo, this, server));
    server->on(F("/dc1_setting"), std::bind(&DC1::httpSetting, this, server));
    server->on(F("/ha"), std::bind(&DC1::httpHa, this, server));
#ifdef USE_HOMEKIT
    server->on(F("/homekit"), std::bind(&homekit_http, server));
#endif
}

String DC1::httpGetStatus(ESP8266WebServer *server)
{
    String data;
    for (size_t ch = 0; ch < channels; ch++)
    {
        data += ",\"power" + String(ch + 1) + "\":";
        data += bitRead(lastState, ch) ? 1 : 0;
    }
    for (size_t ch = 0; ch < channels; ch++)
    {
        uint32_t rem = (ch < config.timer_remaining_count) ? config.timer_remaining[ch] : 0;
        uint8_t tgt = (ch < config.timer_target_count) ? config.timer_target[ch] : 0;
        data += ",\"timer" + String(ch + 1) + "\":";
        data += String(rem);
        data += ",\"timer" + String(ch + 1) + "target\":\"";
        data += tgt ? "on" : "off";
        data += "\"";
    }
    for (size_t ch = 0; ch < channels; ch++)
    {
        uint32_t on = (ch < config.sched_on_count) ? config.sched_on[ch] : 0xFFFFFFFF;
        uint32_t off = (ch < config.sched_off_count) ? config.sched_off[ch] : 0xFFFFFFFF;
        char buf[40];
        if (on != 0xFFFFFFFF && off != 0xFFFFFFFF)
        {
            snprintf_P(buf, sizeof(buf), PSTR("%02d:%02d-%02d:%02d"), on / 60, on % 60, off / 60, off % 60);
        }
        else if (on != 0xFFFFFFFF)
        {
            snprintf_P(buf, sizeof(buf), PSTR("%02d:%02d-"), on / 60, on % 60);
        }
        else if (off != 0xFFFFFFFF)
        {
            snprintf_P(buf, sizeof(buf), PSTR("-%02d:%02d"), off / 60, off % 60);
        }
        else
        {
            strcpy_P(buf, PSTR("--"));
        }
        data += ",\"sched" + String(ch + 1) + "\":\"";
        data += String(buf);
        data += "\"";
    }
    energyShow(false);
    data += String(tmpData);
    return data.substring(1);
}

void DC1::httpHtml(ESP8266WebServer *server)
{
    server->sendContent_P(
        PSTR("<table class='gridtable'><thead><tr><th colspan='2'>开关状态</th></tr></thead><tbody>"
             "<tr style='text-align:center'><td colspan='2'>"));

    for (size_t ch = 0; ch < channels; ch++)
    {
        // 卡片: 左(名称+状态) 中(toggle开关) 右(倒计时按钮+状态)
        snprintf_P(tmpData, sizeof(tmpData),
                   PSTR("<div style='background:#f3edfa;border-radius:12px;padding:6px 8px;margin:8px 0'>"
                        "<table style='width:100%%;border:0;border-collapse:collapse;table-layout:fixed'><tr>"
                        "<td style='border:0;padding:2px;text-align:left;vertical-align:middle;width:15%%'>"
                        "<span style='font-weight:bold;font-size:14px'>开关%d</span><br>"
                        "<span id='stat%d' style='font-size:11px;color:#888'>%s</span></td>"),
                   ch + 1, ch + 1,
                   bitRead(lastState, ch) ? PSTR("已开启") : PSTR("已关闭"));
        server->sendContent_P(tmpData);
        snprintf_P(tmpData, sizeof(tmpData),
                   PSTR("<td style='border:0;padding:2px;width:35%%;text-align:center;vertical-align:middle'>"
                        "<button id='sw%d' onclick=\"toggleSw(%d)\" style='width:44px;height:26px;border-radius:13px;background:%s;border:none;position:relative;outline:none'>"
                        "<span style='position:absolute;top:2px;%s:2px;width:22px;height:22px;border-radius:11px;background:#fff;display:block'></span></button></td>"),
                   ch + 1, ch + 1,
                   bitRead(lastState, ch) ? PSTR("#7c5cbf") : PSTR("#ccc"),
                   bitRead(lastState, ch) ? PSTR("right") : PSTR("left"));
        server->sendContent_P(tmpData);
        snprintf_P(tmpData, sizeof(tmpData),
                   PSTR("<td style='border:0;padding:2px;text-align:right;vertical-align:middle;width:50%%'>"
                        "<span style='display:inline-block;text-align:center'>"
                        "<button type='button' onclick=\"showSchedModal(%d)\" style='background:#7c5cbf;border:none;border-radius:10px;color:#fff;padding:2px 0;font-size:10px;width:54px'>定时任务</button>"
                        "<br><span id='sched%d' style='font-size:10px;color:#888'>无定时</span>"
                        "</span>"),
                   ch + 1, ch + 1);
        server->sendContent_P(tmpData);
        snprintf_P(tmpData, sizeof(tmpData),
                   PSTR("<span style='display:inline-block;text-align:center;margin-left:16%%'>"
                        "<button type='button' onclick=\"showModal(%d)\" style='background:#7c5cbf;border:none;border-radius:10px;color:#fff;padding:2px 0;font-size:10px;width:54px'>倒计时</button>"
                        "<br><span id='timer%d' style='font-size:10px;color:#888'>无倒计时</span>"
                        "</span>"
                        "</td></tr></table></div>"),
                   ch + 1, ch + 1);
        server->sendContent_P(tmpData);
    }

    server->sendContent_P(
        PSTR("</td></tr></tbody></table>"

             "<div id='modal' style='display:none;position:fixed;top:0;left:0;right:0;bottom:0;background:rgba(0,0,0,0.45);z-index:99;align-items:center;justify-content:center' onclick=\"if(event.target==this){closeModal()}\">"
             "<div style='background:#fff;border-radius:14px;padding:20px 24px;width:280px;max-width:86%;text-align:center'>"
             "<div id='mtitle' style='font-weight:bold;font-size:17px;margin-bottom:14px'>倒计时·开关1</div>"
             "<div style='text-align:left;font-size:13px;color:#888;margin-bottom:6px'>时长</div>"
             "<div style='text-align:left'>"
             "<input type='number' id='mh' min='0' max='24' style='width:62px;border:1px solid #ddd;border-radius:6px;padding:5px;font-size:14px' value='0'><span style='font-size:13px;color:#666'>&nbsp;小时</span>"
             "&nbsp;&nbsp;"
             "<input type='number' id='mm' min='0' max='59' style='width:62px;border:1px solid #ddd;border-radius:6px;padding:5px;font-size:14px' value='30'><span style='font-size:13px;color:#666'>&nbsp;分钟</span>"
             "</div>"
             "<div style='text-align:left;font-size:13px;color:#888;margin:14px 0 6px'>倒计时结束后</div>"
             "<div style='text-align:left'>"
             "<label style='font-size:14px'><input type='radio' name='mact' id='mrad_on' value='1' style='accent-color:#7c5cbf'> 开启开关</label>"
             "&nbsp;&nbsp;&nbsp;"
             "<label style='font-size:14px'><input type='radio' name='mact' id='mrad_off' value='0' style='accent-color:#7c5cbf' checked> 关闭开关</label>"
             "</div>"
             "<div style='font-size:12px;color:#aaa;margin-top:10px;line-height:1.5'>到点自动执行，不影响当前开关状态（可随时取消）</div>"
             "<div style='margin-top:16px;display:flex;justify-content:space-between'>"
             "<button type='button' onclick=\"timerCancelModal()\" style='background:none;border:none;color:#7c5cbf;font-size:14px;padding:4px'>取消倒计时</button>"
             "<button type='button' onclick=\"closeModal()\" style='background:none;border:none;color:#7c5cbf;font-size:14px;padding:4px'>取消</button>"
             "<button type='button' onclick=\"timerStartModal()\" style='background:none;border:none;color:#7c5cbf;font-size:14px;font-weight:bold;padding:4px'>开始</button>"
             "</div></div></div>"

             "<div id='modal2' style='display:none;position:fixed;top:0;left:0;right:0;bottom:0;background:rgba(0,0,0,0.45);z-index:99;align-items:center;justify-content:center' onclick=\"if(event.target==this){closeSchedModal()}\">"
             "<div style='background:#fff;border-radius:14px;padding:20px 24px;width:280px;max-width:86%;text-align:center'>"
             "<div id='stitle' style='font-weight:bold;font-size:17px;margin-bottom:14px'>定时任务·开关1</div>"
             "<div style='text-align:left;font-size:13px;color:#888;margin-bottom:6px'>开启时间</div>"
             "<div style='text-align:left'>"
             "<input type='number' id='son_h' min='0' max='23' style='width:62px;border:1px solid #ddd;border-radius:6px;padding:5px;font-size:14px' value='7'><span style='font-size:13px;color:#666'>&nbsp;时</span>"
             "&nbsp;&nbsp;"
             "<input type='number' id='son_m' min='0' max='59' style='width:62px;border:1px solid #ddd;border-radius:6px;padding:5px;font-size:14px' value='0'><span style='font-size:13px;color:#666'>&nbsp;分</span>"
             "</div>"
             "<div style='text-align:left;font-size:13px;color:#888;margin:14px 0 6px'>关闭时间</div>"
             "<div style='text-align:left'>"
             "<input type='number' id='soff_h' min='0' max='23' style='width:62px;border:1px solid #ddd;border-radius:6px;padding:5px;font-size:14px' value='21'><span style='font-size:13px;color:#666'>&nbsp;时</span>"
             "&nbsp;&nbsp;"
             "<input type='number' id='soff_m' min='0' max='59' style='width:62px;border:1px solid #ddd;border-radius:6px;padding:5px;font-size:14px' value='0'><span style='font-size:13px;color:#666'>&nbsp;分</span>"
             "</div>"
             "<div style='font-size:12px;color:#aaa;margin-top:10px;line-height:1.5'>每天到点自动开启/关闭开关（可随时取消）</div>"
             "<div style='margin-top:16px;display:flex;justify-content:space-between'>"
             "<button type='button' onclick=\"schedClearModal()\" style='background:none;border:none;color:#7c5cbf;font-size:14px;padding:4px'>清除定时</button>"
             "<button type='button' onclick=\"closeSchedModal()\" style='background:none;border:none;color:#7c5cbf;font-size:14px;padding:4px'>取消</button>"
             "<button type='button' onclick=\"schedSaveModal()\" style='background:none;border:none;color:#7c5cbf;font-size:14px;font-weight:bold;padding:4px'>保存</button>"
             "</div></div></div>"

             "<table class='gridtable'><thead><tr><th colspan='2'>电量统计</th></tr></thead><tbody>"
             "<tr colspan='2'><td><div style='width:260px;margin:0 auto;text-align:left'>"
             "&#12288;&#12288;&#12288;电压：<span id='voltage'>0</span> V"
             "<br>&#12288;&#12288;&#12288;电流：<span id='current'>0</span> A"
             "<br>&#12288;&#12288;&#12288;功率：<span id='power'>0</span> W"
             "<br>&#12288;视在功率：<span id='apparent_power'>0</span> VA"
             "<br>&#12288;无功功率：<span id='reactive_power'></span> VAr"
             "<br>&#12288;功率因数：<span id='factor'>0</span>"
             "<br>今日用电量：<span id='today'>0</span> kWh"
             "<br>昨日用电量：<span id='yesterday'>0</span> kWh"
             "<br>&#12288;总用电量：<span id='total'>0</span> kWh"
             "<br>&#12288;开始时间：<span id='starttime'>--</span>"
             "</div></td></tr></tbody></table>"));

    server->sendContent_P(
        PSTR("<form method='post' action='/dc1_setting' onsubmit='postform(this);return false'>"
             "<table class='gridtable'><thead><tr><th colspan='2'>DC1插线板设置</th></tr></thead><tbody>"
             "<tr><td>上电状态</td><td>"
             "<label class='bui-radios-label'><input type='radio' name='power_on_state' value='0'/><i class='bui-radios'></i> 开关通电时断开</label><br/>"
             "<label class='bui-radios-label'><input type='radio' name='power_on_state' value='1'/><i class='bui-radios'></i> 开关通电时闭合</label><br/>"
             "<label class='bui-radios-label'><input type='radio' name='power_on_state' value='2'/><i class='bui-radios'></i> 开关通电时状态与断电前相反</label><br/>"
             "<label class='bui-radios-label'><input type='radio' name='power_on_state' value='3'/><i class='bui-radios'></i> 开关通电时保持断电前状态</label>"
             "</td></tr>"));

    server->sendContent_P(
        PSTR("<tr><td>开关模式</td><td>"
             "<label class='bui-radios-label'><input type='radio' name='power_mode' value='0'/><i class='bui-radios'></i> 自锁</label>&nbsp;&nbsp;&nbsp;&nbsp;"
             "<label class='bui-radios-label'><input type='radio' name='power_mode' value='1'/><i class='bui-radios'></i> 互锁</label>"
             "</td></tr>"));

    server->sendContent_P(
        PSTR("<tr><td>LOGO LED</td><td>"
             "<label class='bui-radios-label'><input type='radio' name='logo_led' value='0'/><i class='bui-radios'></i> 常亮</label>&nbsp;&nbsp;&nbsp;&nbsp;"
             "<label class='bui-radios-label'><input type='radio' name='logo_led' value='1'/><i class='bui-radios'></i> 常灭</label>&nbsp;&nbsp;&nbsp;&nbsp;"
             "<label class='bui-radios-label'><input type='radio' name='logo_led' value='2'/><i class='bui-radios'></i> 跟随总开关</label>&nbsp;&nbsp;&nbsp;&nbsp;"
             "<label class='bui-radios-label'><input type='radio' name='logo_led' value='3'/><i class='bui-radios'></i> 与总开关相反</label>"
             "</td></tr>"));

    server->sendContent_P(
        PSTR("<tr><td>WIFI LED</td><td>"
             "<label class='bui-radios-label'><input type='radio' name='wifi_led' value='0'/><i class='bui-radios'></i> 常亮</label>&nbsp;&nbsp;&nbsp;&nbsp;"
             "<label class='bui-radios-label'><input type='radio' name='wifi_led' value='1'/><i class='bui-radios'></i> 常灭</label>&nbsp;&nbsp;&nbsp;&nbsp;"
             "<label class='bui-radios-label'><input type='radio' name='wifi_led' value='2'/><i class='bui-radios'></i> 闪烁</label><br>未连接WIFI或者MQTT时为快闪"
             "</td></tr>"));

    server->sendContent_P(
        PSTR("<tr><td>分开关联动</td><td>"
             "<label class='bui-radios-label'><input type='radio' name='sub_kinkage' value='0'/><i class='bui-radios'></i> 不联动</label>&nbsp;&nbsp;&nbsp;&nbsp;"
             "<label class='bui-radios-label'><input type='radio' name='sub_kinkage' value='1'/><i class='bui-radios'></i> 总关禁开</label>&nbsp;&nbsp;&nbsp;&nbsp;"
             "<label class='bui-radios-label'><input type='radio' name='sub_kinkage' value='2'/><i class='bui-radios'></i> 分开总开</label>"
             "</td></tr>"));

    snprintf_P(tmpData, sizeof(tmpData),
               PSTR("<tr><td>主动上报间隔</td><td><input type='number' min='0' max='3600' name='report_interval' required value='%d'>&nbsp;秒，0关闭</td></tr>"
                    "<tr><td>功率波动</td><td><input type='number' min='0' max='4000' name='energy_power_delta' required value='%d'>&nbsp;0关闭，1-100为百分比，>100是差值(-100)</td></tr>"
                    "<tr><td>过载保护</td><td><input type='number' min='0' max='2600' name='energy_max_power' required value='%d'>&nbsp;W&nbsp;&nbsp;&nbsp;&nbsp;0关闭</td></tr>"),
               config.report_interval, config.energy_power_delta, config.energy_max_power);
    server->sendContent_P(tmpData);

    server->sendContent_P(
        PSTR("<tr><td colspan='2'><button type='submit' class='btn-info'>设置</button><br>"
             "<button type='button' class='btn-success' style='margin-top:10px' onclick='window.location.href=\"/ha\"'>下载HA配置文件</button><br>"
             "<button type='button' class='btn-danger' style='margin-top:10px' onclick=\"javascript:if(confirm('确定要重置用电量？')){ajaxPost('/dc1_setting', 'c=1');}\">重置用电量</button></td></tr>"
             "</tbody></table></form>"));

#ifdef USE_HOMEKIT
    homekit_html(server);
#endif

    server->sendContent_P(
        PSTR("<script type='text/javascript'>"
             "function setDataSub(data,key){if(key.substr(0,5)=='power' && key.length==6){var n=key.substr(5,1);var v=data[key];var sw=id('sw'+n);sw.style.background=v==1?'#7c5cbf':'#ccc';var sl=sw.children[0];if(v==1){sl.style.right='3px';sl.style.left='auto'}else{sl.style.left='3px';sl.style.right='auto'}id('stat'+n).innerHTML=v==1?'已开启':'已关闭';return true}if(key.substr(0,5)=='timer' && key.length==6){var n=key.substr(5,1);var v=data[key];var tgt=data['timer'+n+'target'];var t=id('timer'+n);if(v>0){var m=Math.round(v/60*10)/10;t.innerHTML='剩余'+m+'min→'+(tgt=='on'?'开':'关');t.style.color='#c77'}else{t.innerHTML='无倒计时';t.style.color='#888'}return true}if(key.substr(0,5)=='sched' && key.length==6){var n=key.substr(5,1);var t=id('sched'+n);var v=data[key];if(v&&v!='--'){t.innerHTML=v;t.style.color='#7c5cbf'}else{t.innerHTML='无定时';t.style.color='#888'}return true}return false}"
             "function toggleSw(n){ajaxPost('/dc1_do','do=T&c='+n)}"
             "var curCh=1;function showModal(n){curCh=n;id('mtitle').innerHTML='倒计时·开关'+n;id('mh').value='0';id('mm').value='30';id('mrad_off').checked=true;id('modal').style.display='flex'}"
             "function closeModal(){id('modal').style.display='none'}"
             "function timerStartModal(){var h=parseInt(id('mh').value)||0;var m=parseInt(id('mm').value)||0;var sec=h*3600+m*60;if(sec<=0){toast('请输入倒计时时长',3000,false);return}var tgt=id('mrad_on').checked?'on':'off';ajaxPost('/dc1_setting','timer_ch='+curCh+'&timer_seconds='+sec+'&timer_target='+tgt);closeModal()}"
             "function timerCancelModal(){ajaxPost('/dc1_setting','timer_ch='+curCh+'&timer_seconds=0');closeModal()}"
             "function showSchedModal(n){curCh=n;id('stitle').innerHTML='定时任务·开关'+n;var s=schedInit[n-1];id('son_h').value=s[0]>=0?s[0]:7;id('son_m').value=s[1]>=0?s[1]:0;id('soff_h').value=s[2]>=0?s[2]:21;id('soff_m').value=s[3]>=0?s[3]:0;id('modal2').style.display='flex'}"
             "function closeSchedModal(){id('modal2').style.display='none'}"
             "function schedSaveModal(){var onh=id('son_h').value,onm=id('son_m').value,offh=id('soff_h').value,offm=id('soff_m').value;ajaxPost('/dc1_setting','sched_ch='+curCh+'&sched_on_hh='+onh+'&sched_on_mm='+onm+'&sched_off_hh='+offh+'&sched_off_mm='+offm,function(){var s=schedInit[curCh-1];s[0]=parseInt(onh);s[1]=parseInt(onm);s[2]=parseInt(offh);s[3]=parseInt(offm)});closeSchedModal()}"
             "function schedClearModal(){ajaxPost('/dc1_setting','sched_ch='+curCh+'&sched_clear=1',function(){var s=schedInit[curCh-1];s[0]=-1;s[1]=-1;s[2]=-1;s[3]=-1});closeSchedModal()}"));

    snprintf_P(tmpData, sizeof(tmpData),
               PSTR("setRadioValue('power_on_state', '%d');"
                    "setRadioValue('power_mode', '%d');"
                    "setRadioValue('logo_led', '%d');"
                    "setRadioValue('wifi_led', '%d');"
                    "setRadioValue('sub_kinkage', '%d');"),
               config.power_on_state, config.power_mode, config.logo_led, config.wifi_led, config.sub_kinkage);
    server->sendContent_P(tmpData);

    // 定时任务初始值: [on_h,on_m,off_h,off_m] per channel, -1=禁用
    snprintf_P(tmpData, sizeof(tmpData),
               PSTR("var schedInit=[[%d,%d,%d,%d],[%d,%d,%d,%d],[%d,%d,%d,%d],[%d,%d,%d,%d]];"),
               (config.sched_on_count > 0 && config.sched_on[0] != 0xFFFFFFFF) ? (int)(config.sched_on[0] / 60) : -1,
               (config.sched_on_count > 0 && config.sched_on[0] != 0xFFFFFFFF) ? (int)(config.sched_on[0] % 60) : -1,
               (config.sched_off_count > 0 && config.sched_off[0] != 0xFFFFFFFF) ? (int)(config.sched_off[0] / 60) : -1,
               (config.sched_off_count > 0 && config.sched_off[0] != 0xFFFFFFFF) ? (int)(config.sched_off[0] % 60) : -1,
               (config.sched_on_count > 1 && config.sched_on[1] != 0xFFFFFFFF) ? (int)(config.sched_on[1] / 60) : -1,
               (config.sched_on_count > 1 && config.sched_on[1] != 0xFFFFFFFF) ? (int)(config.sched_on[1] % 60) : -1,
               (config.sched_off_count > 1 && config.sched_off[1] != 0xFFFFFFFF) ? (int)(config.sched_off[1] / 60) : -1,
               (config.sched_off_count > 1 && config.sched_off[1] != 0xFFFFFFFF) ? (int)(config.sched_off[1] % 60) : -1,
               (config.sched_on_count > 2 && config.sched_on[2] != 0xFFFFFFFF) ? (int)(config.sched_on[2] / 60) : -1,
               (config.sched_on_count > 2 && config.sched_on[2] != 0xFFFFFFFF) ? (int)(config.sched_on[2] % 60) : -1,
               (config.sched_off_count > 2 && config.sched_off[2] != 0xFFFFFFFF) ? (int)(config.sched_off[2] / 60) : -1,
               (config.sched_off_count > 2 && config.sched_off[2] != 0xFFFFFFFF) ? (int)(config.sched_off[2] % 60) : -1,
               (config.sched_on_count > 3 && config.sched_on[3] != 0xFFFFFFFF) ? (int)(config.sched_on[3] / 60) : -1,
               (config.sched_on_count > 3 && config.sched_on[3] != 0xFFFFFFFF) ? (int)(config.sched_on[3] % 60) : -1,
               (config.sched_off_count > 3 && config.sched_off[3] != 0xFFFFFFFF) ? (int)(config.sched_off[3] / 60) : -1,
               (config.sched_off_count > 3 && config.sched_off[3] != 0xFFFFFFFF) ? (int)(config.sched_off[3] % 60) : -1);
    server->sendContent_P(tmpData);

    server->sendContent_P(PSTR("</script>"));
}

void DC1::httpDo(ESP8266WebServer *server)
{
    String c = server->arg(F("c"));
    if (c != F("1") && c != F("2") && c != F("3") && c != F("4"))
    {
        server->send_P(200, PSTR("application/json"), PSTR("{\"code\":0,\"msg\":\"参数错误。\"}"));
        return;
    }
    uint8_t ch = c.toInt() - 1;
    if (ch > channels)
    {
        server->send_P(200, PSTR("application/json"), PSTR("{\"code\":0,\"msg\":\"继电器数量错误。\"}"));
        return;
    }
    String str = server->arg(F("do"));
    switchRelay(ch, (str == "on" ? true : (str == "off" ? false : !bitRead(lastState, ch))));

    server->setContentLength(CONTENT_LENGTH_UNKNOWN);
    server->send_P(200, PSTR("application/json"), PSTR("{\"code\":1,\"msg\":\"操作成功\",\"data\":{"));
    server->sendContent(httpGetStatus(server));
    server->sendContent_P(PSTR("}}"));
}

void DC1::httpSetting(ESP8266WebServer *server)
{
    if (server->hasArg(F("c")))
    {
        energyClear();
        server->setContentLength(CONTENT_LENGTH_UNKNOWN);
        server->send_P(200, PSTR("application/json"), PSTR("{\"code\":1,\"msg\":\"重置用电量成功。\",\"data\":{"));
        server->sendContent(httpGetStatus(server));
        server->sendContent_P(PSTR("}}"));
        return;
    }
    config.power_on_state = server->arg(F("power_on_state")).toInt();
    config.power_mode = server->arg(F("power_mode")).toInt();
    config.logo_led = server->arg(F("logo_led")).toInt();
    config.wifi_led = server->arg(F("wifi_led")).toInt();
    config.sub_kinkage = server->arg(F("sub_kinkage")).toInt();

    config.report_interval = server->arg(F("report_interval")).toInt();
    config.energy_power_delta = server->arg(F("energy_power_delta")).toInt();
    config.energy_max_power = server->arg(F("energy_max_power")).toInt();

    // 倒计时: timer_ch=1..4, timer_seconds=剩余秒数(0=取消), timer_target=on|off
    if (server->hasArg(F("timer_ch")))
    {
        uint8_t timerCh = server->arg(F("timer_ch")).toInt();
        uint32_t timerSec = server->arg(F("timer_seconds")).toInt();
        if (timerCh >= 1 && timerCh <= channels)
        {
            if (timerSec > 0)
            {
                timerSet(timerCh - 1, timerSec, server->arg(F("timer_target")) == F("on"));
            }
            else
            {
                timerCancel(timerCh - 1);
            }
        }
    }

    // 定时任务: sched_ch=1..4 + sched_on_hh/sched_on_mm/sched_off_hh/sched_off_mm 或 sched_clear=1
    if (server->hasArg(F("sched_ch")))
    {
        uint8_t schedCh = server->arg(F("sched_ch")).toInt();
        if (schedCh >= 1 && schedCh <= channels)
        {
            if (server->hasArg(F("sched_clear")))
            {
                scheduleClear(schedCh - 1);
            }
            else
            {
                int onM = -1, offM = -1;
                if (server->hasArg(F("sched_on_hh")))
                {
                    onM = server->arg(F("sched_on_hh")).toInt() * 60 + server->arg(F("sched_on_mm")).toInt();
                }
                if (server->hasArg(F("sched_off_hh")))
                {
                    offM = server->arg(F("sched_off_hh")).toInt() * 60 + server->arg(F("sched_off_mm")).toInt();
                }
                scheduleSet(schedCh - 1, onM, offM);
            }
        }
    }

    logoLed();

    Config::saveConfig();
    server->send_P(200, PSTR("application/json"), PSTR("{\"code\":1,\"msg\":\"已经设置成功。\"}"));
}

void DC1::httpHa(ESP8266WebServer *server)
{
    char attachment[100];
    snprintf_P(attachment, sizeof(attachment), PSTR("attachment; filename=%s.yaml"), UID);

    server->setContentLength(CONTENT_LENGTH_UNKNOWN);
    server->sendHeader(F("Content-Disposition"), attachment);
    server->send_P(200, PSTR("Content-Type: application/octet-stream"), "");

    String availability = Mqtt::getTeleTopic(F("availability"));
    char cmndTopic[100];
    strcpy(cmndTopic, Mqtt::getCmndTopic(F("power1")).c_str());
    server->sendContent(F("switch:\r\n"));
    for (size_t ch = 0; ch < channels; ch++)
    {
        cmndTopic[strlen(cmndTopic) - 1] = ch + 49;           // 48 + 1 + ch
        powerStatTopic[strlen(powerStatTopic) - 1] = ch + 49; // 48 + 1 + ch

        snprintf_P(tmpData, sizeof(tmpData),
                   PSTR("  - platform: mqtt\r\n"
                        "    name: \"%s_%d\"\r\n"
                        "    unique_id: \"%s_%d\"\r\n"
                        "    state_topic: \"%s\"\r\n"
                        "    command_topic: \"%s\"\r\n"
                        "    payload_on: \"on\"\r\n"
                        "    payload_off: \"off\"\r\n"
                        "    availability_topic: \"%s\"\r\n"
                        "    payload_available: \"online\"\r\n"
                        "    payload_not_available: \"offline\"\r\n\r\n"),
                   UID, ch + 1,
                   UID, ch + 1,
                   powerStatTopic, cmndTopic, availability.c_str());
        server->sendContent_P(tmpData);
    }

    String tims[] = {F("voltage"), F("current"), F("power"), F("apparent_power"), F("reactive_power"), F("factor"), F("total"), F("yesterday"), F("today"), F("starttime")};
    String tims2[] = {F("V"), F("A"), F("W"), F("VA"), F("VAr"), F(""), F("kWh"), F("kWh"), F("kWh"), F("")};
    server->sendContent_P(PSTR("sensor:\r\n"));
    for (size_t i = 0; i < 10; i++)
    {
        if (tims2[i].length() == 0)
        {
            snprintf_P(tmpData, sizeof(tmpData),
                       PSTR("  - platform: mqtt\r\n"
                            "    name: \"%s_%s\"\r\n"
                            "    unique_id: \"%s_%s\"\r\n"
                            "    state_topic: \"%s\"\r\n"
                            "    value_template: \"{{value_json.%s}}\"\r\n\r\n"),
                       UID, tims[i].c_str(),
                       UID, tims[i].c_str(),
                       energyTeleTopic, tims[i].c_str());
        }
        else
        {
            snprintf_P(tmpData, sizeof(tmpData),
                       PSTR("  - platform: mqtt\r\n"
                            "    name: \"%s_%s\"\r\n"
                            "    unique_id: \"%s_%s\"\r\n"
                            "    state_topic: \"%s\"\r\n"
                            "    value_template: \"{{value_json.%s}}\"\r\n"
                            "    unit_of_measurement: \"%s\"\r\n\r\n"),
                       UID, tims[i].c_str(),
                       UID, tims[i].c_str(),
                       energyTeleTopic, tims[i].c_str(), tims2[i].c_str());
        }
        server->sendContent_P(tmpData);
    }
}
#pragma endregion

void DC1::logoLed()
{
    if (config.logo_led == 0)
    {
        digitalWrite(LOGO_LED_PIN, LOW);
    }
    else if (config.logo_led == 1)
    {
        digitalWrite(LOGO_LED_PIN, HIGH);
    }
    else if (config.logo_led == 2)
    {
        digitalWrite(LOGO_LED_PIN, bitRead(lastState, 0) ? LOW : HIGH);
    }
    else if (config.logo_led == 3)
    {
        digitalWrite(LOGO_LED_PIN, bitRead(lastState, 0) ? HIGH : LOW);
    }
}

void DC1::switchRelay(uint8_t ch, bool isOn, bool isSave)
{
    if (ch > channels)
    {
        Debug::AddInfo(PSTR("invalid channel: %d"), ch);
        return;
    }

    if (ch > 0 || (ch == 0 && config.sub_kinkage == 0))
    {
        if (!bitRead(lastState, 0) && isOn && config.sub_kinkage != 0)
        {
            if (config.sub_kinkage == 1 || !isSave)
            {
                isOn = false;
            }
            else if (config.sub_kinkage == 2)
            {
                switchRelay(0, true);
            }
        }

        if (isOn && config.power_mode == 1)
        {
            for (size_t ch2 = (config.sub_kinkage == 0 ? 0 : 1); ch2 < channels; ch2++)
            {
                if (ch2 != ch && bitRead(lastState, ch2))
                {
                    switchRelay(ch2, false, isSave);
                }
            }
        }
    }
    Debug::AddInfo(PSTR("Relay %d . . . %s"), ch + 1, isOn ? "ON" : "OFF");

    if (!cat9554->digitalWrite(relGPIO[ch], isOn ? HIGH : LOW))
    {
        Debug::AddError(PSTR("CAT9554 digitalWrite Error"));
        if (!cat9554->digitalWrite(relGPIO[ch], isOn ? HIGH : LOW))
        {
            Debug::AddError(PSTR("CAT9554 digitalWrite Error2"));
            return;
        }
    }

    bitWrite(lastState, ch, isOn);

    powerStatTopic[strlen(powerStatTopic) - 1] = ch + 49; // 48 + 1 + ch
    Mqtt::publish(powerStatTopic, isOn ? "on" : "off", globalConfig.mqtt.retain);

    if (isSave && config.power_on_state > 0)
    {
        bitWrite(config.last_state, ch, isOn);
        bitSet(operationFlag, 1);
        Config::delaySaveConfig(10);
    }

    if (ch == 0)
    {
        logoLed();
        if (isSave && config.sub_kinkage != 0)
        {
            for (size_t ch2 = 1; ch2 < channels; ch2++)
            {
                if (isOn)
                {
                    if (bitRead(config.last_state, ch2))
                    {
                        switchRelay(ch2, true, false);
                    }
                }
                else
                {
                    switchRelay(ch2, false, false);
                }
            }
        }
    }
}

void DC1::checkButton(uint8_t ch)
{
    bool buttonState = ch == 0 ? digitalRead(btnGPIO[ch]) : cat9554->digitalRead(btnGPIO[ch]);

    if (buttonState == 0)
    {
        if (!bitRead(buttonTiming, ch))
        {
            bitSet(buttonTiming, ch);
            buttonTimingStart[ch] = millis();
        }
        else
        { // buttonTiming = true
            if (millis() >= (buttonTimingStart[ch] + buttonDebounceTime))
            {
                buttonAction[ch] = 1;
            }
            if (millis() >= (buttonTimingStart[ch] + buttonLongPressTime))
            {
                buttonAction[ch] = 2;
            }
        }
    }
    else
    {
        bitClear(buttonTiming, ch);
        if (buttonAction[ch] != 0)
        {
            if (buttonAction[ch] == 1) // 执行短按动作
            {
                switchRelay(ch, !bitRead(lastState, ch), true);
            }
            else if (buttonAction[ch] == 2) // 执行长按动作
            {
                if (ch == 0)
                {
                    Wifi::setupWifiManager(false);
                }
            }
            buttonAction[ch] = 0;
        }
    }
}

void DC1::energyUpdate()
{
    if (Rtc::rtcTime.valid && config.energy_kWhtotal_time == 0)
    {
        config.energy_kWhtotal_time = Rtc::utcTime;
        TIME_T tmpTime;
        Rtc::breakTime(config.energy_kWhtotal_time, tmpTime);
        snprintf_P(kWhtotalTime, sizeof(kWhtotalTime), PSTR("%04d-%02d-%02d %02d:%02d:%02d"), tmpTime.year, tmpTime.month, tmpTime.day_of_month, tmpTime.hour, tmpTime.minute, tmpTime.second);
    }
    if (Rtc::rtcTime.valid && config.energy_kWhdoy != Rtc::rtcTime.day_of_year)
    {
        Debug::AddInfo(PSTR("day_of_year: %d %d %d"), Rtc::rtcTime.day_of_year, Rtc::rtcTime.day_of_month, Rtc::rtcTime.day_of_week);

        energySync();
        config.energy_kWhdoy = Rtc::rtcTime.day_of_year;
        config.energy_kWhyesterday = config.energy_kWhtoday;
        config.energy_kWhtoday = 0;
        Config::saveConfig();

        cse7766->Energy.daily = (float)(config.energy_kWhtoday + cse7766->Energy.kWhtoday) / 100000;
        cse7766->Energy.total = (float)(config.energy_kWhtotal + cse7766->Energy.kWhtoday) / 100000;
    }
    if (cse7766->everySecond())
    {
        energyUpdateToday();
    }
    if (perSecond % 3600 == 0 && cse7766->Energy.kWhtoday > 0)
    {
        energySync();
        Config::saveConfig();
    }
    energyMarginCheck();
}

void DC1::energySync()
{
    if (cse7766->Energy.kWhtoday > 0)
    {
        config.energy_kWhtoday += cse7766->Energy.kWhtoday;
        config.energy_kWhtotal += cse7766->Energy.kWhtoday;
        cse7766->Energy.kWhtoday = 0;
    }
}

void DC1::energyInit()
{
    cse7766->Energy.kWhtoday = 0;
    cse7766->Energy.kWhtoday_delta = 0;
    cse7766->Energy.daily = (float)(config.energy_kWhtoday) / 100000;
    cse7766->Energy.total = (float)(config.energy_kWhtotal) / 100000;

    TIME_T tmpTime;
    Rtc::breakTime(config.energy_kWhtotal_time, tmpTime);
    snprintf_P(kWhtotalTime, sizeof(kWhtotalTime), PSTR("%04d-%02d-%02d %02d:%02d:%02d"), tmpTime.year, tmpTime.month, tmpTime.day_of_month, tmpTime.hour, tmpTime.minute, tmpTime.second);
}

void DC1::energyClear()
{
    config.energy_kWhtoday = 0;
    config.energy_kWhyesterday = 0;
    config.energy_kWhtotal = 0;
    config.energy_kWhdoy = 0;
    config.energy_kWhtotal_time = 0;

    cse7766->Energy.kWhtoday_delta = 0;

    energyInit();
    Config::saveConfig();
}

void DC1::energyUpdateToday()
{
    if (cse7766->Energy.kWhtoday_delta > 1000)
    {
        unsigned long delta = cse7766->Energy.kWhtoday_delta / 1000;
        cse7766->Energy.kWhtoday_delta -= (delta * 1000);
        cse7766->Energy.kWhtoday += delta;

        cse7766->Energy.daily = (float)(config.energy_kWhtoday + cse7766->Energy.kWhtoday) / 100000;
        cse7766->Energy.total = (float)(config.energy_kWhtotal + cse7766->Energy.kWhtoday) / 100000;
    }
}

void DC1::energyMarginCheck()
{
    if (cse7766->Energy.power_steady_counter)
    {
        cse7766->Energy.power_steady_counter--;
        return;
    }

    uint16_t energy_power_u = (uint16_t)(cse7766->Energy.active_power);
    uint16_t energy_voltage_u = (uint16_t)(cse7766->Energy.voltage);
    uint16_t energy_current_u = (uint16_t)(cse7766->Energy.current * 1000);

    //Debug::AddInfo(PSTR("NRG: W %d, U %d, I %d"), energy_power_u, energy_voltage_u, energy_current_u);
    if (config.energy_power_delta)
    {
        uint16_t delta = abs(cse7766->Energy.power_history[0] - energy_power_u);
        if (delta > 0)
        {
            if (config.energy_power_delta < 101)
            { // 1..100 = Percentage
                uint16_t min_power = (cse7766->Energy.power_history[0] > energy_power_u) ? energy_power_u : cse7766->Energy.power_history[0];
                if (0 == min_power)
                {
                    min_power++;
                } // Fix divide by 0 exception (#6741)
                if (((delta * 100) / min_power) > config.energy_power_delta)
                {
                    cse7766->Energy.power_delta = true;
                }
            }
            else
            { // 101..32000 = Absolute
                if (delta > (config.energy_power_delta - 100))
                {
                    cse7766->Energy.power_delta = true;
                }
            }
            if (cse7766->Energy.power_delta)
            {
                cse7766->Energy.power_history[1] = cse7766->Energy.active_power; // We only want one report so reset history
                cse7766->Energy.power_history[2] = cse7766->Energy.active_power;
            }
        }
    }

    cse7766->Energy.power_history[0] = cse7766->Energy.power_history[1]; // Shift in history every second allowing power changes to settle for up to three seconds
    cse7766->Energy.power_history[1] = cse7766->Energy.power_history[2];
    cse7766->Energy.power_history[2] = energy_power_u;

    if (config.report_interval > 0 && (perSecond % config.report_interval) == 0)
    {
        reportPower();
        cse7766->Energy.power_delta = true;
    }
    if (cse7766->Energy.power_delta)
    {
        cse7766->Energy.power_delta = false;
        reportEnergy();
    }
    if (config.energy_max_power)
    {
        DC1::energyMaxPower();
    }
}

void DC1::energyMaxPower()
{
    if (cse7766->Energy.active_power > config.energy_max_power)
    {
        if (!cse7766->Energy.mplh_counter)
        {
            cse7766->Energy.mplh_counter = MAX_POWER_HOLD;
        }
        else
        {
            cse7766->Energy.mplh_counter--;
            if (!cse7766->Energy.mplh_counter)
            {
                Debug::AddError(PSTR("MaxPowerReached: %d"), (uint16_t)cse7766->Energy.active_power);
                lastState2 = lastState;
                for (size_t ch = 0; ch < channels; ch++)
                {
                    if (bitRead(lastState, ch))
                    {
                        switchRelay(ch, false, false);
                    }
                }
                if (!cse7766->Energy.mplr_counter)
                {
                    cse7766->Energy.mplr_counter = MAX_POWER_RETRY + 1;
                }
                cse7766->Energy.mplw_counter = MAX_POWER_WINDOW;
                cse7766->Energy.mplv_counter = 0;
            }
        }
    }
    else if (lastState && (cse7766->Energy.active_power <= config.energy_max_power))
    {
        cse7766->Energy.mplh_counter = 0;
        cse7766->Energy.mplw_counter = 0;

        if (cse7766->Energy.mplv_counter++ == 60)
        {
            cse7766->Energy.mplv_counter = 0;
            cse7766->Energy.mplr_counter = 0;
        }
    }
    if (!lastState)
    {
        if (cse7766->Energy.mplw_counter)
        {
            cse7766->Energy.mplw_counter--;
        }
        else
        {
            if (cse7766->Energy.mplr_counter)
            {
                cse7766->Energy.mplr_counter--;
                if (cse7766->Energy.mplr_counter)
                {
                    Debug::AddError(PSTR("PowerMonitor ON"));
                    if (lastState2)
                    {
                        for (size_t ch = 0; ch < channels; ch++)
                        {
                            if (bitRead(lastState2, ch))
                            {
                                switchRelay(ch, true, false);
                            }
                        }
                        lastState2 = 0;
                    }
                }
                else
                {
                    Debug::AddInfo(PSTR("MaxPowerReachedRetry OFF"));
                }
            }
        }
    }
}

void DC1::energyShow(bool isMqtt)
{
    const uint8_t current_resolution = 3;
    const uint8_t voltage_resolution = 0;
    const uint8_t wattage_resolution = 0;
    const uint8_t energy_resolution = 3;

    float apparent_power = cse7766->Energy.apparent_power;
    if (isnan(apparent_power))
    {
        apparent_power = cse7766->Energy.voltage * cse7766->Energy.current;
    }
    if (apparent_power < cse7766->Energy.active_power)
    { // Should be impossible
        cse7766->Energy.active_power = apparent_power;
    }

    float power_factor = cse7766->Energy.power_factor;
    if (isnan(power_factor))
    {
        power_factor = (cse7766->Energy.active_power && apparent_power) ? cse7766->Energy.active_power / apparent_power : 0;
        if (power_factor > 1)
        {
            power_factor = 1;
        }
    }

    float reactive_power = cse7766->Energy.reactive_power;
    if (isnan(reactive_power))
    {
        reactive_power = 0;
        uint32_t difference = ((uint32_t)(apparent_power * 100) - (uint32_t)(cse7766->Energy.active_power * 100)) / 10;
        if ((cse7766->Energy.current > 0.005) && ((difference > 15) || (difference > (uint32_t)(apparent_power * 100 / 1000))))
        {
            // calculating reactive power only if current is greater than 0.005A and
            // difference between active and apparent power is greater than 1.5W or 1%
            reactive_power = (float)(Util::RoundSqrtInt((uint32_t)(apparent_power * apparent_power * 100) - (uint32_t)(cse7766->Energy.active_power * cse7766->Energy.active_power * 100))) / 10;
        }
    }

    char apparent_power_chr[16];
    char reactive_power_chr[16];
    char power_factor_chr[16];
    Util::dtostrfd(apparent_power, wattage_resolution, apparent_power_chr);
    Util::dtostrfd(reactive_power, wattage_resolution, reactive_power_chr);
    Util::dtostrfd(power_factor, 2, power_factor_chr);

    char voltage_chr[16];
    char current_chr[16];
    char active_power_chr[16];
    Util::dtostrfd(cse7766->Energy.voltage, voltage_resolution, voltage_chr);
    Util::dtostrfd(cse7766->Energy.current, current_resolution, current_chr);
    Util::dtostrfd(cse7766->Energy.active_power, wattage_resolution, active_power_chr);

    char energy_daily_chr[16];
    char energy_yesterday_chr[16];
    char energy_total_chr[16];
    Util::dtostrfd(cse7766->Energy.daily, energy_resolution, energy_daily_chr);
    Util::dtostrfd((float)config.energy_kWhyesterday / 100000, energy_resolution, energy_yesterday_chr);
    Util::dtostrfd(cse7766->Energy.total, energy_resolution, energy_total_chr);

    snprintf_P(tmpData, sizeof(tmpData),
               PSTR("%s\"starttime\":\"%s\",\"total\":\"%s\",\"yesterday\":\"%s\",\"today\":\"%s\","
                    "\"voltage\":\"%s\",\"current\":\"%s\",\"power\":\"%s\","
                    "\"apparent_power\":\"%s\",\"reactive_power\":\"%s\",\"factor\":\"%s\"%s"),
               isMqtt ? PSTR("{") : PSTR(","),
               kWhtotalTime,
               energy_total_chr, energy_yesterday_chr, energy_daily_chr,
               voltage_chr, current_chr, active_power_chr,
               apparent_power_chr, reactive_power_chr, power_factor_chr,
               isMqtt ? PSTR("}") : PSTR(""));
}

void DC1::reportEnergy()
{
    energyShow(true);
    Mqtt::publish(energyTeleTopic, tmpData, globalConfig.mqtt.retain);
}

void DC1::reportPower()
{
    for (size_t ch = 0; ch < channels; ch++)
    {
        powerStatTopic[strlen(powerStatTopic) - 1] = ch + 49; // 48 + 1 + ch
        Mqtt::publish(powerStatTopic, bitRead(lastState, ch) ? "on" : "off", globalConfig.mqtt.retain);
    }
}
