@echo off
chcp 65001 >nul
REM ============================================
REM DC1 插线板固件刷写脚本
REM 用法: flash_dc1.bat COM端口号
REM 例如: flash_dc1.bat COM5
REM ============================================
if "%1"=="" (
    echo 用法: flash_dc1.bat COM端口号  （例如 flash_dc1.bat COM5）
    echo 串口号在设备管理器-端口(COM和LPT) 里查看
    exit /b 1
)

set ESPTOOL=C:\Users\Administrator\AppData\Local\Programs\Python\Python311\Scripts\esptool.exe
set FW=J:\tmp\esp_dc1_sync\output\dc1.bin

echo.
echo 即将刷写: %FW%
echo 目标端口: %1
echo 固件含定时任务功能 (2026-08-16 构建)
echo.
echo 警告: 刷写时插线板必须断电(不插220V), 由USB-TTL的3.3V供电!
echo.
pause

"%ESPTOOL%" --port %1 --baud 921600 --before default_reset --after hard_reset write_flash -fs 1MB -fm dout -ff 40m 0x00000 "%FW%"

if %errorlevel% equ 0 (
    echo.
    echo ========================================
    echo 刷写成功! 断开串口线, 重新上电即可使用
    echo ========================================
) else (
    echo.
    echo 刷写失败, 请检查接线(GND/TX/RX/3.3V)和COM口号
)
pause
