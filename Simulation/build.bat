@echo off
REM build.bat - Windows MSVC ile derleme
REM Visual Studio Developer Command Prompt'tan calistirin

echo.
echo ========================================
echo   IMU Simulasyon Projesi Derleme
echo ========================================
echo.

REM Derleme
cl /nologo /W3 /O2 /I inc ^
   main.c ^
   src\lsm6dsm_driver.c ^
   src\lsm6dsm_process.c ^
   src\filter.c ^
   src\ring_buffer.c ^
   src\timer.c ^
   src\imu_sim.c ^
   /Fe:imu_sim.exe

if %ERRORLEVEL% == 0 (
    echo.
    echo Derleme basarili: imu_sim.exe
    echo.
    echo Kullanim: imu_sim.exe [senaryo_no]
    echo   0 veya bos: Tum senaryolari calistir
    echo   1: Normal calisma
    echo   2: Veri kaybi testi
    echo   3: Spike + 50 Hz EMI filtreleme testi
    echo   4: Gecikmeli veri testi
) else (
    echo.
    echo HATA: Derleme basarisiz!
)

REM Ara dosyalari temizle
del /Q *.obj 2>nul

echo.
