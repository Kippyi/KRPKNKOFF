@echo off
rem Builds the standalone KartSim test runner with MSVC (no Unreal required).
rem Usage: Tests\build_tests.bat   (from any directory)
setlocal
set ROOT=%~dp0..
set OUT=%ROOT%\Intermediate\KartSimTests
if not exist "%OUT%" mkdir "%OUT%"

where cl >nul 2>nul
if errorlevel 1 (
    call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
)

cl /nologo /std:c++17 /O2 /W4 /WX /EHsc /permissive- /D_CRT_SECURE_NO_WARNINGS ^
   /I"%ROOT%\Source\KartSimCore\Public" ^
   /Fo"%OUT%\\" /Fe"%OUT%\KartSimTests.exe" ^
   "%ROOT%\Tests\Main.cpp" "%ROOT%\Tests\UnitTests.cpp" "%ROOT%\Tests\Scenarios.cpp" "%ROOT%\Tests\Maneuvers.cpp" "%ROOT%\Tests\ClassTests.cpp" "%ROOT%\Tests\EnergyTests.cpp" "%ROOT%\Tests\SetupTests.cpp" "%ROOT%\Tests\LapTimingTests.cpp" "%ROOT%\Tests\TrackConditionTests.cpp" "%ROOT%\Tests\SessionTests.cpp" "%ROOT%\Tests\AiTests.cpp" "%ROOT%\Tests\TyreWearTests.cpp" "%ROOT%\Tests\ReplayTests.cpp" "%ROOT%\Tests\LaunchTests.cpp" "%ROOT%\Tests\SoundRender.cpp" "%ROOT%\Tests\EngineDamageTests.cpp" "%ROOT%\Tests\AcousticsTests.cpp" "%ROOT%\Tests\LicenseTests.cpp" "%ROOT%\Tests\FlexTests.cpp" ^
   "%ROOT%\Source\KartSimCore\Private\KartSim\*.cpp"
exit /b %errorlevel%
