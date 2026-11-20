@echo off
rem Packages KRPKNKOFF for Windows testers -> Build\Windows and Build\KRPKNKOFF_TestBuild_<date>.zip
rem Works with the editor open (editor binaries must already be built: -nocompileeditor).
setlocal
set ROOT=%~dp0..
call "E:\UE_5.8\Engine\Build\BatchFiles\RunUAT.bat" BuildCookRun -project="%ROOT%\KRPKNKOFF.uproject" -noP4 -platform=Win64 -clientconfig=Development -build -nocompileeditor -cook -stage -pak -iostore -compressed -prereqs -archive -archivedirectory="%ROOT%\Build" -utf8output -unattended
if errorlevel 1 exit /b 1
rmdir /s /q "%ROOT%\Build\Windows\KRPKNKOFF\Saved" 2>nul
rem Kart data now lives in the encrypted pak; remove loose copies left by older packages (stale data).
rmdir /s /q "%ROOT%\Build\Windows\KRPKNKOFF\Content\KartData" 2>nul
copy /y "%ROOT%\Docs\PlayerReadme.txt" "%ROOT%\Build\Windows\README.txt" >nul
for /f %%d in ('powershell -NoProfile -Command "Get-Date -Format yyyy-MM-dd"') do set D=%%d
"%SystemRoot%\System32\tar.exe" -a -c -f "%ROOT%\Build\KRPKNKOFF_TestBuild_%D%.zip" -C "%ROOT%\Build" Windows
echo Done: %ROOT%\Build\KRPKNKOFF_TestBuild_%D%.zip
