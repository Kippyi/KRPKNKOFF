@echo off
rem Packages the TESTER build: Shipping configuration (no console, developer -Kart test flags stripped, license
rem required), no debug symbols (.pdb), content + kart data in the AES-encrypted pak (Config/DefaultCrypto.ini).
rem Output: Build\Tester\Windows and Build\KRPKNKOFF_Tester_<date>.zip
rem Works with the editor open (editor binaries must already be built: -nocompileeditor).
setlocal
set ROOT=%~dp0..
if not exist "%ROOT%\Config\DefaultCrypto.ini" (
    echo Config\DefaultCrypto.ini missing: the pak would not be encrypted. Aborting.
    exit /b 1
)
findstr /c:"KrpkLicenseModulusHex = \"\"" "%ROOT%\Source\KartSimCore\Private\KartSim\LicensePublicKey.inc" >nul
if not errorlevel 1 (
    echo No license public key: run  python Tools\License\krpk_license.py init  first. Aborting.
    exit /b 1
)
if exist "%ROOT%\Build\Tester" rmdir /s /q "%ROOT%\Build\Tester"
call "E:\UE_5.8\Engine\Build\BatchFiles\RunUAT.bat" BuildCookRun -project="%ROOT%\KRPKNKOFF.uproject" -noP4 -platform=Win64 -clientconfig=Shipping -build -nocompileeditor -cook -stage -pak -iostore -compressed -prereqs -nodebuginfo -archive -archivedirectory="%ROOT%\Build\Tester" -utf8output -unattended
if errorlevel 1 exit /b 1
rem No symbols in the tester zip (they would map the executable back to every function name).
del /s /q "%ROOT%\Build\Tester\Windows\*.pdb" >nul 2>nul
rem UE lists every file inside the pak in these manifests: not needed to run the game.
del /q "%ROOT%\Build\Tester\Windows\Manifest_*.txt" >nul 2>nul
rmdir /s /q "%ROOT%\Build\Tester\Windows\KRPKNKOFF\Saved" 2>nul
rem Steam (multiplayer) in a Shipping build needs the app id next to the executable: 480 = Valve's test app until
rem KRPKNKOFF has its own Steam app id.
for %%D in ("%ROOT%\Build\Tester\Windows\KRPKNKOFF\Binaries\Win64") do echo 480> "%%~D\steam_appid.txt"
copy /y "%ROOT%\Docs\TesterReadme.txt" "%ROOT%\Build\Tester\Windows\README.txt" >nul
for /f %%d in ('powershell -NoProfile -Command "Get-Date -Format yyyy-MM-dd"') do set D=%%d
"%SystemRoot%\System32\tar.exe" -a -c -f "%ROOT%\Build\KRPKNKOFF_Tester_%D%.zip" -C "%ROOT%\Build\Tester" Windows
echo Done: %ROOT%\Build\KRPKNKOFF_Tester_%D%.zip
