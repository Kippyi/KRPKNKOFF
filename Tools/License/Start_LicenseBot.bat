@echo off
rem Starts the KRPKNKOFF Discord license bot (keep this window open while testers should be able to /activate).
rem Uses its own Python environment (.venv) so other Discord libraries on this PC (nextcord) cannot break it.
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
    echo First start: creating the bot's Python environment...
    python -m venv .venv || goto :fail
    ".venv\Scripts\python.exe" -m pip install --quiet "discord.py==2.6.0" "cryptography==48.0.0" || goto :fail
)
".venv\Scripts\python.exe" krpk_discord_bot.py %*
pause
exit /b 0
:fail
echo Could not set up Python for the bot (is Python 3 installed and on PATH?).
pause
exit /b 1
