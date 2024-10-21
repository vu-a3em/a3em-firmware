@echo off
for /f "tokens=2 delims==" %%a in ('wmic logicaldisk where "drivetype=2" get name /value') do set "driveLetter=%%~a"
if [%driveLetter%] == [] echo No SD cards found & exit /b 0
set /p response=Format drive %driveLetter% for A3EM? [Y/N]: 
if /I '%response%'=='Y' format %driveLetter% /FS:exFAT /A:4096 /V:A3EM /Q /X /Y & pause
exit /b 0
