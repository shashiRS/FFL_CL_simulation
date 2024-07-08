@ECHO OFF

set evalName=TCE_AutoTestExecution
set waitSeconds=5

echo Would you like to delete the previous Results?
choice /c yn /d n /t %waitSeconds%

goto Label%errorlevel%
..\CarMaker\TestRun\
:Label1
echo "Deleting previous Results"
rd /s /q ..\CarMaker\Data\TestRun\%evalName%
rd /s /q ..\CarMaker\SimOutput\%evalName%
goto startReportGeneration


:Label2 
goto startReportGeneration

:startReportGeneration
%PY3PATH% eval_AUP_performance.py -c %evalName%
taskkill /IM HIL.exe /f >nul 2>&1
taskkill /IM Movie.exe /f >nul 2>&1
taskkill /IM CarMaker.win64.exe /f >nul 2>&1
taskkill /IM roadutil.exe /f >nul 2>&1

%PY3PATH% AUP_Eval_STET.py -c %evalName%

set /p DUMMY=Hit ENTER to continue...
