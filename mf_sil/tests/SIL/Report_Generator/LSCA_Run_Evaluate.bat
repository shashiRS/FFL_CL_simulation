@ECHO OFF

set evalName=LSCA_AutoTestExecution
set waitSeconds=5
attrib -r %~dp0\..\CarMaker

echo Would you like to delete the previous Results?
choice /c yn /d n /t %waitSeconds%

goto Label%errorlevel%
%~dp0\..\CarMaker\TestRun\

:Label1
echo "Deleting previous Results"
rd /s /q %~dp0\..\CarMaker\Data\TestRun\%evalName%
rd /s /q %~dp0\..\CarMaker\SimOutput\%evalName%
goto startReportGeneration


:Label2 
goto startReportGeneration

:startReportGeneration
REM use Python virtual environment for test execution
call %~dp0\..\..\..\scripts\create_venv_for_testing.bat
call %~dp0\..\..\..\venv\Scripts\activate.bat
python eval_AUP_performance.py -c %evalName%
taskkill /IM HIL.exe /f >nul 2>&1
taskkill /IM Movie.exe /f >nul 2>&1
taskkill /IM CarMaker.win64.exe /f >nul 2>&1
taskkill /IM roadutil.exe /f >nul 2>&1

python AUP_Eval_STET.py -c %evalName%

%SystemRoot%\explorer.exe "%~dp0..\Test_Results\Reports_LSCA"

set /p DUMMY=Hit ENTER to continue...
