@ECHO OFF

set evalName=AP_AutoTestExecution
attrib -r %~dp0\..\..\CarMaker

echo "Deleting previous Results"
rd /s /q %~dp0\..\..\CarMaker\Data\TestRun\%evalName%
rd /s /q %~dp0\..\..\CarMaker\SimOutput\%evalName%

REM use Python virtual environment for test execution
if not defined VIRTUAL_ENV (
    call %~dp0\..\..\..\..\scripts\create_venv_for_testing.bat
    call %~dp0\..\..\..\..\venv\Scripts\activate.bat
)

echo %evalName%_KPIs
python %~dp0\..\eval_AUP_performance.py -c %evalName%
set errorlevel_run=%errorlevel%
echo "errorlevel_run = %errorlevel_run%"
taskkill /IM HIL.exe /f >nul 2>&1
taskkill /IM Movie.exe /f >nul 2>&1
taskkill /IM CarMaker.win64.exe /f >nul 2>&1
taskkill /IM roadutil.exe /f >nul 2>&1

python %~dp0\..\AUP_Eval_STET_KPIs_L1_L3.py -c %evalName%
set errorlevel_eval=%errorlevel%
echo "errorlevel_eval = %errorlevel_eval%"
python  %~dp0\eval_test_summary.py -c %evalName%
set errorlevel_test=%errorlevel%
echo "errorlevel_test = %errorlevel_test%"
set /a "ERRORLEVEL_TOT=%errorlevel_run%+%errorlevel_eval%+%errorlevel_test%"
echo "Total errorlevel = %ERRORLEVEL_TOT%"

REM Do not crash build if full test catalogue not successful to avoid running it again locally!
REM if %ERRORLEVEL_TOT% neq 0 exit /b %ERRORLEVEL_TOT%
