@ECHO OFF

set evalName=LSCA_AutoTestExecution
set waitSeconds=5

REM use Python virtual environment for test execution
call %~dp0\..\..\..\scripts\create_venv_for_testing.bat
call %~dp0\..\..\..\venv\Scripts\activate.bat
python AUP_Eval_STET.py -c %evalName%

%SystemRoot%\explorer.exe "%~dp0..\Test_Results\Reports_LSCA"

set /p DUMMY=Hit ENTER to continue...
