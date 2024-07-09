@echo off

set venv_path=%~dp0..\venv

set CUR_YYYY=%date:~10,4%
set CUR_MM=%date:~4,2%
set CUR_DD=%date:~7,2%
set CUR_HH=%time:~0,2%
if %CUR_HH% lss 10 (set CUR_HH=0%time:~1,1%)

set CUR_NN=%time:~3,2%
set CUR_SS=%time:~6,2%
set CUR_MS=%time:~9,2%

if "%~5" == "" (
    set LOGNAME=container_%CUR_YYYY%%CUR_MM%%CUR_DD%-%CUR_HH%%CUR_NN%%CUR_SS%.log
) else (
    set LOGNAME=%5\container_%CUR_YYYY%%CUR_MM%%CUR_DD%-%CUR_HH%%CUR_NN%%CUR_SS%.log
)

>%LOGNAME% 2>&1 (
echo %0 %1 %2 %3 %4 %5 %6 %7 %8 %9

if not exist "%venv_path%" (
    echo Creating python virtual environment...
    python -m venv "%venv_path%"
    echo Installing VSP_pyBase dependencies...
    call "%venv_path%\Scripts\activate.bat"
    if exist "%~dp0..\tests\VSP_PYBASE" (
        python -m pip install -r "%~dp0..\tests\VSP_PYBASE\requirements.txt"
    ) else (
        python -m pip install -r "%~dp0..\contrib\VSP_pyBase\requirements.txt"
    )
    call "%venv_path%\Scripts\deactivate.bat"
)

call "%venv_path%\Scripts\activate.bat"
python "%~dp0\..\Scripts\run_simulation.py" %1 %2 %3 %4 %5 %6 %7 %8 %9
call "%venv_path%\Scripts\deactivate.bat"
)
type %LOGNAME%
