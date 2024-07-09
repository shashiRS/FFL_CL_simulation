@ECHO OFF

set venv_path=%~dp0\..\contrib\Reporting_UI\pl_parking\venv
set venv_path_fortest_execution=%~dp0\..\venv

REM Check if requirements have already been installed
if not exist %venv_path%\pip.ini (

	call %venv_path_fortest_execution%\Scripts\activate.bat
	python -m pip install --upgrade pip
	python -m pip install -r %~dp0\..\tests\SIL\Report_Generator\Bricks_CI\requirements.txt
	call %venv_path_fortest_execution%\Scripts\deactivate.bat

	%~dp0\..\contrib\cip\python\python.exe -m venv %venv_path%
	%~dp0\..\contrib\cip\python\python.exe %~dp0\..\contrib\Reporting_UI\pl_parking\utils\copy_pip_to_venv.py

	call %venv_path%\Scripts\activate.bat
	python -m pip install --upgrade pip
	python -m pip install -e %~dp0\..\contrib\Reporting_UI\pl_parking

	call %venv_path%\Scripts\deactivate.bat

)




