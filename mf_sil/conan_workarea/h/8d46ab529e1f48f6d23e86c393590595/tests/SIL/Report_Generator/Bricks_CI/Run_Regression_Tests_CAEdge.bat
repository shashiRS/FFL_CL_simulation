@ECHO OFF

set directory=%1
set outputPath=%2
attrib -r %~dp0\..\..\CarMaker

rd /s /q %~dp0\..\..\CarMaker\SimOutput\%outputPath%

set venv_path=%~dp0\..\..\..\..\venv
if not exist %venv_path% (
    echo Creating python virtual environment...
    call %~dp0\..\..\..\..\scripts\create_venv_for_testing.bat
    echo Installing VSP_pyBase dependencies...
    call %venv_path%\Scripts\activate.bat
    python -m pip install -r %~dp0\requirements.txt 
)

echo Directory: %directory%
echo OutputPath: %outputPath%
python %~dp0\..\eval_AUP_performance_CAEdge.py -d %directory% -o %outputPath%
set errorlevel_run=%errorlevel%
echo "errorlevel_run = %errorlevel_run%"
taskkill /IM HIL.exe /f >nul 2>&1
taskkill /IM Movie.exe /f >nul 2>&1
taskkill /IM CarMaker.win64.exe /f >nul 2>&1
taskkill /IM roadutil.exe /f >nul 2>&1

if %errorlevel_run% neq 0 exit /b %errorlevel_run%