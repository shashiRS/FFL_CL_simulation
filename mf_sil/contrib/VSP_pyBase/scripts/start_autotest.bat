@echo off
:: run autotest
::
:: Synopsis:
::  1) scripts/start_autotest.bat
::     or
::  2) scripts/start_autotest.bat  <CMAKE_INSTALL_PREFIX>
::       typically via python.exe scripts/cip.py build --variant generic --platform vs2017



echo "********************"
echo " start_autotest.bat "
echo "********************"


:: Get Parameter first, if any.
set INSTALL_DIR=%1

set CURRENT_DIR=%~dp0
echo:
echo INSTALL_DIR=%INSTALL_DIR%
echo CURRENT_DIR=%CURRENT_DIR%


:: Donb't use hard coded Bricks package path for Jenkins, like "...fallback...".
:: Check if this script is called manually without parameter
:: or called by "PostInstll.cmake" with a single parameter.
if [%INSTALL_DIR%] == [] (
    echo No Parameter. Will build with hardcoded conan_workarea path.
    set exepath=%CURRENT_DIR%..\conan_workarea\package.SEF_Brake.SMOKE_TEST.1.0.0-fallback..vs2017\bin\carmaker\carmaker.win64.exe
    set resultpath=%CURRENT_DIR%..\conan_workarea\package.SEF_Brake.SMOKE_TEST.1.0.0-fallback.vs2017\bin\

) else (
    echo With Parameter. Will build with given conan_workarea path.
    set exepath=%INSTALL_DIR%\bin\carmaker\carmaker.win64.exe
    set resultpath=%INSTALL_DIR%\bin\
)




set workspace=%CURRENT_DIR%..\tests\

echo:
echo workspace=%workspace%
echo exepath=%exepath%
echo resultpath=%resultpath%
echo Headless mode is enabled


set component=SEF_BRAKE
echo:
echo check if prev. test result directory exist
if exist %resultpath%%component% (
	REM timeout 1
	rd /s /q %resultpath%%component%
	echo prev. test result directory deleted
)
if exist %workspace%carmaker\SimOutput\%component% (
	REM timeout 1
	rd /s /q %workspace%carmaker\SimOutput\%component%
	echo prev. test result directory deleted
)
if exist %workspace%.venv (
	REM timeout 1
	rd /s /q %workspace%.venv
	echo prev. test result directory deleted
)
echo:
cd ..
echo Carmaker binary path
dir /s/d/a:-d "*carmaker.win64.exe*"
echo Check Carmaker Installation Path
set IPGHOME
dir %IPGHOME%\carmaker\win64-10.2.1\lib\

cd %workspace%
echo:
echo init virtual environment
py -3.6 -m venv .venv
call %workspace%.venv\Scripts\activate.bat
call %workspace%pip-installation\install_pip.bat
echo:
echo install autotest
pip install autotest==2.1.71

echo:
echo start autotest tool
autotest.py --configPath %workspace%TestConfig_Brake.xml --carMakerProj %workspace%carmaker --testResultsPath %resultpath% --testType %component% --cmExeFile %exepath% --bricksMode=TRUE

echo:
echo deactivate virtual environment
call %workspace%.venv\Scripts\deactivate.bat

REM exit 