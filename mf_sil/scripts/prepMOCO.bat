@echo off

setlocal
set self=%~n0
set cwd=%cd%

set build_folder=../build
set cmake_folder=../scripts
cd ..

echo %cd%: Creating solution
py -3 scripts/cip.py build --variant local_dep --platform vs2017
IF ERRORLEVEL 1 GOTO error_handling
pause



exit

:error_handling
echo ===============================================
echo "Build failed. Please check the errors above."
echo ===============================================
pause
