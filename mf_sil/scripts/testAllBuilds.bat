@ECHO OFF

set CURRENT_PATH=%~dp0
IF NOT "%MSBuild%" == "c:\Program Files (x86)\Microsoft Visual Studio\2017\Professional\MSBuild\15.0\Bin\MSBuild.exe" (
	CALL :defineMSBuildLocation
	:defineMSBuildLocation 
	set /p msbuild="Please enter the path to MSBuild.exe Eg:c:\Program Files (x86)\Microsoft Visual Studio\2017\Professional\MSBuild\15.0\Bin\MSBuild.exe"
	setx MSBuild "%msbuild%"
	pause
)
ECHO ########### 1.) Get Parking Base components based on build.yml
pause
python getComponents.py --base -autostop
ECHO ########### 1.) Check console output! Press key if no problems.
pause
ECHO ########### 2.) Build Debug_Base configuration.
pause
set BUILD_CONFIG="Debug_Base"
CALL :startBuild
ECHO ########### 2.) Build Debug_Base performed. Check console output!
pause
ECHO ########### 3.) Build Release_Base configuration.
pause
set BUILD_CONFIG="Release_Base"
CALL :startBuild
ECHO ########### 3.) Build Release_Base performed. Check console output!
pause
ECHO ########### 4.) Get Parking Entry components based on build.yml
pause
python getComponents.py --entry -autostop
ECHO ########### 4.) Check console output! Press key if no problems.
pause
ECHO ########### 5.) Build Debug_Entry configuration.
pause
set BUILD_CONFIG="Debug_Entry"
CALL :startBuild
ECHO ########### 5.) Build Debug_Entry performed. Check console output!
pause
ECHO ########### 6.) Build Release_Entry configuration.
pause
set BUILD_CONFIG="Release_Entry"
CALL :startBuild
ECHO ########### 6.) Build Release_Entry performed. Check console output!
pause
python getComponents.py --performance -autostop
ECHO ########### 7.) Check console output! Press key if no problems.
pause
ECHO ########### 7.) Press key if no problems.
pause
ECHO ########### 8.) Build Debug_Performance configuration.
pause
set BUILD_CONFIG="Debug_Performance"
CALL :startBuild
ECHO ########### 8.) Build Debug_Performance performed. Check console output!
pause
ECHO ########### 9.) Build Release_Performance configuration.
pause
set BUILD_CONFIG="Release_Performance"
CALL :startBuild
ECHO ########### 9.) Build Release_Performance performed. Check console output!
pause
ECHO ########### FINSIHED!
pause
EXIT /B 0


:startBuild
cd %CURRENT_PATH%
cd .\..\tests\SIL\CarMaker\prj\
"%MSBuild%" CarMaker.sln /t:Rebuild /p:Configuration=%BUILD_CONFIG% /p:Platform="x64"
cd %CURRENT_PATH%
EXIT /B 0



