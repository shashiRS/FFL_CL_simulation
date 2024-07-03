@ECHO OFF

IF "%MSBuild%" == "c:\Program Files (x86)\Microsoft Visual Studio\2017\Professional\MSBuild\15.0\Bin\MSBuild.exe" (
	goto startBuild
) ELSE (
	goto Label2
	)


:Label2 
set /p msbuild="Please enter the path to MSBuild.exe Eg:c:\Program Files (x86)\Microsoft Visual Studio\2017\Professional\MSBuild\15.0\Bin\MSBuild.exe":
setx MSBuild "%msbuild%"
goto startBuild


:startBuild
set CURRENT_PATH=%~dp0
cd %CURRENT_PATH%
cd .\..\..\..\..\..\..\..\..\..
cd svs3xx\utils\MTS_SimLegacy\
REM echo %CD%
"%MSBuild%" MTSWrapping_VS2017.sln /t:Rebuild /p:Configuration="Debug" /p:Platform="Win32"

set /p DUMMY=Hit ENTER to continue...
