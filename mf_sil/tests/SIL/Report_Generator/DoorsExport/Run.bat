@ECHO OFF
:register
title Register
echo.
echo Please fill in the the requested information
echo.
set waitSeconds = 10
set /p username="Please enter your DOORS username: "
set "psCommand=powershell -Command "$pword = read-host 'Please enter your DOORS password' -AsSecureString ; ^
    $BSTR=[System.Runtime.InteropServices.Marshal]::SecureStringToBSTR($pword); ^
        [System.Runtime.InteropServices.Marshal]::PtrToStringAuto($BSTR)""
for /f "usebackq delims=" %%p in (`%psCommand%`) do set passwd=%%p

IF "%DOORSPATH%" == "" (
	goto Label2
) ELSE (
	goto Label3
	)


:Label3
echo %DOORSPATH% 	
set /p CorrectPath="Type y if the path above is correct and n if it is not: " 	
echo %CorrectPath%
IF "%CorrectPath%" == "y" (
	goto Label1
) ELSE (
	goto Label2
)

:Label1
echo "doors path is set to %DOORSPATH%"
set doorsPath=%DOORSPATH%
goto start_installation

:Label2 
set /p doorsPath="Please enter the path to doors.exe (incl. .exe) for the doors version you want to install the packages (e.g. 'C:\Program Files\IBM\Rational\DOORS\9.7\bin\doors.exe'):
setx DOORSPATH "%doorsPath%"
goto start_installation

:start_installation
SET sModuleName=/ADAS/20_Base_Projects/L0_L1_System/LSM200/L0_L1_System/01_RequirementSet/AP/AUP_Parameters_list
SET sExportView=_ParamExport
SET sSaveLocation=%~dp0
echo.
echo Start doors export for DOORS document
echo %sModuleName%
echo ... in DOORS view...
echo %sExportView%
echo Please wait about 3-4 minutes. Info: Excel will open a empty window...
"%DOORSPATH%" -user %username% -password %passwd% -batch "AD_RE_Multi_EnhExcel_Export.dxl"
echo "Start conversion from xlsm to txt"
%PY3PATH% parameters_export.py
echo "Finished"