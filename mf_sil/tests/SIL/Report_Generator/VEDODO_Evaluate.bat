@ECHO OFF

set evalName=VEDODO_MTS
set waitSeconds=5
set CURRENT_PATH=%~dp0

%PY3PATH% AUP_Eval_STET.py -c %evalName%

cd %CURRENT_PATH%
cd ./../../../../../../../../..
cd bin/aup100/OutputReports_VEDODO
set REPORTS=%CD%
echo %REPORTS%
%SystemRoot%\explorer.exe %REPORTS%

set /p DUMMY=Hit ENTER to continue...
