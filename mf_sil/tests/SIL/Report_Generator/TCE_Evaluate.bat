@ECHO OFF

set evalName=TCE_AutoTestExecution
set waitSeconds=5

%PY3PATH% AUP_Eval_STET.py -c %evalName%

set /p DUMMY=Hit ENTER to continue...
