@ECHO OFF
call auxiliary\_before_eval.bat
%PY3PATH% ..\WHPTestSession.py -eval
call auxiliary\_after_eval.bat
pause