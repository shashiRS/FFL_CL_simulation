@ECHO OFF
call auxiliary\_before_gen_and_run.bat
%PY3PATH% ..\WHPTestSession.py -full
call auxiliary\_after_gen_and_run.bat
pause