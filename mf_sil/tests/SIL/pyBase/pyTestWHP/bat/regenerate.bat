@ECHO OFF
call auxiliary\_before_gen.bat
%PY3PATH% ..\WHPTestSession.py -regen
call auxiliary\_after_gen.bat
pause