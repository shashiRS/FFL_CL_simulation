@ECHO OFF
call auxiliary\_before_gen.bat
C:\LegacyApp\Python36\python.exe ..\PDWTestSession.py -regen
REM %PY3PATH% ..\PDWTestSession.py -regen
call auxiliary\_after_gen.bat
pause