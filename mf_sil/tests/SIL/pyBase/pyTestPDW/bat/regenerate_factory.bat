@ECHO OFF
mkdir %~dp0\..\..\..\CarMaker\Data\TestRun\DWF\PDW\temp
REM %PY3PATH% ..\TestRunGenerator.py
REM %PY3PATH% ..\CopyGeneratedTestRuns.py -factory
C:\LegacyApp\Python36\python.exe ..\TestRunGenerator.py
C:\LegacyApp\Python36\python.exe ..\CopyGeneratedTestRuns.py -factory
rmdir /s /q %~dp0\..\..\..\CarMaker\Data\TestRun\DWF\PDW\temp\ >nul
pause