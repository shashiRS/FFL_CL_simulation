@ECHO OFF
mkdir %~dp0\..\..\..\CarMaker\Data\TestRun\UDP\temp
%PY3PATH% ..\TestRunGenerator.py
%PY3PATH% ..\CopyGeneratedTestRuns.py
rmdir /s /q %~dp0\..\..\..\CarMaker\Data\TestRun\UDP\temp\ >nul
pause