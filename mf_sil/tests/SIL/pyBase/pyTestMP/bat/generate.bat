@ECHO OFF
mkdir %~dp0\..\..\..\CarMaker\Data\TestRun\AP\temp
%PY3PATH% ..\TestRunGenerator.py
%PY3PATH% ..\CopyGeneratedTestRuns.py
rmdir /s /q %~dp0\..\..\..\CarMaker\Data\TestRun\AP\temp\ >nul
pause