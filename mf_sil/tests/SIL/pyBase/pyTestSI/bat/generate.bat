@ECHO OFF
mkdir %~dp0\..\..\..\CarMaker\Data\TestRun\SI\temp\01_Perpendicular_Left
mkdir %~dp0\..\..\..\CarMaker\Data\TestRun\SI\temp\02_Perpendicular_Right
mkdir %~dp0\..\..\..\CarMaker\Data\TestRun\SI\temp\03_Parallel_Left
mkdir %~dp0\..\..\..\CarMaker\Data\TestRun\SI\temp\04_Parallel_Right
%PY3PATH% ..\TestRunGenerator.py
%PY3PATH% ..\CopyGeneratedTestRuns.py
rmdir /s /q %~dp0\..\..\..\CarMaker\Data\TestRun\SI\temp\ >nul
pause