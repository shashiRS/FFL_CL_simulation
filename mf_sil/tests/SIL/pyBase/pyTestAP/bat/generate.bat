@ECHO OFF
mkdir %~dp0\..\..\..\CarMaker\Data\TestRun\AP\temp\01_Perpendicular_Left
mkdir %~dp0\..\..\..\CarMaker\Data\TestRun\AP\temp\02_Perpendicular_Right
mkdir %~dp0\..\..\..\CarMaker\Data\TestRun\AP\temp\03_Parallel_Left
mkdir %~dp0\..\..\..\CarMaker\Data\TestRun\AP\temp\04_Parallel_Right
mkdir %~dp0\..\..\..\CarMaker\Data\TestRun\AP\temp\05_Angled_Left
mkdir %~dp0\..\..\..\CarMaker\Data\TestRun\AP\temp\06_Angled_Right
%PY3PATH% ..\TestRunGenerator.py
%PY3PATH% ..\CopyGeneratedTestRuns.py
rmdir /s /q %~dp0\..\..\..\CarMaker\Data\TestRun\AP\temp\ >nul
pause