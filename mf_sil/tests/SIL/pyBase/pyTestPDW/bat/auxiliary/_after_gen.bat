%PY3PATH% ..\CopyGeneratedTestRuns.py
rmdir /s /q %~dp0\..\..\..\..\CarMaker\Data\TestRun\DWF\PDW\temp\ >nul
call auxiliary\handle_catalogue.bat reopen