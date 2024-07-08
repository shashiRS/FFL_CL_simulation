%PY3PATH% ..\CopyGeneratedTestRuns.py
rmdir /s /q %~dp0\..\..\..\..\CarMaker\Data\TestRun\DWF\WHP\temp\ >nul
call auxiliary\handle_catalogue.bat reopen