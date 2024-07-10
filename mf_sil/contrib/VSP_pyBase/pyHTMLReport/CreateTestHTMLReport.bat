@echo off

if not exist "test_output" (
	mkdir test_output
	echo Folder tesT_output created.
	) else (
	echo Folder test_output already exists.
	)

echo Create test html report...
python test_htmlReport_extended.py
pause