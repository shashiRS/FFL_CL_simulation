@echo off
TITLE MTS SIMULATION

::USER CONFIG
::----------------------------------------------------------------------------------------
set MTS_INST=1
set CURRENT_PATH=%~dp0
cd %CURRENT_PATH%
cd .\..\..\..\..\..\..\..\..\..\..\..
cd recordings

set REC_PATH=%CD%
cd .\..
cd .\mts\bin
set MTS_CONFIGURATION_PATH=%CD%
cd %CURRENT_PATH%
echo %CD%
cd .\..\..\..\..\..\..\..\..

cd utils/mts/mts_config/cfg/algo/vedodo/
set configname=\sim_adc426_vedodo.cfg
set CONFIG_NAME=%CD%%configname% 
:: ---------------------------------------------------------------------------------------

::Print in console
echo Start
echo ----------------------------------------------------------------------------------
echo Recording used : %REC_PATH%
echo ----------------------------------------------------------------------------------
echo ...
echo ----------------------------------------------------------------------------------
echo MTS version used : %MTS_CONFIGURATION_PATH%
echo ----------------------------------------------------------------------------------
echo ...
echo ----------------------------------------------------------------------------------
echo Configuration file used : %CONFIG_NAME%
echo ----------------------------------------------------------------------------------
echo ...

cd %CURRENT_PATH%
%PY3PATH% VEDODO_simulation.py --rec_path %REC_PATH% --mts_config %MTS_CONFIGURATION_PATH% --config_name %CONFIG_NAME%
TIMEOUT /T 5 /NOBREAK

cd %MTS_CONFIGURATION_PATH%
cd .\mts_measurement\ini
set BPL_PATH=%CD%

:count_BPLfiles
set no_bpl_files=0
for %%x in (*.bpl) do set /a no_bpl_files+=1
echo Number of BPL files: %no_bpl_files%z

cd %MTS_CONFIGURATION_PATH%
cd mts
rem -lc"config file" -lb"playlist file in ini folder"

SET /A BPL_file=0

:check_Number_of_BPL
if %BPL_file% EQU %no_bpl_files% (
	goto sim_FINISHED
) ELSE (
goto check_MTS
)

:start_MTS
SET /A BPL_file=%BPL_file%+1
start "" ".\measapp.exe" -lc%CONFIG_NAME% -lb"%BPL_PATH%\BPL_file_%BPL_file%.bpl" -pal -eab -show"maximized" -silent
echo BPL_file_%BPL_file%.bpl
TIMEOUT /T 10 /NOBREAK
goto check_Number_of_BPL

:check_MTS
wmic process where name="measapp.exe" | find "measapp.exe" /c > SomeFile.txt  
set /p MTS_instances= < SomeFile.txt  
del SomeFile.txt  

echo Number of MTS instances open in this moment: %MTS_instances%

if %MTS_instances% LSS %MTS_INST% (
	echo Open another instance of MTS
	goto start_MTS
) ELSE (
	echo Number of simulated recordings: %BPL_file% / %no_bpl_files%
	TIMEOUT /T 15 /NOBREAK
    goto check_MTS
)

:sim_FINISHED
wmic process where name="measapp.exe" | find "measapp.exe" /c > SomeFile.txt  
set /p MTS_instances= < SomeFile.txt  
del SomeFile.txt  

echo Number of MTS instances open in this moment: %MTS_instances%

if %MTS_instances% LSS 1 (
	echo ALL recordings were simulated and the output data can be found in ..mts_measurement/data
	TIMEOUT /T 15 /NOBREAK

	goto END
) ELSE (
	TIMEOUT /T 15 /NOBREAK
    goto sim_FINISHED
)


:END
pause
