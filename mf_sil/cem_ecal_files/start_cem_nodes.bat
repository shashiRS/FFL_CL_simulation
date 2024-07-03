@echo off
setlocal
echo:
echo [PL Parking mf_sil] Updating path variable.

SET ECAL_DATA=%~dp0\ecal\
SET PATH=%~dp0\ecal\bin\;%PATH%
echo [PL Parking mf_sil] Starting ecal sys for simulation. Please open the TestRun of choice in the CarMaker GUI. Follow the instructions to start or kill nodes if needed. Just type "exit" to exit all nodes and close CarMaker manually.
%~dp0\ecal\bin\ecal_sys.exe -c cem_automization.ecalsys -r --use-localhost-for-all-tasks true -i

@REM endlocal
