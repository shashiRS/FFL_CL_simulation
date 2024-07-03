
Before running any of the bat files below, please make sure you have the setup configured as described below:
https://confluence-adas.zone2.agileci.conti.de/display/SYSAUP/SIL+-+Setup%3A+Testrun+evaluation

1. AUP_Evaluate.bat 
	- Evaluates all measurements available inside the folder: mf_sil\tests\SIL\CarMaker\SimOutput\AP_AutoTestExecution\
	- The evaluation is done based on KPI requirements defined in Doors
2. AUP_Evaluate_KPIs_L1_L3.bat
	- Evaluates all measurements available inside the folder: mf_sil\tests\SIL\CarMaker\SimOutput\AP_AutoTestExecution\
	- The evaluation is done based on "old" KPIs, defined in AUP_TC_Replaced.xls
3. AUP_Evaluate_Regression.bat
	- Evaluates all measurements available inside the folder: mf_sil\tests\SIL\CarMaker\SimOutput\AP_RegressionTests\
	- The evaluation is done based on KPI requirements defined in Doors
4. AUP_Evaluate_Regression_KPIs_L1_L3.bat
	- Evaluates all measurements available inside the folder: mf_sil\tests\SIL\CarMaker\SimOutput\AP_RegressionTests\
	- The evaluation is done based on "old" KPIs, defined in AUP_TC_Replaced.xls
5. AUP_Run_Eval_Regression.bat
	- Executes all scenarios from the following folder: mf_sil\tests\SIL\CarMaker\Data\TestRun\AP\06_Regression
	- Evaluates all measurements available inside the folder: mf_sil\tests\SIL\CarMaker\SimOutput\AP_RegressionTests\
	- The evaluation is done based on KPI requirements defined in Doors
6. AUP_Run_Eval_Regression_KPIs_L1_L3.bat
	- Executes all scenarios from the following folder: mf_sil\tests\SIL\CarMaker\Data\TestRun\AP\06_Regression
	- Evaluates all measurements available inside the folder: mf_sil\tests\SIL\CarMaker\SimOutput\AP_RegressionTests\
	- The evaluation is done based on "old" KPIs, defined in AUP_TC_Replaced.xls
7. AUP_Run_Evaluate.bat
	- Executes all scenarios from the following folder: mf_sil\tests\SIL\CarMaker\Data\TestRun\AP\04_Generated\
	- Evaluates all the measurements from the following folder: mf_sil\tests\SIL\CarMaker\SimOutput\AP_AutoTestExecution\
	- The evaluation is done based on KPI requirements defined in Doors
8. AUP_Run_Evaluate_KPIs_L1_L3.bat
	- Executes all scenarios from the following folder: mf_sil\tests\SIL\CarMaker\Data\TestRun\AP\04_Generated\
	- Evaluates all the measurements from the following folder: mf_sil\tests\SIL\CarMaker\SimOutput\AP_AutoTestExecution\
	- The evaluation is done based on "old" KPIs, defined in AUP_TC_Replaced.xls
9. LSCA_Evaluate.bat
	- Evaluates all the measurements from the following folder: mf_sil\tests\SIL\CarMaker\SimOutput\LSCA\
	- The evaluation is done based on LSCA_TC_Replaced.xls
        -***** haven't been used in a long time, might not work anymore***** 
10. LSCA_Run_Evaluate.bat
 	- Executes all scenarios from the following folder:mf_sil\tests\SIL\CarMaker\Data\TestRun\LSCA\
	- Evaluates all the measurements from the following folder: mf_sil\tests\SIL\CarMaker\SimOutput\LSCA\
	- The evaluation is done based on LSCA_TC_Replaced.xls
	-***** haven't been used in a long time, might not work anymore***** 
11. Run_Replaced.bat
	- Generates AUP_TC_KPI_Replaced.xlsx based on the export from Doors - AUP_TC_KPI.xlsx
12. TCE_Evaluate.bat
	- Evaluates all the measurements from the following folder: mf_sil\tests\SIL\CarMaker\SimOutput\TCE\
	- The evaluation is done based on TCE_TC_Replaced.xls
	-***** haven't been used in a long time, might not work anymore***** 
13. TCE_Run_Evaluate.bat
	- Executes all scenarios from the following folder:mf_sil\tests\SIL\CarMaker\Data\TestRun\TCE\
        - Evaluates all the measurements from the following folder: mf_sil\tests\SIL\CarMaker\SimOutput\TCE\
	- The evaluation is done based on TCE_TC_Replaced.xls
	-***** haven't been used in a long time, might not work anymore***** 
14. VEDODO_Evaluate.bat
	- evaluates all bsig files generated after MTS resimulation
	- evaluation is done based on VEDODO_TC_Replaced.xls
	-****** in order to work, path to the measurement location should be updated: PATH_measurements_VEDODO inside AUP_Eval_STET.py*******
15. MTS_Run_Eval\Build_Run_Eval_MTS_VEDODO.bat
	- Generates the dlls necessary for the MTS resimulation
	- Resimulates all the rrecs
	- Evaluates all bsig files generated after MTS resimulation
	- evaluation is done based on VEDODO_TC_Replaced.xls
	-****** in order to work please update the following: *****
        	- the bat file should be updated, to get the correct path for building the dlls. See lines below from MTS_Run_Eval\Build_Run_Eval_MTS_VEDODO.bat file: 
			- 	cd .\..\..\..\..\..\..\..\..\..
			-	cd svs3xx\utils\MTS_SimLegacy\	
		- path to rrecs - REC_PATH ,MTS configuration - MTS_CONFIGURATION_PATH and configuration name - CONFIG_NAME. See lines below from MTS_Run_Eval\Run_MTS.bat:
			set CURRENT_PATH=%~dp0
			cd %CURRENT_PATH%
			cd .\..\..\..\..\..\..\..\..\..\..\..
			cd recordings

			set REC_PATH=%CD%
			cd .\..
			cd .\mts\bin
			set =%CD%
			cd %CURRENT_PATH%
			echo %CD%
			cd .\..\..\..\..\..\..\..\..

			cd utils/mts/mts_config/cfg/algo/vedodo/
			set configname=\sim_adc426_vedodo.cfg
			set CONFIG_NAME=%CD%%configname% 
		- path to the measurement location should be updated: PATH_measurements_VEDODO inside AUP_Eval_STET.py
		
16. MTS_Run_Eval\Run_MTS.bat
	- Resimulates all the rrecs
	-****** in order to work please update the following: *****
			- path to rrecs - REC_PATH ,MTS configuration - MTS_CONFIGURATION_PATH and configuration name - CONFIG_NAME. See lines below from MTS_Run_Eval\Run_MTS.bat:
			set CURRENT_PATH=%~dp0
			cd %CURRENT_PATH%
			cd .\..\..\..\..\..\..\..\..\..\..\..
			cd recordings

			set REC_PATH=%CD%
			cd .\..
			cd .\mts\bin
			set =%CD%
			cd %CURRENT_PATH%
			echo %CD%
			cd .\..\..\..\..\..\..\..\..

			cd utils/mts/mts_config/cfg/algo/vedodo/
			set configname=\sim_adc426_vedodo.cfg
			set CONFIG_NAME=%CD%%configname% 
17. MTS_Run_Eval\Run_MTS_Eval_VEDODO.bat
	- Resimulates all the rrecs
	- evaluates all bsig files generated after MTS resimulation
	- evaluation is done based on VEDODO_TC_Replaced.xls
	-****** in order to work please update the following: *****
		- path to rrecs - REC_PATH ,MTS configuration - MTS_CONFIGURATION_PATH and configuration name - CONFIG_NAME. See lines below from MTS_Run_Eval\Run_MTS.bat:
			set CURRENT_PATH=%~dp0
			cd %CURRENT_PATH%
			cd .\..\..\..\..\..\..\..\..\..\..\..
			cd recordings

			set REC_PATH=%CD%
			cd .\..
			cd .\mts\bin
			set =%CD%
			cd %CURRENT_PATH%
			echo %CD%
			cd .\..\..\..\..\..\..\..\..

			cd utils/mts/mts_config/cfg/algo/vedodo/
			set configname=\sim_adc426_vedodo.cfg
			set CONFIG_NAME=%CD%%configname% 
		- path to the measurement location should be updated: PATH_measurements_VEDODO inside AUP_Eval_STET.py
18. MTS_Run_Eval\VEDODO_Evaluate_HIL.bat
	- evaluates all bsig files generated after MTS resimulation on HIL
	- evaluation is done based on VEDODO_HIL_TC_Replaced.xls
	-****** in order to work, path to the measurement location should be updated: PATH_measurements_VEDODO inside AUP_Eval_STET.py*******
19. MTS_Run_Eval\VS_MTS_Build.bat
	- generates all the dlls necesary for MTS resimulation
        - *******in order to work, the bat file should be updated, to get the correct path. See lines below from the bat file: *****
	- 	cd .\..\..\..\..\..\..\..\..\..
	-	cd svs3xx\utils\MTS_SimLegacy\
20. SI_Evaluate.bat
	- Evaluates all measurements available inside the folder: mf_sil\tests\SIL\CarMaker\SimOutput\SI_AutoTestExecution\
21. SI_Run_Evaluate.bat
	- Executes all scenarios from the following folder: mf_sil\tests\SIL\CarMaker\Data\TestRun\SI\01_Generated
	- Evaluates all measurements available inside the folder: mf_sil\tests\SIL\CarMaker\SimOutput\SI_AutoTestExecution\
22. SI_Evaluate_Regression.bat
	- Evaluates all measurements available inside the folder: mf_sil\tests\SIL\CarMaker\SimOutput\SI_RegressionTests\
23. SI_Run_Evaluate_Regression.bat
	- Executes all scenarios from the following folder: mf_sil\tests\SIL\CarMaker\Data\TestRun\SI\02_Regression
	- Evaluates all measurements available inside the folder: mf_sil\tests\SIL\CarMaker\SimOutput\SI_RegressionTests\
	
    