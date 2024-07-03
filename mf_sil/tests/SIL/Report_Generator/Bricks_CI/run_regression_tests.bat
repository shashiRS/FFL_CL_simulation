@ECHO OFF
setlocal enabledelayedexpansion

REM project variant "base" or "entry" or "performance"
set variant=%1

REM regression test execution can be disabled in the conf/test_config.json file

REM set this variable to any value to enable virtual testing regression evaluation
set "perform_virtual_testing_evaluation="

REM skip regression tests on "release/PreCheck_ReadOnly" branch
if "%CHANGE_BRANCH%"=="release/PreCheck_ReadOnly" (
    exit /b 0
)

REM Number of stroke file path
set no_of_strokes_file=%~dp0\..\NoOfStrokes_TRJPLA.txt
if %variant%==entry (
    set no_of_strokes_file=%~dp0\..\NoOfStrokes_TRJPLA_CUS.txt
)
if %variant%==performance (
    set no_of_strokes_file=%~dp0\..\NoOfStrokes_TRJPLA_Performance.txt
)

REM Backup and restore selected_variant.txt to not interfere with the variant written by getComponents.py
set variant_file=%~dp0\..\..\..\..\scripts\selected_variant.txt
set variant_file_backup=%~dp0\..\..\..\..\scripts\backup_selected_variant.txt
if exist %variant_file% (
    move /y %variant_file% %variant_file_backup%
)
REM The regression test framework reads the variant from this file.
echo %variant% > %variant_file%
set errorlevel_MP_small=0
if %variant%==base (
    REM run MP small regression KPIs test catalogue
    call %~dp0\MP_Run_Eval_Regression_KPIs_Small.bat
    set errorlevel_MP_small=!errorlevel!
    xcopy %~dp0\..\..\Test_Results\Small_Regr_MP\LastReport %~dp0\..\..\..\..\conan_workarea\mf_sil-%variant%-test_results\small_memory_parking /s /e /i /y
)


REM run small regression KPIs test catalogue
call %~dp0\AUP_Run_Eval_Regression_KPIs_L1_L3_Small.bat
set errorlevel_AUP_small=%errorlevel%
xcopy %~dp0\..\..\Test_Results\Small_Regr_AUP\LastReport %~dp0\..\..\..\..\conan_workarea\mf_sil-%variant%-test_results\small /s /e /i /y
xcopy %no_of_strokes_file% %~dp0\..\..\..\..\conan_workarea\mf_sil-%variant%-test_results\ /i /y


REM evaluate Virtual Testing criteria for small regression test catalogue (no need to run CarMaker again)
set errorlevel_AUP_VT_small=0
if defined perform_virtual_testing_evaluation (
    call %~dp0\AUP_Eval_Regression_Small.bat
    set errorlevel_AUP_VT_small=%errorlevel%
    xcopy %~dp0\..\..\Test_Results\Small_Regr_AUP\LastReport %~dp0\..\..\..\..\conan_workarea\mf_sil-%variant%-test_results\small-VT /s /e /i /y
    xcopy %no_of_strokes_file% %~dp0\..\..\..\..\conan_workarea\mf_sil-%variant%-test_results\small-VT /i /y
)

REM Execute mid regression KPIs only on protected branches ("master" or "release/RC_*")
REM or if the pull request is targeted to the "master" or "release/RC_Parking" branch
set errorlevel_AUP_mid=0
set "run_mid_regression="
if "%BRANCH_NAME%"=="master" set run_mid_regression=true
if "%BRANCH_NAME:~0,11%"=="release/RC_" set run_mid_regression=true
if "%CHANGE_TARGET%"=="master" set run_mid_regression=true
if "%CHANGE_TARGET%"=="release/RC_Parking" set run_mid_regression=true
if "%CHANGE_TARGET%"=="release/RC_Cobolt" set run_mid_regression=true
if "%CHANGE_TARGET%"=="release/RC_Cobolt_R3_0_0" set run_mid_regression=true
if defined run_mid_regression (
    call %~dp0\AUP_Run_Eval_Regression_KPIs_L1_L3_Mid.bat
    set errorlevel_AUP_mid=!errorlevel!
    xcopy %~dp0\..\..\Test_Results\Regr_Reports_AUP\LastReport %~dp0\..\..\..\..\conan_workarea\mf_sil-%variant%-test_results\mid /s /e /i /y
    xcopy %no_of_strokes_file% %~dp0\..\..\..\..\conan_workarea\mf_sil-%variant%-test_results\ /i /y
)

REM Restore the selected_variant.txt file
if exist %variant_file_backup% (
    move /y %variant_file_backup% %variant_file%
)

if %errorlevel_AUP_small% neq 0 (
    echo "ERROR: Small AUP KPIs test catalogue not passed"
)
if !errorlevel_MP_small! neq 0 (
    echo "ERROR: Small Memory Parking KPIs test catalogue not passed"
)
if %errorlevel_AUP_VT_small% neq 0 (
    echo "ERROR: Small AUP test catalogue not passed"
)
if !errorlevel_AUP_mid! neq 0 (
    echo "ERROR: Mid AUP KPIs test catalogue not passed"
)

set /a "errorlevel_total=%errorlevel_AUP_small%+%errorlevel_AUP_VT_small%+%errorlevel_AUP_mid%+%errorlevel_MP_small%"

REM customevent log message for CIP Stream reporting of regression test pass/fail
set "run_custom_logging=false"

if "%BRANCH_NAME%"=="master" set run_custom_logging=true
if "%BRANCH_NAME:~0,11%"=="release/RC_" set run_custom_logging=true

if "%run_custom_logging%"=="true" (
    if !errorlevel_AUP_small! neq 0 (
        echo "customevent.info.v1|SIL|EEX_DEVSIM_CLS|FAIL|"
    ) else (
        echo "customevent.info.v1|SIL|EEX_DEVSIM_CLS|PASS|"
    )
)

exit /b %errorlevel_total%
