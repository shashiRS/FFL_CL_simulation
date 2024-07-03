@ECHO OFF
setlocal EnableDelayedExpansion
set TEST_CATALOGUE_NAME=Scenario_Catalogue_PDW.xlsm
set TEST_CATALOGUE_PATH=d:\IMS\AUP_100\02_System\04_System_Design\02_System_Requirements\03_Use_Cases\%TEST_CATALOGUE_NAME%

attrib -r %TEST_CATALOGUE_PATH%

IF %1==close (
    tasklist /V | findstr -N %TEST_CATALOGUE_NAME% | find /c ":">temp.txt

    for /f %%a in (temp.txt) do (
        IF %%a==1 (
            echo Close test catalogue
            taskkill /F /FI "WindowTitle eq %TEST_CATALOGUE_NAME%*" /T > nul
        )
    )
)
IF %1==reopen (
    for /f %%a in (temp.txt) do (
        IF %%a==1 (
            echo Reopen test catalogue
            start excel %TEST_CATALOGUE_PATH%
        )
    )
    del temp.txt
)


