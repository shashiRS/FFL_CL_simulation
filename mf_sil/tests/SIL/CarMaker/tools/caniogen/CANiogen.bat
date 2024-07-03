@ECHO OFF

REM --- If only one dbc in   mf_sil\tests\SIL\CarMaker\CANdb\, then script can be called with no arguments --> e.g. CANiogen.bat
REM --- If more dbc files in mf_sil\tests\SIL\CarMaker\CANdb\, then call script with dbc name as argument  --> e.g. CANiogen.bat AP_Private_CAN_v1.30.dbc

set BUS_NAME=PRIVATE_CAN
set SIMULATED_ECUS=SalvatorX
set EXCLUDED_ECUS=CAN2LIN_GW,HFM,IMU
set DBC_FODLER=..\..\CANdb
set OUTPUT_DIR=%cd%\output

set CANIOGEN_CMD=%cd%\CANiogen.exe -outdir %OUTPUT_DIR% -simECU %SIMULATED_ECUS% -excECU %EXCLUDED_ECUS% -ioModule %BUS_NAME% 

IF NOT [%1]==[] (
    %CANIOGEN_CMD% %DBC_FODLER%\%1
    GOTO FINISHED
)
IF [%1]==[] (
    pushd %DBC_FODLER%
    for %%a in (*.dbc) do (
        popd | %CANIOGEN_CMD% %%a
        GOTO FINISHED
    )
)
:FINISHED
popd
ECHO -------------------------------------------------------
ECHO Generated code in %OUTPUT_DIR%.
ECHO Please merge with code in mf_sil\tests\SIL\CarMaker\src.
