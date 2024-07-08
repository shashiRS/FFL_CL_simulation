#====================================================================
# Constants
#====================================================================
# Requirement

# CarMaker
EGO_LENGTH                  = 4.787 # m
EGO_WIDTH                   = 1.832 # m
EGO_WIDTH_MIRRORS           = 2.083 # m
EGO_OVERHANG                = 1.096 # m
EGO_WHEELBASE               = 2.786 # m
VHCL_SROAD_STARTOFF         = 3.882 # m - Default Vhcl.sRoad value for 0m initial offset (offset of the front wheels)

# CarMaker scenario
P_minDrivingDist            = 7 # m - minimum distance traveled before colliding with a static pole
P_drivingTime               = 4.6 # s - time traveled before colliding with a static pole

#Constants related to CarMaker test execution
MEAS_EXTENTION                = ".erg"
MAX_EXECUTION_TIME_S          = 300
MAX_SIMULATION_TIME_S         = 200
TEST_CASE_SET_NAME = "PDW" #outputFolderName inside mf_sil\tests\SIL\CarMaker\SimOutput\
CM_VERSION = "8.1.1"
PDW_QUANTITIES = [ # List of signals to be output in the erg files; TODO: extend if necessary 
    "Car.v",
    "Car.Distance",
    "Vhcl.sRoad",
    "DM.ManTime",
    "DM.ManNo",
    "DM.ManDist",
    "DM.BrakePark",
    "DM.SelectorCtrl",
    "Vehicle.FL.rot",
    "Vehicle.FR.rot",
    "Vehicle.RL.rot",
    "Vehicle.RR.rot",
    "AP.odoInputPort.odoSigFcanPort.gearboxCtrlStatus.gearCur_nu",
    "AP.LoDMCStatusPort.standstillSecureCur_nu",
    "AP.planningCtrlPort.apStates",
    "AP.hmiOutputPort.userActionHeadUnit_nu",
    "AP.headUnitVisualizationPort.screen_nu",
    "AP.headUnitVisualizationPort.message_nu",
    "AP.drvWarnStatus.pdwState_nu",
    "AP.drvWarnStatus.pdwSystemState_nu",
    "AP.drvWarnStatus.pdwShutdownCause_nu",
    "AP.drvWarnStatus.pdwActiveState_nu",
    "AP.drvWarnStatus.pdwCoreState_nu",
    "AP.drvWarnStatus.reduceToMuteSoundReq_nu",
    "AP.logicToProcPort.drvTubeDisplayReq_nu",
    "AP.logicToProcPort.sideResetReq_nu_Front",
    "AP.logicToProcPort.sideResetReq_nu_Rear",
    "AP.logicToProcPort.sideResetReq_nu_Left",
    "AP.logicToProcPort.sideResetReq_nu_Right",
    "AP.drvWarnDebugPort.minDistFrontSensors",
    "AP.drvWarnDebugPort.prevCondMask",
    "AP.pdwDisabled_nu",
    "AP.pdwFailure_nu"
]