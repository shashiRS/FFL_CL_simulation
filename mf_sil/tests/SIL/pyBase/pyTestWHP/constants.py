#Constants related to CarMaker test execution
MEAS_EXTENTION                = ".erg"
MAX_EXECUTION_TIME_S          = 300
MAX_SIMULATION_TIME_S         = 200
TEST_CASE_SET_NAME = "WHP" #outputFolderName inside mf_sil\tests\SIL\CarMaker\SimOutput\
CM_VERSION = "8.1.1"
WHP_QUANTITIES = [ # List of signals to be output in the erg files; TODO: extend if necessary 
    "Car.v",
    "Car.Distance",
    "Vhcl.sRoad",
    "DM.ManTime",
    "DM.ManNo",
    "DM.ManDist",
    "DM.BrakePark",
    "DM.SelectorCtrl",
    "DM.Steer.Ang",
    "Car.SteerAngleFL",
    "Car.SteerAngleFR",
    "Car.SteerAngleRL",
    "Car.SteerAngleRR",
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
    "AP.drvWarnStatus.whpState_nu",
    "AP.drvWarnStatus.whpDisplayReq_nu",
    "AP.drvWarnStatus.whpCoreState_nu",
    "AP.logicToProcPort.whlWarningType_nu_FL",
    "AP.logicToProcPort.whlWarningType_nu_FR",
    "AP.logicToProcPort.whlWarningType_nu_RL",
    "AP.logicToProcPort.whlWarningType_nu_RR",
    "AP.whpDisabled_nu"
]