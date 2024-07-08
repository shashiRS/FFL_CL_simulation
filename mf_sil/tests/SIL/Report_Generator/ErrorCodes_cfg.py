

#====================================================================
# Exit Error Codes        /  bit position 
#====================================================================
NO_ERROR                   = 0
CM_CONNECTION_ERROR        = 0 #'CMRC: Could not connect to running CarMaker instance at '
CM_NO_INSTANCE_RUNNING     = 1 #'CMRC: No CarMaker instance runnning'
CM_GUI_EXE_ERROR           = 2 #'CMRC: Could not find any CarMaker GUI Executable'
CM_NO_STATUS_RECEIVED      = 3 #'CMRC: CarMaker Status was not received within maximum wait time'
CM_SAFE_MODE_NOT_SUPPORTED = 4 #'CMRC: CarMaker SafeMode not supported'
CM_SIMSPEED_NOT_SUPPORTED  = 5 #'CMRC: CarMaker SimSpeed not supported'
CM_ERROR                   = 6 #'CMRC: CarMaker Error: '
SIMULATION_TIME_EXCEEDED   = 7 #'Simulation Time longer than expected! See SIMTIMEMAX_S!'
EXECUTION_TIME_EXCEEDED    = 8 #'Execution Time longer than expected!'
QUANTITY_ERROR             = 9 #'Error while receiving quantity/ Could not subscribe to Quantity'
TEST_SCENARIO_ERROR        = 10
#Error Code for test evaluation dropped
EVALUATION_DROPPED         = 11