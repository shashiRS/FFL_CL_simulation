

  # ID:  type.eco.AlgoDataTimeStamp uml:DataType
    ->:  type.uint64 uml:PrimitiveType
----------------------------------------------------------------------------------------------------------


  # ID:  type.eco.AlgoCycleCounter uml:DataType
    ->:  type.uint16 uml:PrimitiveType
----------------------------------------------------------------------------------------------------------


  # ID:  type.eco.AlgoSignalState uml:Enumeration
    // Algo signal state enumeration  values: enum { AL_SIG_STATE_INIT=0
    // ,AL_SIG_STATE_OK=1,AL_SIG_STATE_INVALID=2,}
    // @range{0,2}
  Members:
    AL_SIG_STATE_INIT = 0 (DEFAULT)           
    AL_SIG_STATE_OK = 1            
    AL_SIG_STATE_INVALID = 2            
----------------------------------------------------------------------------------------------------------

  # ID:  type.eco.SignalHeader uml:Class
    // Common header for all structured data types
  Members:
    type.eco.AlgoDataTimeStamp uiTimeStamp     // Time stamp of the signal (for SIP: T0 timestamp of input signal;, otherwise: T7 timestamp to which signal has been adjusted)
    type.eco.AlgoCycleCounter uiMeasurementCounter     // Deprecated, not used. Should be set to 0.
    type.eco.AlgoCycleCounter uiCycleCounter     // Rolling counter of source component. Will be incremented in every;call to the exec method of the component.
    type.eco.AlgoSignalState eSigStatus     // Validity status of the signal. If not set to AL_SIG_STATE_OK,;then do not evaluate the content of the associated signal!
----------------------------------------------------------------------------------------------------------

  # ID:  type.eco.ComponentExecutionMode uml:Class
    // The execution mode of the components. It contains the configuration and sub-configuration id.
  Members:
    type.uint8 configurationID     // The component execution configuration id.
    type.uint8 subConfigurationID     // The component execution sub-configuration id.
----------------------------------------------------------------------------------------------------------

  # ID:  type.eco.CompOpState uml:Enumeration
    // Represents the operation state of components updated by
    // the framework
    // through shell with return provided by the components.
    // values: enum { OPS_INIT=0,OPS_SCHEDULED=1,OPS_RUNNING=2,OPS_DONE=3
    // ,OPS_CANCELED=4,OPS_UNDEFINED=255,}
    // @range{0,255}
  Members:
    OPS_INIT = 0            
    OPS_SCHEDULED = 1            
    OPS_RUNNING = 2            
    OPS_DONE = 3            
    OPS_CANCELED = 4            
    OPS_UNDEFINED = 255            
----------------------------------------------------------------------------------------------------------

  # ID:  type.eco.TraceData uml:Class
  Members:
    type.eco.SignalHeader sigHeader     // Signal header indicating data validity
    type.uint16 rangeCheckFailures     // Stores range check failures for trace ports
----------------------------------------------------------------------------------------------------------



  # ID:  type.eco.VehicleParam uml:Class
    // Parameters specific to the ego vehicle
  Members:
    type.eco.SignalHeader sigHeader     // General information about the signal
    type.float32 vehicleMass     // @unit{kg};information about the mass of the vehicle in kg
    type.float32 axisLoadDistr     // overall axis load distribution of the vehicle
    type.float32 length     // @unit{meters};Length of the ego vehicle in meters
    type.float32 width     // @unit{meters};Width of the ego vehicle in meters
    type.float32 wheelbase     // @unit{meters};Absolute distance in longitudinal direction between front axle and rear axle in meters
    type.float32 trackWidthFront     // @unit{meters};Absolute distance between the wheel mounting positions on the front axle in meters
    type.float32 trackWidthRear     // @unit{meters};Absolute distance between the wheel mounting positions on the rear axle in meters
    type.float32 overhangFront     // @unit{meters};Absolute distance in longitudinal direction between front axle and front most point of the chassis in meters
    type.float32 overhangRear     // @unit{meters};Absolute distance in longitudinal direction between rear axle and rear most point of the chassis in meters
    type.float32 wheelCircumferenceFront     // @unit{meters};Rear wheel Circumference of the ego vehicle in meters
    type.float32 wheelCircumferenceRear     // @unit{meters};Front wheel Circumference of the ego vehicle in meters
    type.float32 centerOfGravityX     // @unit{meters};Center of gravity X of the ego vehicle in meters
    type.float32 centerOfGravityY     // @unit{meters};Center of gravity Y of the ego vehicle in meters
    type.float32 centerOfGravityZ     // @unit{meters};Center of gravity Z of the ego vehicle in meters
    type.boolean isVehicleMassAvailable     // Specifies if vehicle VehicleMass parameter is available or not
    type.boolean isAxisLoadDistrAvailable     // Specifies if vehicle AxisLoadDistr parameter is available or not
    type.boolean isLengthAvailable     // Specifies if vehicle length parameter is available or not
    type.boolean isWidthAvailable     // Specifies if vehicle width parameter is available or not
    type.boolean isWheelbaseAvailable     // Specifies if vehicle wheel base parameter is available or not
    type.boolean isTrackWidthFrontAvailable     // Specifies if vehicle track width front parameter is available or not
    type.boolean isTrackWidthRearAvailable     // Specifies if vehicle track width rear parameter is available or not
    type.boolean isOverhangFrontAvailable     // Specifies if vehicle overhang front parameter is available or not
    type.boolean isOverhangRearAvailable     // Specifies if vehicle overhang rear parameter is available or not
    type.boolean isWheelCircumferenceFrontAvailable     // Specifies if vehicle front wheel circumference parameter is available or not
    type.boolean isWheelCircumferenceRearAvailable     // Specifies if vehicle rear wheel circumference parameter is available or not
    type.boolean isCenterOfGravityXAvailable     // Specifies if center of gravity X parameter is available or not
    type.boolean isCenterOfGravityYAvailable     // Specifies if center of gravity Y parameter is available or not
    type.boolean isCenterOfGravityZAvailable     // Specifies if center of gravity Z parameter is available or not
----------------------------------------------------------------------------------------------------------



  # ID:  type.eco.SensorMountPosition uml:Class
    // Sensor mount position and sensor ID
  Members:
    type.eco.SignalHeader sigHeader     // General information about the signal
    type.eco.SensorID sensorId     // Sensor ID of the related sensor
    type.eco.SensorMountingPositionDetailed sensorMountPos     // Mounting position details of the sensor relative to the environment model;coordinate system
    type.boolean isSensorMountPosAvailable     // Specifies if sensor mount position is available or not
----------------------------------------------------------------------------------------------------------

  # ID:  type.eco.SensorMountingPositionDetailed uml:Class
    // Mounting position details of the sensor relative to the environment model
    // coordinate system
  Members:
    type.float32 roll     // @unit{rad};Rolling angle of the sensor in environment model coordinate system
    type.float32 pitch     // @unit{rad};Pitch angle of the sensor in environment model coordinate system
    type.float32 yaw     // @unit{rad};Yaw angle of the sensorin environment model coordinate system
    type.float32 x     // @unit{meters};X-coordinate position of the sensor in environment model coordinate system
    type.float32 y     // @unit{meters};Y-coordinate position of the sensor in environment model coordinate system
    type.float32 z     // @unit{meters};Z-coordinate position of the sensor in environment model coordinate system
----------------------------------------------------------------------------------------------------------

  # ID:  type.eco.SensorID uml:DataType
    ->:  type.uint16 uml:PrimitiveType
----------------------------------------------------------------------------------------------------------

  # ID:  type.eco.ComponentResponse uml:Class
  Members:
    type.eco.SignalHeader sigHeader     // Returned SignalHeader Common header for all structure.
    type.uint64 errorCode     // Returned error code by the component
    type.uint64 functionalErrorCode     // Returned functional error codes (range check results) by the component
    type.eco.CompOpState opState     
----------------------------------------------------------------------------------------------------------

  # ID:  type.eco.TimeServiceData uml:Class
  Members:
    type.eco.SignalHeader sigHeader     // Returned SignalHeader Common header for all structure.
    type.uint64 cycleStart     // cycle start time
    type.uint64 estimatedCycleEnd     // Estimated Cycle End Time
----------------------------------------------------------------------------------------------------------


  # ID:  type.eco.BaseCtrlData uml:Class
  Members:
    type.uint32 uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint16 eOpMode     
    type.uint8 eSchedulingMode     
    type.uint8 eSchedulingSubMode     
----------------------------------------------------------------------------------------------------------

  # ID:  type.eco.AlgoCompState uml:Class
  Members:
    type.uint32 uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint32 uiAlgoVersionNumber     
    type.uint8 eCompState     
    type.uint32 eErrorCode     
----------------------------------------------------------------------------------------------------------

  # ID:  type.eco.ExecutionMode uml:Class
    // Execution information needed to execute a component
  Members:
    type.eco.SignalHeader sigHeader     // ADAS signal header
    type.eco.ComponentExecutionMode mode     // The component execution mode and sub mode information
----------------------------------------------------------------------------------------------------------

  # ID:  type.eco.SystemTime uml:Class
  Members:
    type.eco.SignalHeader sigHeader     // Returned SignalHeader Common header for all structure.
    type.uint64 systemTime     // System time by the component
----------------------------------------------------------------------------------------------------------

  # ID:  type.eco.AlgoInterfaceVersionNumber uml:DataType
    ->:  type.uint32 uml:PrimitiveType
----------------------------------------------------------------------------------------------------------

  # ID:  type.eco.CompState_t uml:Enumeration
    // The state of the algo  values: enum { COMP_STATE_NOT_INITIALIZED=0
    // ,COMP_STATE_RUNNING=1,COMP_STATE_TEMPORARY_ERROR=2,COMP_STATE_PERMANENT_ERROR=3
    // ,COMP_STATE_SUCCESS=4,COMP_STATE_REDUCED_AVAILABILITY=5,COMP_STATE_NOT_RUNNING=6
    // ,}
    // @range{0,6}
  Members:
    COMP_STATE_NOT_INITIALIZED = 0            
    COMP_STATE_RUNNING = 1            
    COMP_STATE_TEMPORARY_ERROR = 2            
    COMP_STATE_PERMANENT_ERROR = 3            
    COMP_STATE_SUCCESS = 4            
    COMP_STATE_REDUCED_AVAILABILITY = 5            
    COMP_STATE_NOT_RUNNING = 6            
----------------------------------------------------------------------------------------------------------

  # ID:  type.eco.OpMode uml:Enumeration
    // Represents the component execution mode context  which is provided
    // by the Framework to all components.  values: enum { OPM_INIT=0
    // ,OPM_EXEC1=1,OPM_EXEC2=2,OPM_EXEC3=3,OPM_EXEC4=4,OPM_EXEC5=5,
    // OPM_EXEC6=6,OPM_NOP=254,OPM_UNDEF=255,}
    // @range{0,255}
  Members:
    OPM_INIT = 0            
    OPM_EXEC1 = 1            
    OPM_EXEC2 = 2            
    OPM_EXEC3 = 3            
    OPM_EXEC4 = 4            
    OPM_EXEC5 = 5            
    OPM_EXEC6 = 6            
    OPM_NOP = 254            
    OPM_UNDEF = 255            
----------------------------------------------------------------------------------------------------------

  # ID:  type.eco.FirstLastComponentExecution uml:Class
    // Represents the operation mode of the first and last execution of the component within a cycle.
    // If the operation mode for first and last execution are equal the component is triggered only once.
    // The data is undefined if both operation modes are set to "OPM_UNDEF".
  Members:
    type.eco.OpMode firstExecutionOpMode     
    type.eco.OpMode lastExecutionOpMode     
----------------------------------------------------------------------------------------------------------

  # ID:  type.eco.FrameworkControl uml:Class
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     // Version number of interface
    type.eco.SignalHeader sSigHeader     // Signal header with common signal information
    type.eco.OpMode opMode     // Represents the component execution mode context  which is provided;by the Framework to all components.
    type.eco.AlgoDataTimeStamp cemCycleInitTimeStamp     // Represents the time stamp at the begin of the cycle.
    type.eco.AlgoDataTimeStamp endCycleEstimatedTimeStamp     // Represents the time stamp at the end of the cycle.
    type.eco.CompOpState opState     // Represents the operation state of components updated by;the framework;through shell with return provided by the components.
    type.eco.AlgoDataTimeStamp opModeExecTime     // @unit{us};Represents the execution time, of the last operation mode of components (exec1() .. exec6()).
    type.uint64 errorCode     // Returned error code by the component
    type.uint64 functionalErrorCode     // Returned functional error codes (range check results) by the component
    type.eco.AlgoDataTimeStamp windowsCycleStart     // Used for computing the time offset of the recorded data to the windows time. This reports the windows time at the;start of the cycle.
    type.eco.AlgoDataTimeStamp rawCycleStart     // Used for computing the time offset of the recorded data to the windows time. This reports the recorded time at the;start of the cycle without the latency compensation.
    type.eco.FirstLastComponentExecution firstLastExecution     // Used to determine if the execution is the first or last or both execution in the current cycle.
    type.boolean oseMode     // Used to determine if the system mode is ose mode or not.
----------------------------------------------------------------------------------------------------------

  # ID:  type.eco.CompStage uml:Enumeration
    // values: enum { INIT=0,SUSPEND=1,RESUME=2,SHUTDOWN=3
    // ,STAGE_UNDEFINED=255,}
    // @range{0,255}
  Members:
    STAGE_INIT = 0            
    STAGE_SUSPEND = 1            
    STAGE_RESUME = 2            
    STAGE_SHUTDOWN = 3            
    STAGE_UNDEFINED = 255            
----------------------------------------------------------------------------------------------------------

  # ID:  type.eco.DiagnosisEventStatus uml:Enumeration
    // values: enum { DIAGNOSIS_EVENT_STATUS_PASSED=0,DIAGNOSIS_EVENT_STATUS_FAILED=1,
    // DIAGNOSIS_EVENT_STATUS_PREPASSED=2,DIAGNOSIS_EVENT_STATUS_PREFAILED=3}
    // @range{0,255}
  Members:
    DIAGNOSIS_EVENT_STATUS_PASSED = 0            
    DIAGNOSIS_EVENT_STATUS_FAILED = 1            
    DIAGNOSIS_EVENT_STATUS_PREPASSED = 2            
    DIAGNOSIS_EVENT_STATUS_PREFAILED = 3            
----------------------------------------------------------------------------------------------------------

  # ID:  type.com.ComResult uml:Enumeration
    // ! /brief enum for function call results
  Members:
    COMRES_OK = 0            // !< Call result ok
    COMRES_WARNING = 1            // !< wrong configuration, or errors found, run is possible
    COMRES_WARN_WRONG_CONFIG = 1            
    COMRES_ERROR = 2            // !< Error no run possible SAVESTATE
----------------------------------------------------------------------------------------------------------

  # ID:  type.com.ComState uml:Enumeration
    // ! \brief return value enum for getStateInfo()  for all modules
  Members:
    COMSTATE_UNDEFINED = 0            // !< COMSTATE_UNDEFINED
    COMSTATE_OFF = 1            // !< COMSTATE_OFF
    COMSTATE_INITIALISING = 2            // !< COMSTATE_INITIALISING
    COMSTATE_READY = 3            // !< COMSTATE_READY
    COMSTATE_RUNNING = 4            // !< COMSTATE_RUNNING
    COMSTATE_FAILSAFE = 5            // !< Filesafe state
    COMSTATE_DEINITIALISING = 6            // !< COMSTATE_DEINITIALISING
----------------------------------------------------------------------------------------------------------

  # ID:  type.com.ebool_t uml:Enumeration
    //    ! \brief ebool typedef for export or measurement struct
    // full ranges from 0..0xFF is defined
    // !! never use bool   only  0 and 1 is defined
  Members:
    FALSE_e = 0            // !< FALSE_e
    TRUE_e = 1            // !< TRUE_e
----------------------------------------------------------------------------------------------------------

  # ID:  type.com.ComSSM_State_t uml:Enumeration
    // ! \brief global System state Machine states
  Members:
    COM_SSM_SYS_INIT = 0            // !< Init state default
    COM_SSM_SYS_RUN = 1            // !< normal running state
    COM_SSM_SYS_RUN_LIMITED = 2            // !< run with limited function
    COM_SSM_SYS_FAILSAFE = 3            // !< failsave, no fuction run, only support save output data
    COM_SSM_SYS_SHUTDOWN = 4            // !< shutdown no functionality switch to other states only after restart
    COM_SSM_SD_OFF = 0            
    COM_SSM_SD_RESTART = 1            
    COM_SSM_SD_GET_PERS_DATA = 2            
----------------------------------------------------------------------------------------------------------

  # ID:  type.com.ComVersion uml:Class
    // ! \brief Version information struct
    //   usage for global version information
  Members:
    type.uint8 major     
    type.uint8 minor     
    type.uint16 patch     
----------------------------------------------------------------------------------------------------------

  # ID:  type.CFG_MGR.EcuCodingState uml:Enumeration
  Members:
    CODING_STATE_INIT = 0            // CODING_STATE_INIT data or struct is not valid
    CODING_STATE_VALID = 1            // CODING_STATE_VALID data is valid for other usage
    CODING_STATE_DEFAULT = 2            // CODING_STATE_DEFAULT data is invalid or out of range, usage with care
    CODING_STATE_UNKNOWN = 128            // CODING_STATE_UNKNOWN data in error state
----------------------------------------------------------------------------------------------------------

  # ID:  type.CFG_MGR.UsSensorDegradationType uml:Enumeration
  Members:
    DEGRADATION_MODE_NO_DEGRADATION = 0            // DEGRADATION_MODE_NO_DEGRADATION data or struct is not valid
    DEGRADATION_MODE_GROUP_DEGRADATION = 1            // DEGRADATION_MODE_GROUP_DEGRADATION data is valid for other usage
    DEGRADATION_MODE_SENSOR_DEGRADATION = 2            // DEGRADATION_MODE_SENSOR_DEGRADATION data is invalid or out of range, usage with care
----------------------------------------------------------------------------------------------------------

  # ID:  type.CFG_MGR.EcuCodingPort uml:Class
  Members:
    type.uint64 timestamp_us_u64     
    type.CFG_MGR.EcuCodingState codingState_e     
    type.boolean isUsSupported     
    type.uint8 numOfUsSensors     
    type.CFG_MGR.UsSensorDegradationType UsDegradationMode_e     
    type.boolean isPdwFrontOnlySupported     
    type.boolean isPdwRearOnlySupported     
    type.boolean isPdw360Supported     
    type.boolean isBrakingFrontOnlySupported     
    type.boolean isBrakingRearOnlySupported     
    type.boolean isBraking360Supported     
    type.boolean isSteeringSuggestSupported     
    type.boolean isSteeringProtectSupported     
    type.boolean isWhlProtectSupported     
    type.boolean isSemiAuParkSupported     
    type.boolean isFullAuParkSupported     
    type.boolean isRemoteParkSupported     
    type.boolean isBasicGarageParkSupported     
----------------------------------------------------------------------------------------------------------

  # ID:  type.cml.Vec2Df_POD uml:Class
    // 2-dimensional vector as float
  Members:
    type.float32 x_dir     
    type.float32 y_dir     
----------------------------------------------------------------------------------------------------------

  # ID:  type.ECU_CTRL.VoltageState uml:Enumeration
  Members:
    VOLT_STATE_INIT = 0            // VOLT_STATE_INIT data or struct is not valid
    VOLT_STATE_NORMAL = 1            // VOLT_STATE_NORMAL data is valid for other usage
    VOLT_STATE_OVER_VOLTAGE = 2            // VOLT_STATE_OVER_VOLTAGE data is invalid or out of range, use with care
    VOLT_STATE_UNDER_VOLTAGE = 3            // VOLT_STATE_UNDER_VOLTAGE data in timeout state
    VOLT_STATE_UNKNOWN = 128            // VOLT_STATE_UNKNOWN data in error state
----------------------------------------------------------------------------------------------------------

  # ID:  type.ECU_CTRL.EcuState uml:Enumeration
  Members:
    ECU_STATE_INIT = 0            // ECU_STATE_INIT data or struct is not valid
    ECU_STATE_STARTUP = 1            // ECU_STATE_STARTUP data is valid for other usage
    ECU_STATE_RUNNING = 2            // ECU_STATE_RUNNING data is invalid or out of range, use with care
    ECU_STATE_SHUTDOWN = 3            // ECU_STATE_SHUTDOWN data in timeout state
    ECU_STATE_UNKNOWN = 128            // ECU_STATE_UNKNOWN data in error state
----------------------------------------------------------------------------------------------------------

  # ID:  type.ECU_CTRL.CommState uml:Enumeration
  Members:
    COMM_STATE_INIT = 0            // COMM_STATE_INIT data or struct is not valid
    COMM_STATE_BUS_OFF = 1            // COMM_STATE_BUS_OFF data is valid for other usage
    COMM_STATE_NO_COMM = 2            // COMM_STATE_NO_COMM data is invalid or out of range, use with care
    COMM_STATE_FULL_COMM = 3            // COMM_STATE_FULL_COMM data in timeout state
    COMM_STATE_UNKNOWN = 128            // COMM_STATE_UNKNOWN data in error state
----------------------------------------------------------------------------------------------------------

  # ID:  type.ECU_CTRL.TempState uml:Enumeration
  Members:
    TEMP_STATE_INIT = 0            // TEMP_STATE_INIT data or struct is not valid
    TEMP_STATE_NORMAL = 1            // TEMP_STATE_NORMAL data is valid for other usage
    TEMP_STATE_OVER_TEMP = 2            // TEMP_STATE_OVER_TEMP data is invalid or out of range, use with care
    TEMP_STATE_UNDER_TEMP = 3            // TEMP_STATE_UNDER_TEMP data in timeout state
    TEMP_STATE_UNKNOWN = 128            // TEMP_STATE_UNKNOWN data in error state
----------------------------------------------------------------------------------------------------------

  # ID:  type.ECU_CTRL.EcuHealthStatusPort uml:Class
  Members:
    type.uint64 timestamp_us_u64     
    type.float32 ecuVoltageValue_V     
    type.ECU_CTRL.EcuState globalState_e     
    type.ECU_CTRL.VoltageState ecuVoltageState_e     
    type.ECU_CTRL.CommState commState_e     
    type.ECU_CTRL.TempState temperatureState_e     
----------------------------------------------------------------------------------------------------------

  # ID:  type.LSM_GEOML.size_type uml:DataType
    ->:  type.uint32 uml:PrimitiveType
----------------------------------------------------------------------------------------------------------

  # ID:  type.LSM_GEOML.Pose_POD uml:Class
    // Vehicle pose as float
  Members:
    type.float32 x_dir     
    type.float32 y_dir     
    type.float32 yaw_rad     
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_Common.DrivingDirection uml:Enumeration
  Members:
    DIRECTION_UNKNOWN = 0            
    STANDSTILL = 1            
    DRIVING_FORWARDS = 2            
    DRIVING_BACKWARDS = 3            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_Common.SteeringDirection uml:Enumeration
  Members:
    CIRCLE_LEFT = 0            
    CLOTHOID_LEFT_BACKEND = 1            
    CLOTHOID_LEFT_FRONTEND = 2            
    STRAIGHT = 3            
    CIRCLE_RIGHT = 4            
    CLOTHOID_RIGHT_BACKEND = 5            
    CLOTHOID_RIGHT_FRONTEND = 6            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_Common.VehicleShapeCorners uml:Enumeration
  Members:
    FRONT_LEFT = 0            
    AXIS_LEFT = 1            
    REAR_LEFT = 2            
    REAR_RIGHT = 3            
    AXIS_RIGHT = 4            
    FRONT_RIGHT = 5            
    NUM_RELEVANT_CORNERS = 6            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_Common.VehicleBoundingShapeCorners uml:Enumeration
  Members:
    FRONT_LEFT_INNER = 0            
    FRONT_LEFT_OUTER = 1            
    AXIS_LEFT = 2            
    REAR_LEFT_OUTER = 3            
    REAR_LEFT_INNER = 4            
    REAR_RIGHT_INNER = 5            
    REAR_RIGHT_OUTER = 6            
    AXIS_RIGHT = 7            
    FRONT_RIGHT_OUTER = 8            
    FRONT_RIGHT_INNER = 9            
    NUM_RELEVANT_CORNERS = 10            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_Common.VehicleSide uml:Enumeration
  Members:
    VEHICLE_FRONT = 0            
    VEHICLE_LEFT_SIDE = 1            
    VEHICLE_REAR = 2            
    VEHICLE_RIGHT_SIDE = 3            
    VEHICLE_NO_SIDE = 4            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_Common.VehicleQuadrant uml:Enumeration
  Members:
    VEHICLE_FRONT_LEFT = 0            
    VEHICLE_REAR_LEFT = 1            
    VEHICLE_REAR_RIGHT = 2            
    VEHICLE_FRONT_RIGHT = 3            
    VEHICLE_NUM_QUADRANTS = 4            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_Common.EgoVehicleShapeType uml:Enumeration
  Members:
    EGO_VEH_SHAPE_STANDARD = 0            
    EGO_VEH_SHAPE_BOUNDING = 1            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_Common.VehiclePart uml:Enumeration
  Members:
    WHEEL_FRONT_LEFT = 0            
    WHEEL_REAR_LEFT = 1            
    WHEEL_REAR_RIGHT = 2            
    WHEEL_FRONT_RIGHT = 3            
    CAR_BODY = 4            
    MIRROR_LEFT = 5            
    MIRROR_RIGHT = 6            
    TRAILER_HITCH = 7            
    MAX_NUM_VEH_PARTS = 8            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_Common.AP_COMMON_TYPES_Consts uml:Class
  Members:
    type.uint8 AP_G_MAX_NUM_STATIC_OBJ_NU = 32    
    type.uint8 AP_G_MAX_NUM_SENSOR_TYPES_NU = 1    
    type.uint8 AP_G_MAX_NUM_PTS_STATIC_POLY_NU = 10    
    type.uint8 AP_G_MAX_NUM_DYN_OBJECTS_NU = 4    
    type.uint8 AP_G_MAX_NUM_PTS_FOR_DYN_POLY_NU = 4    
    type.uint8 AP_G_MAX_NUM_P_SPACE_MARKINGS_NU = 50    
    type.uint8 AP_G_NUM_POINTS_PER_PARKING_LINE_NU = 2    
    type.uint8 AP_G_MAX_NUM_POSES_PREV_PATH_NU = 40    
    type.uint8 AP_G_MAX_NUM_LANE_BOUNDARIES_NU = 3    
    type.uint8 AP_G_MAX_NUM_LANE_BOUND_PTS_NU = 20    
    type.uint8 AP_G_MAX_NUM_LANES_NU = 2    
    type.uint8 AP_G_MAX_NUM_US_SENSORS_NU = 12    
    type.uint8 AP_G_MAX_NUM_PARKING_BOXES_NU = 6    
    type.uint8 AP_G_MAX_NUM_EXTERNAL_POSES_NU = 4    
    type.uint8 AP_G_MAX_NUM_SV_CAMS_NU = 4    
    type.uint8 AP_G_MAX_NUM_P_BOX_VERTICES_NU = 4    
    type.uint8 AP_G_MAX_NUM_P_BOX_DELIMITERS_NU = 8    
    type.uint8 AP_G_MAX_NUM_VIRTUAL_LINES_NU = 4    
    type.uint8 AP_G_NUM_TAPOS_INFLATED_OBJ_NU = 8    
    type.uint8 AP_G_MAX_NUM_TARGET_POSES_NU = 8    
    type.uint8 COLL_G_MAX_NUM_DYN_OBJECTS_NU = 1    
    type.uint8 COLL_G_MAX_NUM_STATIC_OBJ_NU = 32    
    type.uint8 AP_G_MAX_NUM_PTS_IN_VIRTUAL_LINE_NU = 2    
    type.uint8 AP_V_MIRROR_SHAPE_MAX_SIZE_NU = 4    
    type.uint8 AP_V_HITCH_SHAPE_MAX_SIZE_NU = 4    
    type.uint8 AP_V_WHEEL_SHAPE_MAX_SIZE_NU = 6    
    type.uint8 AP_V_VEHICLE_SHAPE_MAX_SIZE_NU = 16    
    type.uint8 AP_V_NUM_WHEELS_NU = 4    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_Common.Sys_Func_Params uml:Class
  version: ::ap_common::Sys_Func_Params_InterfaceVersion::Sys_Func_Params_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 AP_G_MAX_AVG_V_MPS     // Absolute maximum allowed velocity during automated vehicle control: 10 km/h
    type.float32 AP_G_MAX_WAIT_TIME_REM_S     // Time, the vehicle waits for driver interaction during RC or RM, (e.g. operating the dead man"s switch), until the function aborts the current AP Maneuver
    type.float32 AP_G_REM_MAX_V_MPS     // Maximum velocity during remote maneuvering
    type.boolean AP_G_VARIANT_SEMI_AP_ACTIVE_NU     // Parameter to activate/dezactivate SEMI-AP variant; 1:Semi-AP is active; 0:Semi-AP is not active
    type.float32 AP_G_SEMI_AP_MAX_SPEED_WARN_MPS     // Parameter used in Semi-AP mode. Bellow this threshold, the maxSpeed10MPHwarning is a continuous warning; in case of exceeding this threshold, the warning will start blinking
    type.float32 AP_G_STANDSTILL_V_THRESH_MPS     // Velocity threshold below which the car is considered in standstill.
    type.float32 AP_G_WS_OVERSHOOT_LENGTH_M     // Length to overshoot a detected wheelstopper to compensate sensing unaccuracy of the wheelstopper position.
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_Common.Sys_Func_Params_InterfaceVersion uml:Class
  Members:
    type.uint32 Sys_Func_Params_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_Common.Vehicle_Params uml:Class
  version: ::ap_common::Vehicle_Params_InterfaceVersion::Vehicle_Params_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 AP_V_STEER_RATIO_NU     // Steer ratio (steer wheel angle to effective wheel angle).
    type.float32 AP_V_STEER_LOOKUP_ST_WHL_RAD     // Steer ratio look up table; values for steer wheel angle
    type.float32 AP_V_STEER_LOOKUP_IN_WHL_RAD     // Steer ratio look up table; values for inner wheel angle
    type.float32 AP_V_STEER_LOOKUP_OUT_WHL_RAD     // Steer ratio look up table; values for outer wheel angle
    type.float32 AP_V_STEER_LOOKUP_CTR_WHL_RAD     // Steer ratio look up table; values for effective steer angle
    type.float32 AP_V_WHEELBASE_M     // Vehicle wheelbase
    type.float32 AP_V_OVERHANG_REAR_M     // Rear overhang of vehicle.
    type.float32 AP_V_LENGTH_M     // Length of vehicle.
    type.float32 AP_V_WIDTH_M     // Width of vehicle.
    type.uint8 AP_V_WHEEL_NUMBER_OF_TEETH_NU     // Number of teeth used to measure wheel pulses.
    type.float32 AP_V_TYRE_CIRCUMFERENCE_FRONT_M     // Circumference of front wheels.
    type.float32 AP_V_TYRE_CIRCUMFERENCE_REAR_M     // Circumference of rear wheels.
    type.uint8 AP_V_NUM_STANDARD_SHAPE_PTS     // Number of valid vertices of the standard ego vehicle shape polygon
    type.float32 AP_V_STANDARD_SHAPE_X_M     // x-Values of the standard ego vehicle shape polygon. Center of rear axle is at origin.
    type.float32 AP_V_STANDARD_SHAPE_Y_M     // y-Values of the standard ego vehicle shape polygon. Center of rear axle is at origin.
    type.uint8 AP_V_NUM_BOUNDING_PTS     // Maximal number of vertices of the ego vehicle bounding box
    type.float32 AP_V_BOUNDINGBOX_X_M     // x-Values of the ego vehicle bounding box
    type.float32 AP_V_BOUNDINGBOX_Y_M     // y-Values of the ego vehicle bounding box
    type.float32 AP_V_TRACK_FRONT_M     // Track at front axle.
    type.float32 AP_V_TRACK_REAR_M     // Track at rear axle.
    type.uint8 AP_V_MIRROR_SHAPE_SIZE_NU     // Actual number of mirror vertices
    type.uint8 AP_V_HITCH_SHAPE_SIZE_NU     // Actual number of hitch vertices
    type.uint8 AP_V_WHEEL_SHAPE_SIZE_NU     // Actual number of wheel vertices
    type.float32 AP_V_LEFT_MIRROR_SHAPE_X_M     // x-values of the left mirror shape
    type.float32 AP_V_LEFT_MIRROR_SHAPE_Y_M     // y-values of the left mirror shape
    type.float32 AP_V_RIGHT_MIRROR_SHAPE_X_M     // x-values of the right mirror shape
    type.float32 AP_V_RIGHT_MIRROR_SHAPE_Y_M     // y-values of the right mirror shape
    type.float32 AP_V_HITCH_SHAPE_X_M     // x-values of the trailer hitch shape
    type.float32 AP_V_HITCH_SHAPE_Y_M     // y-values of the trailer hitch shape
    type.float32 AP_V_FL_WHEEL_SHAPE_X_M     // x-values of the left front wheel shape
    type.float32 AP_V_FL_WHEEL_SHAPE_Y_M     // y-values of the left front wheel shape
    type.float32 AP_V_RL_WHEEL_SHAPE_X_M     // x-values of the left rear wheel shape
    type.float32 AP_V_RL_WHEEL_SHAPE_Y_M     // y-values of the left rear wheel shape
    type.float32 AP_V_RR_WHEEL_SHAPE_X_M     // x-values of the right rear wheel shape
    type.float32 AP_V_RR_WHEEL_SHAPE_Y_M     // y-values of the right rear wheel shape
    type.float32 AP_V_FR_WHEEL_SHAPE_X_M     // x-values of the right front wheel shape
    type.float32 AP_V_FR_WHEEL_SHAPE_Y_M     // y-values of the right front wheel shape
    type.float32 AP_V_MAX_STEER_ANG_VEL_RADPS     // Maximum steering angle velocity (at wheels). (based on safery analysis)
    type.float32 AP_V_COMF_STEER_ANG_VEL_RADPS     // Maximum comfortable steering angle velocity (at wheels).
    type.float32 AP_V_STEER_ANG_TO_YAW_ANG_NU     // Influence of front steering angle on yaw angle (vehicle at standstill)
    type.float32 AP_V_STEER_POLY_OUT_WHL_RAD     // 3nd deg polynomial coefficients of characteristic curve steergain outer wheel
    type.float32 AP_V_STEER_POLY_CTR_WHL_RAD     // 3nd deg polynomial coefficients of characteristic curve steergain "virtual" center wheel
    type.float32 AP_V_STEER_POLY_IN_WHL_RAD     // 3nd deg polynomial coefficients of characteristic curve steergain inner wheel
    type.float32 AP_V_MAX_INFL_DIST_M     // Maximum distance for ego vehicle shape part inflation (max of all situations)
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_Common.Vehicle_Params_InterfaceVersion uml:Class
  Members:
    type.uint32 Vehicle_Params_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_Common.FC_PARKSM_Sys_Func_Params uml:Class
  version: ::ap_common::FC_PARKSM_Sys_Func_Params_InterfaceVersion::FC_PARKSM_Sys_Func_Params_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 AP_G_ESC_TIME_THRESH_S     
    type.float32 AP_G_MAX_REM_DIST_M     
    type.float32 AP_G_MAX_IDLE_TIME_S     
    type.float32 AP_G_ROLLED_DIST_IN_THRESH_M     
    type.float32 AP_G_ROLLED_DIST_OUT_THRESH_M     
    type.float32 AP_G_V_SCANNING_THRESH_MPS     
    type.float32 AP_G_V_START_AVG_THRESH_MPS     
    type.float32 AP_G_EBA_TIME_THRESH_S     
    type.float32 AP_G_RCTA_TIME_THRESH_S     
    type.float32 AP_G_TCS_TIME_THRESH_S     
    type.float32 AP_G_ABS_TIME_THRESH_S     
    type.float32 AP_G_EBD_TIME_THRESH_S     
    type.float32 AP_G_THROTTLE_THRESH_PERC     
    type.float32 AP_G_MAX_HANDOVER_TIME_S     
    type.float32 AP_G_STEERING_TORQUE_THRESH_NM     
    type.float32 AP_G_MAX_RM_DRIVE_DIST_M     
    type.float32 AP_G_PERC_HMI_DIST_BAR_M     
    type.float32 AP_G_ROLLED_DIST_RA_THRESH_M     
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_Common.FC_PARKSM_Sys_Func_Params_InterfaceVersion uml:Class
  Members:
    type.uint32 FC_PARKSM_Sys_Func_Params_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_Common.TrafficSide uml:Enumeration
    // 
  Members:
    LEFT_HAND_TRAFFIC = 0            
    RIGHT_HAND_TRAFFIC = 1            
    MAX_NUM_TRAFFIC_SIDE_TYPES = 2            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_Common.FC_TRJPLA_Vehicle_Params uml:Class
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 AP_V_MIN_SAFETY_DIST_M     // Minimum safety distance which shall be applied to vehicle shape by inflation
    type.float32 AP_V_MAX_ROBUST_DIST_M     // Maximum robustness distance which shall be applied to vehicle shape by inflation
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_Common.FC_TRJPLA_Sys_Func_Params uml:Class
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 AP_G_MAX_DECEL_COMFORTABLE_MPS2     // (L3 AUP Core) Maximum longitudinal deceleration in automated parking mode in situations that requires no emergency stop. Source of value 0.17: Winner - Fahrdynamik
    type.float32 AP_G_MAX_CURV_STEP_VEL_RED_1PM     // Limitation of absolute curvature step between two trajectory segments (same driving direction). In case of a larger curvature step in the planned trajectory the driving velocity limit will be reduced.
    type.float32 AP_G_DIST_MIN_DEFAULT_M     // Default minimal distance of ego vehicle to obstacles in final parking position
    type.float32 AP_G_DIST_MIN_NO_DELIMITER_M     // Minimal distance of ego vehicle to parking box edge without delimiter in final parking position
    type.float32 AP_G_DIST_MIN_LSIDE_HIGH_OBST_PERP_M     // Minimal distance of ego vehicle to high obstacle on long side of perpendicular parking box in final parking position. Final value should be 0.36 m. Currently (2019-05-10) CEM polygons are inflated a bit.
    type.float32 AP_G_DIST_CMF_LSIDE_HIGH_OBST_PERP_M     // Comfort distance of ego vehicle to high obstacle on long side of perpendicular parking box in final parking position
    type.float32 AP_G_DIST_MIN_SSIDE_HIGH_OBST_M     // Minimal distance of ego vehicle to high obstacle on short side of parking box in final parking position
    type.float32 AP_G_DIST_CMF_FRONT_HIGH_OBST_M     // Comfort distance of ego vehicle to high obstacle on front side of parking box in final parking position
    type.float32 AP_G_DIST_CMF_REAR_HIGH_OBST_M     // Comfort distance of ego vehicle to high obstacle on rear side of parking box in final parking position
    type.float32 AP_G_DIST_MIN_LSIDE_DOOR_OPEN_PERP_M     // Minimal distance of ego vehicle to door-openable-object on long side of perpendicular parking box in final parking position
    type.float32 AP_G_DIST_CMF_LSIDE_DOOR_OPEN_PERP_M     // Comfort distance of ego vehicle to door-openable-object on long side of perpendicular parking box in final parking position
    type.float32 AP_G_DIST_MIN_LSIDE_TRAV_PERP_M     // Minimal distance of ego vehicle to traversable obstacle on long side of perpendicular parking box in final parking position
    type.float32 AP_G_DIST_CMF_LSIDE_TRAV_PERP_M     // Comfort distance of ego vehicle to traversable obstacle on long side of perpendicular parking box in final parking position
    type.float32 AP_G_DIST_MIN_SSIDE_TRAV_M     // Minimal distance of ego vehicle to traversable obstacle on short side of parking box in final parking position
    type.float32 AP_G_DIST_CMF_FRONT_TRAV_M     // Comfort distance of ego vehicle to traversable obstacle on front side of parking box in final parking position
    type.float32 AP_G_DIST_CMF_REAR_TRAV_M     // Comfort distance of ego vehicle to traversable obstacle on rear side of parking box in final parking position
    type.float32 AP_G_DIST_MIN_LSIDE_CURB_DOWN_PERP_M     // Minimal distance of ego vehicle to descending curbstone on long side of perpendicular parking box in final parking position.
    type.float32 AP_G_DIST_MIN_FRONT_CURB_DOWN_M     // Minimal distance of ego vehicle to descending curbstone on front side parking box in final parking position.
    type.float32 AP_G_DIST_MIN_REAR_CURB_DOWN_M     // Minimal distance of ego vehicle to descending curbstone on rear side of parking box in final parking position.
    type.float32 AP_G_DIST_CMF_LSIDE_CURB_DOWN_PERP_M     // Comfort distance of ego vehicle to descending curbstone on long side of perpendicular parking box in final parking position.
    type.float32 AP_G_DIST_CMF_SSIDE_CURB_DOWN_M     // Comfort distance of ego vehicle to descending curbstone on short side of parking box in final parking position.
    type.float32 AP_G_DIST_MIN_LANE_M     // Minimal distance of ego vehicle to lane marker in final parking position.
    type.float32 AP_G_DIST_CMF_LANE_M     // Comfort distance of ego vehicle to lane marker in final parking position.
    type.float32 AP_G_DIST_MIN_PARKSLOT_MARKER_M     // Minimal distance of ego vehicle to parking slot marker in final parking position.
    type.float32 AP_G_DIST_CMF_PARKSLOT_MARKER_M     // Comfort distance of ego vehicle to parking slot marker in final parking position. TODO: Differentiate more here -> Additional parameters necessary
    type.float32 AP_G_PAR_MAX_DEVIATION_LONG_M     // Maximal deviation between ego vehicle position and target position in longitudinal direction that allows successful finish of parking maneuver for;parallel parking slots. Final value should be around 0.1 m. Current value deactivates TAPOSD check.
    type.float32 AP_G_PAR_MAX_DEVIATION_LAT_M     // Maximal deviation between ego vehicle position and target position in lateral direction that allows successful finish of parking maneuver for perpendicular parking slots
    type.float32 AP_G_PAR_MAX_DEVIATION_ANGL_RAD     // Maximal deviation between ego vehicle angle and target angle that allows successful finish of parking maneuver for parallel parking slots. (0,05 rad = 3 deg)
    type.float32 AP_G_PER_MAX_DEVIATION_LONG_M     // Maximal deviation between ego vehicle position and target position in longitudinal direction that allows successful finish of parking maneuver for perpendicular parking;slots for parallel parking slots. Final value should be around 0.1 m. Current value deactivates TAPOSD check.
    type.float32 AP_G_PER_MAX_DEVIATION_LAT_M     // Maximal deviation between ego vehicle position and target position in lateral direction that allows successful finish of parking maneuver for perpendicular parking slots
    type.float32 AP_G_PER_MAX_DEVIATION_ANGL_RAD     // Maximal deviation between ego vehicle angle and target angle that allows successful finish of parking maneuverfor perpendicular parking slots  (0,05 rad = 3 deg)
    type.AP_Common.TrafficSide AP_G_TRAFFIC_SIDE_NU     // Traffic side in current country
    type.float32 AP_G_PAR_MAX_ORI_ANGLE_RAD     // Max deviation between average scanning orientation and orientation in the final parking pose for parallel parking
    type.float32 AP_G_PERP_MAX_ORI_ANGLE_RAD     // In the final parking pose in a perp. Parking slot, the orientation of the ego vehicle shall be in an intervall of [pi -[#PARAM], pi +[#PARAM]) with reference to the average scanning orientation
    type.boolean AP_G_FREEZE_AP_EM_INPUT_NU     // Define if the EM input of the parking function shall be traced or not. It is used for the current state of the software only. Will never be part of any requirement!
    type.float32 AP_G_DIST_CMF_NO_DELIMITER_M     // Comfort distance of ego vehicle to parking box edge without delimiter in final parking position
    type.float32 AP_G_WHEEL_DIST_ROAD_LVL_PAR_M     // Min. distance of the wheels of the ego vehicle shall keep to the curbstone, when the tires are on the level of the road (for parallel parking situations)
    type.float32 AP_G_WHEEL_DIST_CURB_LVL_PAR_M     // Min.  distance of the wheels of the ego vehicle shall keep to the curbstone, when the tires are on the level of the curb (for parallel parking situations)
    type.float32 AP_G_WHEEL_DIST_ROAD_LVL_PERP_M     // Min.  distance of the wheels of the ego vehicle shall keep to the curbstone, when the tires are on the level of the road (for perpendicular parking situations)
    type.float32 AP_G_WHEEL_DIST_CURB_LVL_PERP_M     // Min. distance of the wheels of the ego vehicle shall keep to the curbstone, when the tires are on the level of the curb(for perpendicular parking situations)
    type.float32 AP_G_PARK_OUT_PAR_MIN_ANG_RAD     // Min. absolute orientation of the ego vehicle during a parallel parking out maneuver, in case that the parking slot is not delimited by an static obstacle in the front of the vehicle.
    type.float32 AP_G_LONG_SLOT_REL_LENGTH_M     // If length of a parking slot minus vehicle length is bigger or equal to this value it is considered as "long".
    type.float32 AP_G_MAX_DEVIATION_LONG_SLOT_M     // Max allowed deviation in case of a long parking slot.
    type.float32 AP_G_WIDE_SLOT_REL_WIDTH_M     // If width of a pakring slot minus vehicle width is bigger or equal to this value it is considered as "wide".
    type.float32 AP_G_MAX_DEVIATION_WIDE_SLOT_M     // Max allowed deviation in case of a wide parking slot.
    type.float32 AP_G_DIST_MIN_GP_ENTRANCE_CLR_M     // Minimum distance vehicle shall have to the garage door in final parking pose
    type.float32 AP_G_DIST_CMF_GP_ENTRANCE_CLR_M     // Comfort distance vehicle shall have to the garage door in final parking pose
    type.float32 AP_G_DIST_CMF_GP_DRIVER_M     // Comfort distance of ego vehicle to side wall on driver"s side in garage
    type.float32 AP_G_DIST_CMF_GP_PASSENGER_M     // Comfort distance of ego vehicle to side wall on passenger"s side in garage
    type.float32 AP_G_PAR_MAX_LAT_MISALIGNMENT_M     // Maxiumum lateral misaligment of two objects, bordering a parallel parking slot at the first side edge and second side edge.
    type.float32 AP_G_PAR_LON_PLAN_DEV_RATIO_NU     // Ratio of the longitudinal deviation (i.e., of AP_G_MAX_DEVIATION_LONG_SLOT_M or AP_G_PAR_MAX_DEVIATION_LONG_M) the planner is allowed to use during planning for parallel parking.
    type.float32 AP_G_PAR_LAT_PLAN_DEV_RATIO_NU     // Ratio of the lateral deviation (i.e., of AP_G_PAR_MAX_DEVIATION_LAT_M) the planner is allowed to use during planning for parallel parking.
    type.float32 AP_G_PAR_YAW_PLAN_DEV_RATIO_NU     // Ratio of the yaw deviation (i.e., of AP_G_PAR_MAX_DEVIATION_ANGL_RAD) the planner is allowed to use during planning for parallel parking.
    type.float32 AP_G_PER_LON_PLAN_DEV_RATIO_NU     // Ratio of the longitudinal deviation (i.e., of AP_G_PER_MAX_DEVIATION_LONG_M) the planner is allowed to use during planning for perpendicular parking.
    type.float32 AP_G_PER_LAT_PLAN_DEV_RATIO_NU     // Ratio of the lateral deviation (i.e., of AP_G_MAX_DEVIATION_WIDE_SLOT_M or AP_G_PER_MAX_DEVIATION_LAT_M) the planner is allowed to use during planning for perpendicular parking.
    type.float32 AP_G_PER_YAW_PLAN_DEV_RATIO_NU     // Ratio of the yaw deviation (i.e., of AP_G_PER_MAX_DEVIATION_ANGL_RAD) the planner is allowed to use during planning for perpendicular parking.
    type.float32 AP_G_POUT_PERP_LAT_PLAN_DEV_RATIO_NU     // Ratio of the lateral deviation (i.e., of AP_T_POUT_PERP_DEV_LAT_M) the planner is allowed to use during planning for perpendicular parking out.
    type.float32 AP_G_POUT_PERP_YAW_PLAN_DEV_RATIO_NU     // Ratio of the yaw deviation (i.e., of AP_T_POUT_PERP_DEV_YAW_RAD) the planner is allowed to use during planning for perpendicular parking out.
    type.float32 AP_G_PERP_MAX_LONG_MISALIGNMENT_M     // Maxiumum longitudinal misaligment of two objects, bordering a perpendicular slot at the first side edge and second side edge.
    type.float32 AP_G_ANGLED_MAX_LONG_MISALIGNMENT_M     // Maxiumum longitudinal misaligment of two objects, bordering a angled parking slot at the first side edge and second side edge.
    type.float32 AP_G_DIST_MIN_LSIDE_HIGH_OBST_PAR_M     // Minimal distance of ego vehicle to high obstacle on long side of parallel parking box in final parking position.
    type.float32 AP_G_DIST_CMF_LSIDE_HIGH_OBST_PAR_M     // Comfort distance of ego vehicle to high obstacle on long side of parallel parking box in final parking position.
    type.float32 AP_G_DIST_MIN_LSIDE_DOOR_OPEN_PAR_M     // Minimal distance of ego vehicle to door-openable-object on long side of parallel parking box in final parking position.
    type.float32 AP_G_DIST_CMF_LSIDE_DOOR_OPEN_PAR_M     // Comfort distance of ego vehicle to door-openable-object on long side of parallel parking box in final parking position.
    type.float32 AP_G_DIST_MIN_LSIDE_TRAV_PAR_M     // Minimal distance of ego vehicle to traversable obstacle on long side of parallel parking box in final parking position.
    type.float32 AP_G_DIST_CMF_LSIDE_TRAV_PAR_M     // Comfort distance of ego vehicle to traversable obstacle on long side of parallel parking box in final parking position.
    type.float32 AP_G_DIST_MIN_LSIDE_CURB_DOWN_PAR_M     // Minimal distance of ego vehicle to descending curbstone on long side of parallel parking box in final parking position.
    type.float32 AP_G_DIST_CMF_LSIDE_CURB_DOWN_PAR_M     // Comfort distance of ego vehicle to descending curbstone on long side of parallel parking box in final parking position.
    type.float32 AP_G_DIST_MIN_LSIDE_HIGH_OBST_FB_M     // Fallback distance of ego vehicle to high obstacle on long side of parallel/perp/angled parking box in final parking position.
    type.float32 AP_G_DIST_MIN_SSIDE_HIGH_OBST_FB_M     // Fallback distance of ego vehicle to high obstacle on short side of parallel/perp/angled parking box in final parking position.
    type.float32 AP_G_DIST_STATIC_OBJ_ROAD_SHIFT_PERP_M     // External shift on target pose to compensate e.g. environment inaccuracies in perp/angled parking.
    type.float32 AP_G_DIST_STATIC_OBJ_ROAD_SHIFT_PAR_M     // External shift on target pose to compensate e.g. environment inaccuracies in perp/angled parking.
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.AmbientDataPort uml:Class
  version: ::ap_commonvehsigprovider::AmbientDataPort_InterfaceVersion::AmbientDataPort_VERSION
    // None
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 Ambient_temperature     // @unit{deg C};@range{-40,85};Ambient temperature
    type.float32 Ambient_pressure     // @unit{bar};Ambient pressure
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.AmbientDataPort_InterfaceVersion uml:Class
  Members:
    type.uint32 AmbientDataPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.BrakeCtrlStatusPort uml:Class
  version: ::ap_commonvehsigprovider::BrakeCtrlStatusPort_InterfaceVersion::BrakeCtrlStatusPort_VERSION
    // None
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.boolean vehicleBraked_nu     // information if vehicle is braked by driver or system) or not
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.BrakeCtrlStatusPort_InterfaceVersion uml:Class
  Members:
    type.uint32 BrakeCtrlStatusPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.DdsPort uml:Class
  version: ::ap_commonvehsigprovider::DdsPort_InterfaceVersion::DdsPort_VERSION
    // None
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.boolean Dds_warning_active     // @unit{boolean};Tire pressure loss warning active
    type.boolean Tire_change_reset_button     // @unit{boolean};driver indicated that tire was changed
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.DdsPort_InterfaceVersion uml:Class
  Members:
    type.uint32 DdsPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.DrvWarnSMCoreSampleTimePort uml:Class
  version: ::ap_commonvehsigprovider::DrvWarnSMCoreSampleTimePort_InterfaceVersion::DrvWarnSMCoreSampleTimePort_VERSION
    // None
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint64 drvWarnSMCoreSampleTime_us     // @unit{microsecond};Sample time between last and current call of DrvWarnSMCore component
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.DrvWarnSMCoreSampleTimePort_InterfaceVersion uml:Class
  Members:
    type.uint32 DrvWarnSMCoreSampleTimePort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.DrvWarnSMSampleTimePort uml:Class
  version: ::ap_commonvehsigprovider::DrvWarnSMSampleTimePort_InterfaceVersion::DrvWarnSMSampleTimePort_VERSION
    // None
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint64 drvWarnSMSampleTime_us     // @unit{microsecond};Sample time between last and current call of  DrvWarnSM function component
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.DrvWarnSMSampleTimePort_InterfaceVersion uml:Class
  Members:
    type.uint32 DrvWarnSMSampleTimePort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------


  # ID:  type.AP_CommonVehSigProvider.EngineCtrlStatusPort uml:Class
  version: ::ap_commonvehsigprovider::EngineCtrlStatusPort_InterfaceVersion::EngineCtrlStatusPort_VERSION
    // Signals from Engine ECU
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 throttlePos_perc     // @unit{Percentage};@range{0,101.6};Throttle position in percent
    type.float32 throttleGradient_percps     // @unit{Percentage per second};Throttle gradient is calculated by two 4 ms scanned throttle positions
    type.float32 axleTorque_nm     // @unit{torque in Nm};Torquq at axle in Nm
    type.boolean engineOn_nu     // Engine is running
    type.AP_CommonVehSigProvider.StartStopStatus startStopStatus_nu     // @range{0,3};Status of start stop function should be in status N/A or "no clearance" to prevent turning off the engine while parking
    type.boolean remoteStartPossible_nu     // Remote start possible
    type.boolean throttlePos_QF_nu     // Qualifier-Bit of Throttle position
    type.boolean throttleGradient_QF_nu     // Qualifier-Bit Throttle position
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.EngineCtrlStatusPort_InterfaceVersion uml:Class
  Members:
    type.uint32 EngineCtrlStatusPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------




  # ID:  type.AP_CommonVehSigProvider.GPSData uml:Class
    // Data specifications of the GPS System.
  Members:
    type.float32 gpsAntennaHeight_m     // @unit{m};@range{0,99999.9};
    type.sint32 gpsLatitude_dd     // @unit{deg};@range{0,90};
    type.float32 gpsLatitude_mm     // @unit{min};@range{0,60};
    type.sint32 gpsLongitude_dd     // @unit{grad};@range{0,180};
    type.float32 gpsLongitude_mm     // @unit{min};@range{0,60};
    type.float32 gpsSpeed_mps     // @unit{m/s};@range{0,500};GPS speed over ground
    type.float32 gpsR32SpeedOverGround_mps     // @unit{m/s};@range{0,500};GPS speed over ground. (Filled signal)
    type.float32 gpsCourseOverGround     // @unit{rad};@range{0,6.28};
    type.float32 verticalDOP     // @unit{nu};@range{0,25.575};Vertical Dilution Of Precision (VDOP) and its is a value of probability for the height inaccuracy effect on GPS accuracy
    type.float32 horizontalDOP     // @unit{nu};@range{0,25.575};Horizontal Dilution Of Precision (HDOP) and it is a value of probability for the horizontal geometric effect (latitude and longitude) on GPS accuracy.
    type.float32 timeDOP     // @unit{nu};@range{0,25.575};Time Dilution Of Precision (TDOP) and its is a value of probability for the time inaccuracy effect on GPS accuracy.
    type.float32 geometricDOP     // @unit{nu};@range{0,25.575};Geometric dilution of precision (Overall accuracy including 3D position und Time)
    type.float32 positionDOP     // @unit{nu};@range{0,25.575};Positional dilution of precision (Position accuracy, 3D)
    type.uint16 gpsUtcTime_hh     // @unit{std};@range{0,24};
    type.uint16 gpsUtcTime_mm     // @unit{min};@range{0,59};
    type.uint16 gpsUtcTime_ss     // @unit{sec};@range{0,59};
    type.AP_CommonVehSigProvider.Hemisphere gpsLatitudeHemisphere_nu     // 
    type.AP_CommonVehSigProvider.Hemisphere gpsLongitudeHemisphere_nu     // 
    type.uint8 gpsDateDay_dd     // @unit{day};@range{0,31};
    type.uint8 gpsDateMonth_mm     // @unit{month};@range{0,12};
    type.uint8 gpsDateYear_yy     // @unit{year};@range{0,99};
    type.uint8 gpsFix     // @unit{nu};@range{0,3};Dimension accuracy of current GPS measurement. (ex: 2 -> 2D, 3 -> 3D)
    type.uint8 gpsNoOfSatellites     // @unit{nu};@range{0,255};GPS number of satellites
    type.AP_CommonVehSigProvider.GpsReceiverStatus ReceiverStatus_nu     // GPS Receiver status
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.Gear uml:Enumeration
    // System status/Availability of the gearbox control and current gear.
  Members:
    GEAR_N = 0            
    GEAR_1 = 1            
    GEAR_2 = 2            
    GEAR_3 = 3            
    GEAR_4 = 4            
    GEAR_5 = 5            
    GEAR_6 = 6            
    GEAR_7 = 7            
    GEAR_8 = 8            
    GEAR_P = 9            
    GEAR_S = 10            
    GEAR_D = 11            
    GEAR_INTERMEDIATE_POS = 12            
    GEAR_R = 13            
    GEAR_NOT_DEFINED = 14            
    GEAR_ERROR = 15            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.GearBoxCtrlSystemState uml:Enumeration
    // System status/Availability of the gearbox control.
  Members:
    GCTRL_NOT_AVAILABLE = 0            
    GCTRL_INITIALISATION = 1            
    GCTRL_AVAILABLE = 2            
    GCTRL_ERROR = 7            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.GearInformation uml:Class
    // None
  Members:
    type.AP_CommonVehSigProvider.GearBoxCtrlSystemState gearboxCtrlSystemState_nu     // @range{0,7};System status/Availability of the gearbox control.
    type.AP_CommonVehSigProvider.Gear gearCur_nu     // @range{0,15};System status/Availability of the gearbox control and current gear.
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.GearLeverInformation uml:Class
    // None
  Members:
    type.AP_CommonVehSigProvider.Gear gearLeverPositionCur_nu     // @range{0,15};Current position of gear lever.
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.GearboxCtrlStatusPort uml:Class
  version: ::ap_commonvehsigprovider::GearboxCtrlStatusPort_InterfaceVersion::GearboxCtrlStatusPort_VERSION
    // From Gearbox Control.
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.AP_CommonVehSigProvider.GearInformation gearInformation     // @unit{nu};
    type.AP_CommonVehSigProvider.GearLeverInformation gearLeverInformation     // @unit{nu};
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.GearboxCtrlStatusPort_InterfaceVersion uml:Class
  Members:
    type.uint32 GearboxCtrlStatusPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.GpsReceiverStatus uml:Enumeration
    // GPS Receiver status
  Members:
    ODO_GPS_INVALID = 0            
    ODO_GPS_VALID = 1            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.HMIHandlerSampleTimePort uml:Class
  version: ::ap_commonvehsigprovider::HMIHandlerSampleTimePort_InterfaceVersion::HMIHandlerSampleTimePort_VERSION
    // None
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint64 hmiHandlerSampleTime_us     // @unit{microsecond};
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.HMIHandlerSampleTimePort_InterfaceVersion uml:Class
  Members:
    type.uint32 HMIHandlerSampleTimePort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.Hemisphere uml:Enumeration
    // None
  Members:
    ODO_GPS_NO_SIGNAL = 0            
    ODO_GPS_NORTH = 1            
    ODO_GPS_SOUTH = 2            
    ODO_GPS_EAST = 3            
    ODO_GPS_WEST = 4            
----------------------------------------------------------------------------------------------------------



  # ID:  type.AP_CommonVehSigProvider.MotionStatePort uml:Class
  version: ::ap_commonvehsigprovider::MotionStatePort_InterfaceVersion::MotionStatePort_VERSION
    // Measurements desribing the current motion state
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 vRefESC_mps     // @unit{m/s};@range{0,104.854};reference vehicle velocity ESC
    type.AP_CommonVehSigProvider.VehicleDrivingDirection vehicleDrivingDirection_nu     // @range{-1,2};vehicle driving direction
    type.boolean vRefESC_QF_nu     // Qualifier-Bit vehicle direction
    type.boolean vehicleStandstill_nu     // Vehicle standstill information
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.MotionStatePort_InterfaceVersion uml:Class
  Members:
    type.uint32 MotionStatePort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.DebugConstants uml:Class
    // 
  Members:
    type.uint8 NUM_OF_WHEELS_NU = 4    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.OdoExtCtrlPort uml:Class
  version: ::ap_commonvehsigprovider::OdoExtCtrlPort_InterfaceVersion::OdoExtCtrlPort_VERSION
    // None
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.boolean resetPoseEstimation_nu     // Signal in order to reset the pose estimation
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.OdoExtCtrlPort_InterfaceVersion uml:Class
  Members:
    type.uint32 OdoExtCtrlPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.OdoGpsPort uml:Class
  version: ::ap_commonvehsigprovider::OdoGpsPort_InterfaceVersion::OdoGpsPort_VERSION
    // None
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.AP_CommonVehSigProvider.GPSData gpsData     // @unit{nu};Data specifications of the GPS System.
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.OdoGpsPort_InterfaceVersion uml:Class
  Members:
    type.uint32 OdoGpsPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.PARKSMCoreSampleTimePort uml:Class
  version: ::ap_commonvehsigprovider::PARKSMCoreSampleTimePort_InterfaceVersion::PARKSMCoreSampleTimePort_VERSION
    // None
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint64 parksmCoreSampleTime_us     // @unit{microsecond};
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.PARKSMCoreSampleTimePort_InterfaceVersion uml:Class
  Members:
    type.uint32 PARKSMCoreSampleTimePort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.PSMSampleTimePort uml:Class
  version: ::ap_commonvehsigprovider::PSMSampleTimePort_InterfaceVersion::PSMSampleTimePort_VERSION
    // None
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint64 psmSampleTime_us     // @unit{microsecond};
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.PSMSampleTimePort_InterfaceVersion uml:Class
  Members:
    type.uint32 PSMSampleTimePort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.StartStopStatus uml:Enumeration
    // Status of start stop function should be in status N/A or "no clearance" to prevent turning off the engine while parking
  Members:
    STOP_GRANTED = 0            
    STOP_PROHIBITED = 1            
    START_REQUESTED = 2            
    START_STOP_ERROR = 3            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.SteerCtrlStatusPort uml:Class
  version: ::ap_commonvehsigprovider::SteerCtrlStatusPort_InterfaceVersion::SteerCtrlStatusPort_VERSION
    // Measurements from the Steer Control.
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 steeringWheelAngle_rad     // @unit{rad};@range{-32.767,32.767};steer column angle (VW uncalibrated). Measured by power steering system.
    type.float32 steeringWheelAngleOffset_rad     // @unit{rad};steering wheel angle signal offset estimated by signal provider. (to calculate raw value signal is calibrated)
    type.float32 steeringWheelAngleVelocity_radps     // @unit{radps};@range{-50,50};steer column turning velocity
    type.float32 epsAppliedTieRodForce_Nm     // @unit{Nm};@range{0,12};Tie rod force applied by electric power steering
    type.boolean steeringWheelAngle_QF_nu     // Qualifier-Bit of steer column angle
    type.boolean steeringWheelAngleVelocity_QF_nu     // Qualifier-Bit of steer column angle velocity
    type.boolean calculatedSteeringWheelAngle_QF_nu     // Qualifier-Bit of calculated steering wheel angle
    type.boolean epsAppliedTieRodForce_QF_Nm     // @unit{boolean};Qualifier-Bit of tie rod force
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.SteerCtrlStatusPort_InterfaceVersion uml:Class
  Members:
    type.uint32 SteerCtrlStatusPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.SuspensionPort uml:Class
  version: ::ap_commonvehsigprovider::SuspensionPort_InterfaceVersion::SuspensionPort_VERSION
    // Measurements from the suspension
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 suspensionTravelFL_m     // @range{-0.254,0.254};@unit{m};calibrated suspension travel front left
    type.float32 suspensionTravelFR_m     // @range{-0.254,0.254};@unit{m};calibrated suspension travel front right
    type.float32 suspensionTravelRL_m     // @range{-0.254,0.254};@unit{m};calibrated suspension travel rear left
    type.float32 suspensionTravelRR_m     // @range{-0.254,0.254};@unit{m};calibrated suspension travel rear right
    type.float32 suspensionTravelFL_pct     // @range{0,100};@unit{%};raw suspension travel front left
    type.float32 suspensionTravelFR_pct     // @range{0,100};@unit{%};raw suspension travel front right
    type.float32 suspensionTravelRL_pct     // @range{0,100};@unit{%};raw suspension travel rear left
    type.float32 suspensionTravelRR_pct     // @range{0,100};@unit{%};raw suspension travel rear right
    type.boolean isSuspensionCalibrated     // @unit{boolean};None
    type.boolean suspensionTravelFL_QF     // @unit{boolean};None
    type.boolean suspensionTravelFR_QF     // @unit{boolean};None
    type.boolean suspensionTravelRL_QF     // @unit{boolean};None
    type.boolean suspensionTravelRR_QF     // @unit{boolean};None
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.SuspensionPort_InterfaceVersion uml:Class
  Members:
    type.uint32 SuspensionPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.SystemTimePort uml:Class
  version: ::ap_commonvehsigprovider::SystemTimePort_InterfaceVersion::SystemTimePort_VERSION
    // None
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.SystemTimePort_InterfaceVersion uml:Class
  Members:
    type.uint32 SystemTimePort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.THSampleTimePort uml:Class
  version: ::ap_commonvehsigprovider::THSampleTimePort_InterfaceVersion::THSampleTimePort_VERSION
    // None
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint64 thSampleTime_us     // @unit{microsecond};Sample time between last and current call of  ToneHandler component.
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.THSampleTimePort_InterfaceVersion uml:Class
  Members:
    type.uint32 THSampleTimePort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.TRJCTLGeneralInputPort uml:Class
  version: ::ap_commonvehsigprovider::TRJCTLGeneralInputPort_InterfaceVersion::TRJCTLGeneralInputPort_VERSION
    // From base software/ECU.
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 trjctlSampleTime_s     // @unit{s};Sample time between last and current call of function component Trajectory Control.
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.TRJCTLGeneralInputPort_InterfaceVersion uml:Class
  Members:
    type.uint32 TRJCTLGeneralInputPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.VehDynamicsPort uml:Class
  version: ::ap_commonvehsigprovider::VehDynamicsPort_InterfaceVersion::VehDynamicsPort_VERSION
    // Measurements describing general vehicle dynamics
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 yawRate_radps     // @unit{rad/s};@range{-3.2767,3.2767};vehicles yaw rate around z-axis. (VW Uncalibrated)
    type.float32 yawRateOffset_radps     // @unit{rad/s};@range{-0.1785471824790199,0.17837264955382048};Estimatet offset of yaw rate calculated in ESP.
    type.float32 pitchRate_radps     // @unit{rad/s};pitch rate
    type.float32 pitchRateOffset_radps     // @unit{rad/s};pitch rate offset
    type.float32 rollRate_radps     // @unit{rad/s};roll rate
    type.float32 rollRateOffset_radps     // @unit{rad/s};roll rate offset
    type.float32 lateralAcceleration_mps2     // @unit{m/s^2};@range{-10,10};vehicle acceleration on y-axis (VW calibrated)
    type.float32 lateralAccelerationOffset_mps2     // @unit{m/s^2};@range{-0.3,0.3};lateral acceleration signal offset estimated by signal provider. (to calculate raw value signal is calibrated) Lat_acc = raw_value - offset
    type.float32 longitudinalAcceleration_mps2     // @unit{m/s^2};@range{-10,10};vehicle acceleration on x-axis (VW calibrated)
    type.float32 longitudinalAccelerationOffset_mps2     // @unit{m/s^2};@range{-3.9375,3.9375};longitudinal acceleration signal offset estimated by signal provider. (to calculate raw value signal is calibrated) Long_acc = raw_value - offset
    type.float32 verticalAcceleration_mps2     // @unit{nu};vertical acceleration
    type.float32 verticalAccelerationOffset_mps2     // @unit{nu};vertical acceleration offset
    type.boolean yawRate_QF_nu     // Qualifier-Bit of yaw rate signal
    type.boolean pitchRate_QF_nu     // Qualifier-Bit of pitch rate signal
    type.boolean rollRate_QF_nu     // Qualifier-Bit of roll rate signal
    type.boolean lateralAcceleration_QF_nu     // Qualifier-Bit of lateral acceleration signal
    type.boolean longitudinalAcceleration_QF_nu     // Qualifier-Bit of longitudinal signal
    type.boolean verticalAcceleration_QF_nu     // Qualifier-Bit of vertical signal
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.VehDynamicsPort_InterfaceVersion uml:Class
  Members:
    type.uint32 VehDynamicsPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.VehicleDrivingDirection uml:Enumeration
    // vehicle driving direction
  Members:
    VEHICLE_DIRECTION_REVERSE = -1            
    VEHICLE_DIRECTION_INIT_INVALID = 0            
    VEHICLE_DIRECTION_FORWARD = 1            
    VEHICLE_DIRECTION_STANDSTILL = 2            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.WheelDrivingDirection uml:Enumeration
    // rotational direction wheel front left
  Members:
    WHEEL_DIRECTION_REVERSE = -1            
    WHEEL_DIRECTION_INIT_INVALID = 0            
    WHEEL_DIRECTION_FORWARD = 1            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.WheelDrivingDirectionsPort uml:Class
  version: ::ap_commonvehsigprovider::WheelDrivingDirectionsPort_InterfaceVersion::WheelDrivingDirectionsPort_VERSION
    // Measurements describing the driving directions provided by the wheel speed sensor
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.AP_CommonVehSigProvider.WheelDrivingDirection wheelDrivingDirection_FL_nu     // @range{-1,1};rotational direction wheel front left
    type.AP_CommonVehSigProvider.WheelDrivingDirection wheelDrivingDirection_FR_nu     // @range{-1,1};rotational direction wheel front right
    type.AP_CommonVehSigProvider.WheelDrivingDirection wheelDrivingDirection_RL_nu     // @range{-1,1};rotational direction wheel rear left
    type.AP_CommonVehSigProvider.WheelDrivingDirection wheelDrivingDirection_RR_nu     // @range{-1,1};rotational direction wheel rear right
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.WheelDrivingDirectionsPort_InterfaceVersion uml:Class
  Members:
    type.uint32 WheelDrivingDirectionsPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.WheelLinSpeedPort uml:Class
  version: ::ap_commonvehsigprovider::WheelLinSpeedPort_InterfaceVersion::WheelLinSpeedPort_VERSION
    // Measurements describing the linear wheel speeds
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 wheelLinSpeedFL_mps     // @unit{m/s};wheel linear velocity front left
    type.float32 wheelLinSpeedFR_mps     // @unit{m/s};wheel linear velocity front right
    type.float32 wheelLinSpeedRL_mps     // @unit{m/s};wheel linear velocity rear left
    type.float32 wheelLinSpeedRR_mps     // @unit{m/s};wheel linear velocity rear right
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.WheelLinSpeedPort_InterfaceVersion uml:Class
  Members:
    type.uint32 WheelLinSpeedPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.WheelPulsePort uml:Class
  version: ::ap_commonvehsigprovider::WheelPulsePort_InterfaceVersion::WheelPulsePort_VERSION
    // Measurements describing the wheel pulses
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint16 wheelPulsesFL_nu     // @range{0,1000};Number of wheel-sensor impulses front left. Rising and falling edge.
    type.uint16 wheelPulsesFR_nu     // @range{0,1000};Number of wheel-sensor impulses front right. Rising and falling edge.
    type.uint16 wheelPulsesRL_nu     // @range{0,1000};Number of wheel-sensor impulses rear left. Rising and falling edge.
    type.uint16 wheelPulsesRR_nu     // @range{0,1000};Number of wheel-sensor impulses rear right. Rising and falling edge.
    type.boolean wheelPulsesFL_QF_nu     // Qualifier-Bit Number of wheel-sensor impulse signal front left
    type.boolean wheelPulsesFR_QF_nu     // Qualifier-Bit Number of wheel-sensor impulse signal front right.
    type.boolean wheelPulsesRL_QF_nu     // Qualifier-Bit Number of wheel-sensor impulse signal rear left.
    type.boolean wheelPulsesRR_QF_nu     // Qualifier-Bit Number of wheel-sensor impulse signal rear right.
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.WheelPulsePort_InterfaceVersion uml:Class
  Members:
    type.uint32 WheelPulsePort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.WheelSpeedPort uml:Class
  version: ::ap_commonvehsigprovider::WheelSpeedPort_InterfaceVersion::WheelSpeedPort_VERSION
    // Measurements describing the rotational wheel speeds
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 wheelRotSpeedFL_radps     // @unit{rad/s};wheel rotational velocity front left
    type.float32 wheelRotSpeedFR_radps     // @unit{rad/s};wheel rotational velocity front right
    type.float32 wheelRotSpeedRL_radps     // @unit{rad/s};wheel rotational velocity rear left
    type.float32 wheelRotSpeedRR_radps     // @unit{rad/s};wheel rotational velocity rear right
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.WheelSpeedPort_InterfaceVersion uml:Class
  Members:
    type.uint32 WheelSpeedPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.WheelTireEscPort uml:Class
  version: ::ap_commonvehsigprovider::WheelTireEscPort_InterfaceVersion::WheelTireEscPort_VERSION
    // Measurements describing the tyre circumference used by the brake ecu to calculate the wheel velocity
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 tireCircumFL_m     // @unit{m};@range{0,4.095};tyre circumference on front left wheel
    type.float32 tireCircumFR_m     // @unit{m};@range{0,4.095};tyre circumference on front right wheel
    type.float32 tireCircumRL_m     // @unit{m};@range{0,4.095};tyre circumference on rear left wheel
    type.float32 tireCircumRR_m     // @unit{m};@range{0,4.095};tyre circumference on rear right wheel
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_CommonVehSigProvider.WheelTireEscPort_InterfaceVersion uml:Class
  Members:
    type.uint32 WheelTireEscPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------


  # ID:  type.AP_HMIToAP.HMIOutputPort uml:Class
  version: ::ap_hmitoap::HMIOutputPort_InterfaceVersion::HMIOutputPort_VERSION
    // None
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.AP_HMIToAP.UserActionHeadUnit userActionHeadUnit_nu     // @range{0,255};User interaction with HMI (Head Unit)
    type.uint8 userActionHUCounter_nu     
    type.boolean pdwAutoActivate_nu     
    type.boolean lscaAutoActivate_nu     
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_HMIToAP.HMIOutputPort_InterfaceVersion uml:Class
  Members:
    type.uint32 HMIOutputPort_VERSION = 3U    
----------------------------------------------------------------------------------------------------------






  # ID:  type.AP_HMIToAP.VisuInputData uml:Class
  version: ::ap_hmitoap::VisuInputData_InterfaceVersion::VisuInputData_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.sint16 firstClickEventX_px_u16     
    type.sint16 firstClickEventY_px_u16     
    type.sint16 secondClickEventX_px_u16     
    type.sint16 secondClickEventY_px_u16     
    type.AP_HMIToAP.ScreenTypes HmiOutUserActScreenReq_u8     
    type.AP_HMIToAP.GestureCode GestureCode_nu_u8     
    type.AP_HMIToAP.BlindSpotView blindSpotActivated_nu     
    type.AP_HMIToAP.ParkingAugmentationType parkingAugmentationType_nu     
    type.uint8 GestureFinger_nu_u8     
    type.uint8 gestureCounter_nu     
    type.uint8 visibilityTags_nu     
    type.boolean videoRecorderRequest_nu     
    type.boolean screenCaptureRequest_nu     
    type.boolean isSequence_nu     
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_HMIToAP.VisuInputData_InterfaceVersion uml:Class
  Members:
    type.uint32 VisuInputData_VERSION = 3U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_HMIToAP.UserActionHeadUnit uml:Enumeration
    // User interaction with HMI (Head Unit)
    // Gesture suported by HMI.
    // Types of blindspot detection suported by HMI.
    // Types of parking augmentation types suported by HMI.
  Members:
    NO_USER_ACTION = 0            
    TAP_ON_PARKING_SPACE_LEFT_1 = 1            
    TAP_ON_PARKING_SPACE_LEFT_2 = 2            
    TAP_ON_PARKING_SPACE_LEFT_3 = 3            
    TAP_ON_PARKING_SPACE_LEFT_4 = 4            
    TAP_ON_PARKING_SPACE_RIGHT_1 = 5            
    TAP_ON_PARKING_SPACE_RIGHT_2 = 6            
    TAP_ON_PARKING_SPACE_RIGHT_3 = 7            
    TAP_ON_PARKING_SPACE_RIGHT_4 = 8            
    TAP_ON_PARKING_SPACE_FRONT_1 = 9            
    TAP_ON_PARKING_SPACE_FRONT_2 = 10            
    TAP_ON_PARKING_SPACE_FRONT_3 = 11            
    TAP_ON_PARKING_SPACE_FRONT_4 = 12            
    TAP_ON_PARKING_SPACE_REAR_1 = 13            
    TAP_ON_PARKING_SPACE_REAR_2 = 14            
    TAP_ON_PARKING_SPACE_REAR_3 = 15            
    TAP_ON_PARKING_SPACE_REAR_4 = 16            
    TAP_ON_START_SELECTION = 17            
    TAP_ON_START_PARKING = 18            
    TAP_ON_INTERRUPT = 19            
    TAP_ON_CONTINUE = 20            
    TAP_ON_UNDO = 21            
    TAP_ON_CANCEL = 22            
    TAP_ON_REDO = 23            
    TAP_ON_START_REMOTE_PARKING = 24            
    TAP_ON_SWITCH_DIRECTION = 25            
    TAP_ON_SWITCH_ORIENTATION = 26            
    TAP_ON_PREVIOUS_SCREEN = 27            
    TOGGLE_AP_ACTIVE = 28            
    TAP_ON_LSCA_RELEASE_BRAKE = 29            
    TAP_ON_FULLY_AUTOMATED_PARKING = 30            
    TAP_ON_SEMI_AUTOMATED_PARKING = 31            
    TAP_ON_START_KEY_PARKING = 32            
    TAP_ON_GP = 33            
    TAP_ON_RM = 34            
    TAP_ON_PDC = 35            
    TAP_ON_AP_PDC_TOGGLE_VIEW = 36            
    TAP_ON_SWITCH_TO_REMOTE_KEY = 37            
    TAP_ON_SWITCH_TO_REMOTE_APP = 38            
    TAP_ON_WHP = 39            
    TAP_ON_USER_SLOT_LEFT_PAR = 40            
    TAP_ON_USER_SLOT_LEFT_PERP_BWD = 41            
    TAP_ON_USER_SLOT_LEFT_PERP_FWD = 42            
    TAP_ON_USER_SLOT_RIGHT_PAR = 43            
    TAP_ON_USER_SLOT_RIGHT_PERP_BWD = 44            
    TAP_ON_USER_SLOT_RIGHT_PERP_FWD = 45            
    TAP_ON_USER_SLOT_MOVE_UP = 46            
    TAP_ON_USER_SLOT_MOVE_DOWN = 47            
    TAP_ON_USER_SLOT_MOVE_LEFT = 48            
    TAP_ON_USER_SLOT_MOVE_RIGHT = 49            
    TAP_ON_USER_SLOT_ROT_CLKWISE = 50            
    TAP_ON_USER_SLOT_ROT_CTRCLKWISE = 51            
    TAP_ON_USER_SLOT_RESET = 52            
    TAP_ON_USER_SLOT_SAVE = 53            
    TAP_ON_EXPLICIT_SCANNING = 54            
    TAP_ON_REVERSE_ASSIST = 55            
    TAP_ON_MUTE = 56            
    TAP_ON_MEMORY_PARKING = 57            
    TAP_ON_MEMORY_SLOT_1 = 58            
    TAP_ON_MEMORY_SLOT_2 = 59            
    TAP_ON_MEMORY_SLOT_3 = 60            
    TAP_ON_USER_SLOT_REFINE = 61            
    TAP_ON_USER_SLOT_CLOSE = 62            
    TAP_ON_USER_SLOT_DEFINE = 63            
    TAP_ON_LVMD = 64            
    TAP_ON_LSCA = 65            
    TAP_ON_USER_SLOT_DELETE = 66            
    TAP_ON_LVMD_MUTE_AUDIO = 67            
    TAP_ON_SVS_ACTIVE = 68            
    TAP_ON_SVS_2D = 69            
    TAP_ON_SVS_3D = 70            
    TAP_ON_SVS_KERB_VIEW_FRONT = 71            
    TAP_ON_SVS_KERB_VIEW_REAR = 72            
    TAP_ON_SVS_CROSS_BUTTON = 73            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_HMIToAP.ScreenTypes uml:Enumeration
  Members:
    BLANK_SCREEN = 0            
    BOWL_VIEW_1 = 1            
    BOWL_VIEW_2 = 2            
    BOWL_VIEW_3 = 3            
    BOWL_VIEW_4 = 4            
    BOWL_VIEW_5 = 5            
    BOWL_VIEW_6 = 6            
    BOWL_VIEW_7 = 7            
    BOWL_VIEW_8 = 8            
    BOWL_VIEW_9 = 9            
    BOWL_VIEW_10 = 10            
    BOWL_VIEW_11 = 11            
    BOWL_VIEW_12 = 12            
    BOWL_VIEW_13 = 13            
    BOWL_VIEW_14 = 14            
    BOWL_VIEW_MODIFIED = 15            
    FRONT_VIEW_FULL = 16            
    FRONT_VIEW_SPLIT = 17            
    PANORAMIC_FRONT_VIEW = 18            
    BLACK_SCREEN = 19            
    FRONT_TOP_ZOOM = 20            
    LAST_FRONT_VIEW = 30            
    TOP_VIEW = 31            
    TOP_VIEW_SFM = 32            
    TOP_VIEW_PSD = 33            
    TOP_VIEW_PMD = 34            
    LAST_TOP_VIEW = 45            
    REAR_VIEW_FULL = 46            
    REAR_VIEW_SPLIT = 47            
    PANORAMIC_REAR_VIEW = 48            
    TRAILER_VIEW = 49            
    REAR_TOP_ZOOM = 50            
    LAST_REAR_VIEW = 60            
    KERB_VIEW_FRONT = 61            
    KERB_VIEW_REAR = 62            
    LAST_KERB_VIEW = 75            
    ARA_REAR_NORMAL = 76            
    ARA_REAR_IRREGULAR = 77            
    ARA_PANORAMIC_NORMAL = 78            
    ARA_PANORAMIC_IRREGULAR = 79            
    LEFT_VIEW_SPLIT = 80            
    LEFT_VIEW_FULL = 81            
    PANORAMIC_LEFT_VIEW = 82            
    RIGHT_VIEW_SPLIT = 83            
    RIGHT_VIEW_FULL = 84            
    PANORAMIC_RIGHT_VIEW = 85            
    LAST_ARA_VIEW = 90            
    IPA_FRONT_VIEW = 91            
    IPA_REAR_VIEW = 92            
    IPA_TOP_VIEW = 93            
    PARK_FRONT_VIEW = 94            
    PARK_REAR_VIEW = 95            
    PARK_TOP_VIEW = 96            
    PARK_SPACES_TOP_VIEW = 97            
    PARK_WARN_TOP_VIEW = 98            
    PARK_TRANSPARENT_PDW = 99            
    LAST_IPA_VIEW = 105            
    MORPH_FRONT = 106            
    MORPH_FRONT_FULL = 107            
    LAST_MORPH_FRONT = 120            
    MORPH_REAR = 121            
    MORPH_REAR_FULL = 122            
    MORPH_ARA = 123            
    LAST_MORPH_VIEW = 135            
    SMARTPHONE_VIEW = 151            
    BLIND_SPOT_LEFT = 152            
    BLIND_SPOT_RIGHT = 153            
    LAST_SPECIAL_VIEW = 165            
    TRANSPARENT_HOOD = 166            
    TRANSPARENT_ALPHA = 167            
    TRANSPARENT_GHOST = 168            
    TRANSPARENT_TRUNK = 169            
    TRANSPARENT_ALPHA_SPLIT = 170            
    TRANSPARENT_HOOD_SPLIT = 171            
    TRANSPARENT_TRUNK_SPLIT = 172            
    ADAPTIVE_FULL_BOWL = 173            
    STATIC_BOWL = 174            
    ADAPTIVE_SPLIT_BOWL = 175            
    STATIC_SPLIT_BOWL = 176            
    RAW_CAMERAS = 181            
    RAW_CAMERAS_SFM = 182            
    DEBUG_FRONT_CAMERA_EXTERN = 200            
    DEBUG_FRONT_CAMERA_VECTORS = 201            
    DEBUG_FRONT_CAMERA_EDGES = 202            
    DEBUG_FRONT_CAMERA_CLEANING = 203            
    DEBUG_REAR_CAMERA_EXTERN = 204            
    DEBUG_REAR_CAMERA_VECTORS = 205            
    DEBUG_REAR_CAMERA_EDGES = 206            
    DEBUG_REAR_CAMERA_CLEANING = 207            
    DEBUG_LEFT_CAMERA_EXTERN = 208            
    DEBUG_LEFT_CAMERA_VECTORS = 209            
    DEBUG_LEFT_CAMERA_EDGES = 210            
    DEBUG_LEFT_CAMERA_CLEANING = 211            
    DEBUG_RIGHT_CAMERA_EXTERN = 212            
    DEBUG_RIGHT_CAMERA_VECTORS = 213            
    DEBUG_RIGHT_CAMERA_EDGES = 214            
    DEBUG_RIGHT_CAMERA_CLEANING = 215            
    DEBUG_CALIBRATION_FLMC_QUAD_VIEW = 216            
    DEBUG_CALIBRATION_PGM_QUAD_VIEW = 217            
    DEBUG_TESTSCREEN_4_CAMS = 218            
    DEBUG_CALIBRATION_OLMC_QUAD_VIEW = 219            
    DEBUG_COLOUR_BARS = 220            
    DEBUG_RAW_CAMERAS = 221            
    DEBUG_RAW_CAMERAS_SFM = 222            
    DEBUG_CURB_MESH_DEMO = 223            
    DEBUG_HEIGHT_MAP = 224            
    DEBUG_PGM_QUAD_OVERLAY = 225            
    DEBUG_IMAGE_HARMONIZATION = 226            
    PARKING_IN_SLOT_SELECTION_VIEW = 227            
    PARKING_IN_SLOT_CONFIRMATION_VIEW = 228            
    PARKING_IN_MANEUVER_PHASE_FORWARD_VIEW = 229            
    PARKING_IN_MANEUVER_PHASE_BACKWARD_VIEW = 230            
    PARKING_IN_MANEUVER_APPROACH_PHASE_VIEW = 231            
    PARKING_IN_MANEUVER_APPROACH_WHEELSTOPPER_PHASE_FORWARD_VIEW = 232            
    PARKING_IN_MANEUVER_APPROACH_WHEELSTOPPER_PHASE_BACKWARD_VIEW = 233            
    PARKING_OUT_DIRECTION_SELECTION_VIEW = 234            
    PARKING_OUT_MANEUVER_FORWARD_VIEW = 235            
    PARKING_OUT_MANEUVER_BACKWARD_VIEW = 236            
    PARKING_OUT_MANEUVER_FINISH_PHASE_VIEW = 237            
    NO_STREAM_CHANGE = 255            
    SIZE = 256            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_HMIToAP.GestureCode uml:Enumeration
  Members:
    NO_ACTION = 0            
    PRESS = 1            
    LONGPRESS = 2            
    RELEASE = 3            
    CLICK = 4            
    RIGHTCLICK = 5            
    DRAG = 6            
    ZOOM = 7            
    ROTATE = 8            
    RAW = 9            
    DOUBLECLICK = 10            
    TRIPLECLICK = 11            
    FLICK = 12            
    INVALID = 31            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_HMIToAP.BlindSpotView uml:Enumeration
  Members:
    BLIND_SPOT_VIEW_ACTIVE_HMI = 0            
    BLIND_SPOT_VIEW_ACTIVE_IC = 1            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_HMIToAP.ParkingAugmentationType uml:Enumeration
  Members:
    PARKING_AUGMENTATION_OFF = 0            
    PARKING_IN_ON = 1            
    PARKING_OUT_ON = 2            
----------------------------------------------------------------------------------------------------------


  # ID:  type.AP_HMIToAP.RemoteHMIOutputPort uml:Class
  version: ::ap_hmitoap::RemoteHMIOutputPort_InterfaceVersion::RemoteHMIOutputPort_VERSION
    // None
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 batteryLevel_perc     // @unit{Percent};@range{0,102};Returns battery level of remote device to ensure enough usage time of the remote device for parking function
    type.uint16 fingerPositionX_px     // @unit{Pixel};@range{0,4095};x Finger position on Smartphone display 4095: Finger is not on screen (0xfff)
    type.uint16 fingerPositionY_px     // @unit{Pixel};@range{0,4095};y Finger position on Smartphone display 4095: Finger is not on screen (0xfff)
    type.AP_HMIToAP.UserActionRemoteDevice userActionRemoteDevice_nu     // @range{0,255};User interaction with HMI (Remote Parking App)  27: Switches to a screen, where the user can see the surround-view view / us-distance view. 28 / 29: Forward / Backward-Button for Remote Maneuvering
    type.uint8 aliveCounter_nu     // @range{0,255};Increases on each cycle by one (to clarify: needed here)
    type.boolean deadMansSwitchBtn_nu     // Dead Man"s Switch pressed (test purpose only)
    type.boolean paired_nu     // true if bluetooth device is paried
    type.boolean connected_nu     // true if bluetooth device is connected
    type.uint16 screenResolutionX_px     // @unit{Pixel};@range{0,16383};resolution screen curent device in X
    type.uint16 screenResolutionY_px     // @unit{Pixel};@range{0,16383};resolution screen curent device in Y
    type.uint8 userActionRemCounter_nu     
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_HMIToAP.RemoteHMIOutputPort_InterfaceVersion uml:Class
  Members:
    type.uint32 RemoteHMIOutputPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_HMIToAP.UserActionRemoteDevice uml:Enumeration
    //     User interaction with HMI (Remote Parking App)
    // 
    // 27: Switches to a screen, where the user can see the surround-view view / us-distance view.
    // 28 / 29: Forward / Backward-Button for Remote Maneuvering
  Members:
    REM_NO_USER_ACTION = 0            
    REM_TAP_ON_PARKING_SPACE_1 = 1            
    REM_TAP_ON_PARKING_SPACE_2 = 2            
    REM_TAP_ON_PARKING_SPACE_3 = 3            
    REM_TAP_ON_PARKING_SPACE_4 = 4            
    REM_APP_STARTED = 16            
    REM_APP_CLOSED = 17            
    REM_TAP_ON_START_PARKING = 18            
    REM_TAP_ON_INTERRUPT = 19            
    REM_TAP_ON_CONTINUE = 20            
    REM_TAP_ON_UNDO = 21            
    REM_TAP_ON_CANCEL = 22            
    REM_TAP_ON_REDO = 23            
    REM_TAP_ON_PARK_IN = 24            
    REM_TAP_ON_PARK_OUT = 25            
    REM_TAP_ON_REM_MAN = 26            
    REM_TAP_ON_REM_SV = 27            
    REM_TAP_ON_REM_FWD = 28            
    REM_TAP_ON_REM_BWD = 29            
    REM_TAP_ON_PREVIOUS_SCREEN = 30            
    REM_TAP_ON_GP = 31            
    REM_TAP_ON_SWITCH_TO_HEAD_UNIT = 32            
----------------------------------------------------------------------------------------------------------


  # ID:  type.AP_LADMC.LaDMCStatusPort uml:Class
  version: ::ap_ladmc::LaDMCStatusPort_InterfaceVersion::LaDMCStatusPort_VERSION
    // Signals for control logic communication with other Funtionc Components. (e.g. Trajectory Control)
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 handSteeringTorque_Nm     // @unit{Nm};@range{-15,15};Detected Steering Torque by driver
    type.boolean handSteeringTorque_QF_nu     // Qualifier-Bit of handSteeringTorque
    type.AP_LADMC.LaDMCSystemState laDMCSystemState_nu     // @range{0,7};System status/Availability of the lateral dynamic motion control and the underlaid actuator EPS. Used for communication between LaDMC and EPS as well as LaDMC and StateMachine.
    type.boolean driverIntervention_nu     // Flag about detected driver intervention. (e.g. steer wheel hand torque detected)
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_LADMC.LaDMCStatusPort_InterfaceVersion uml:Class
  Members:
    type.uint32 LaDMCStatusPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_LADMC.LaDMCSystemState uml:Enumeration
    // System status/Availability of the lateral dynamic motion control and the underlaid actuator EPS. Used for communication between LaDMC and EPS as well as LaDMC and StateMachine.
  Members:
    LADMC_NOT_AVAILABLE = 0            
    LADMC_INITIALISATION = 1            
    LADMC_AVAILABLE = 2            
    LADMC_CTRL_ACTIVE = 3            
    LADMC_CANCEL_BY_DRIVER = 4            
    LADMC_ERROR = 7            
----------------------------------------------------------------------------------------------------------





  # ID:  type.AP_LODMC.LoDMCStatusPort uml:Class
  version: ::ap_lodmc::LoDMCStatusPort_InterfaceVersion::LoDMCStatusPort_VERSION
    // Signals for control logic communication with other Funtionc Components. (e.g. Trajectory Control)
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 dynamicSlope_perc     // @unit{%};@range{-30,30};Current estimation of dynamic slope.
    type.AP_LODMC.SlopeAccuracy dynamicSlopeAccuracy_nu     // @range{0,3};Accurcay of current estimation of dynamic slope.
    type.AP_LODMC.LoDMCSystemState loDMCSystemState_nu     // @range{0,15};System status and availability of the longitudinal dynamic motion control and the underlaid actuators.
    type.AP_LODMC.maneuveringFinished maneuveringFinished_nu     // Information that the requested maneuvering is in progress or finished by the LoDMC. (e.g. requested DistanceToStopReq_nu reached and vehicle in stand still)
    type.AP_LODMC.longitudinalControlActiveStatus longitudinalControlActiveStatus_nu     
    type.boolean standstillHoldCur_nu     // Status defines whether the vehicle standstill is hold by the LoDMC. (e.g. using the hydraulic brake)
    type.boolean standstillSecureCur_nu     // Status defines whether the vehicle standstill is secured by the LoDMC. (e.g. closed electric park brake)
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_LODMC.LoDMCStatusPort_InterfaceVersion uml:Class
  Members:
    type.uint32 LoDMCStatusPort_VERSION = 2U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_LODMC.LoDMCSystemState uml:Enumeration
    // System status and availability of the longitudinal dynamic motion control and the underlaid actuators.
  Members:
    LODMC_NOT_AVAILABLE = 0            
    LODMC_INITIALISATION = 1            
    LSCA_SUPPORT_AVAILABLE = 2            
    LSCA_MSP_SUPPORT_AVAILABLE = 3            
    LSCA_MSP_APA_SUPPORT_AVAILABLE = 4            
    LSCA_MSP_APA_REMOTE_SUPPORT_AVAILABLE = 5            
    LSCA_MSP_APA_REMOTE_SUPPORT_AVAILABLE_REMOTE_READY = 6            
    LODMC_CTRL_ACTIVE = 7            
    LODMC_CANCEL_BY_DRIVER = 8            
    LODMC_ERROR = 9            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_LODMC.SlopeAccuracy uml:Enumeration
    // Accurcay of current estimation of dynamic slope.
  Members:
    HIGH = 0            
    MID = 1            
    LOW = 2            
    UNDEFINED = 3            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_LODMC.maneuveringFinished uml:Enumeration
    // Status of the maneuvering.
  Members:
    MANEUVERING_NOT_STARTED = 0            
    MANEUVERING_IN_PROGRESS = 1            
    MANEUVERING_FINISHED = 2            
    MANEUVERING_SATURATED = 3            
    MANEUVERING_FAULT = 4            
    RESERVED = 5            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_LODMC.longitudinalControlActiveStatus uml:Enumeration
    // LoDMC Acknowledgement signal whether requested control type is granted and active
  Members:
    LODMC_NO_ACTIVE_HANDSHAKE = 0            
    LODMC_AUP_HANDSHAKE_ACTIVE = 1            
    LODMC_LSCA_HANDSHAKE_ACTIVE = 2            
    LODMC_REMOTE_PARKING_HANDSHAKE_ACTIVE = 3            
    LODMC_MSP_HANDSHAKE_ACTIVE = 4            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.ABSState uml:Enumeration
    // ABS active
  Members:
    ABS_INACTIVE = 0            
    ABS_ACTIVE = 1            
    ABS_ERROR = 2            
----------------------------------------------------------------------------------------------------------


  # ID:  type.AP_VehStateSigProvider.ACCInformationPort uml:Class
  version: ::ap_vehstatesigprovider::ACCInformationPort_InterfaceVersion::ACCInformationPort_VERSION
    // None
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.AP_VehStateSigProvider.ACCStatus accStatus_nu     // @range{0,7};Status ACC
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.ACCInformationPort_InterfaceVersion uml:Class
  Members:
    type.uint32 ACCInformationPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.ACCStatus uml:Enumeration
    // Status ACC
  Members:
    ACC_OFF = 0            
    ACC_INIT = 1            
    ACC_STANDBY = 2            
    ACC_ACTIVE = 3            
    ACC_OVERRIDE = 4            
    ACC_TURN_OFF = 5            
    ACC_ERROR_REVERSIBLE = 6            
    ACC_ERROR_IRREVERSIBLE = 7            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.APCtrlStatusPort uml:Class
  version: ::ap_vehstatesigprovider::APCtrlStatusPort_InterfaceVersion::APCtrlStatusPort_VERSION
    // Signals to control AP status externally
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.boolean apActivationGranted_nu     // Activation of Automatic Parking Maneuvering granted by Function Coordinator (signals such as roller bench, diagnostic mode, alarm system, steering wheel locked are checked on higher level)
    type.boolean scanningActivationReq_nu     // Request to activate scanning of environment for parking spaces
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.APCtrlStatusPort_InterfaceVersion uml:Class
  Members:
    type.uint32 APCtrlStatusPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------








  # ID:  type.AP_VehStateSigProvider.AdditionalBCMStatusPort uml:Class
  version: ::ap_vehstatesigprovider::AdditionalBCMStatusPort_InterfaceVersion::AdditionalBCMStatusPort_VERSION
    // Signals from additional Body Controller
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.AP_VehStateSigProvider.Light light     // @unit{nu};Status of relevant lights
    type.AP_VehStateSigProvider.Ignition ignition     // @unit{nu};Ignition signals
    type.boolean frontLidOpen_nu     // Front lid (aka. Front hood, Frunk, Front Trunk) open.
    type.boolean tankCapOpen_nu     // Tank cap open
    type.AP_VehStateSigProvider.OuterRearViewMirror outerRearViewMirrorState     // Status of relevant OuterRearViewMirror
    type.AP_VehStateSigProvider.SunroofStatus sunroofStatus_nu     // Status of Sunroof
    type.AP_VehStateSigProvider.ChargingStatus chargingStatus_nu     // Status of the EV charging systems.
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.AdditionalBCMStatusPort_InterfaceVersion uml:Class
  Members:
    type.uint32 AdditionalBCMStatusPort_VERSION = 3U    
----------------------------------------------------------------------------------------------------------


  # ID:  type.AP_VehStateSigProvider.AllDoorStatus uml:Class
    // Status of all doors
  Members:
    type.AP_VehStateSigProvider.DoorStatus frontPsgr_nu     // @range{0,3};Door of the front passenger is open
    type.AP_VehStateSigProvider.DoorStatus driver_nu     // @range{0,3};Door of the driver is open
    type.AP_VehStateSigProvider.DoorStatus rearRight_nu     // @range{0,3};Rear door on the right (behind front passenger) open
    type.AP_VehStateSigProvider.DoorStatus rearLeft_nu     // @range{0,3};Rear door on the left (behind driver) open
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.AuthenticationStatusPort uml:Class
  version: ::ap_vehstatesigprovider::AuthenticationStatusPort_InterfaceVersion::AuthenticationStatusPort_VERSION
    // Authentication signals
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.boolean authKeyDetected_nu     // Authorized key detected
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.AuthenticationStatusPort_InterfaceVersion uml:Class
  Members:
    type.uint32 AuthenticationStatusPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.BeltBuckle uml:Enumeration
    // Passenger"s front seat belt buckle status
  Members:
    BELT_STATUS_NOT_INSTALLED = 0            
    BELT_STATUS_NOT_AVAILABLE = 1            
    BELT_STATUS_OPEN = 2            
    BELT_STATUS_LOCKED = 3            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.BeltBuckleStatus uml:Class
    // Status of the belt buckle for each seat
  Members:
    type.AP_VehStateSigProvider.BeltBuckle frontPsgr_nu     // @range{0,3};Passenger"s front seat belt buckle status
    type.AP_VehStateSigProvider.BeltBuckle driver_nu     // @range{0,3};Driver"s seat belt buckle status
    type.AP_VehStateSigProvider.BeltBuckle backrowRight_nu     // @range{0,3};Seat belt buckle status in backrow on the right (row 2 or row 3)
    type.AP_VehStateSigProvider.BeltBuckle backrowLeft_nu     // @range{0,3};Seat belt buckle status in bacjrow on the left (row 2 or row 3)
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.ChassisAxleHeightPort uml:Class
  version: ::ap_vehstatesigprovider::ChassisAxleHeightPort_InterfaceVersion::ChassisAxleHeightPort_VERSION
    // axle height information
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 chassisAxleHeightFront_mm     // @unit{nu};Height in mm between chassis and front axle
    type.float32 chassisAxleHeightRear_mm     // @unit{nu};Height in mm between chassis and rear axle
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.ChassisAxleHeightPort_InterfaceVersion uml:Class
  Members:
    type.uint32 ChassisAxleHeightPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.ConvertibleTopState uml:Enumeration
    // Status of convertible top
  Members:
    LOCKED_OPEN = 0            
    LOCKED_CLOSED = 1            
    UNLOCKED = 2            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.ConvertibleTopStatusPort uml:Class
  version: ::ap_vehstatesigprovider::ConvertibleTopStatusPort_InterfaceVersion::ConvertibleTopStatusPort_VERSION
    // Signals from Convertible Top ECU
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.AP_VehStateSigProvider.ConvertibleTopState cTopState_nu     // @range{0,3};Status of convertible top
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.ConvertibleTopStatusPort_InterfaceVersion uml:Class
  Members:
    type.uint32 ConvertibleTopStatusPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.DoorStatus uml:Enumeration
    // Door of the front passenger is open
  Members:
    DOOR_STATUS_INIT = 0            
    DOOR_STATUS_OPEN = 1            
    DOOR_STATUS_CLOSED = 2            
    DOOR_STATUS_LOCKED = 3            
    DOOR_STATUS_ERROR = 4            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.DoorStatusPort uml:Class
  version: ::ap_vehstatesigprovider::DoorStatusPort_InterfaceVersion::DoorStatusPort_VERSION
    // Signals from door ECUs
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.AP_VehStateSigProvider.AllDoorStatus status     // @unit{nu};Status of all doors
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.DoorStatusPort_InterfaceVersion uml:Class
  Members:
    type.uint32 DoorStatusPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.EBAStatus uml:Enumeration
    // Status of the emergency brake assist
  Members:
    EBA_OFF = 0            
    EBA_PASSIVE = 1            
    EBA_ACTIVE_PREWARNING = 2            
    EBA_ACTIVE_ACUTE_WARNING = 3            
    EBA_ACTIVE_BRAKING = 4            
    EBA_ERROR = 5            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.EBDState uml:Enumeration
    // Electronic Brakeforce Distribution active
  Members:
    EBD_INACTIVE = 0            
    EBD_ACTIVE = 1            
    EBD_ERROR = 2            
----------------------------------------------------------------------------------------------------------



  # ID:  type.AP_VehStateSigProvider.ESCInformationPort uml:Class
  version: ::ap_vehstatesigprovider::ESCInformationPort_InterfaceVersion::ESCInformationPort_VERSION
    // Signals from ESC
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 brakePressureGradient_barps     // @unit{bar per second};@range{0,2500};Brake pressure gradient of driver"s brake pressure
    type.float32 brakePressureDriver_bar     // @unit{bar};@range{-30,276.6};Brake pressure by driver
    type.AP_VehStateSigProvider.TCSState tcsState_nu     // @range{0,1};Traction Control System active
    type.AP_VehStateSigProvider.ESCState escState_nu     // @range{0,1};ESC active
    type.AP_VehStateSigProvider.ABSState absState_nu     // @range{0,1};ABS active
    type.AP_VehStateSigProvider.EBDState ebdState_nu     // @range{0,1};Electronic Brakeforce Distribution active
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.ESCInformationPort_InterfaceVersion uml:Class
  Members:
    type.uint32 ESCInformationPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.ESCState uml:Enumeration
    // ESC active
  Members:
    ESC_INACTIVE = 0            
    ESC_ACTIVE = 1            
    ESC_ERROR = 2            
----------------------------------------------------------------------------------------------------------


  # ID:  type.AP_VehStateSigProvider.ExternalFunctionStatusPort uml:Class
  version: ::ap_vehstatesigprovider::ExternalFunctionStatusPort_InterfaceVersion::ExternalFunctionStatusPort_VERSION
    // Status of other functions that have special influence on AP
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.AP_VehStateSigProvider.RCTAStatus rctaStatus_nu     // @range{0,3};Rear cross traffic alert status
    type.AP_VehStateSigProvider.EBAStatus ebaStatus_nu     // Status of the emergency brake assist
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.ExternalFunctionStatusPort_InterfaceVersion uml:Class
  Members:
    type.uint32 ExternalFunctionStatusPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.Ignition uml:Class
    // Ignition signals
  Members:
    type.boolean ignitionOn_nu     // Ignition on
    type.AP_VehStateSigProvider.IgnitionSourceStatus ignitionSource_nu     // IgnitionSource on
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.KeyFobInRange uml:Enumeration
    // information regarding the location of the key
  Members:
    AP_KEY_OUT_OF_RANGE = 0            
    AP_KEY_IN_VEHICLE = 1            
    AP_KEY_NEAR_RANGE = 2            
    AP_KEY_FAR_RANGE = 3            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.KeyFobUserAction uml:Enumeration
    // User action from the key fob
  Members:
    AP_KEY_NO_USER_ACTION = 0            
    AP_KEY_TAP_ON_START_PARKING = 1            
    AP_KEY_TAP_ON_INTERRUPT = 2            
    AP_KEY_TAP_ON_CONTINUE = 3            
    AP_KEY_TAP_ON_UNDO = 4            
    AP_KEY_TAP_ON_CANCEL = 5            
    AP_KEY_TAP_ON_REDO = 6            
    AP_KEY_FOB_MANEUVER_AUTHORIZED = 7            
    AP_KEY_FOB_MANEUVER_ABORT = 8            
    AP_KEY_FOB_MANEUVER_PAUSED = 9            
    AP_KEY_FOB_ACTIVATED = 10            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.KeylessStatusPort uml:Class
  version: ::ap_vehstatesigprovider::KeylessStatusPort_InterfaceVersion::KeylessStatusPort_VERSION
    // Signals from Keyless Go (VW: Kessy) ECU
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.AP_VehStateSigProvider.KeyFobUserAction keyFobUserAction     // User action from the key fob
    type.uint8 keyFobButtonAliveCounter     // @unit{nu};@range{0,15};Will be increased by one each time the key communicates with HFM ( when the key is in the range)
    type.AP_VehStateSigProvider.KeyFobInRange keyFobInRange     // @range{0,1};information regarding the location of the key
    type.uint8 keylessStatusPortCANAlive_nu     // @range{0,15};Will be increased with each can message  (has to be dropped because it will be checked and abstracted on higher level)
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.KeylessStatusPort_InterfaceVersion uml:Class
  Members:
    type.uint32 KeylessStatusPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.Light uml:Class
    // Status of relevant lights
  Members:
    type.boolean lowBeamOn_nu     // Low beam light on
    type.boolean highBeamOn_nu     // High beam light on
    type.boolean indicatorLeftOn_nu     // Left light indicator on
    type.boolean indicatorRightOn_nu     // Right light indicator on
    type.boolean brakeLightOn_nu     // Brake light on
    type.boolean frontFogLightOn_nu     // Front fog light on
    type.boolean rearFogLightOn_nu     // rear fog light on
    type.boolean daytimeRunningLightStateOn_nu     // daytimeRunning light on
    type.boolean positionLampOn_nu     // parking light on
    type.boolean reverseLampOn_nu     // reverse light on
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.OuterRearViewMirror uml:Class
    // Status of relevant OuterRearViewMirror
  Members:
    type.boolean leftOuterRearViewMirrorState_nu     // Left OuterRearViewMirror unfold
    type.boolean rightOuterRearViewMirrorState_nu     // Right OuterRearViewMirror unfold
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.RCTAStatus uml:Enumeration
    // Rear cross traffic alert status
  Members:
    RCTA_OFF = 0            
    RCTA_PASSIVE = 1            
    RCTA_ACTIVE_PREWARNING = 2            
    RCTA_ACTIVE_ACUTE_WARNING = 3            
    RCTA_ACTIVE_BRAKING = 4            
    RCTA_ERROR = 5            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.SeatOccupancy uml:Enumeration
    // Passenger on front-seat detected
  Members:
    OCC_STATUS_NOT_INSTALLED = 0            
    OCC_STATUS_NOT_AVAILABLE = 1            
    OCC_STATUS_FREE = 2            
    OCC_STATUS_OCCUPIED = 3            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.SeatOccupancyStatus uml:Class
    // Status of the seat occupancy for each seat
  Members:
    type.AP_VehStateSigProvider.SeatOccupancy frontPsgr_nu     // @range{0,3};Passenger on front-seat detected
    type.AP_VehStateSigProvider.SeatOccupancy driver_nu     // @range{0,3};Driver on seat detected
    type.AP_VehStateSigProvider.SeatOccupancy backrowRight_nu     // @range{0,3};Passenger on right seat of backrow detected (row 2 or row 3)
    type.AP_VehStateSigProvider.SeatOccupancy backrowLeft_nu     // @range{0,3};Passenger on left seat of backrow detected (row 2 or row 3)
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.StarterStatusPort uml:Class
  version: ::ap_vehstatesigprovider::StarterStatusPort_InterfaceVersion::StarterStatusPort_VERSION
    // None
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.boolean starterLocked_nu     // indicates that starter is locked
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.StarterStatusPort_InterfaceVersion uml:Class
  Members:
    type.uint32 StarterStatusPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.SteeringColSwitchesStatusPort uml:Class
  version: ::ap_vehstatesigprovider::SteeringColSwitchesStatusPort_InterfaceVersion::SteeringColSwitchesStatusPort_VERSION
    // Signals from switches next to the steering wheel
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.boolean ctrlLeft_nu     // Direction indicator control on left side is set by driver. Controls the left turn indicator (blinker, turn lever).
    type.boolean ctrlRight_nu     // Direction indicator control on right side is set by driver. Controls the left turn indicator (blinker, turn lever).
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.SteeringColSwitchesStatusPort_InterfaceVersion uml:Class
  Members:
    type.uint32 SteeringColSwitchesStatusPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.TCSState uml:Enumeration
    // Traction Control System active
  Members:
    TCS_INACTIVE = 0            
    TCS_ACTIVE = 1            
    TCS_ERROR = 2            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.TrailerHitchStatus uml:Enumeration
    // Status of trailer hitch
  Members:
    NOT_FOLDED = 0            
    FOLDED = 1            
    NOT_INSTALLED = 2            
    TRAILER_ATTACHED = 3            
    HITCH_NOT_ATTACHED = 4            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.TrailerStatusPort uml:Class
  version: ::ap_vehstatesigprovider::TrailerStatusPort_InterfaceVersion::TrailerStatusPort_VERSION
    // Signals from Trailer ECU
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.boolean trailerAttached_nu     // Attached trailer recognized
    type.AP_VehStateSigProvider.TrailerHitchStatus trailerHitchStatus_nu     // @range{0,5};Status of trailer hitch
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.TrailerStatusPort_InterfaceVersion uml:Class
  Members:
    type.uint32 TrailerStatusPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.TrunkLidStatusPort uml:Class
  version: ::ap_vehstatesigprovider::TrunkLidStatusPort_InterfaceVersion::TrunkLidStatusPort_VERSION
    // Signals from Trunk Lid ECU
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.boolean open_nu     // Trunk lid open
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.TrunkLidStatusPort_InterfaceVersion uml:Class
  Members:
    type.uint32 TrunkLidStatusPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.VehicleOccupancyStatusPort uml:Class
  version: ::ap_vehstatesigprovider::VehicleOccupancyStatusPort_InterfaceVersion::VehicleOccupancyStatusPort_VERSION
    // Signals from Airbag ECU
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.AP_VehStateSigProvider.SeatOccupancyStatus seatOccupancyStatus     // @unit{nu};Status of the seat occupancy for each seat
    type.AP_VehStateSigProvider.BeltBuckleStatus beltBuckleStatus     // @unit{nu};Status of the belt buckle for each seat
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.VehicleOccupancyStatusPort_InterfaceVersion uml:Class
  Members:
    type.uint32 VehicleOccupancyStatusPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.SunroofStatus uml:Enumeration
    // Status of vehicle sunroof
  Members:
    SUNROOF_STATUS_CLOSED = 0            
    SUNROOF_STATUS_OPEN = 1            
    SUNROOF_STATUS_OPENING = 2            
    SUNROOF_STATUS_CLOSING = 3            
    SUNROOF_STATUS_INVALID = 4            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.IgnitionSourceStatus uml:Enumeration
    // Status of the ignitionsource
  Members:
    DRIVER = 0            
    REMOTE = 1            
    RESERVED = 2            
    INVALID = 3            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.ChargingStatus uml:Class
    // Status of the EV charging system.
  Members:
    type.boolean ev_charging_is_installed_nu     
    type.AP_VehStateSigProvider.ChargingConnectorStatus chargingConnectorStatus_nu     // Operating condition of the EV battery charging port connector.
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_VehStateSigProvider.ChargingConnectorStatus uml:Enumeration
    // Operating condition of the EV battery charging port connector.
  Members:
    CC_STATUS_CABLE_NOT_CONNECTED = 0            
    CC_STATUS_CABLE_CONNECTED = 1            
    CC_STATUS_CHARGING = 2            
----------------------------------------------------------------------------------------------------------

  # ID:  type.PDCP.PDCP_Consts uml:Class
  Members:
    type.uint8 MAX_NUM_SECTORS_PER_SIDE = 4    
    type.uint8 NUM_MTS_DEBUG_FREESPACE_PDCP = 10    
----------------------------------------------------------------------------------------------------------

  # ID:  type.PDCP.CriticalityLevel uml:Enumeration
    // Criticality level for this sector (the higher the criticality, the closer the obstacle is to the car).
  Members:
    PDC_CRIT_LVL_OUTSIDE = 0            
    PDC_CRIT_LVL_GREEN = 1            
    PDC_CRIT_LVL_YELLOW = 2            
    PDC_CRIT_LVL_RED = 3            
----------------------------------------------------------------------------------------------------------

  # ID:  type.PDCP.DrvTubeDisplay uml:Enumeration
    // Information of where the driving tube should be displayed
  Members:
    PDC_DRV_TUBE_NONE = 0            
    PDC_DRV_TUBE_FRONT = 1            
    PDC_DRV_TUBE_REAR = 2            
----------------------------------------------------------------------------------------------------------

  # ID:  type.PDCP.PDCPDebugPort uml:Class
  version: ::pdcp::PDCPDebugPort_InterfaceVersion::PDCPDebugPort_VERSION
    // PDCP internal signals for debug
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.sint32 debugInt     // @unit{nu};freespace for MTS debug values
    type.float32 debugFloat     // @unit{nu};freespace for MTS debug values
----------------------------------------------------------------------------------------------------------

  # ID:  type.PDCP.PDCPDebugPort_InterfaceVersion uml:Class
  Members:
    type.uint32 PDCPDebugPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.PDCP.PDCPDrivingTubePort uml:Class
  version: ::pdcp::PDCPDrivingTubePort_InterfaceVersion::PDCPDrivingTubePort_VERSION
    // Signals from PDC Processing containing the driving tube information
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 frontRadius_m     // @unit{m};Radius of the driving tube on the front
    type.float32 rearRadius_m     // @unit{m};Radius of the driving tube on the rear
    type.cml.Vec2Df_POD turningCircleCenter_nu     // Center of the turning circle
    type.PDCP.DrvTubeDisplay drvTubeDisplay_nu     // @range{0,2};Information of where the driving tube should be displayed
    type.boolean straightDrvTube_nu     // Indicates if the driving tube is perfectly straight (radiuses should be ignored)
----------------------------------------------------------------------------------------------------------

  # ID:  type.PDCP.PDCPDrivingTubePort_InterfaceVersion uml:Class
  Members:
    type.uint32 PDCPDrivingTubePort_VERSION = 1    
----------------------------------------------------------------------------------------------------------


  # ID:  type.PDCP.PDCPSectorsPort uml:Class
  version: ::pdcp::PDCPSectorsPort_InterfaceVersion::PDCPSectorsPort_VERSION
    // Signals from PDC Processing containg the information about sectors
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.PDCP.SectorInfo sectorsFront     // @unit{nu};Information for the PDC sectors from the front side (numbering is done from left side towards right side)
    type.PDCP.SectorInfo sectorsRear     // @unit{nu};Information for the PDC sectors from the rear side (numbering is done from left side towards right side)
    type.PDCP.SectorInfo sectorsLeft     // @unit{nu};Information for the PDC sectors from the left side (numbering is done from front towards rear)
    type.PDCP.SectorInfo sectorsRight     // @unit{nu};Information for the PDC sectors from the right side (numbering is done from front towards rear)
    type.float32 PDC_P_SECTOR_INNER_COORDS_X_M     // The x coordinates of the inner sectors contour
    type.float32 PDC_P_SECTOR_INNER_COORDS_Y_M     // The y coordinates of the inner sectors contour
    type.float32 PDC_P_SECTOR_OUTER_COORDS_X_M     // The x coordinates of the outer sectors contour
    type.float32 PDC_P_SECTOR_OUTER_COORDS_Y_M     // The y coordinates of the outer sectors contour
----------------------------------------------------------------------------------------------------------

  # ID:  type.PDCP.PDCPSectorsPort_InterfaceVersion uml:Class
  Members:
    type.uint32 PDCPSectorsPort_VERSION = 3    
----------------------------------------------------------------------------------------------------------

  # ID:  type.PDCP.ProcToLogicPort uml:Class
  version: ::pdcp::ProcToLogicPort_InterfaceVersion::ProcToLogicPort_VERSION
    // Signals from PDC Processing to PDC Logic
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 minimalDistance_m     // @unit{m};distance to the closest object around the car
    type.boolean processingError_nu     // PDCP error information
----------------------------------------------------------------------------------------------------------

  # ID:  type.PDCP.ProcToLogicPort_InterfaceVersion uml:Class
  Members:
    type.uint32 ProcToLogicPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.PDCP.SectorInfo uml:Class
    // Information for the PDC sectors from the front side (numbering is done from left side towards right side)
  Members:
    type.float32 smallestDistance_m     // @unit{m};@range{0,2.55};Distance between the car contour and the closest static obstacle for this sector
    type.float32 dynamicSmallestDistance_m     // @unit{m};@range{0,2.55};Distance between the car contour and the closest dynamic obstacle for this sector
    type.uint8 sectorID_nu     // @range{0,24};Sector ID (unique over all sectors)
    type.PDCP.CriticalityLevel criticalityLevel_nu     // @range{0,3};Criticality level for this sector (the higher the criticality, the closer the obstacle is to the car).
    type.boolean scanned_nu     // Indicates if this sector was scanned (scanned means that PDCP had valid information to calculate the smallestDistance_nu).
    type.boolean intersectsDrvTube_nu     // Indicates if the closest point that determines the criticality of this sector intersects the driving tube
----------------------------------------------------------------------------------------------------------

  # ID:  type.PDCP.FC_PDCP_Params uml:Class
  version: ::pdcp::FC_PDCP_Params_InterfaceVersion::FC_PDCP_Params_VERSION
    // PDCP Parameters
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 PDC_P_SECTOR_INNER_COORDS_X_M     // The x coordinates of the inner sectors contour
    type.float32 PDC_P_SECTOR_INNER_COORDS_Y_M     // The y coordinates of the inner sectors contour
    type.float32 PDC_P_SECTOR_OUTER_COORDS_X_M     // The x coordinates of the outer sectors contour
    type.float32 PDC_P_SECTOR_OUTER_COORDS_Y_M     // The y coordinates of the outer sectors contour
    type.float32 PDC_P_SECTOR_LENGTH_M     // Sector length
    type.float32 PDC_P_DIST_HYST_PERCENTAGE_NU     // Percentage from slice length, based on which the hysteresis length is computed
    type.uint8 PDC_P_NUM_FRONT_SECTORS_NU     // Number of sectors for the front side of the car
    type.uint8 PDC_P_NUM_RIGHT_SECTORS_NU     // Number of sectors for the right side of the car
    type.uint8 PDC_P_NUM_REAR_SECTORS_NU     // Number of sectors for the rear side of the car
    type.uint8 PDC_P_NUM_LEFT_SECTORS_NU     // Number of sectors for the left side of the car
    type.uint8 PDC_P_NUM_CRITICALITY_LEVELS_NU     // Number of criticality levels
    type.uint8 PDC_P_NUM_SLICES_PER_LEVEL_NU     // Number of slices in each level
    type.uint8 PDC_P_LOW_OBSTACLE_MIN_CONFIDENCE_NU     // Minimum confidence required for low obstacle validation by the PDCP component
    type.uint8 PDC_P_MIN_EXISTANCE_PROB_STATIC_OBJ_NU     // The minimum existance probability for static obstacles
    type.uint8 PDC_P_MIN_EXISTANCE_PROB_DYNAMIC_OBJ_NU     // The minimum existance probability for dynamic obstacles
    type.boolean PDC_P_WARN_ON_LOW_OBSTACLE_NU     // Flag for switching on/off warning for low obstacles
    type.boolean PDC_P_ACTIVATE_INTERNAL_ROI_NU     // Flag for switching on/off filtering of obstacles based on their positions relative to an internal PDCP Region of Interest
----------------------------------------------------------------------------------------------------------

  # ID:  type.PDCP.FC_PDCP_Params_InterfaceVersion uml:Class
  Members:
    type.uint32 FC_PDCP_Params_VERSION = 2    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_DRVWARNSM.MF_DRVWARNSM_Consts uml:Class
  Members:
    type.uint8 NUM_WHEELS = 4    
    type.uint8 NUM_SECTOR_SIDES = 4    
    type.uint8 NUM_MTS_DEBUG_FREESPACE_APP_DWF = 10    
----------------------------------------------------------------------------------------------------------


  # ID:  type.MF_DRVWARNSM.AppToCoreSMPort uml:Class
  version: ::mf_drvwarnsm::AppToCoreSMPort_InterfaceVersion::AppToCoreSMPort_VERSION
    // Commands from customer dependent state machine(application sm) to drvwarnsm core
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.MF_DRVWARNSM.RequestMode pdwRequestMode_nu     // @range{0,5};The PDW request mode to the core state machines
    type.MF_DRVWARNSM.RequestMode whpRequestMode_nu     // @range{0,5};The WHP request mode to the core state machines
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_DRVWARNSM.AppToCoreSMPort_InterfaceVersion uml:Class
  Members:
    type.uint32 AppToCoreSMPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_DRVWARNSM.DrvTubeDisplayReq uml:Enumeration
    // DrvWarn logic determines the side where the driving tube should be displayed and informs PDW processing about it.
  Members:
    PDW_DRV_TUBE_REQ_NONE = 0            
    PDW_DRV_TUBE_REQ_FRONT = 1            
    PDW_DRV_TUBE_REQ_REAR = 2            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_DRVWARNSM.DrvWarnDebugPort uml:Class
  version: ::mf_drvwarnsm::DrvWarnDebugPort_InterfaceVersion::DrvWarnDebugPort_VERSION
    // DrvWarn state machines internal signals for debug
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.sint32 debugInt     // @unit{nu};freespace for MTS debug values
    type.float32 debugFloat     // @unit{nu};freespace for MTS debug values
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_DRVWARNSM.DrvWarnDebugPort_InterfaceVersion uml:Class
  Members:
    type.uint32 DrvWarnDebugPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------






  # ID:  type.MF_DRVWARNSM.DrvWarnStatusPort uml:Class
  version: ::mf_drvwarnsm::DrvWarnStatusPort_InterfaceVersion::DrvWarnStatusPort_VERSION
    // Main output from Driver Warning State machines (status information)
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.MF_DRVWARNSM.WHPState whpState_nu     // @range{0,4};Wheel Protection function status(internal state).
    type.boolean whpDisplayReq_nu     // Wheel Protection function display request.
    type.MF_DRVWARNSM.PDWSystemState pdwSystemState_nu     // @range{0,5};PDW system state information.
    type.MF_DRVWARNSM.PDWShutdownCause pdwShutdownCause_nu     // @range{0,3};The reason why  PDW system state is off
    type.MF_DRVWARNSM.PDWState pdwState_nu     // @range{0,13};Internal state of the PDW state machine
    type.MF_DRVWARNSM.ReduceToMuteReq reduceToMuteSoundReq_nu     // @range{0,5};reduced to mute sound volume in standstill _PDW_REDUCE_NONE=0,_PDW_REDUCE_LVL1=1,_PDW_REDUCE_LVL2=2,_PDW_REDUCE_LVL3=3,_PDW_REDUCE_LVL4=4,_PDW_NUM_LEVELS=5
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_DRVWARNSM.DrvWarnStatusPort_InterfaceVersion uml:Class
  Members:
    type.uint32 DrvWarnStatusPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------


  # ID:  type.MF_DRVWARNSM.LogicToProcPort uml:Class
  version: ::mf_drvwarnsm::LogicToProcPort_InterfaceVersion::LogicToProcPort_VERSION
    // Signals from DrvWarn Logic to DrvWarn Functions Processing
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.MF_DRVWARNSM.WhlWarningType whlWarningType_nu     // @range{0,2};Wheel warning suppression level for each wheel. Positions front left: 0, front right: 1, rear left: 2; rear right: 3, to be defined in an enum.
    type.boolean sideResetReq_nu     // Reset flags for each side of the car. Positions front: 0, rear: 1, left: 2; right: 3, to be defined in an enum.
    type.MF_DRVWARNSM.DrvTubeDisplayReq drvTubeDisplayReq_nu     // @range{0,2};DrvWarn logic determines the side where the driving tube should be displayed and informs PDW processing about it.
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_DRVWARNSM.LogicToProcPort_InterfaceVersion uml:Class
  Members:
    type.uint32 LogicToProcPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_DRVWARNSM.PDWShutdownCause uml:Enumeration
    // The reason why  PDW system state is off
  Members:
    PDW_NONE_SYS_ACTIVE = 0            
    PDW_SHUTDOWN_BY_BUTTON = 1            
    PDW_SHUTDOWN_BY_DISENGAGED_R_GEAR = 2            
    PDW_SHUTDOWN_BY_SPEED = 3            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_DRVWARNSM.PDWState uml:Enumeration
    // Internal state of the PDW state machine
  Members:
    PDW_INT_STATE_INIT = 0            
    PDW_INT_STATE_ACT_BTN = 1            
    PDW_INT_STATE_ACT_R_GEAR = 2            
    PDW_INT_STATE_ACT_AP = 3            
    PDW_INT_STATE_ACT_AUTO = 4            
    PDW_INT_STATE_ACT_ROLLBACK = 5            
    PDW_INT_STATE_ACT_RA = 6            
    PDW_INT_STATE_DEACT_INIT = 7            
    PDW_INT_STATE_DEACT_BTN = 8            
    PDW_INT_STATE_DEACT_SPEED = 9            
    PDW_INT_STATE_DEACT_P_GEAR = 10            
    PDW_INT_STATE_DEACT_EPB = 11            
    PDW_INT_STATE_DEACT_AP_FIN = 12            
    PDW_INT_STATE_FAILURE = 13            
    PDW_INT_NUM_STATES = 14            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_DRVWARNSM.PDWSystemState uml:Enumeration
    // PDW system state information.
  Members:
    PDW_INIT = 0            
    PDW_OFF = 1            
    PDW_ACTIVATED_BY_R_GEAR = 2            
    PDW_ACTIVATED_BY_BUTTON = 3            
    PDW_AUTOMATICALLY_ACTIVATED = 4            
    PDW_FAILURE = 5            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_DRVWARNSM.RequestMode uml:Enumeration
    // The PDW request mode to the core state machines
  Members:
    REQ_NO_REQUEST = 0            
    REQ_INIT = 1            
    REQ_INACTIVE = 2            
    REQ_ACTIVE = 3            
    REQ_FAILURE = 4            
    REQ_NUM = 5            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_DRVWARNSM.WHPState uml:Enumeration
    // Wheel Protection function status(internal state).
  Members:
    WHP_INIT = 0            
    WHP_INACTIVE = 1            
    WHP_ACTIVE = 2            
    WHP_FAILURE = 3            
    WHP_NUM_STATES = 4            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_DRVWARNSM.WhlWarningType uml:Enumeration
    // Wheel warning suppression level for each wheel. Positions front left: 0, front right: 1, rear left: 2; rear right: 3, to be defined in an enum.
  Members:
    WHP_SUPPRESS_WARNING = 0            
    WHP_REDUCE_WARNING = 1            
    WHP_FULL_WARNING = 2            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_DRVWARNSM.ReduceToMuteReq uml:Enumeration
    // 
  Members:
    PDW_REDUCE_NONE = 0            
    PDW_REDUCE_LVL1 = 1            
    PDW_REDUCE_LVL2 = 2            
    PDW_REDUCE_LVL3 = 3            
    PDW_REDUCE_LVL4 = 4            
    PDW_NUM_LEVELS = 5            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_DRVWARNSM.FC_DrvWarnSM_Params uml:Class
  version: ::mf_drvwarnsm::FC_DrvWarnSM_Params_InterfaceVersion::FC_DrvWarnSM_Params_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 DWF_L_SPEED_HYSTERESIS_MPS     // The hysteresis to be applied to the speed threshold when switching between active/inactive states
    type.float32 PDW_L_INIT_TIME_S     // Maximum duration for initialization phase
    type.float32 PDW_L_STABILIZATON_WAIT_TIME_S     // Maximum waiting time for the system to be stable during initialization.
    type.float32 PDW_L_STANDSTILL_TIME_S     // Waiting time before the vehicle is considered in standstill (time with no ticks detected at the rear wheels)
    type.float32 PDW_L_REV_GEAR_DEB_TIME_MAN_S     // Waiting time, before PDW function consider R gear engaged in case of manual transmission
    type.float32 PDW_L_REV_GEAR_DEB_TIME_AUT_S     // Waiting time, before PDW function consider R gear engaged in case of automatic transmission
    type.float32 PDW_L_VEL_THRESH_OFF_MPS     // Maximum allowed velocity in order to use PDW
    type.float32 WHP_L_MAX_SPEED_MPS     // Maximum speed for which the wheel protection function is enabled
    type.float32 WHP_L_MIN_WHL_ANGLE_DIFF_RAD     // Parameter for the minimum change in wheel angle to reactivate the wheel warning level
    type.float32 PDW_L_ROLLBACK_W_OBST_DIST_M     // Define a distance threshold while rolling backwards for PDW activation caused by backwards rolling with obstacle
    type.float32 PDW_L_ROLLBACK_WO_OBST_DIST_M     // Define a distance threshold while rolling backwards for PDW activation caused by backwards rolling without obstacle
    type.float32 PDW_L_MAX_DIST_CONT_TONE_M     // Define maximum distance threshold  for continous tone range.
    type.float32 PDW_L_MIN_DIST_REAR_SENSORS_M     // Define minimum distance threshold for rear sensors.
    type.uint8 DWF_C_OPS_VARIANT_NU     // Coding parameter that specifies the variant: OPS or 360 OPS.
    type.boolean DWF_C_TRANSMISSION_TYPE_NU     // Coding parameter that specifices which transmission is available: 1 = automatic; 0 = manual
    type.boolean PDW_L_DRIVING_TUBE_NU     // Parameter to enable the driving tube
    type.boolean PDW_L_AUTOMATIC_ACTIVATION_NU     // Parameter to activate/deactivate the automatic activation: 1= active
    type.boolean PDW_L_AUTO_ACT_BUTTON_NU     // Parameter that determines the action taken when the PDW button is pressed during automatic activation: 0 = PDW becomes activated by button; 1 = PDW becomes deactivated by button
    type.boolean PDW_L_AUTOM_ACTIV_STANDSTILL_NU     // Parameter to allow the PDW function to be automatically activated in standstill
    type.boolean PDW_L_ROLLBACK_ACTIVATION_NU     // Parameter to activate/deactivate the rolling backwards: 1 = active
    type.boolean PDW_L_DEACTIV_BY_P_GEAR_NU     // Parameter to allow or not, to automatically deactivate the PDW function by P gear:  1 = deactivate  by P gear
    type.boolean PDW_L_DEACTIV_BY_EPB_NU     // Parameter to allow or not to automatically deactivate the PDW function by EPB intervention: 1 = deactivate by EPB
    type.boolean PDW_L_P_GEAR_RESET_AUTO_ACT_NU     // Parameter to reset automatic activation after P-gear is engaged and then disengaged.
    type.boolean PDW_L_EPB_RESET_AUTO_ACT_NU     // Parameter to reset automatic activation after EPB is enabled and then disabled.
    type.boolean PDW_L_ROLLBACK_FULL_VIEW_NU     // Parameter to allow full view in case of rollback activation. 1 = full view active
    type.boolean PDW_L_ROLLBACK_TRAILER_NU     // Parameter that specifies if rollback activation shall be available or not in case of a attached trailer. 1 = rollback activation available even with attached trailer
    type.boolean PDW_L_ACT_PDW_BY_AP_NU     // Parameter to enable/disable PDW activation by AUP.
    type.boolean WHP_L_DEACT_WHP_BY_AP_NU     // Parameter to enable/disable WHP deactivation by AUP.
    type.boolean DWF_L_TONE_SUPPRESS_BY_AP_NU     // Parameter to enable/disable DWF tone deactivation during AUP: 1 = tone deactivation is allowed
    type.boolean DWF_L_TONE_REDUCT_INTER_STANDSTILL_NU     // Parameter to enable/disable tone volume reduction in standstill for intermittent: 1 = tone reduction allowed
    type.boolean DWF_L_TONE_REDUCT_CONT_STANDSTILL_NU     // Parameter to enable/disable tone volume reduction in standstill for continuous : 1 = tone reduction allowed
    type.float32 PDW_L_STANDSTILL_INTER_TIME_S     // Waiting time before the vehicle is considered in standstill for intermittent tone(time with no ticks detected at the rear wheels)
    type.float32 PDW_L_STANDSTILL_CONT_TIME_S     // Waiting time before the vehicle is considered in standstill for continuos tone(time with no ticks detected at the rear wheels)
    type.boolean PDW_L_AUTO_ACTIV_CRITICAL_NU     // Parameter to select the autoactivation for obstacles 1= autoactivation for obstacles in the continuous zone or in driving tube, 0= autoactivation for activation region
    type.float32 PDW_L_MAX_DIST_CONT_TONE_FWD_M     // Define maximum distance threshold  for continous tone range for front side
    type.float32 PDW_L_MAX_DIST_CONT_TONE_REAR_M     // Define maximum distance threshold  for continous tone range for rear side
    type.float32 PDW_L_MAX_DIST_CONT_TONE_LAT_M     // Define maximum distance threshold  for continous tone range for lateral sides
    type.boolean PDW_L_CONT_DIST_EXTEND_NU     // Parameter to enable/disable modification of continuous zone length
    type.float32 PDW_AUTO_ACTIV_THRESHOLD     // Define maximum distance for activation range
    type.boolean PDW_L_CRIT_ACT_PDW_BY_AP_NU     // Parameter to enable/disable PDW activation by critical obstacles during AUP.
    type.boolean PDW_L_FRONT_SECT_ACTIV_NU     // Parameter to enable/disable PDW for front sectors: 1 = PDW for front sectors is enable
    type.boolean PDW_L_LAT_SECT_ACTIV_NU     // Parameter to enable/disable PDW for lateral sectors: 1 = PDW for lateral sectors is enable
    type.boolean PDW_L_REAR_SECT_ACTIV_NU     // Parameter to enable/disable PDW for rear sectors: 1 = PDW for rear sectors is enable
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_DRVWARNSM.FC_DrvWarnSM_Params_InterfaceVersion uml:Class
  Members:
    type.uint32 FC_DrvWarnSM_Params_VERSION = 2    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM_APP.RemoteMode uml:Enumeration
    // [Optional] Indicates the currently active mode of AP related to remote functionality
  Members:
    REM_MODE_INACTIVE = 0            
    REM_MODE_PARK_IN = 1            
    REM_MODE_PARK_OUT = 2            
    REM_MODE_MAN = 3            
    REM_MODE_GARAGE_PARKING = 4            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM_APP.MaxSpeed10KPHwarning uml:Enumeration
    // For manual gear box, warning maximum speed limit : 10KPH
  Members:
    OFF = 0            
    CONTINUOUS = 1            
    BLINKING = 2            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM_APP.HMIMessage uml:Enumeration
  Members:
    NO_MESSAGE = 0            
    VERY_CLOSE_TO_OBJECTS = 1            
    START_PARKING_IN = 2            
    PARKING_IN_FINISHED = 3            
    PARKING_CANCELLED = 4            
    PARKING_FAILED = 5            
    SHIFT_TO_R = 6            
    SHIFT_TO_D = 7            
    SHIFT_TO_1 = 8            
    RELEASE_BRAKE = 9            
    START_PARKING_OUT = 10            
    PARKING_OUT_FINISHED = 11            
    START_REM_MAN = 12            
    REM_MAN_OBSTACLE_STOP = 13            
    REDUCE_DISTANCE_TO_VEHICLE = 14            
    CLOSE_DOOR_OR_START_REMOTE = 15            
    RELEASE_HANDBRAKE = 16            
    LEAVE_VEHICLE = 17            
    NO_DRIVER_DETECTED_ON_SEAT = 18            
    INTERNAL_SYSTEM_ERROR = 19            
    PARKING_OUT_HANDOVER = 20            
    STEERING_ACTIVE = 21            
    STOP = 22            
    MAX_WAITING_TIME_EXCEEDED = 23            
    SHIFT_TO_P = 24            
    SHIFT_TO_N = 25            
    SELECT_PARKING_VARIANT = 26            
    NO_REMOTE_DEVICE_PAIRED = 27            
    NO_REMOTE_DEVICE_CONNECTED = 28            
    PARKING_CANCELLED_THROTTLE = 29            
    PARKING_CANCELLED_STEER = 30            
    GARAGE_NOT_OPEN = 31            
    UNDO_FINISHED = 32            
    LOW_ENERGY = 33            
    WAIT = 34            
    SLOW_DOWN = 35            
    KEY_NOT_IN_RANGE = 36            
    KEY_NOT_ALIVE = 37            
    PARKING_IN_FINISHED_FALLBACK = 38            
    PARKING_OUT_FINISHED_FALLBACK = 39            
    REVERSE_ASSIST_FINISHED = 40            
    REVERSE_ASSIST_CANCELLED = 41            
    REVERSE_ASSIST_FAILED = 42            
    FRONT_CAM_VISION_UNRELIABLE = 43            
    REAR_CAM_VISION_UNRELIABLE = 44            
    LEFT_CAM_VISION_UNRELIABLE = 45            
    RIGHT_CAM_VISION_UNRELIABLE = 46            
    FRONT_CAM_PRE_PROC_UNRELIABLE = 47            
    REAR_CAM_PRE_PROC_UNRELIABLE = 48            
    LEFT_CAM_PRE_PROC_UNRELIABLE = 49            
    RIGHT_CAM_PRE_PROC_UNRELIABLE = 50            
    FRONT_ULTRASONICS_UNRELIABLE = 51            
    REAR_ULTRASONICS_UNREALIBLE = 52            
    HOST_TEMPERATURE_WARNING = 53            
    VEHICLE_COMMUNICATION_ERROR = 54            
    MEMORY_PARKING_FINISHED = 55            
    MEMORY_PARKING_CANCELLED = 56            
    MEMORY_PARKING_FAILED = 57            
    MEMORY_PARKING_REFINE = 58            
    PARKING_SLOT_FOUND = 59            
    DRIVER_IN_OUT_SELECTION = 60            
    PARKING_RESUMED = 61            
    PARKING_TIMER_EXCEEDED = 62            
    DMS_TIMER_EXCEEDED = 63            
    PARK_BRAKE_ENGAGED = 64            
    REFERENCE_VEHICLE_MOVED = 65            
    PRESS_BRAKE_PEDAL = 66            
    STEEP_ROAD = 67            
    POOR_LIGHTING = 68            
    NO_VISIBILTY = 69            
    AVG_SPPED_HIGH = 70            
    SPACE_CONSTRAINT = 71            
    SYSTEM_FAILURE = 72            
    ORVM_MANUAL_HANDLING = 73            
    ONLY_SAFETY_CORE_AVAILABLE = 74            
    ODOMETRY_UNRELIABLE = 75            
    ENV_MODEL_STATIC_OBJS_UNRELIABLE = 76            
    ENV_MODEL_TRAFFIC_PARTICIPANTS_UNRELIABLE = 77            
    ENV_MODEL_PARKING_FEATURES_UNRELIABLE = 78            
    LOCALIZATION_UNRELIABLE = 79            
    PDW_UNRELIABLE = 80            
    LSCA_UNRELIABLE = 81            
    RA_UNRELIABLE = 82            
    HV_UNRELIABLE = 83            
    AVGA_FAILED = 84            
    MF_MANAGER_FAILED = 85            
    MOCO_FAILED = 86            
    FRONT_CAM_BLOCKAGE = 87            
    REAR_CAM_BLOCKAGE = 88            
    LEFT_CAM_BLOCKAGE = 89            
    RIGHT_CAM_BLOCKAGE = 90            
    HMI_COMMUNICATION_ERROR = 91            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM_APP.GarageParking uml:Enumeration
  Members:
    GP_NOT_AVAILABLE = 0            
    GP_IN_SCAN_FWD = 1            
    GP_IN_SCAN_BWD = 2            
    GP_OUT_FWD = 3            
    GP_OUT_BWD = 4            
    GP_IN_DETECTED_FWD = 5            
    GP_IN_DETECTED_BWD = 6            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM_APP.PPCParkingMode uml:Enumeration
  Members:
    PARKING_MODE_NOT_VALID = 0            
    PARK_IN_FULL_MANEUVERING_AREA = 1            
    PARK_IN_RESTRICTED_MANEUVERING_AREA = 13            
    PARK_OUT_UNTIL_CRITICAL_POINT_REACHED = 2            
    PARK_OUT_TO_TARGET_POSE = 14            
    GARAGE_PARKING_IN = 3            
    GARAGE_PARKING_OUT = 4            
    TRAINED_PARKING_TRAIN = 5            
    TRAINED_PARKING_EXEC = 6            
    REMOTE_MANEUVERING = 7            
    MEMORY_PARKING_TRAIN = 8            
    MEMORY_PARKING_EXEC = 9            
    UNDO_MANEUVER = 10            
    REVERSE_ASSIST_ACTIVE = 11            
    REMOTE_SELF_TEST = 12            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM_APP.APFinishType uml:Enumeration
  Members:
    AP_NOT_FINISHED = 0            
    AP_SUCCESS = 1            
    AP_SUCCESS_FALLBACK = 2            
    AP_CANCELED = 3            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM_APP.PPCState uml:Enumeration
    // State of the parking procedure state machine
  Members:
    PPC_INACTIVE = 0            
    PPC_BEHAVIOR_INACTIVE = 1            
    PPC_SCANNING_INACTIVE = 2            
    PPC_SCANNING_IN = 3            
    PPC_SCANNING_OUT = 4            
    PPC_SCANNING_RM = 5            
    PPC_PARKING_INACTIVE = 6            
    PPC_PREPARE_PARKING = 7            
    PPC_PERFORM_PARKING = 8            
    PPC_PARKING_PAUSE = 9            
    PPC_PARKING_FAILED = 10            
    PPC_OFFER_HANDOVER = 11            
    PPC_SUCCESS = 12            
    PPC_PARKING_CANCELED = 13            
    PPC_IRREVERSIBLE_ERROR = 14            
    PPC_REVERSIBLE_ERROR = 15            
    PPC_DEMO_OFF = 23            
    PPC_SCANNING_GP = 24            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM_APP.ESMState uml:Enumeration
  Members:
    ESM_INACTIVE = 0            
    ESM_NO_ERROR = 1            
    ESM_REVERSIBLE_ERROR = 2            
    ESM_IRREVERSIBLE_ERROR = 3            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM_APP.VSMState uml:Enumeration
  Members:
    VSM_INACTIVE = 0            
    VSM_SCANNING_CONDITIONS_NOT_MET = 1            
    VSM_SCANNING_CONDITIONS_MET = 2            
    VSM_MANEUVERING_CONDITIONS_MET = 3            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM_APP.DMState uml:Enumeration
  Members:
    DM_INACTIVE = 0            
    DM_DRIVER_MANEUVERING = 1            
    DM_DRIVER_ASSISTS_AP_MANEUVER = 2            
    DM_DRIVER_NOT_MANEUVERING = 3            
    DM_PASSENGER_IN_VEHICLE = 4            
    DM_NOONE_IN_VEHICLE = 5            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM_APP.RDMState uml:Enumeration
  Members:
    RDM_INACTIVE = 0            
    RDM_OUT_OF_RANGE = 1            
    RDM_IN_RANGE = 2            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM_APP.StdRequest uml:Enumeration
  Members:
    STD_REQ_NO_REQUEST = 0            
    STD_REQ_INIT = 1            
    STD_REQ_SCAN = 2            
    STD_REQ_PREPARE = 3            
    STD_REQ_DEGRADE = 4            
    STD_REQ_START = 5            
    STD_REQ_PAUSE = 6            
    STD_REQ_SECURE = 7            
    STD_REQ_FINISH = 8            
    STD_REQ_ERROR = 9            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM_APP.DegradeType uml:Enumeration
  Members:
    NO_DEGRADATION = 0            
    INCREASE_SAFETY_MARGIN = 1            
    LONG_CTRL_ONLY = 2            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM_APP.MotionControlRequestType uml:Enumeration
  Members:
    AP_MCRT_NORMAL_REQUEST = 0            
    AP_MCRT_OVERRIDE_REQUEST = 1            
    AP_MCRT_DEGRADATION_REQUEST = 2            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM_APP.DriverInOutRequestType uml:Enumeration
  Members:
    AP_DRIVER_IN_MANEUVER = 0            
    AP_DRIVER_OUT_MANEUVER = 1            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM_APP.AP_PSM_APP_Consts uml:Class
  Members:
    type.uint8 NUM_MTS_DEBUG_FREESPACE_APP_PSM = 10    
    type.uint8 NUM_PRE_CONDITION_FAILURE_APP_PSM = 15    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM_APP.EngineReq uml:Class
  Members:
    type.boolean stopEngine_nu     // Stop engine
    type.boolean startEngine_nu     // Start engine
    type.AP_CommonVehSigProvider.StartStopStatus startStopStatus_nu     // Status of start stop function
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM_APP.BCMReq uml:Class
  Members:
    type.boolean dirIndLeftReq_nu     // Direction indicator on left side is requested
    type.boolean dirIndRightReq_nu     // Direction indicator on right side is requested
    type.boolean hazardWarning_nu     // Hazard warning is active
    type.boolean environmentLightReq_nu     // Special environmental lights for surround view requested
    type.boolean lowBeamLightReq_nu     // Low beam light requested
    type.boolean backUpLightReq_nu     // Back-up light requested
    type.boolean foldMirrors_nu     // Request to fold mirrors
    type.boolean unFoldMirrors_nu     // Request to unfold mirrors
    type.boolean blockConvTopActivation_nu     // Block activation of convertible top
----------------------------------------------------------------------------------------------------------


  # ID:  type.AP_PSM_APP.APUserInformationPort uml:Class
  version: ::ap_psm_app::APUserInformationPort_InterfaceVersion::APUserInformationPort_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.AP_PSM_APP.RemoteMode remoteModeActive_nu     // [Optional] Indicates the currently active mode of AP related to remote functionality
    type.boolean remoteAppActive_nu     // Indicates, whether the Remote App is active (= is opened and is not breaked down)
    type.boolean remoteAppAuthorized_nu     // True if the usage of the remote Control is authorized
    type.boolean remoteAppCoded_nu     // Indicates, that the remote control is coded for the current sw
    type.boolean remoteKeySelected_nu     // True if user selected Remote Key instead of in vehicle parking;True if user selected Remote Key instead of in vehicle parking
    type.boolean remoteKeyPoss_nu     // Indicates, whether Remote Key is possible in the vehicle
    type.boolean continuePoss_nu     // Indicates wether it is possible to continue the interrupted maneuver
    type.boolean remManPoss_nu     // True, if remote maneuvering is possible. (In this case, user can select it via menu of the remote parking app)
    type.boolean undoPoss_nu     // Indicates wether it is possible to undo the interrupted maneuver
    type.boolean svPoss_nu     // True, if the vehicle can stream a surround-view view to the remote parking app
    type.boolean btnForwardPoss_nu     // [for Remote Maneuvering]True, if it is possible to move the ego vehicle in forward direction -> Forward-Button is active in REM_MAN-Screen
    type.boolean btnBackwardPoss_nu     // [for Remote Maneuvering]True, if it is possible to move the ego vehicle in backward direction -> Backward-Button is active in REM_MAN-Screen
    type.boolean btnFullyAutomParkingPoss_nu     // [Indicates, if Fully Automated Parking is possible (shows the corespondent button)
    type.boolean btnSemiAutomParkingPoss_nu     // Indicates, if Semi Automated Parking is possible (shows the corespondent button)
    type.boolean garageOpenerAvail_nu     // Indicates, that a remote garage door opener is available and configured (true, when GARAGE_OPENER_STATUS_NU == GOS_CONFIGURED)
    type.uint8 distanceToStop_perc     // Distance to next stopping point during parking maneuver (could be shown in a filling bar)
    type.AP_PSM_APP.MaxSpeed10KPHwarning maxSpeed10KPHwarning_nu     // For manual gear box, warning maximum speed limit : 10KPH
    type.AP_Common.DrivingDirection drivingDirection_nu     // Vehicle direction for the current maneuver part
    type.AP_PSM_APP.GarageParking garageParking_nu     // 0: set, when either GARAGE_PARKING_CODED_NU==false or when no garage has been detected;1: set, when a garage scan is active in the front of the vehicle;2: set, when a garage scan is active in the back of the vehicle;3: set, wehen a forward parking out situation has been detected;4: set, when a backward parking out situation has been detected;5: set, when a garage has been detected in the front of the vehicle;6: set, when a garage has been detected in the back of the vehicle
    type.AP_CommonVehSigProvider.Gear currentGear_nu     // Current gear of the ego vehicle. This information is displayed in automated vehicle guidance (AVG) mode.
    type.AP_PSM_APP.HMIMessage generalUserInformation_nu     // general information for the User. Extended User interface. Currently NOT used. Can be used to get rid of the visualization ports
    type.AP_PSM_APP.PPCParkingMode ppcParkingMode_nu     // current active parking mode. Extended User interface. Currently NOT used. Can be used to get rid of the visualization ports
    type.AP_PSM_APP.APFinishType finishType_nu     // type of the finish event. Extended User interface. Currently NOT used. Can be used to get rid of the visualization ports
    type.boolean switchInputDevPoss_nu     // indicates that it is possible to switch the device at the moment (remote to hmi or the other way around)
    type.boolean reverseAssistPoss_nu     // indicates that it is possible to activate reverse assist function
    type.boolean memoryParkingPoss_nu     // indicates that it is possible to activate memory parking function
    type.boolean mpEasyRegisterAvailable     // to indicate easy register feature is ready
    type.boolean preConditionFailureState     // indicates any pre conditions for parking failed
    type.boolean preConditionFailureReason     // array of bits that represent which all pre conditions failed
    type.AP_PSM_APP.HUIntrPauseCondition apaIntrPauseCondition     // This signal Indicates the Pause condition messages
    type.uint8 apaInteractionPauseCount     // This signal indicates the number of pause count
    type.uint8 apaInteractionTimer     // This signal Indicates timer for the pause conditions
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM_APP.APUserInformationPort_InterfaceVersion uml:Class
  Members:
    type.uint32 APUserInformationPort_VERSION = 5    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM_APP.PSMToSSPPort uml:Class
  version: ::ap_psm_app::PSMToSSPPort_InterfaceVersion::PSMToSSPPort_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.AP_PSM_APP.EngineReq engineReq     // Signals from Engine ECU
    type.AP_PSM_APP.BCMReq bcmReq     // Signals to BCM
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM_APP.PSMToSSPPort_InterfaceVersion uml:Class
  Members:
    type.uint32 PSMToSSPPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM_APP.PSMDebugPort uml:Class
  version: ::ap_psm_app::PSMDebugPort_InterfaceVersion::PSMDebugPort_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.sint32 debugInt     // freespace for MTS debug values
    type.float32 debugFloat     // freespace for MTS debug values
    type.AP_PSM_APP.PPCState stateVarPPC_nu     // State of the parking procedure state machine
    type.AP_PSM_APP.ESMState stateVarESM_nu     // State of the error state monitoring state machine
    type.AP_PSM_APP.VSMState stateVarVSM_nu     // State of the vehicle state monitoring state machine
    type.AP_PSM_APP.DMState stateVarDM_nu     // State of the driver monitoring state machine
    type.AP_PSM_APP.RDMState stateVarRDM_nu     // State of the remote hmi communication state machine
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM_APP.PSMDebugPort_InterfaceVersion uml:Class
  Members:
    type.uint32 PSMDebugPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM_APP.OverrideLSCAPort uml:Class
  version: ::ap_psm_app::OverrideLSCAPort_InterfaceVersion::OverrideLSCAPort_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 ignoreDistanceLSCA_m     // @range{0,1};Request for distance to STOP from AP and LSCA in order to ignore the object inside the given distance
    type.AP_Common.DrivingDirection ignoreDirection_nu     // Request for direction of distance to ignore the object in this direction
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM_APP.OverrideLSCAPort_InterfaceVersion uml:Class
  Members:
    type.uint32 OverrideLSCAPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM_APP.CtrlCommandPort uml:Class
  version: ::ap_psm_app::CtrlCommandPort_InterfaceVersion::CtrlCommandPort_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.AP_PSM_APP.StdRequest stdRequest_nu     // the std request to the ap psm core
    type.AP_PSM_APP.PPCParkingMode ppcParkingMode_nu     // inform the psm core of the current parking mode
    type.AP_PSM_APP.DegradeType degradeType_nu     // in case of function degradation is requeseted the type of the degradation has to be specified
    type.uint8 selectedTargetPoseId_nu     // id of the selected target pose
    type.boolean emergencyBrake_nu     // request emergency brake of core parksm (if this flag is set the stop request will be send as emergency brake request to the controller)
    type.uint8 currentSelectedMpSlotId     // Id of the current memory parking slot selected by the user
    type.boolean deleteMemorizedParkingData     // request to delete
    type.boolean storeMemorizedParkingData     // store request
    type.AP_PSM_APP.MotionControlRequestType motionControlRequestType_nu     // Type of requests to the Motion Controller
    type.AP_PSM_APP.DriverInOutRequestType driverInOutRequestType_nu     // Type of requests based on driver Inside or outside
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM_APP.CtrlCommandPort_InterfaceVersion uml:Class
  Members:
    type.uint32 CtrlCommandPort_VERSION = 4    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM_APP.HUIntrPauseCondition uml:Enumeration
    // This signal Indicates the Pause condition messages
  Members:
    NO_MSG = 0            
    DRIVER_DOOR_NOT_CLOSED = 1            
    CO_DRIVER_DOOR_NOT_CLOSED = 2            
    DYN_STAT_OBJ_TRAJ = 3            
    SEAT_BELT_OPEN = 4            
    ORVM_CLOSE = 5            
    BRAKE_PRESS_TH = 6            
    APA_HOLD_RELEASED = 7            
    REAR_PSG_DOOR_NOT_CLOSED = 8            
    TRUNK_LID_NOT_CLOSED = 9            
    TAIL_GATE_NOT_CLOSED = 10            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM_APP.FC_PARKSM_Params uml:Class
  version: ::ap_psm_app::FC_PARKSM_Params_InterfaceVersion::FC_PARKSM_Params_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 AP_S_BRAKE_PERSS_THRESH_P_BAR     
    type.float32 AP_S_BRAKE_PERSS_THRESH_C_BAR     
    type.float32 AP_S_CONFIRMATION_TIME_S     
    type.float32 AP_S_DEADMANSWITCH_THRESHOLD_TIMER_S     
    type.float32 AP_S_STEERING_TORQUE_THRESH_NM     
    type.uint32 AP_S_REM_FINGER_MOV_THRESH_PX     
    type.uint32 AP_S_REM_FINGER_MOVE_DELAY_US     
    type.float32 AP_S_RESET_MAP_DIST_M     
    type.float32 AP_S_STOP_DIST_THRESH_M     
    type.float32 AP_S_UNDERSHOOT_DIST_THRESH_M     
    type.float32 AP_S_OVERSHOOT_DIST_THRESH_M     
    type.uint8 AP_S_NUM_IGNORED_REM_ALIVE_NU     
    type.uint16 AP_S_APP_RES_X_NU     
    type.uint16 AP_S_APP_RES_Y_NU     
    type.boolean AP_S_CHECK_PDC_SYSTEM_STATE_NU     
    type.uint8 AP_S_NUM_IGNORED_KEY_ALIVE_NU     
    type.uint8 AP_S_MIN_APP_ENERGY_LEVEL_PERC     
    type.boolean AP_S_DO_NOT_CHECK_REM_KEY_NU     
    type.AP_Common.FC_PARKSM_Sys_Func_Params sysFuncParams     
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM_APP.FC_PARKSM_Params_InterfaceVersion uml:Class
  Members:
    type.uint32 FC_PARKSM_Params_VERSION = 2    
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.Constants uml:Class
    // Constants used in the SI interface
  Members:
    type.uint8 MAX_NUM_OF_DIAGNOSTIC_EVENT_IDS = 4    // @unit{nu};Maximum number of diagnostic event id
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.ObjMeasurementState uml:Enumeration
    // Measurement state of the dynamic object
  Members:
    MEAS_STATE_NEW = 0            
    MEAS_STATE_MEASURED = 1            
    MEAS_STATE_PREDICTED = 2            
    MEAS_STATE_DELETED = 3            
    MAX_NUM_MEASUREMENT_STATES = 4            
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.ObjectTrend uml:Enumeration
    // Trend information about the object.
  Members:
    OBJ_TRND_UNKNOWN = 0            
    OBJ_TRND_APPROACHING = 1            
    OBJ_TRND_MOVING_AWAY = 2            
    OBJ_TRND_FIXED_DIST = 3            
    OBJ_TRND_MAX_NUM = 4            
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.StaticObjHeigthType uml:Enumeration
    // height calss of the shape
  Members:
    SO_HI_UNKNOWN = 0            
    SO_HI_WHEEL_TRAVERSABLE = 1            
    SO_HI_BODY_TRAVERSABLE = 2            
    SO_HI_DOOR_OPENABLE = 3            
    SO_HI_HIGH_OBSTACLE = 4            
    SO_HI_HANGING_OBJECT = 5            
    SO_HI_LOWER_BUMPER_HEIGHT = 6            
    MAX_NUM_HEIGHT_TYPES = 7            
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.ParkingScenarioTypes uml:Enumeration
    // type of parking sccenario associated with this parking box
  Members:
    PARALLEL_PARKING = 0            
    PERPENDICULAR_PARKING = 1            
    ANGLED_PARKING_OPENING_TOWARDS_BACK = 2            
    ANGLED_PARKING_OPENING_TOWARDS_FRONT = 3            
    GARAGE_PARKING = 4            
    DIRECT_PARKING = 5            
    EXTERNAL_TAPOS_PARALLEL = 6            
    EXTERNAL_TAPOS_PERPENDICULAR = 7            
    EXTERNAL_TAPOS_PARALLEL_OUT = 8            
    EXTERNAL_TAPOS_PERPENDICULAR_OUT = 9            
    MAX_NUM_PARKING_SCENARIO_TYPES = 10            
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.ParkingScenarioSideTypes uml:Enumeration
    // type of parking sccenario associated with this parking box
  Members:
    UNKNOWN = 0            
    LEFT_SIDE_OF_ROAD = 1            
    RIGHT_SIDE_OF_ROAD = 2            
    MAX_NUM_PARKING_SCENARIO_SIDE_TYPES = 3            
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.RelativeLocationToParkingBox uml:Enumeration
    // side of the parking box for which this delimiter is relevant
  Members:
    UNDEFINED_EDGE = 0            
    ROAD_SIDE_EDGE = 1            
    RIGHT_EDGE = 2            
    CURB_SIDE_EDGE = 3            
    LEFT_EDGE = 4            
    INSIDE_ROAD_SIDE_EDGE = 5            
    INSIDE_RIGHT_EDGE = 6            
    INSIDE_CURB_SIDE_EDGE = 7            
    INSIDE_LEFT_EDGE = 8            
    CORNER_LEFT_AT_OPENING = 9            
    CORNER_RIGHT_AT_OPENING = 10            
    CORNER_LEFT_AT_CURB = 11            
    CORNER_RIGHT_AT_CURB = 12            
    RELATED_OTHERWISE = 13            
    MAX_NUM_PARKING_BOX_EDGE_TYPES = 14            
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.SIMotionState uml:Enumeration
    // motion state of the vehicle
  Members:
    SI_ODO_STANDSTILL = 0            
    SI_ODO_NO_STANDSTILL = 1            
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.AvailabilityStatus uml:Enumeration
    // availability state of the USS sensors (enum values tbd)
  Members:
    ITEM_IN_DEGREADED_STATE = 0            
    ITEM_IN_ERROR_STATE = 1            
    ITEM_NOT_AVAILABLE = 2            
    ITEM_AVAILABLE = 3            
    MAX_NUM_AVAILABILITY_STATES = 4            
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.EgoMotionPort uml:Class
  version: ::si::EgoMotionPort_InterfaceVersion::EgoMotionPort_VERSION
    // None
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.SI.SIMotionState motionState_nu     // @range{0,1};motion state of the vehicle
    type.float32 pitch_rad     // @unit{Radian};@range{-3.14159265359,3.14159265359};pitch angle of the vehicle
    type.float32 roll_rad     // @unit{Radian};@range{-3.14159265359,3.14159265359};roll angle of the vehicle
    type.float32 vel_mps     // @unit{m / s};@range{-27.8,100.0};velocity vehicle (rear axle based)
    type.float32 yawRate_radps     // @unit{Radian / s};yaw rate of the vehicle
    type.float32 accel_mps2     // @unit{m / s^2};@range{-27.8,100.0};acceleration vector of the vehicle
    type.float32 drivenDistance_m     // @unit{m};@range{-1000.0,1000.0};driven distance of the vehicle
    type.float32 frontWheelAngle_rad     // @unit{Radian};@range{-0.8,0.8};front wheel angle
    type.float32 rearWheelAngle_rad     // @unit{Radian};@range{-0.8,0.8};rear wheel angle
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.EgoMotionPort_InterfaceVersion uml:Class
  Members:
    type.uint32 EgoMotionPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.ParkingBoxDelimiter uml:Class
    // delimiter of a parking box
  Members:
    type.uint8 indexInList_nu     // @range{0,255};Index of the item in either the static structure array, parking space marking array or lane boundary array
    type.uint8 virtLineIdx_nu     // @range{0,255};index of the virtual line in the array of virtual lines
    type.SI.RelativeLocationToParkingBox delimitingSide_nu     // @range{0,255};Relative location of this delimiter to the parking box
----------------------------------------------------------------------------------------------------------


  # ID:  type.SI.VirtualLineSerializable uml:Class
    // virtual line constructed to facilitate a smooth orientation alignment of the parked vehicle
  Members:
    type.SI.VirtLineVertices_mSerializable virtLineVertices_m     // @unit{m};start and end point of the virtual line
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.VirtLineVertices_mSerializable uml:Class
  Members:
    type.LSM_GEOML.size_type actualSize     
    type.cml.Vec2Df_POD array     
----------------------------------------------------------------------------------------------------------


  # ID:  type.SI.DynamicObjectSerializable uml:Class
    // dynamic objects
  Members:
    type.uint8 existenceProb_perc     // @unit{Percent};@range{0,100};Existence probability of the dynamic object
    type.SI.DynamicObjShapeSerializable objShape_m     
    type.cml.Vec2Df_POD vel_mps     // @unit{m / s};velocity vector of the object
    type.cml.Vec2Df_POD accel_mps2     // @unit{m / s^2}
    type.float32 headingAngle_rad     // @unit{Radian};@range{-3.14159265359,3.14159265359}
    type.SI.ObjMeasurementState measurementState_nu     // @range{0,4};Measurement state of the dynamic object
    type.uint16 refObjID_nu     
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.DynamicObjShapeSerializable uml:Class
  Members:
    type.LSM_GEOML.size_type actualSize     
    type.cml.Vec2Df_POD array     
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.ResetOriginResult uml:Class
    // Information about the previously used coordinate system.
  Members:
    type.LSM_GEOML.Pose_POD originTransformation     // @unit{no unit};Transformation from old to new coordinate system.
    type.uint8 resetCounter_nu     // @range{0,255};reset counter increasing by one on every reset of the coordinate system origin
----------------------------------------------------------------------------------------------------------


  # ID:  type.SI.StaticObjectSerializable uml:Class
    // structures comprising the static environment of the vehicle
  Members:
    type.uint16 refObjID_nu     // @range{0,65535};Unique identifier of the object of which this structure is part (a physical object maybe split into multiple structures to comply to the convexity requirement)
    type.uint8 existenceProb_perc     // @unit{Percent};@range{0,100};Existence probability of the structure
    type.uint16 objAgeInCycles_nu     // timestamp of the first corresponding detection
    type.uint16 objMeasLastUpdateInCycles_nu     // timestamp of the last time, the object has been updated
    type.uint16 objTrendLastUpdateInCycles_nu     // timestamp of the last time, the object-trend has been updated
    type.SI.ObjectTrend objTrend_nu     // @range{0,255};Trend information about the object.
    type.boolean readFromNVRAM_nu     // true if structure has only been read from non volatile RAM and not (yet) been confirmed by new measurements
    type.SI.StaticObjShapeSerializable objShape_m     // @unit{m};Positions of the vertices
    type.SI.StaticObjHeigthType objHeightClass_nu     // @range{0,5};height calss of the shape
    type.uint8 objHeightClassConfidence_perc     // @unit{Percent};@range{0,100};confidence in the height classification
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.StaticObjShapeSerializable uml:Class
  Members:
    type.LSM_GEOML.size_type actualSize     
    type.cml.Vec2Df_POD array     
----------------------------------------------------------------------------------------------------------


  # ID:  type.SI.ParkingBoxSerializable uml:Class
    // description of potential parking boxes
  Members:
    type.uint16 parkingBoxID_nu     // @range{0,65535};ID of the parking box
    type.SI.SlotCoordinates_mSerializable slotCoordinates_m     // @unit{m};vertices describing the parking box polygon
    type.uint8 existenceProb_perc     // @unit{Percent};@range{0,100};existence probability of the parking box
    type.SI.ParkingScenarioTypes parkingScenario_nu     // @range{0,10};type of parking sccenario associated with this parking box
    type.SI.ParkingScenarioSideTypes parkingScenarioSide_nu     // @range{0,3};marks which side of the road is the parking scenario
    type.SI.ParkingBoxDelimiter delimiters     // @unit{nu};delimiter of a parking box
    type.uint8 numValidDelimiters_nu     // @range{0,255};number of valid entries in delimiter list
    type.uint8 numVirtualLines_nu     // @range{0,255};number of virtual lines for this parking box
    type.SI.VirtualLineSerializable virtualLines     // @unit{nu};virtual line constructed to facilitate a smooth orientation alignment of the parked vehicle
    type.uint16 groupID_nu     
    type.uint8 priority_nu     
    type.cml.Vec2Df_POD parkingBoxRoadSideEdge     // the edge of the parking box on the road side
    type.boolean hasValueParkingBoxRoadSideEdge     // boolean showing if parkingBoxRoadSideEdge value had been filled with valid data
    type.cml.Vec2Df_POD cnnBoxRoadSideEdge     // the road side edge of the CNN parking box
    type.boolean hasValueCnnBoxRoadSideEdge     // boolean showing if cnnBoxRoadSideEdge value had been filled with valid data
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.SlotCoordinates_mSerializable uml:Class
  Members:
    type.LSM_GEOML.size_type actualSize     
    type.cml.Vec2Df_POD array     
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.ExternalPoseData uml:Class
  Members:
    type.LSM_GEOML.Pose_POD externalTargetPose_m     
    type.float32 curvature_1pm     
    type.uint16 relatedParkingBoxId     
    type.boolean isParkingPose     
    type.boolean isForwardDriving     
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.ApEnvModelPort uml:Class
  version: ::si::ApEnvModelPort_InterfaceVersion::ApEnvModelPort_VERSION
    // None
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint8 numberOfStaticObjects_u8     // @unit{nu};Number of valid static objects
    type.uint8 numberOfDynamicObjects_u8     // @unit{nu};Number of valid dynamic objects
    type.SI.ResetOriginResult resetOriginResult     // @unit{nu};Information about the previously used coordinate system.
    type.LSM_GEOML.Pose_POD transformationToOdometry     // @unit{no unit};Transformation information from AP (performance core) to odometry coordinate system (realtime core)
    type.LSM_GEOML.Pose_POD egoVehiclePoseForAP     // @unit{no unit};A pose defined by an x and y coordinate as well as a yaw angle
    type.SI.DynamicObjectSerializable dynamicObjects     // @unit{nu};dynamic objects
    type.SI.StaticObjectSerializable staticObjects     // @unit{nu};structures comprising the static environment of the vehicle
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.ApEnvModelPort_InterfaceVersion uml:Class
  Members:
    type.uint32 ApEnvModelPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.ApParkingBoxPort uml:Class
  version: ::si::ApParkingBoxPort_InterfaceVersion::ApParkingBoxPort_VERSION
    // None
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint8 numValidParkingBoxes_nu     // @range{0,MAX_NUM_PARKING_BOXES};number of valid parking boxes
    type.uint8 numValidExternalPoses     
    type.SI.ParkingBoxSerializable parkingBoxes     // @unit{nu};description of potential parking boxes
    type.SI.ExternalPoseData extPoseData     
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.ApParkingBoxPort_InterfaceVersion uml:Class
  Members:
    type.uint32 ApParkingBoxPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.CollEnvModelPort uml:Class
  version: ::si::CollEnvModelPort_InterfaceVersion::CollEnvModelPort_VERSION
    // None
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint8 numberOfStaticObjects_u8     // @unit{nu};Number of valid static objects
    type.uint8 numberOfDynamicObjects_u8     // @unit{nu};Number of valid dynamic objects
    type.uint8 firstStatObjOutDetZoneIdx_u8     // @unit{nu};Idx of first static object out of the detection zone
    type.uint8 firstDynObjOutDetZoneIdx_u8     // @unit{nu};Idx of first dynamic object out of the detection zone
    type.SI.DynamicObjectSerializable dynamicObjects     // @unit{nu};dynamic objects
    type.SI.StaticObjectSerializable staticObjects     // @unit{nu};structures comprising the static environment of the vehicle
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.CollEnvModelPort_InterfaceVersion uml:Class
  Members:
    type.uint32 CollEnvModelPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.PerceptionAvailabilityPort uml:Class
  version: ::si::PerceptionAvailabilityPort_InterfaceVersion::PerceptionAvailabilityPort_VERSION
    // availability states of the sensors and EM
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.SI.AvailabilityStatus statusUSSensors_nu     // @range{0,5};availability state of the USS sensors (enum values tbd)
    type.SI.AvailabilityStatus statusEnvModel_nu     // @range{0,5};availability state of the env model (enum values tbd)
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.PerceptionAvailabilityPort_InterfaceVersion uml:Class
  Members:
    type.uint32 PerceptionAvailabilityPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.SiParams uml:Class
  version: ::si::SiParams_InterfaceVersion::SiParams_VERSION
    // SI Parameters
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.boolean isStaticObjectInputCCWOriented     // If true the polygons read from the input are oriented counter-clockwise. If false they are clockwise
    type.float32 increaseOverhangLowObject_m     // Additional overhang to compensate the fact that in the 2D plane a wheel stopper is partly covered by the wheels.
    type.float32 coveredIntervalsRoiWidth_m     // It controls the width of the ROI, that determines which objects taking into account of the covered interval calculation of the parking box.
    type.boolean useLineMarkings     // Base and performance only: if true reads lineMarkings from Input. Wheelstoppers in input will still be read regardless
    type.boolean treatUnknownCurbsideObjectsAsBodyTraversable     // If set to true, treat unknown curbside objects as body traversable objects.
    type.boolean enableSlotIdentFromTrajectory     // Enable/Disable the estimation of a parking maneuver based on the sampled trajectory
    type.float32 slotIdentMaxYawDevParallel_rad     // Threshold for the max. yaw deviation from the end position within the recorded trajectory for slot identification
    type.float32 slotIdentMaxDevFromStraightLine_m     // Threshold for max. deviation of a straight line within the recorded trajectory for slot identification
    type.boolean enableSecondSideDelimiterOnlyScenario     // Enable parking scenarios where we only have the second delimiter and a curb object
    type.boolean enableFirstSideDelimiterOnlyScenario     // Enable parking scenarios where we only have the first delimiter and a curb object
    type.boolean preferSecondSideOSD     // If true prefer second side OSD over first side OSD if both are possible
    type.float32 osdSideRoiXFront_m     // Delta from vehicle front for OSD scan roi when moving forward (Otherwise use normal scanRoi)
    type.float32 osdSideRoiXBack_m     // Delta from vehicle back for OSD scan roi when moving backward (Otherwise use normal scanRoi)
    type.boolean requiredCurbObjectForOneSideDelimitingSlots     // If set to false, SI won"t require curb objects for one side delimiting (OSD) slots
    type.float32 horizontalMovementLimitOfASlotSlotCoords_m     // COMA shall not allow shrinking which results in total slot movement exceeding this value
    type.float32 maxAllowedVehCurbOverlapForPerp_m     // Max. allowed distance the vehicle may overlap a low curb in a perpendicular slot.
    type.float32 minCurbObjLengthForOneSideDel_m     // the length of all low curb side objects combined must exceed this threshold in order to accept a one side delimiter parking slot
    type.boolean allowNakedSlots     // SI won"t kill selected slot during park-in because of a missing delimiting object as long as the slot has at least one delimiting object.
    type.boolean requireAtLeastOneDelimitingObjectForNakedSlot     // Set to false to allow naked slots without any delimiting object. Parameter used only when naked slots are enabled.
    type.float32 maxRoadsideExtensionPerpendicular_m     // (aligned) slot may be shifted towards road by that value (reaching out between delimiters). Side-effect: slotPenetrationDepth_m
    type.float32 maxRoadsideExtensionParallel_m     // (aligned) slot may be shifted towards road by that value (reaching out between delimiters. Side-effect: slotPenetrationDepth_m
    type.float32 maxRoadsideExtensionAngular_m     // slot may be shifted towards road by that value (reaching out between delimiters. Side-effect: slotPenetrationDepth_m
    type.float32 minAppliedRoadsideExtension_m     // If applied, roadside extension shall not be smaller than this value
    type.float32 minDistanceToObject_m     // Minimum distance vehicle has to keep to objects
    type.float32 minDistanceNoObject_m     // Minimum distance vehicle has to keep to slot edges if no object on that side
    type.boolean enableAngularParking     // Disable/Enable angular slot detection
    type.boolean enableAngularRightOpeningTowardsBack     // Disable/Enable angular slot detection on the right with opening towards back of ego vehicle
    type.boolean enableAngularRightOpeningTowardsFront     // Disable/Enable angular slot detection on the right with opening towards front of ego vehicle
    type.boolean enableAngularLeftOpeningTowardsBack     // Disable/Enable angular slot detection on the left with opening towards back of ego vehicle
    type.boolean enableAngularLeftOpeningTowardsFront     // Disable/Enable angular slot detection on the left with opening towards front of ego vehicle
    type.boolean enableAngularSlotBruteForceDetection     // if set to true, the detector will try different angles to detect angular slots if no line marking was detected.
    type.boolean enableSlotPoseDetectionOnVirtualLines     // if set to true, the detector will calculate the virtual lines of side delimiting objects and use them to determine the slot orientation, if the VLs are long enough.
    type.float32 outlierDistSqrParallelSlot_m2     // Replacing parallel slots with squared distance equal or higher than this threshold regardless of their score to avoid having distant slots in the system only because they have a high score.
    type.float32 outlierDistSqrNonParallelSlot_m2     // Replacing non parallel slots with squared distance equal or higher than this threshold regardless of their score to avoid having distant slots in the system only because they have a high score.
    type.float32 slotReplacementDeltaOutlierDistSqr_m2     // Do a replacement if new slot"s squared distance is smaller than the worst slot to replace by slotReplacementDeltaOutlierDistSqr_m2.
    type.float32 angularParkingDetectionWindowAbsMin_m     // absolute (not relative to ego vehicle) min width of the slot window for angular parking space detection
    type.float32 angularParkingDetectionWindowAbsMax_m     // absolute (not relative to ego vehicle) max width of the slot window for angular parking space detection
    type.float32 angularSlotMaxDepthExtension_m     // TO BE TESTED: extension on the max depth of angular slots. So far an angular slot uses the same parameters as a;perpendicular slot. However, if an angular slot is aligned on lines, the one closer to the road is picked for;alignment and the slot is shifted a lot to the road. In this case it is too short if only the per max length is used.
    type.boolean enableParallelSlots     // Disable/Enable parallel slot detection
    type.float32 parallelSlotLengthDeltaMin_m     // (aligned) Parallel slots must have opening greater or equal vehicle length + this margin
    type.float32 parallelSlotLengthDeltaModel_m     // (NOT aligned) Parallel slots with unknown opening are modeled as vehicle length + this margin.Required for one-side delimiter parking
    type.float32 parallelSlotLengthDeltaMax_m     // (aligned) Parallel slots must have opening lesser or equal vehicle length + this margin
    type.float32 parallelSlotWidthDeltaModel_m     // (aligned) If there is no curbside object available, the slot depth is modeled as vehicle width + this margin
    type.float32 parallelSlotWidthDeltaMin_m     // (aligned) Parallel slots must have a depth greater or equal vehicle width + this margin
    type.float32 parallelSlotWidthDeltaMax_m     // (NOT aligned) Parallel slots must have depth lesser or equal vehicle width + this margin
    type.float32 parallelSlotOpeningDeltaMaxPark_m     // max slot opening: offset to vehilce length during park in.
    type.float32 parallelSlotOpeningDeltaMinPark_m     // min slot opening: offset to vehilce length during park in.
    type.float32 parallelSlotDepthDeltaMaxPark_m     // max slot depth: offset to vehilce width during park in.
    type.float32 parallelSlotDepthDeltaMinPark_m     // min slot depth: offset to vehilce width during park in.
    type.boolean enablePerpendicularSlots     // Disable/Enable perpendicular slot detection
    type.float32 perpendicularSlotWidthDeltaMin_m     // (aligned) Perpendicular slots must have opening greater or equal vehicle width + this margin
    type.float32 perpendicularSlotWidthDeltaModel_m     // (NOT aligned) Perpendicular slots with unknown opening are modeled as vehicle width + this margin.
    type.float32 perpendicularSlotWidthDeltaMax_m     // (aligned) Perpendicular slots must have opening lesser or equal vehicle width + this margin
    type.float32 perpendicularSlotLengthDeltaModel_m     // (aligned) If there is no curbside object available, the slot depth is modeled as vehicle length + this margin
    type.float32 perpendicularSlotLengthDeltaMin_m     // (aligned) Perpendicular slots must have a depth greater or equal vehicle length + this margin
    type.float32 perpendicularSlotLengthDeltaMax_m     // (NOT aligned) Perpendicular slots must have a depth lesser or equal vehicle length + this margin
    type.float32 perpendicularSlotOpeningDeltaMaxPark_m     // max slot opening: offset to vehilce width during park in.
    type.float32 perpendicularSlotOpeningDeltaMinPark_m     // min slot opening: offset to vehilce width during park in.
    type.float32 perpendicularSlotDepthDeltaMaxPark_m     // max slot depth: offset to vehilce length during park in.
    type.float32 perpendicularSlotDepthDeltaMinPark_m     // min slot depth: offset to vehilce length during park in.
    type.boolean enableGarageParkingSlots     // Disable/Enable garage slot detection
    type.float32 garageParkingSlotWidthDeltaMin_m     // (NOT aligned) Garage slots must have opening greater or equal vehicle width + this margin
    type.float32 garageParkingSlotWidthDeltaMax_m     // (NOT aligned) Garage slots must have opening lesser or equal vehicle width + this margin
    type.float32 garageParkingSlotLengthDeltaMin_m     // (NOT aligned) Garage slots must have depth greater or equal vehicle width + this margin
    type.float32 garageParkingSlotLengthDeltaModel_m     // (NOT aligned) If there is no curbside object detected, thes slot depth is modeled as vehicle length + this margin
    type.float32 garageParkingSlotLengthDeltaMax_m     // (NOT aligned) Garage slots must have depth lesser or equal vehicle width + this margin
    type.float32 garageParkingSlotDoorAreaLength_m     // (NOT aligned) The entrange of a garage slot tha overlaps with the closing door is modeled by this value
    type.float32 maxAllowedSlotOpeningOverlap_m     // new detected slot are allowed to have a certain overlap due to corner cases
    type.boolean preferWideSlots     // If true prefer shift to top if shifting means going below preferred width even if that means going below preferred depth
    type.boolean offerOverlappingSlots     // If set to true, SI will offer overlapping slots where applicable, Currently this works for OSD-perp and parallel
    type.boolean performOptimization     // Enable/Disable rotation optimization
    type.float32 shrinkSlotForNumericalInaccuracy_m     // 0.0001F: distance to shrink the slot after adustment to objects and lines to account for numerical inaccuracy
    type.float32 slotPositioningRoiLeftRightDelta_m     // Slot opening positioning takes object parts this far to the left/right of the slot into account
    type.float32 slotPositioningRoiRoadDelta_m     // Slot opening positioning takes object parts this for towards to the roadside of the slot into account
    type.float32 slotEdgePullSearchDistFront_m     // Objects within this radius are considered for the corner alignment part of the cost function
    type.float32 maxDrivenRotationParallel_rad     // max rotation of ego pose since slot detection before the parallel slot is invalidated
    type.float32 maxDrivenRotationPerpendicular_rad     // max rotation of ego pose since slot detection before the perpendicular slot is invalidated
    type.float32 maxDrivenRotationGarage_rad     // max rotation of ego pose since slot detection before the garage slot is invalidated
    type.float32 maxDrivenRotationAngled_rad     // max rotation of ego pose since slot detection before the angled slot is invalidated
    type.float32 slotDepthHorizontalLineAlignment_m     // depth of ROI into the slot to align slots to horizontal lines
    type.boolean enableSlotShrinkingBelowUsualSizeDuringTrack     // if set to true, CollisionManager will maintain the selected slot during track even if its size falls below expected minimums
    type.boolean invalidateOverlappingEstablishedSlots     // If two slots are fully established and overlap each other one of them should be invalidated
    type.float32 weightCurbOrientation     // weight of curb orientation cost
    type.float32 weightLineOrientation     // TO BE TESTED: weight for orientation of the slot according to vertical lines
    type.float32 minCoveragePerSideNonPar_m     // if this much coverage is available per side, the slot is considered as sandwich slot and line orientation is not used in the cost function for non-parallel slots
    type.boolean enableSlotOrientationOnSideVLs     // Disable/Enable the slot orientation according to the virtual lines along the slot of side delimiting objects (i.e. the deviation to the virtual lines becomes a part of the optimization criterion)
    type.boolean enableSlotOrientationOnWingVLs     // Disable/Enable the slot orientation according to the virtual lines along the street of side delimiting objects (i.e. the deviation to the virtual lines becomes a part of the optimization criterion)
    type.float32 sideAlignmentROIWidth_m     // It controls the width of the ROI, that is used for the side alignment calculation in the cost function.
    type.float32 sideAlignmentWeight_mpr     // Side alignment weight factor, that is used during the calculation of the performance score.
    type.float32 wingAlignmentWeight_mpr     // Wing alignment weight factor, that is used during the calculation of the performance score.
    type.float32 minimumObstacleFittedLineLength_m     // Length of the smallest acceptable "virtual line", fitted down an obstacle to determine slot orientation.
    type.float32 lineOrientationRoiDelta_m     // roi for slot orientation according to adjacent lines
    type.boolean highSideDelimiterRequired     // if set to true, a slot that has only low side delimiting objects and no line markings is invalid.
    type.float32 curbObjectEvaluationRoiDepthDelta_m     // depth delta for the roi of curb object evaluation. Object within this range below the slot are considered as curb objects.
    type.float32 parallelLineAllowedDeviation_rad     // -
    type.boolean enlargeDynObj     // if set to true, dynamic objects are enlarged to compensate uncertainties.
    type.float32 dynObjMaxVel_mps     // The higher the assumed max. object velocity, the larger the the dyn. object search ROI is set
    type.float32 dynObjMaxPreviewTime_s     // The longer the preview time, the larger the dyn. object search ROI is set
    type.uint8 dynObjMinExistenceProb_perc     // Objects with an existence probability below this threshold shall not be forwarded
    type.boolean doExactShiftVectorToEgoCollisionCheck     // if set to true, the collision check between slot shifts and the ego vehicle takes into account the exact object position
    type.float32 minSlotOpeningOffsetSideBlowUp_m     // Minimum slot opening offset used to calculate the maximum side blow up.
    type.float32 parallelSlotLineTooLongPenalty     // Penalty parameter in case of one of the parking line which belongs to the parkingbox is long.
    type.boolean enableSlotLengthAdaptionToAdjacentObjects     // if set to true, the slot length adaption to adjacent objects is enabled. The slot length will only be adjusted if no curb object or line is present.
    type.boolean enableSlotLengthAdaptionToAdjacentLines     // if set to true, the slot length adaption to adjacent lines is enabled. The slot length will only be adjusted if no curb object or line is present.
    type.boolean allowSlotExtensionOverLinesToMinOpening     // if set to true, the slot opening may be extended over lines to have minimum width
    type.float32 maxAllowedSlotExtensionOverLinesToMinOpening_m     // The maximum length the slot may be extended over a line for each side.
    type.boolean allowShrinkOnLinesDespiteSideObjects     // Allow the slot to be shrunk down to lines, even if there are objects available
    type.boolean preferShiftToTopOnCurbObjects     // if set to true, shrinking to top on objects in curb area shall be preferred over shrinking to side
    type.float32 sideShiftPenaltyForCurbObjects     // penalty added to costs of unpreferred shrinks to side on objects in curb area
    type.float32 preferShiftToTopRoiYBorderOffset_m     // vertical offset to the default border of the curb area (positive value means closer to the slot opening)
    type.boolean enableSlotFallback     // if set to true, an invalid slot will be kept alive for mTBD_numCyclesSlotFallback_nu SI cycles
    type.boolean enableUnspecifiedShift     // if set to true, a shift in unspecified direction (to a side and to top) is allowed for objects
    type.float32 penaltyForShrinkToMinDim     // Penalty for shrinking to minimum slot dimensions.
    type.float32 shrinkPenaltyForClassifyingObjectAsDynamic     // Apply these penalty to zero-shifts which result from declaring static objects as dynamic.
    type.float32 shrinkPenaltyForShiftBelowMin     // Apply this penalty to shifts beyond minimal values, highest penalty atm because trajpla no like small slots.
    type.float32 shrinkPenaltyForExceedingGivenLimits     // Apply a penalty to shift beyond the given limitations.
    type.float32 sideAndCurbDelimiterRoiExtension_m     // ROI extension parameter for side and curb delimiter calculation.
    type.float32 sideAndCurbDelimiterRoiBottomShift_m     // It shift the bottom of the ROI for side and curb delimiter calculation.
    type.float32 typicalOrientationOfAngledSlot_rad     // Likely deviation of an angled slot, from orthogonality. Note: required to be non-negative.
    type.boolean doLineCategorization     // If set to true SI will try to find parallel lines which match well together to delimit a parking box
    type.float32 smallSlotOpeningPenalty     // Slot rotation penalty if a slot has an opening smaller than the minimum
    type.float32 lcAngleCostFactor     // LineCategorization Pair Match cost factor for angle diff
    type.float32 lcLineDistCostFactor     // LineCategorization Pair Match cost factor for line dist smaller than model line dist based on slot dimensions
    type.float32 lcLineLengthCostFactor     // LineCategorization Pair Match cost factor for common line length smaller than model length based on slot dimensions
    type.boolean doCurbOnlyLineCategorization     // if left/right were found, but no road line is present try to categorize curb only
    type.float32 lcCurbOnlyAngleDiff_rad     // Maximum difference of a potential curb line angle to left/right line (Optimum is HALF_PI) Not applicable for angular parking
    type.float32 lcCurbOnlySpatialDiff_m     // Maximum spatial difference between left/right curb endpoints and curb line
    type.boolean ignoreLinesForSandwichSlots     // Disable/Enable whether line markings shall be ignored for sandwich slots
    type.boolean lcNoCurbLineOptimizationIfNoCurbLc     // LineCategorization no curb line was found through line categorization then do not optimize on curb lines
    type.float32 sensingRange_m     // Determines a Rectangular around the ego vehicle. Slots outside this window are not updated since we consider the data as unreliable
    type.float32 slotPenetrationDepth_m     // Detection zone is automatically adjusted so that it reaches this far into potential slots. MUST be below any minimum slot depth and should cover the relevant part of the entrance. Side-effect: Roadside Extension
    type.boolean enableLeftSideScanning     // Disable/Enable left side slot scanning
    type.boolean enableRightSideScanning     // Disable/Enable right side slot scanning
    type.float32 sideRoiYStart_m     // Side detection zone starts this far away from the ego flank
    type.float32 sideRoiYEnd_m     // Side detection zone ends this far away from the ego flank
    type.float32 sideRoiXFront_m     // Side detection zone ends this far in front of the ego
    type.float32 sideRoiXBack_m     // Side detection zone ends this far behind of the ego
    type.boolean enableFrontScanning     // Disable/Enable front slot scanning
    type.float32 frontRoiYSide_m     // Front detection zone goes this much to the left and right, seen from ego sides
    type.float32 frontRoiXStart_m     // Front detection zone starts this far in front of the ego
    type.float32 frontRoiXEnd_m     // Front detection zone starts ends this far in front of the ego
    type.boolean allowNarrowSlotsScan     // Allow the detection of slots below min opening during scanning
    type.float32 lineOpeningDelimiterMaxRoadProtrusion_m     // The maximum a line may protrude out of the scanRoi in a "Bad" direction (e.g. for right roi to the left)
    type.float32 lineOpeningDelimiterToCurbProjectionFactor     // Project Line onto scanRoi axis (e.g. for right Roi {0.0, -1.0F}). Separate projection in good(curb) and bad(road) side (e.g. for right roi cutoff point is the top of the roi and good is away from the vehicle and bad is towards vehicle). The good projection has to be factor times larger than the bad projection to still be considered a lineOpeningDelimiter
    type.boolean enableCnnWheelStoppers     // Disable/Enable CNN wheelstopper handling
    type.boolean enableCnnWheelStopperStatObjVerification     // only effective if enableCnnWheelStoppers is set to true
    type.float32 allowedWsPosOutsideSlot_m     // Wheelstopper belongs to a parking slot if it"s center is not outside the slot by this margin.
    type.boolean wsStatObjIgnoreEvaluation     // if set to true, the result of the static object evaluation is ignored and any object within the respective ROI is considered to be a wheelstopper and replaced by the body traversable artificial object
    type.float32 allowedDistToExistingWsWithWlSuppression_m     // allowed distance between two CNN wheel stoppers for merging the wheel stoppers in case that wheel locker suppression is activated
    type.float32 allowedDistToExistingWsWithoutWlSuppression_m     // allowed distance between two CNN wheel stoppers for merging the wheel stoppers in case that wheel locker suppression is deactivated
    type.uint8 integrationMessagesLevel_nu     // 1. Level determines what kind of messages are outputted
    type.uint8 integrationMessagesComponent_nu     // 0. Determine for which internal component messages shall be provided. 0 means all
    type.float32 collFunctionRoiWidthDelta_m     // Collision function roi is this far to the sides relative to ego
    type.float32 collFunctionRoiLengthDelta_m     // Collision function roi is this far to the Front/back relative to ego
    type.float32 barrierDelimiterWidth_m     // Default delimiter object width
    type.float32 minBarrierDelimiterLength_m     // If the calculated barrier is below this value, it is considered as irrelevant and not provided in the output
    type.float32 minBarrierPointOverhang_m     // The next point of an object must be at least this far away to the side in slot coordinates to take the current point as outer base
    type.float32 delZoneRoadMaxRelToVehWidthPar_nu     // Maximum Size of "Unspecified Zone" from "Center" towards Road for parallel Slots
    type.float32 delZoneRoadMaxRelToVehLengthNonPar_nu     // Maximum Size of "Unspecified Zone" from "Center" towards Road for non-parallel Slots
    type.float32 delZoneRoadFactorPar_nu     // Size of "Unspecified Zone" from "Center" towards Road for parallel Slots is (Depth * delZoneRoadFactorPar_nu_m) - delZoneRoadMinDistPar_m
    type.float32 delZoneRoadFactorNonPar_nu     // Size of "Unspecified Zone" from "Center" towards Road for non-parallel Slots is (Depth * delZoneRoadFactorNonPar_nu) - delZoneRoadMinDistNonPar_m
    type.float32 delZoneRoadMinDistNonPar_m     // Minimum distance the "Unspecified Zone" has to have from the road side edge for parallel slots
    type.float32 delZoneRoadMinDistPar_m     // Minimum distance the "Unspecified Zone" has to have from the road side edge for non -parallel slots
    type.float32 delZoneCurbMaxRelToVehWidthPar_nu     // Maximum Size of "Unspecified Zone" from "Center" towards Curb for parallel Slots
    type.float32 delZoneCurbMaxRelToVehLengthNonPar_nu     // Maximum Size of "Unspecified Zone" from "Center" towards Curb for non-parallel Slots
    type.float32 delZoneCurbFactorPar_nu     // Size of "Unspecified Zone" from "Center" towards curb for parallel Slots is (Depth * delZoneCurbFactorPar_nu)
    type.float32 delZoneCurbFactorNonPar_nu     // Size of "Unspecified Zone" from "Center" towards curb for non-parallel Slots is (Depth * delZoneCurbFactorNonPar_nu)
    type.float32 delZoneLeftRightMaxRelToVehLengthPar_nu     // Maximum Size of "Unspecified Zone" from "Center" towards left/right for parallel Slots
    type.float32 delZoneLeftRightFactorNonPar_nu     // Size of "Unspecified Zone" from "Center" towards left Right for non-parallel slots is ((Opening of Left/Right) * delZoneLeftRightFactorNonPar_nu)
    type.float32 delZoneLeftRightMaxRelToVehWidthNonPar_nu     // Maximum Size of "Unspecified Zone" from "Center" towards left/right for non-parallel Slots
    type.float32 delZoneLeftRightFactorPar_nu     // Size of "Unspecified Zone" from "Center" towards left Right for parallel slots is ((Opening of Left/Right) * delZoneLeftRightFactorPar_nu)
    type.float32 thDelimiterDist_m     // All Delimiter zones are extended outwards (left and right) perpendicular to the slotby delimiterDist.
    type.float32 insideZoneDist_m     // (aligned) Dist between slot and inside zone
    type.float32 minRequiredDelimiterProjectionRatio_nu     // (aligned) the best projection must be at least as good as this factor compared to the second best for clear assignment to a zone
    type.boolean enableInsideObjectPenetrationDepth     // Make objects inside delimiter even if they do not touch inside zone as long as they are not doo deep into the slot
    type.float32 insideObjectMaxProjectionLength_m     // Determine how far an object may reach into the slot to still be considered as <inside> object
    type.boolean vlProjectOntoBoxIfFailed     // If calculation of a VL fails for delimiters than provide a vl projected onto rect parking box instead if min length is met
    type.float32 vlMaxTotalAngle_rad     // Maximum total angles between startingEdge and all vertices in one direction used for line fitting.
    type.float32 vlMaxNextEdgeAngle_rad     // Maximum angle between current edge and edge to next vertex possibly used for line fitting.
    type.float32 vlMinVirtualLineLengthPerpSide_m     // Minimum length of all relevant vertices projected onto the virtual line for a perpendicular left | right delimiter
    type.float32 vlMinVirtualLineLengthParSide_m     // Minimum length of all relevant vertices projected onto the virtual line for a parallel left | right delimiter
    type.float32 vlMinVirtualLineLengthCurb_m     // Minimum length of all relevant vertices projected onto the virtual line for a curb delimiter
    type.float32 vlMinVirtualLineLengthRoad_m     // Minimum length of all relevant vertices projected onto the virtual line for a road delimiter
    type.boolean vlDoRoadInsideOptimization     // If and only if set to true calculate inside VL and road-facing VL for side delimiters in NON-PaS scenarios and generate an optimized VL; Note: overrules vlUseRoadFacingForNonParallelSlots
    type.boolean vlUseRoadFacingForNonParallelSlots     // If set to true MEMM will create VL on road-facing side of an object for NON-PaS; Note: overruled by vlDoRoadInsideOptimization
    type.float32 vlMinLengthStartDirectionSq_m     // When determining the vl algorithm"s start direction the direction most parallel to the related pBox edge is chosen. The first vertex in either direction with this value as a minimum distance is chosen to determine the parallelism.
    type.float32 vlInsideLargerThanRoadFactor     // if this * insideVlUsedForFitting > roadVlUsedForFitting then use inside Vl, otherwise road vl if vlDoRoadInsideOptimization is active
    type.boolean vlUseMaxAngleToParkingBoxEdge     // Only allow a maximum angle of a virtual line to the parking box edge of the parking box rect
    type.float32 vlMaxAngleToParkingBoxEdge_rad     // The maximum angle a virtual line may have to the edge of the parking box rect if vlUseMaxAngleToParkingBoxEdge is set to true
    type.boolean slotExpansion     // Whether to do slotExpansion at all. This will enable expansion on staticObjects.
    type.boolean slotExpansionOnLines     // If SlotExpansion is done, then whether to do it on lines as well
    type.uint8 slotExpFrameCounterStable_nu     // Number of frames a calculated expansion has to be stable to actually take place
    type.float32 slotExpMaxAngleDiff_rad     // Maximum difference of angles for a calculated expansion between frames to still be considered stable
    type.float32 slotExpMaxTriangleExpansion_m     // The maximum expansion for one triangle
    type.float32 slotExpMinTriangleExpansion_rad     // The smallest necessary angle for an expansion to take place.
    type.float32 slotExpansionSideRoiWidth_m     // Width of the roi to check if the slot expansion should orientate itself on vertices in this roi
    type.float32 relevantObjectMinBoxSide_m     // Current immature definition: An object is considered as irrelevant if it fits in a square box with side = this value in ego coordinates
    type.boolean doObjectInflation     // Enable/Disable object inflation
    type.boolean useCovMatrixInflation     // Enable/Disable usage of covariance matrix for object inflation
    type.float32 inflationSafetyDistanceDefault_m     // (aligned?) default value to be used for object inflation
    type.float32 inflationSafetyDistanceParkOut_m     // (aligned?) specific value to be used for object inflation during park out
    type.float32 ncSplitMaxSkippableNCAreaPerStep_m2     // The maximum area that can be skipped per dent in the polygon
    type.boolean ncSplitUseMostXStartIdx     // Whether to use vertex with smallest X as start vertex or default method
    type.boolean ncSplitOn     // Whether the non-convex split feature should be turned off or on
    type.boolean doObjectClustering     // If and only if set to true SI will cluster objects in vicinity of parking slots.
    type.float32 ocPerpMaxDistanceSide_m     // Maximum distance of objects from the top down to still be clustered for side zones
    type.float32 ocPerpMaxDistanceCurb_m     // Maximum distance of objects from the top down to still be clustered for curb zones
    type.float32 ocPerpendicularMaxDistanceXForCurbSideClustering_m     // The maximum distance objects may have (ordered by x direction) to still be clustered for perpendicular slots
    type.float32 ocParallelMaxDistanceXForCurbSideClustering_m     // The maximum distance objects may have (ordered by x direction) to still be clustered for parallel slots
    type.float32 ocAngMaxDistanceCurb_m     // Maximum distance of objects from the top down to still be clustered for curb zones
    type.float32 ocPrimZonePerpSideOpening_m     // The Opening(Width) of a Primary Side(Left/Right) Zone for a Perpendicular slot
    type.float32 ocPrimZoneAngSideOpening_m     // The Opening(Width) of a Primary Side(Left/Right) Zone for an Angular slot
    type.float32 ocPrimZoneParSideOpening_m     // The Opening(Width) of a Primary Side(Left/Right) Zone for a Parallel slot
    type.float32 ocPrimZoneCurbBelowSlot_m     // How far the Curb primary zone gets extended in Curb direction starting from slot curb edge
    type.float32 ocPrimZonePerpCurbIntoSlot_m     // How far the Curb primary zone gets extended in road direction starting from slot curb edge for perpendicular slots
    type.float32 ocPrimZoneAngCurbIntoSlot_m     // How far the Curb primary zone gets extended in road direction starting from slot curb edge for angular slots
    type.float32 ocPrimZoneParCurbIntoSlot_m     // How far the Curb primary zone gets extended in road direction starting from slot curb edge for parallel slots
    type.float32 ocPrimZonePerpSideToRoad_m     // How far the side primary zone gets extended in road direction starting from slot road edge for perpendicular slots
    type.float32 ocPrimZoneParSideToRoad_m     // How far the side primary zone gets extended in road direction starting from slot road edge for parallel slots
    type.float32 ocSecZonePerpSideToSide_m     // The offset of a Secondary Zone to a Perpendicular Slot Primary Side Zone further away from the slot
    type.float32 ocSecZonePerpSideToSlot_m     // The offset of a Secondary Zone to a Perpendicular Slot Primary Side Zone into the slot
    type.float32 ocSecZonePerpSideToRoad_m     // The offset of a Secondary Zone to a Perpendicular Slot Primary Side Zone towards the road
    type.float32 ocSecZonePerpSideToCurb_m     // The offset of a Secondary Zone to a Perpendicular Slot Primary Side Zone towards the curb
    type.float32 ocSecZonePerpCurbToSide_m     // The offset of a Secondary Zone to a Perpendicular Slot Primary Curb Zone towards the Side(Left/Right)
    type.float32 ocSecZonePerpCurbToRoad_m     // The offset of a Secondary Zone to a Perpendicular Slot Primary Curb Zone towards the Road
    type.float32 ocSecZonePerpCurbToCurb_m     // The offset of a Secondary Zone to a Perpendicular Slot Primary Curb Zone towards the curb
    type.float32 ocSecZoneAngSideToSide_m     // The offset of a Secondary Zone to an Angular Slot Primary Side Zone further away from the slot
    type.float32 ocSecZoneAngSideToSlot_m     // The offset of a Secondary Zone to an Angular Slot Primary Side Zone into the slot
    type.float32 ocSecZoneAngSideToRoad_m     // The offset of a Secondary Zone to an Angular Slot Primary Side Zone towards the road
    type.float32 ocSecZoneAngSideToCurb_m     // The offset of a Secondary Zone to an Angular Slot Primary Side Zone towards the curb
    type.float32 ocSecZoneAngCurbToSide_m     // The offset of a Secondary Zone to an Angular Slot Primary Curb Zone towards the Side(Left/Right)
    type.float32 ocSecZoneAngCurbToRoad_m     // The offset of a Secondary Zone to an Angular Slot Primary Curb Zone towards the Road
    type.float32 ocSecZoneAngCurbToCurb_m     // The offset of a Secondary Zone to an Angular Slot Primary Curb Zone towards the curb
    type.float32 ocSecZoneParSideToSide_m     // The offset of a Secondary Zone to a Perpendicular Slot Primary Side Zone further away from the slot
    type.float32 ocSecZoneParSideToSlot_m     // The offset of a Secondary Zone to a Perpendicular Slot Primary Side Zone into the slot
    type.float32 ocSecZoneParSideToRoad_m     // The offset of a Secondary Zone to a Perpendicular Slot Primary Side Zone towards the road
    type.float32 ocSecZoneParSideToCurb_m     // The offset of a Secondary Zone to a Perpendicular Slot Primary Side Zone towards the curb
    type.float32 ocSecZoneParCurbToSide_m     // The offset of a Secondary Zone to a Perpendicular Slot Primary Curb Zone towards the Side(Left/Right)
    type.float32 ocSecZoneParCurbToRoad_m     // The offset of a Secondary Zone to a Perpendicular Slot Primary Curb Zone towards the Road
    type.float32 ocSecZoneParCurbToCurb_m     // The offset of a Secondary Zone to a Perpendicular Slot Primary Curb Zone towards the curb
    type.float32 outsideInAllowedOverlap_m     // Maximum allowed overlap of triangle expansion over objects, perpendicularly from the resulting triangular expansion edge towards most critical vertex of objects
    type.float32 sideAllowedOverlap_m     // Allowed overlap of side ROI over main ROI for Triangle expansion
    type.boolean useAreaCriteria     // Whether to use area criteria to determine which side to expand, or angle criteria
    type.float32 angularCurbRoadAreaFactor     // Factor by which to prefer Road/Curb expansion over Left/Right
    type.float32 prohibitionZoneRelativeLength     // Relative length of a single prohibition zone to the edge of a slot for triangle expansion
    type.boolean useLinearOverlapDropOff     // Whether to use the linear overlap drop off for triangle expansion
    type.float32 vlInsideLargeEnough_m     // If inside.Fitting_length > this value then consider Inside Vl even if inside.Fitting_length < Factor * roadside.Fitting_length
    type.float32 vlMaxAllowedRoadInsideAngleDiff_rad     // if angle between roadSideVl and insideVl is larger than this value (Assuming they should be parallel) then use the roadSideVl
    type.boolean teCalculateRoadSideExpansion     // Whether to do triangular expansion for the road side or not
    type.boolean delZonesUseHighComplexityGetScore     // If and only if set to true MEMM will utilize getScoreHighComplexity() else getScoreLowComplexity()
    type.boolean cnnSlotsEnableSiSlotDetectionVerification     // enables the verification of slot prototypes with CNN based slots
    type.boolean cnnCompareSlotTypeFPSuppression     // If true compare slot types of cnn slots to si detected slots
    type.uint8 cnnSlotsMinScenarioConfidenceTheshold     // the cnn slot type confidence of one scenario has to be this value larger than others to be a distinct Scenario (perp, par, ang)
    type.boolean cnnSlotsPerpendicularValidatesAngular     // If true and enableSlotDetectionVerificationByCnnBasedSlot and cnnCompareSlotTypeFPSuppression are true as well then perpendicular cnn slots validate angular SI slots
    type.float32 cnnSlotsValidationMaxOpeningAngleDiff_rad     // Maximum angle difference that the cnn and SI slot may have for the SI slot to be validated
    type.boolean cnnSlotsEnableCnnSlotPrototypes     // enables CNN based slot prototypes. These prototypes will be added to the internal parking space list.
    type.boolean allowNakedCnnSlots     // Slots linked to a cnn slot may be naked slots.
    type.float32 cnnSlotBlowUpLimitParSide_m     // Maximum distance an object may have to a parallel slot on side to be considered for slot blow up
    type.float32 cnnSlotBlowUpLimitParCurb_m     // Maximum distance an object may have to a parallel slot on the curb side to be considered for slot blow up
    type.float32 cnnSlotRoadAlignmentLimitPar_m     // Maximum distance an object may have to a parallel slot towards the road side to be considered for road edge alignment
    type.float32 cnnSlotBlowUpLimitNonParSide_m     // Maximum distance an object may have to a non-parallel slot on the side to be considered for slot blow up
    type.float32 cnnSlotBlowUpLimitNonParCurb_m     // Maximum distance an object may have to a non-parallel slot on the curb side to be considered for slot blow up
    type.float32 cnnSlotRoadAlignmentLimitNonPar_m     // Maximum distance an object may have to a non-parallel slot towards the road side to be considered for road edge alignment
    type.float32 cnnOrientationErrorCostFunctionFactor     // cnnOrientationError = -(this * angleDiff_rad)^2
    type.boolean cnnSlotsAllowSiSlotOverride     // If an SI Slot was not validated by a CNN Slot still allow the slot if it matches certain criteria
    type.float32 slotScoringMaxParLineLength_m     // SI High If an SI parallel slot was detected on lines, only increase the score if the lines are below max length
    type.float32 slotScoringMinParWingLength_m     // SI High If an SI parallel slot was detected on objects, only increase the score if the object has a WingVlLength of this param or higher
    type.float32 slotScoringMinNonParSideLength_m     // SI High If an SI non parallel slot was detected on objects, only increase the score if the object has a SideVlLength of this param or higher
    type.float32 slotScoringMaxVlOrLineAngleDiff_rad     // SI High If an SI non parallel slot was detected on objects or lines, only increase the score if the respective vl or surrounding lines are at most this angle towards the slot
    type.float32 cnnSlotsMinValidationOverlap_m     // How much a cnn slot and a SI Slot have to overlap for the cnn slot to be able to validate the SI Slot
    type.float32 cnnSlotsMinInvalidationOverlap_m     // how much a cnn slot and a SI Slot have to overlap for the cnn slot to be able to invalidate the SI Slot
    type.float32 minDistGapCnnSlotVsTrackedSlot_m     // Distance threshold to calculate if the cnn slot is closer than any slot currently in parkingSpaceList if it is full. Goal is to avoid toggling between to CNN Slot Based slots.
    type.float32 cnnSlotOdSlotEnlargement_m     // Enlarging the opening of od only slots to make sure tapos fits.
    type.float32 memParkExtendSlot_m     // Extension of slot compared to vehicle"s bounding box in all directions
    type.float32 leftRightMemParkOffsetX_m     // Initial slot position is centered in x-direction at ego vehicle"s center. This value will adjust the x-position relatively (ego vehicle coordinate system).
    type.float32 leftRightMemParkGapY_m     // Gap between ego vehicle and slot at right or left side of the ego vehicle
    type.float32 frontMemParkGap_m     // Gap between ego vehicle and slot in front of the ego vehicle
    type.float32 rearMemParkGap_m     // Gap between ego vehicle and slot rear to the ego vehicle
    type.float32 maxMemParkRotAdj_rad     // In user-defined slots, the maximum allowed abs rotation of the user adjustment.
    type.float32 maxMemParkLinearAdj_m     // In user-defined slots, the maximum allowed linear translation of the user adjustment.
    type.boolean preapplyRoadsideExtension     // If set to true, the maximum allowed road side extension shall be applied to parking spaces, if possible.
    type.float32 safetyMarginDynObjEnlargingPed_m     // Safety margin of dynamic object enlarging for pedestrians
    type.float32 safetyMarginDynObjEnlargingCar_m     // Safety margin of dynamic object enlarging for cars
    type.uint8 minHeightConfDeltaStaticObj     // HIGH ONLY | Minimum height confidence delta threshold of static ojbects, which used to decide that the object is body traversable or high.
    type.float32 developerGenericFloat0     // Dummy floats for "experimental" features
    type.float32 developerGenericFloat1     // Dummy floats for "experimental" features
    type.float32 developerGenericFloat2     // Dummy floats for "experimental" features
    type.float32 developerGenericFloat3     // Dummy floats for "experimental" features
    type.float32 developerGenericFloat4     // Dummy floats for "experimental" features
    type.float32 developerGenericFloat5     // Dummy floats for "experimental" features
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.SiParams_InterfaceVersion uml:Class
  Members:
    type.uint32 SiParams_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.PB_Side uml:Enumeration
    // @brief Describes on which side and how is the parking box positioned.
  Members:
    PB_SIDE_LEFT = 0            // @brief The parking box is on the left side.
    PB_SIDE_RIGHT = 1            // @brief The parking box is on the right side.
    PB_SIDE_CENTER = 2            // @brief The parking box is in front or behind.
    PB_SIDE_RIGHT_ANGLED_OPENING_TOWARDS_BACK = 3            // @brief The angled parking box is on the right side facing towards the back.
    PB_SIDE_RIGHT_ANGLED_OPENING_TOWARDS_FRONT = 4            // @brief The angled parking box is on the right side facing towards the front.
    PB_SIDE_LEFT_ANGLED_OPENING_TOWARDS_BACK = 5            // @brief The angled parking box is on the left side facing towards the back.
    PB_SIDE_LEFT_ANGLED_OPENING_TOWARDS_FRONT = 6            // @brief The angled parking box is on the left side facing towards the front.
    MAX_NUM_PB_SIDE = 7            // @brief Placeholder at the end of the enumeration, it defines the maximum value of the enumeration.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.SlotStatus uml:Enumeration
    // @brief Describes the possible states of a slot.
  Members:
    UNKNOWN = 0            // @brief The state of the slot is unknown.
    VALID = 1            // @brief The slot is a valid slot.
    INVALID_OPENING = 2            // @brief The slot opening does not match any defined prototype.
    INVALID_WINDOW = 3            // @brief The area of the slot is invalid due to some reason (e.g. too many lines overlapping with it).
    NOT_MATCHING_WINDOW = 4            // @brief Slot area does not match the slot.
    SLOT_ALREADY_KNOWN = 5            // @brief The slot defines and already managed slot.
    SLOT_INVALIDATED_BY_OVERLAP_WITH_HIGHER_PRIO = 6            // @brief Slot overlaps some higher priority object.
    SIDE_OR_CURB_OBJECTS_MISSING = 7            // @brief An expected side or curb object is missing.
    TOO_MANY_ITERATIONS_IN_SHRINKING = 8            // @brief Too many iterations occurring during slot shrinking.
    NO_VALID_SHIFT_VECTOR_FOUND = 9            // @brief No valid shift vector is found during slot shrinking.
    SLOT_TOO_SMALL_AFTER_SHRINKING = 10            // @brief Slot becomes too small after shrinking.
    SLOT_INVALID_BY_DEF = 11            // @brief Slot definition is invalid.
    SLOT_PARALLEL_WINDOW_VALID = 12            // @brief Opening classified as parallel.
    SLOT_PARALLEL_WINDOW_INVALID = 13            // @brief Parallel slot with invalid area.
    SLOT_PERP_WINDOW_VALID = 14            // @brief Opening classified as perpendicular.
    SLOT_PERP_WINDOW_INVALID = 15            // @brief Perpendicular slot with invalid area.
    SLOT_GARAGE_WINDOW_VALID = 16            // @brief Opening classified as garage parking slot.
    SLOT_GARAGE_WINDOW_INVALID = 17            // @brief Garage parking slot with invalid area.
    SLOT_ANGLED_WINDOW_VALID = 18            // @brief Opening classified as angled slot.
    SLOT_ANGLED_WINDOW_INVALID = 19            // @brief Angular slot with invalid area.
    SLOT_NOT_RECTANGULAR = 20            // @brief Parking slot is not rectangular.
    SLOT_OPENING_TOO_SHORT = 21            // @brief Parking slot is not long enough.
    SLOT_OPENING_TOO_LONG = 22            // @brief Parking slot is too long.
    SLOT_DEPTH_TOO_SHORT = 23            // @brief Parking slot is not deep enough.
    SLOT_DEPTH_TOO_LONG = 24            // @brief Parking slot is too deep.
    SLOT_ORIENTATION_OVER_THRESHOLD = 25            // @brief Invalid parking slot because its orientation changed over the threshold.
    SIDE_DELIMITER_INVALID = 26            // @brief Slot has an invalid delimiter object.
    SI_LOW_MINI_EM_OVERFLOW = 27            // @brief Mini EM cannot handle the slot anymore.
    SLOT_NOT_OVERLAPPING_EGO = 28            // @brief If the slot is ready and not containing the ego shape, then it is not valid.
    SLOT_ROTATED_TOO_FAR = 29            // @brief Kill slot whose rotation exceeds a limit.
    SLOT_NOT_VALIDATED_BY_CNN_SLOT = 30            // @brief Slot does not have a CNN counterpart.
    SLOT_IS_INVALIDATED_BY_CNN_SLOT = 31            // @brief Slot overlaps too much with an already managed CNN slot.
    SLOT_EXTENDED_BEYOND_LINE_TOO_FAR = 32            // @brief Slot extends too much beyond the side delimiter line.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.LineSegmentSerializable uml:Class
  version: ::si::LineSegmentSerializable_InterfaceVersion::LineSegmentSerializable_VERSION
    // @brief A structure describing a line segment with its two end vertices.
  Members:
    type.LSM_GEOML.size_type actualSize     // @range{0,2};@unit{nu};@brief Describes how many vertices were already added to the line segment.
    type.cml.Vec2Df_POD array     // @brief Array containing the vertices.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.LineSegmentSerializable_InterfaceVersion uml:Class
  Members:
    type.uint32 LineSegmentSerializable_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.LineSegmentSerializableArray uml:Class
  version: ::si::LineSegmentSerializableArray_InterfaceVersion::LineSegmentSerializableArray_VERSION
    // @brief Array of line segments.
  Members:
    type.LSM_GEOML.size_type actualSize     // @range{0,AP_Common.AP_COMMON_TYPES_Consts.AP_G_MAX_NUM_PARKING_BOXES_NU};@unit{nu};@brief Describes how many line segments were already added to the array.
    type.SI.LineSegmentSerializable array     // @brief Array containing the line segments.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.LineSegmentSerializableArray_InterfaceVersion uml:Class
  Members:
    type.uint32 LineSegmentSerializableArray_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.TriangleSerializable uml:Class
  version: ::si::TriangleSerializable_InterfaceVersion::TriangleSerializable_VERSION
    // @brief A structure describing a triangle with its vertices.
  Members:
    type.LSM_GEOML.size_type actualSize     // @range{0,3};@unit{nu};@brief Describes how many vertices were already added to the triangle.
    type.cml.Vec2Df_POD array     // @brief Array containing the vertices.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.TriangleSerializable_InterfaceVersion uml:Class
  Members:
    type.uint32 TriangleSerializable_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.TriangleSerializableArray uml:Class
  version: ::si::TriangleSerializableArray_InterfaceVersion::TriangleSerializableArray_VERSION
    // @brief Array of triangles.
  Members:
    type.LSM_GEOML.size_type actualSize     // @range{0,AP_Common.AP_COMMON_TYPES_Consts.AP_G_MAX_NUM_PARKING_BOXES_NU};@unit{nu};@brief Describes how many triangles were already added to the array.
    type.SI.TriangleSerializable array     // @brief Array containing the triangles.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.TriangleSerializableArray_InterfaceVersion uml:Class
  Members:
    type.uint32 TriangleSerializableArray_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.QuadrilateralSerializable uml:Class
  version: ::si::QuadrilateralSerializable_InterfaceVersion::QuadrilateralSerializable_VERSION
    // @brief A structure describing a shape with four vertices.
  Members:
    type.LSM_GEOML.size_type actualSize     // @range{0,4};@unit{nu};@brief Describes how many vertices were already added to the quadrilateral.
    type.cml.Vec2Df_POD array     // @brief Array containing the vertices.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.QuadrilateralSerializable_InterfaceVersion uml:Class
  Members:
    type.uint32 QuadrilateralSerializable_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.QuadrilateralSerializableArray uml:Class
  version: ::si::QuadrilateralSerializableArray_InterfaceVersion::QuadrilateralSerializableArray_VERSION
    // @brief Array of quadrilaterals.
  Members:
    type.LSM_GEOML.size_type actualSize     // @range{0,AP_Common.AP_COMMON_TYPES_Consts.AP_G_MAX_NUM_PARKING_BOXES_NU};@unit{nu};@brief Describes how many quadrilaterals were already added to the array.
    type.SI.QuadrilateralSerializable array     // @brief Array containing the quadrilaterals.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.QuadrilateralSerializableArray_InterfaceVersion uml:Class
  Members:
    type.uint32 QuadrilateralSerializableArray_VERSION = 1    
----------------------------------------------------------------------------------------------------------













  # ID:  type.SI.CorePlotData uml:Class
  version: ::si::CorePlotData_InterfaceVersion::CorePlotData_VERSION
    // @brief SI Core Plot Data contains the debug data needed for plotting core features.
  Members:
    type.SI.PullCornerRectSerializable pullRects     // @brief Array of pull corner rectangles.
    type.SI.PullCornerPolySerializable pullPolys     // @brief Array of pull corner polygons.
    type.SI.QuadrilateralSerializable parkingSlotUpdateRoi     // @brief " TODO: Write the correct brief documentation.
    type.SI.QuadrilateralSerializable parkingSlotTrackingRoi     // @brief " TODO: Write the correct brief documentation.
    type.SI.CoreSlotPrototype perpendicular     // @brief Describes a parking slot prototype in case of perpendicular parking.
    type.SI.CoreSlotPrototype parallel     // @brief Describes a parking slot prototype in case of parallel parking.
    type.SI.CoreSlotPrototype garage     // @brief Describes a parking slot prototype in case of garage parking.
    type.SI.QuadrilateralSerializableArray rawGpParkingSlots     // @brief Array of raw parking boxes in a garage parking scenario.
    type.SI.QuadrilateralSerializableArray diagonalActual     // @brief Array of detected angular slots.
    type.SI.QuadrilateralSerializableArray positionManagerInitialSlots     // @brief Array of initial slots in ego coordinates.
    type.SI.QuadrilateralSerializableArray positionManagerRois     // @brief Array of RoIs in ego coordinates.
    type.SI.QuadrilateralSerializableArray positionManagerAdjustedRois     // @brief Array of adjusted RoIs in ego coordinates.
    type.SI.QuadrilateralSerializableArray blowUpManagerRoiLeft     // @brief Array of left side RoIs in ego coordinates.
    type.SI.QuadrilateralSerializableArray blowUpManagerAdjustedRoiLeft     // @brief Array of left side adjusted RoIs in ego coordinates.
    type.SI.QuadrilateralSerializableArray blowUpManagerRoiRight     // @brief Array of right side RoIs in ego coordinates.
    type.SI.QuadrilateralSerializableArray blowUpManagerAdjustedRoiRight     // @brief Array of right side adjusted RoIs in ego coordinates.
    type.SI.QuadrilateralSerializableArray blowUpManagerRoiCurb     // @brief Array of curb side RoIs in ego coordinates.
    type.SI.QuadrilateralSerializableArray blowUpManagerAdjustedRoiCurb     // @brief Array of curb side adjusted RoIs in ego coordinates.
    type.SI.OptimizationResultSerializable collisionManagerResult     // @brief Array containing the optimization results.
    type.SI.LineSegmentSerializableArray slotOpeningBeforeExtension     // @brief Array containing the slot openings before extension.
    type.uint16 selectedSlot_nu     // @range{0,65535};@unit{nu};@brief ID of the selected parking box.
    type.SI.QuadrilateralSerializable sensingArea     // @brief The bounding box which is the perimeter of the sensing Region Of Interest.
    type.SI.QuadrilateralSerializable roiLeft     // @brief Adjusted detection zone for left side scanning.
    type.SI.QuadrilateralSerializable roiRight     // @brief Adjusted detection zone for right side scanning.
    type.SI.QuadrilateralSerializable roiFront     // @brief Adjusted detection zone for front side scanning.
    type.SI.QuadrilateralSerializable roiLeftCore     // @brief Initial detection zone for left side scanning before adjustment.
    type.SI.QuadrilateralSerializable roiRightCore     // @brief Initial detection zone for right side scanning before adjustment.
    type.SI.NewDetectedSlotCandidateSerializable newDetectedSlotCandidate     // @brief Array of newly detected slot candidates.
    type.SI.ParkingScenarioTypes lastEstimatedScenario_nu     // @range{0,SI.ParkingScenarioTypes.MAX_NUM_PARKING_SCENARIO_TYPES};@unit{nu};@brief Type of the last estimated parking scenario.
    type.SI.ParkingScenarioSideTypes lastEstimatedScenarioSide_nu     
    type.SI.Uint16Serializable irrelevantObjId     // @brief Array containing the IDs of irrelevant objects.
    type.SI.Sint32Serializable scores     // @brief Array of performance scores of the slots.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.PullCornerRect uml:Class
    // @brief Corner pulling rectangles.
  Members:
    type.SI.QuadrilateralSerializable fl     // @brief Axis aligned bounding box around rectangular front left corner RoI.
    type.SI.QuadrilateralSerializable fr     // @brief Axis aligned bounding box around rectangular front right corner RoI.
    type.SI.QuadrilateralSerializable curb     // @brief Axis aligned bounding box around the RoI for curb alignment.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.PullCornerRectSerializable uml:Class
    // @brief Pull corner rectangles serializable.
  Members:
    type.LSM_GEOML.size_type actualSize     // @range{0,AP_Common.AP_COMMON_TYPES_Consts.AP_G_MAX_NUM_PARKING_BOXES_NU};@unit{nu};@brief Describes how many pull corner rectangles were already added to the array.
    type.SI.PullCornerRect array     // @brief Array of pull corner rectangles.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.PullCornerPoly uml:Class
    // @brief Corner pulling polygon.
  Members:
    type.SI.QuadrilateralSerializable fl     // @brief Rectangular RoI centered around the front left corner of the slot.
    type.SI.QuadrilateralSerializable fr     // @brief Rectangular RoI centered around the front right corner of the slot.
    type.SI.QuadrilateralSerializable curb     // @brief RoI for curb alignment.
    type.boolean flOn_nu     // @unit{nu};@brief Defines whether front left RoI is on or not.
    type.boolean frOn_nu     // @unit{nu};@brief Axis aligned bounding box around the RoI for curb alignment.
    type.float32 flDist_m     // @range{0,3.4028237e+38};@unit{m};@brief Distance from the front left RoI.
    type.float32 frDist_m     // @range{0,3.4028237e+38};@unit{m};@brief Distance from the front right RoI.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.PullCornerPolySerializable uml:Class
    // @brief Pull corner polygon serializable.
  Members:
    type.LSM_GEOML.size_type actualSize     // @range{0,AP_Common.AP_COMMON_TYPES_Consts.AP_G_MAX_NUM_PARKING_BOXES_NU};@unit{nu};@brief Describes how many pull corner polygons were already added to the array.
    type.SI.PullCornerPoly array     // @brief Array of pull corner polygons.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.SlotDimension uml:Class
    // @brief Structure containing the minimum, typical and maximum of a given dimension.
  Members:
    type.float32 min_m     // @range{0,100};@unit{m};@brief Minimum value for the dimension.
    type.float32 typical_m     // @range{0,100};@unit{m};@brief Typical value for the dimension.
    type.float32 max_m     // @range{0,100};@unit{m};@brief Maximum value for the dimension.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.CoreSlotPrototype uml:Class
    // @brief Geometry of a slot prototype.
  Members:
    type.boolean enabled_nu     // @unit{nu};@brief Switch to enable or disable this slot type.
    type.SI.SlotDimension length_m     // @unit{m};@brief Length (longer side) information.
    type.SI.SlotDimension width_m     // @unit{m};@brief Width (shorter side) information.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.NewDetectedSlotCandidate uml:Class
    // @brief Newly detected slot candidate.
  Members:
    type.cml.Vec2Df_POD start     // @brief One side of the opening.
    type.cml.Vec2Df_POD end     // @brief Other side of the opening.
    type.SI.PB_Side side     // @brief The position of the new slot candidate.
    type.SI.SlotStatus status     // @brief The status of the new slot candidate.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.NewDetectedSlotCandidateSerializable uml:Class
    // @brief New detected slot candidate serializable.
  Members:
    type.LSM_GEOML.size_type actualSize     // @range{0,25};@unit{nu};@brief Describes how many new detected slot candidates were already added to the array.
    type.SI.NewDetectedSlotCandidate array     // @brief Array of new detected slot candidates.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.Uint16Serializable uml:Class
    // @brief Unsigned 16bit integer serializable.
  Members:
    type.LSM_GEOML.size_type actualSize     // @range{0,AP_Common.AP_COMMON_TYPES_Consts.AP_G_MAX_NUM_STATIC_OBJ_NU};@unit{nu};@brief Describes how many unsigned 16bit integers were already added to the array.
    type.uint16 array     // @brief Array of unsigned 16bit integers.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.Sint32Serializable uml:Class
    // @brief Signed 32bit integer serializable.
  Members:
    type.LSM_GEOML.size_type actualSize     // @range{0,AP_Common.AP_COMMON_TYPES_Consts.AP_G_MAX_NUM_PARKING_BOXES_NU};@unit{nu};@brief Describes how many signed 32bit integers were already added to the array.
    type.sint32 array     // @brief Array of signed 32bit integers.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.OptimizationResult uml:Class
    // @brief Structure containing the optional optimization result.
  Members:
    type.boolean valid_nu     // @unit{nu};@brief Describes whether the optimization result is valid or not.
    type.SI.QuadrilateralSerializable shape     // @brief End shape after the optimization.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.OptimizationResultSerializable uml:Class
    // @brief Optimization result serializable.
  Members:
    type.LSM_GEOML.size_type actualSize     // @range{0,AP_Common.AP_COMMON_TYPES_Consts.AP_G_MAX_NUM_PARKING_BOXES_NU};@unit{nu};@brief Describes how many optimization results were already added to the array.
    type.SI.OptimizationResult array     // @brief Array of optimization results.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.CorePlotData_InterfaceVersion uml:Class
  Members:
    type.uint32 CorePlotData_VERSION = 1    
----------------------------------------------------------------------------------------------------------











  # ID:  type.SI.LowPlotData uml:Class
  version: ::si::LowPlotData_InterfaceVersion::LowPlotData_VERSION
    // @brief SI Low Plot Data contains the debug data needed for plotting low features.
  Members:
    type.SI.QuadrilateralSerializableArray shrinkingObjectLeftRoi     // @brief Array of left object RoI. If object overlaps this, the slot is NOT shrunk (we assume that the area was sensed correctly).
    type.SI.QuadrilateralSerializableArray shrinkingObjectRightRoi     // @brief Array of right object RoI. If object overlaps this, the slot is NOT shrunk (we assume that the area was sensed correctly).
    type.SI.TriangleSerializableArray shrinkingEgoLeftRoi     // @brief Array of left ego RoI. If the ego overlaps that area, the slot is NOT shrunk (we assume that the ego is close enough to sense the relevant area correctly).
    type.SI.TriangleSerializableArray shrinkingEgoRightRoi     // @brief Array of right ego RoI. If the ego overlaps that area, the slot is NOT shrunk (we assume that the ego is close enough to sense the relevant area correctly).
    type.SI.QuadrilateralSerializableArray slotsBeforePostprocessing     // @brief Array of slots before postprocessing.
    type.SI.QuadrilateralSerializableArray slotsAfterPostprocessing     // @brief Array of slots after postprocessing.
    type.SI.QuadrilateralSerializableArray perSlotSubEmRoi     // @brief Array of per-slot sub-em RoIs.
    type.SI.QuadrilateralSerializableArray perSlotAboveSubEmRoi     // @brief Array of current maneuvering RoIs.
    type.SI.QuadrilateralSerializableArray perSlotFollowROI     // @brief Array of plot per-slot follow-em RoIs (used for slot entrance ROIs because follow-em ROI doesn"t exist any more).
    type.SI.QuadrilateralSerializableArray perSlotOncomingROI     // @brief Array of per-slot oncoming-em RoIs.
    type.SI.EgoShapePolygonSerializable inflatedEgoShape     // @brief Array of vertices corresponding to the inflated shape of the ego vehicle.
    type.SI.TransformedScenarioSerializable slot1InSlotCoordinates     // @brief Array of aggregate information related to the different steps of optimization.
    type.SI.QuadrilateralSerializable dynObjClassifierROI     // @brief Dynamic object classification RoI.
    type.SI.QuadrilateralSerializable dynObjClassifierROI2     // @brief Dynamic object classification RoI for overhead beams.
    type.SI.SlotCostFunDataPerSlotSerializable slotCostFunctionData     // @brief Array of slot cost function data for slots.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.EgoShapePolygonSerializable uml:Class
    // @brief Array of vertices.
  Members:
    type.LSM_GEOML.size_type actualSize     // @range{0,AP_Common.AP_COMMON_TYPES_Consts.AP_V_VEHICLE_SHAPE_MAX_SIZE_NU};@unit{nu};@brief Describes how many vertices were already added to the array.
    type.cml.Vec2Df_POD array     // @brief Array containing the vertices.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.ObjectShapeSerializable uml:Class
    // @brief Array of vertices representing an object.
  Members:
    type.LSM_GEOML.size_type actualSize     // @range{0,AP_Common.AP_COMMON_TYPES_Consts.AP_G_MAX_NUM_PTS_STATIC_POLY_NU};@unit{nu};@brief Describes how many vertices were already added to the array.
    type.cml.Vec2Df_POD array     // @brief Array containing the vertices.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.ObjectShapeSerializableArray uml:Class
    // @brief Array of objects.
  Members:
    type.LSM_GEOML.size_type actualSize     // @range{0,AP_Common.AP_COMMON_TYPES_Consts.COLL_G_MAX_NUM_STATIC_OBJ_NU};@unit{nu};@brief Describes how many objects were already added to the array.
    type.SI.ObjectShapeSerializable array     // @brief Array containing the objects.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.TransformedScenario uml:Class
    // @brief Aggregate structure containing relevant information about the transformed parking slots.
  Members:
    type.SI.QuadrilateralSerializable parkingSlotBefore     // @brief Parking slot before transformation.
    type.SI.QuadrilateralSerializable parkingSlotEnlarged     // @brief Enlarged parking slot.
    type.SI.QuadrilateralSerializable parkingSlotAfter     // @brief Parking slot after transformation.
    type.SI.ObjectShapeSerializableArray objects     // @brief Array of object related to a parking slot.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.TransformedScenarioSerializable uml:Class
    // @brief Array of aggregated transformed scenario information.
  Members:
    type.LSM_GEOML.size_type actualSize     // @range{0,3};@unit{nu};@brief Describes how many aggregate information were already added to the array.
    type.SI.TransformedScenario array     // @brief Array containing the aggregate information.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.SlotCostFunDataPerOptPhase uml:Class
  Members:
    type.SI.QuadrilateralSerializable roiLeft     // @brief Rectangular left side RoI.
    type.SI.QuadrilateralSerializable roiRight     // @brief Rectangular right side RoI.
    type.SI.QuadrilateralSerializable roiCurb     // @brief RoI for curb alignment.
    type.SI.VirtualLineSerializableArray virtualLines     // @brief Array of virtual lines.
    type.SI.VirtualLineIndexSerializable bestVLIndices     // @brief Index array for the best virtual lines.
    type.float32 curbAlignment_deg     // @range{0,90};@unit{°};@brief Rotation angle between the parking slot and the curb virtual line.
    type.float32 curbAlignmentPerf_nu     // @range{-90° * SI_params.weightCurbOrientation,0};@unit{nu};@brief Alignment performance.
    type.float32 closestFL_m     // @range{0,3.4028237e+38};@unit{m};@brief Distance to the closest object from the front left corner of the parking slot.
    type.float32 closestFR_m     // @range{0,3.4028237e+38};@unit{m};@brief Distance to the closest object from the front right corner of the parking slot.
    type.float32 pullPerformFront_nu     // @range{-3.4028237e+38,3.4028237e+38};@unit{nu};@brief Pull performance. Currently only high objects at opening are targeted.
    type.float32 opening_m     // @range{0,3.4028237e+38};@unit{m};@brief Size of the parking slot opening.
    type.float32 openingPerf_nu     // @range{1.175494e-38,3.4028237e+38};@unit{nu};@brief The cost function that can be adjusted to control the slot rotation optimization.
    type.float32 sideAlignmentError_deg     // @range{-90,90};@unit{°};@brief Side alignment error in degrees. Rotation angle between the side virtual line and the road normal vector. 0° means perfect alignment while +/-90° means total misalignment or no side object to align to.
    type.float32 sideAlignmentPerf_m     // @range{-0.5F * π * SI_params.sideAlignmentWeight_mpr,1.5F * π * SI_params.sideAlignmentWeight_mpr};@unit{m};@brief Side alignment performance.
    type.float32 wingAlignmentError_deg     // @range{-90,90};@unit{°};@brief Wing alignment error in degrees. Rotation angle between the wing virtual line and the road side edge. 0° means perfect alignment while +/-90° means total misalignment or no wing object to align to.
    type.float32 wingAlignmentPerf_m     // @range{-0.5F * π * SI_params.wingAlignmentWeight_mpr,1.5F * π * SI_params.wingAlignmentWeight_mpr};@unit{m};@brief Wing alignment performance.
    type.float32 overallPerf_m     // @range{-3.4028237e+38,3.4028237e+38};@unit{m};@brief Combined performance based on all cost function components.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.VirtualLineSerializableArray uml:Class
    // @brief Array of line segments.
  Members:
    type.LSM_GEOML.size_type actualSize     // @range{0,16};@unit{nu};@brief Describes how many line segments were already added to the array.
    type.SI.LineSegmentSerializable array     // @brief Array containing the line segments.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.VirtualLineIndexSerializable uml:Class
    // @brief Array of virtual line indices.
  Members:
    type.LSM_GEOML.size_type actualSize     // @range{0,16};@unit{nu};@brief Describes how many virtual line indices were already added to the array.
    type.LSM_GEOML.size_type array     // @brief Array containing the virtual line indices.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.SlotCostFunDataPerSlot uml:Class
    // @brief Array of slot cost function data for every optimization phase.
  Members:
    type.LSM_GEOML.size_type actualSize     // @range{0,3};@unit{nu};@brief Describes how many slot cost function data were already added to the array.
    type.SI.SlotCostFunDataPerOptPhase array     // @brief Array containing the slot cost function data.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.SlotCostFunDataPerSlotSerializable uml:Class
    // @brief Array of slot cost function data for parking slots.
  Members:
    type.LSM_GEOML.size_type actualSize     // @range{0,AP_Common.AP_COMMON_TYPES_Consts.AP_G_MAX_NUM_PARKING_BOXES_NU};@unit{nu};@brief Describes how many slot cost function data were already added to the array.
    type.SI.SlotCostFunDataPerSlot array     // @brief Array containing the slot cost function data.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.LowPlotData_InterfaceVersion uml:Class
  Members:
    type.uint32 LowPlotData_VERSION = 1    
----------------------------------------------------------------------------------------------------------














  # ID:  type.SI.HighPlotData uml:Class
  version: ::si::HighPlotData_InterfaceVersion::HighPlotData_VERSION
    // @brief SI High Plot Data contains the debug data needed for plotting high features.
  Members:
    type.SI.DelimiterZonesSerializable delimiterZones     // @brief Array of aggregated delimiter zones.
    type.SI.SlotSurroundingLinesSerializable surroundingLinesPerSlot     // @brief Array of lines surrounding slots.
    type.SI.VirtualLineInfoSerializable virtualLinesInfo     // @brief Array of virtual line information.
    type.SI.TriangleExpansionSerializable triangleExpansionRois     // @brief Array containing the triangular side expansion RoIs.
    type.SI.QuadrilateralExpansionSerializable sideExpansionRois     // @brief Array containing the rectangular side expansion RoIs.
    type.SI.QuadrilateralSerializableArray cnnBlowUpLimits     // @brief Array of blow up rectangles, which describe how far CNN slots may be blown up.
    type.SI.QuadrilateralSerializableArray slotBlowUpLimits     // @brief Array of blow up rectangles, which describe how far slots may be blown up.
    type.SI.QuadrilateralSerializableArray rectSlots     // @brief Array of rectangular slots.
    type.SI.HighClusterZoneSerializable clusterZones     // @brief Array of cluster zones.
    type.SI.CnnBasedParkingSlotSerializable cnnBasedParkingSlots     // @brief Array of CNN based parking slots.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.DelimiterZones uml:Class
    // @brief Structure containing the different delimiting bounding boxes.
  Members:
    type.SI.QuadrilateralSerializable curbZone     // @brief Bounding box describing the curb zone.
    type.SI.QuadrilateralSerializable roadZone     // @brief Bounding box describing the road zone.
    type.SI.QuadrilateralSerializable leftZone     // @brief Bounding box describing the left zone.
    type.SI.QuadrilateralSerializable rightZone     // @brief Bounding box describing the right zone.
    type.SI.QuadrilateralSerializable insideZone     // @brief Bounding box describing the inside zone.
    type.SI.QuadrilateralSerializable all     // @brief Bounding box for summation of all of the above.
    type.uint8 slotId_nu     // @range{0,255};@unit{nu};@brief ID of the specific slot.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.DelimiterZonesSerializable uml:Class
    // @brief Optimization result serializable.
  Members:
    type.LSM_GEOML.size_type actualSize     // @range{0,AP_Common.AP_COMMON_TYPES_Consts.AP_G_MAX_NUM_PARKING_BOXES_NU};@unit{nu};@brief Describes how many optimization results were already added to the array.
    type.SI.DelimiterZones array     // @brief Array of optimization results.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.SlotSurroundingLines uml:Class
    // @brief Aggregate structure for the lines surrounding the slot.
  Members:
    type.SI.LineSegmentSerializable leftLine     // @brief Describes a surrounding line on the left side.
    type.SI.LineSegmentSerializable rightLine     // @brief Describes a surrounding line on the right side.
    type.SI.LineSegmentSerializable roadLine     // @brief Describes a surrounding line on the road side.
    type.SI.LineSegmentSerializable curbLine     // @brief Describes a surrounding line on the curb side.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.SlotSurroundingLinesSerializable uml:Class
    // @brief Slot surrounding lines serializable.
  Members:
    type.LSM_GEOML.size_type actualSize     // @range{0,AP_Common.AP_COMMON_TYPES_Consts.AP_G_MAX_NUM_PARKING_BOXES_NU};@unit{nu};@brief Describes how many surrounding lines were already added to the array.
    type.SI.SlotSurroundingLines array     // @brief Array of slot surrounding lines.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.VirtualLineInfo uml:Class
    // @brief Aggregating information relevant for virtual lines in one structure.
  Members:
    type.LSM_GEOML.size_type parkingSlotId_nu     // @range{0,65535};@unit{nu};@brief Describes to which parking slot does the virtual line belong.
    type.LSM_GEOML.size_type delimiterId_nu     // @range{0,65535};@unit{nu};@brief Describes to which delimiter does the virtual line belong.
    type.LSM_GEOML.size_type staticStructureId_nu     // @range{0,65535};@unit{nu};@brief Describes to which static structure does the virtual line belong.
    type.LSM_GEOML.size_type virtualLineId_nu     // @range{0,65535};@unit{nu};@brief Defines the ID of the virtual line.
    type.LSM_GEOML.size_type startVertex_nu     // @range{0,65535};@unit{nu};@brief Defines the ID of the virtual line"s starting vertex.
    type.LSM_GEOML.size_type endVertex_nu     // @range{0,65535};@unit{nu};@brief Defines the ID of the virtual line"s starting vertex.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.VirtualLineInfoSerializable uml:Class
    // @brief Virtual line information serializable.
  Members:
    type.LSM_GEOML.size_type actualSize     // @range{0,AP_Common.AP_COMMON_TYPES_Consts.AP_G_MAX_NUM_PARKING_BOXES_NU};@unit{nu};@brief Describes how many virtual line information were already added to the array.
    type.SI.VirtualLineInfo array     // @brief Array of virtual line information.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.TriangleExpansionSerializable uml:Class
    // @brief Array of side expansions.
  Members:
    type.LSM_GEOML.size_type actualSize     // @range{0,AP_Common.AP_COMMON_TYPES_Consts.AP_G_MAX_NUM_PARKING_BOXES_NU};@unit{nu};@brief Describes how many side expansions were already added to the array.
    type.SI.TriangleSerializableArray array     // @brief Array containing the side expansions.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.QuadrilateralExpansionSerializable uml:Class
    // @brief Array of side expansions.
  Members:
    type.LSM_GEOML.size_type actualSize     // @range{0,AP_Common.AP_COMMON_TYPES_Consts.AP_G_MAX_NUM_PARKING_BOXES_NU};@unit{nu};@brief Describes how many side expansions were already added to the array.
    type.SI.QuadrilateralSerializableArray array     // @brief Array containing the side expansions.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.HighClusterZone uml:Class
    // @brief Aggregate structure for primary and secondary zones.
  Members:
    type.SI.QuadrilateralSerializable zonePrimary     // @brief Primary zone.
    type.SI.QuadrilateralSerializable zoneSecondary     // @brief Secondary zone.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.HighClusterZoneSerializable uml:Class
    // @brief Array of cluster zones.
  Members:
    type.LSM_GEOML.size_type actualSize     // @range{0,AP_Common.AP_COMMON_TYPES_Consts.AP_G_MAX_NUM_PARKING_BOXES_NU};@unit{nu};@brief Describes how many cluster zones were already added to the array.
    type.SI.HighClusterZone array     // @brief Array containing the cluster zones.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.ParkingScenarioConfidence uml:Class
    // @brief The confidence values for the different parking scenarios. The sum of the different confidence might not add up to 100%.
  Members:
    type.uint8 perpendicular_perc     // @range{0,100};@unit{%};@brief Defines the confidence in the parking scenario being perpendicular.
    type.uint8 parallel_perc     // @range{0,100};@unit{%};@brief Defines the confidence in the parking scenario being parallel.
    type.uint8 angled_perc     // @range{0,100};@unit{%};@brief Defines the confidence in the parking scenario being angled.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.CnnBasedParkingSlot uml:Class
    // @brief Aggregate structure containing all CNN based parking slot related information.
  Members:
    type.uint16 slotId_nu     // @range{0,65535};@unit{nu};@brief Defines the ID of the corresponding slot.
    type.SI.QuadrilateralSerializable slotShape_m     // @unit{m};@brief Describes the shape of the corresponding parking slot.
    type.uint8 existenceProb_perc     // @range{0,100};@unit{%};@brief Defines the existence probability of the corresponding slot.
    type.SI.ParkingScenarioConfidence parkingScenarioConfidence_perc     // @unit{%};@brief Defines the confidence in the different types of parking scenarios for the corresponding slot.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.CnnBasedParkingSlotSerializable uml:Class
    // @brief Array of CNN based parking slots.
  Members:
    type.LSM_GEOML.size_type actualSize     // @range{0,24};@unit{nu};@brief Describes how many CNN based parking slots were already added to the array.
    type.SI.CnnBasedParkingSlot array     // @brief Array containing the CNN based parking slots.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.HighPlotData_InterfaceVersion uml:Class
  Members:
    type.uint32 HighPlotData_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.PlotData uml:Class
  version: ::si::PlotData_InterfaceVersion::PlotData_VERSION
    // @brief SI Plot Data contains the aggregate of all the debug data needed for plotting.
  Members:
    type.SI.CorePlotData core     // @brief Data structure containing debug data related to core features.
    type.SI.LowPlotData low     // @brief Data structure containing debug data related to low features.
    type.SI.HighPlotData high     // @brief Data structure containing debug data related to high features.
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.PlotData_InterfaceVersion uml:Class
  Members:
    type.uint32 PlotData_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.DiagnosticEvent uml:Class
  Members:
    type.uint32 diagnosisEventID     // @unit{nu};@range{494,497};Diagnostic event ID
    type.eco.DiagnosisEventStatus diagnosisEventStatus     // @unit{nu};@range{0,3};Type of parking scenario side associated with this parking box
----------------------------------------------------------------------------------------------------------

  # ID:  type.SI.DiagnosticEventSerializable uml:Class
  Members:
    type.LSM_GEOML.size_type actualSize     // @unit{nu};@range{0,SI.Constants.MAX_NUM_OF_DIAGNOSTIC_EVENT_IDS}
    type.SI.DiagnosticEvent array     // @unit{nu};Array holding diagnostic events
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM.APState uml:Enumeration
    // State of the Autonumous Parking.
  Members:
    AP_INACTIVE = 0            
    AP_SCAN_IN = 1            
    AP_SCAN_OUT = 2            
    AP_AVG_ACTIVE_IN = 3            
    AP_AVG_ACTIVE_OUT = 4            
    AP_AVG_PAUSE = 5            
    AP_AVG_UNDO = 6            
    AP_ACTIVE_HANDOVER_AVAILABLE = 7            
    AP_AVG_FINISHED = 8            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM.CoreStopReason uml:Enumeration
    // Reason why the PARKSM Core has stopped the maneuver.
  Members:
    CORE_STOP_REASON_NOT_VALID = 0            
    PARKING_FAILED_TPD_LOST = 1            
    PARKING_FAILED_PATH_LOST = 2            
    PARKING_SUCCESS = 3            
    INPUT_CORRUPTED = 4            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM.DrivingModeTRC uml:Enumeration
    // Define the requested driving mode. Only in the mode "FOLLOW_TRAJ" the requested trajectory will be used as input for the trajectory control.
  Members:
    NO_INTERVENTION = 0            
    FOLLOW_TRAJ = 1            
    MAKE_PAUSE = 2            
    MAKE_SECURE = 3            
    MAKE_SECURE_AND_ADJUST_ANGLE = 4            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM.DriverInOutRequestType uml:Enumeration
    // Define the Driver In or Out request .
  Members:
    PSM_DRIVER_IN_REQUEST = 0            
    PSM_DRIVER_OUT_REQUEST = 1            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM.MotionControlRequestType uml:Enumeration
    // Define the request type for the Motion control.
  Members:
    PSM_MCRT_NORMAL_REQUEST = 0            
    PSM_MCRT_OVERRIDE_REQUEST = 1            
    PSM_MCRT_DEGRADATION_REQUEST = 2            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM.GPState uml:Enumeration
    // State of the Garage Parking.
  Members:
    GP_INACTIVE = 0            
    GP_SCAN_IN = 1            
    GP_SCAN_OUT = 2            
    GP_AVG_ACTIVE_IN = 3            
    GP_AVG_ACTIVE_OUT = 4            
    GP_AVG_PAUSE = 5            
    GP_AVG_UNDO = 6            
    GP_AVG_FINISHED = 7            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM.AP_PSM_Consts uml:Class
  Members:
    type.uint8 NUM_MTS_DEBUG_FREESPACE_PSM = 10    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM.PARKSMCoreDebugPort uml:Class
  version: ::ap_psm::PARKSMCoreDebugPort_InterfaceVersion::PARKSMCoreDebugPort_VERSION
    // Freespace for MTS debug.
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.sint32 debugInt     // Freespace for MTS debug integer values. Size of the array: 10.
    type.float32 debugFloat     // Freespace for MTS debug float values. Size of the array: 10.
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM.PARKSMCoreDebugPort_InterfaceVersion uml:Class
  Members:
    type.uint32 PARKSMCoreDebugPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM.PARKSMCoreState uml:Enumeration
    // State of the PARKSM Core.
  Members:
    CORE_INIT = 0            
    CORE_SCANNING = 1            
    CORE_PARKING = 2            
    CORE_PAUSE = 3            
    CORE_FINISH = 4            
    CORE_ERROR = 5            
----------------------------------------------------------------------------------------------------------


  # ID:  type.AP_PSM.PARKSMCoreStatusPort uml:Class
  version: ::ap_psm::PARKSMCoreStatusPort_InterfaceVersion::PARKSMCoreStatusPort_VERSION
    // Status information of PARKSM Core.
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.AP_PSM.PARKSMCoreState parksmCoreState_nu     // @range{0,255};State of the PARKSM Core.
    type.AP_PSM.CoreStopReason coreStopReason_nu     // @range{0,255};Reason of the maneuver"s failure.
    type.AP_PSM.UnexpectedEvent unexpectedEvent_nu     // @range{0,255};Signals that the system detected that the driver grabbed the steering wheel during the maneuver.
    type.boolean parkingReady_nu     // Indicates that automatic vehicle guidance (maneuvering mode) can be started (map reseted).
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM.PARKSMCoreStatusPort_InterfaceVersion uml:Class
  Members:
    type.uint32 PARKSMCoreStatusPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM.APPlanningSpecification uml:Enumeration
    // Additional information to consider for the planning of the requested parking maneuver.
  Members:
    APPS_INVALID = 0            
    APPS_NONE = 1            
    APPS_PARK_IN_FULL_MANEUVERING_AREA = 2            
    APPS_PARK_IN_RESTRICTED_MANEUVERING_AREA = 3            
    APPS_PARK_OUT_UNTIL_CRITICAL_POINT_REACHED = 4            
    APPS_PARK_OUT_TO_TARGET_POSE = 5            
----------------------------------------------------------------------------------------------------------





  # ID:  type.AP_PSM.PlanningCtrlCommands uml:Class
    // From Parking State Machine or Planning and Control State Machine.
    // This is in fact not a port! It was a port earlier but now is just a substructure in SlotCtrlPort!
    // No interface version number signal needed!
  Members:
    type.uint8 apChosenTargetPoseId_nu     // @range{0,255};Unique identifier of selected target pose. If no pose is selected, ID is 255.
    type.AP_PSM.APState apState     // @range{0,128};State of the Autonumous Parking.
    type.AP_PSM.APPlanningSpecification apPlanningSpecification     // @range{0,4};Additional information to consider for the planning of the requested parking maneuver.
    type.AP_PSM.RMState rmState     // @range{0,128};State of the Remote Parking.
    type.AP_PSM.GPState gpState     // @range{0,128};State of the Garage Parking.
    type.AP_PSM.MPState mpState     // @range{0,128};State of the Memory Parking.
    type.AP_PSM.TPState tpState     // @range{0,128};State of the Trained Parking.
    type.AP_PSM.RAState raState     // @range{0,128};State of the Reverse Assist.
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM.RMState uml:Enumeration
    // State of the Remote Parking.
  Members:
    RM_INACTIVE = 0            
    RM_SCANNING = 1            
    RM_AVG_ACTIVE = 2            
    RM_AVG_PAUSED = 3            
    RM_AVG_FINISHED = 4            
----------------------------------------------------------------------------------------------------------


  # ID:  type.AP_PSM.ResetOriginRequestPort uml:Class
  version: ::ap_psm::ResetOriginRequestPort_InterfaceVersion::ResetOriginRequestPort_VERSION
    // Reset request data in order to request the map reset from SI.
  Members:
    type.LSM_GEOML.Pose_POD transformation     // @unit{nu};Relative to current origin, will be disregarded unless resetOrigin_nu equals RRT_RESET_CUSTOM. Contains target pose coordinates.
    type.AP_PSM.ResetOriginType resetOrigin_nu     // Type of the reset request.
    type.uint8 resetCounter_nu     // Will start from zero and increase by one on every reset request.
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM.ResetOriginRequestPort_InterfaceVersion uml:Class
  Members:
    type.uint32 ResetOriginRequestPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM.ResetOriginType uml:Enumeration
    // Type of the reset request.
  Members:
    RRT_NONE = 0            
    RRT_RESET_XY = 1            
    RRT_RESET_PSI = 2            
    RRT_RESET_XY_PSI = 3            
    RRT_RESET_CUSTOM = 4            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM.SlotCtrlPort uml:Class
  version: ::ap_psm::SlotCtrlPort_InterfaceVersion::SlotCtrlPort_VERSION
    // Slot control data.
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.AP_PSM.ResetOriginRequestPort resetOriginRequestPort     // @unit{nu};Reset request data in order to request the map reset from SI.
    type.AP_PSM.PlanningCtrlCommands planningCtrlCommands     
    type.uint16 selectedParkingBoxId_nu     // ID of the currently selected parking box.
    type.boolean storeMemorizedParkingData     // @unit{nu};Flag indicating if we want to store the current map or not
    type.boolean deleteMemorizedParkingData     // @unit{nu};Flag indicating if we want to delete the current map or not
    type.uint16 currentSelectedMemParkSlotId     // ID of the currently selected map
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM.SlotCtrlPort_InterfaceVersion uml:Class
  Members:
    type.uint32 SlotCtrlPort_VERSION = 3    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM.TrajCtrlRequestPort uml:Class
  version: ::ap_psm::TrajCtrlRequestPort_InterfaceVersion::TrajCtrlRequestPort_VERSION
    // Control commands from Parking State Machine to Trajectory Control.
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.AP_PSM.DrivingModeTRC drivingModeReq_nu     // @range{0,3};Define the requested driving mode. Only in the mode "FOLLOW_TRAJ" the requested trajectory will be used as input for the trajectory control.
    type.boolean trajCtrlActive_nu     // Control command for the activation/deactivation of the trajectory control (automatic vehicle guidance).
    type.boolean emergencyBrakeRequest     // @unit{boolean};Triggers an emergency braking intervention.
    type.AP_PSM.MotionControlRequestType MotionControlRequestType_nu     // Type of requests to the Motion Controller .
    type.AP_PSM.DriverInOutRequestType driverInOutRequestType_nu     // Type of requests based on driver Inside or outside.
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM.TrajCtrlRequestPort_InterfaceVersion uml:Class
  Members:
    type.uint32 TrajCtrlRequestPort_VERSION = 3    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM.UnexpectedEvent uml:Enumeration
    // PARKSM Core can inform the user dependent PARKSM about internal events. Reaction to this events has to be defined by the user dependent PARKSM
  Members:
    NO_EVENT = 0            
    DRIVER_STEERING_INTERVENTION = 1            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM.FC_PARKSM_Core_Params uml:Class
  version: ::ap_psm::FC_PARKSM_Core_Params_InterfaceVersion::FC_PARKSM_Core_Params_VERSION
    // FC_PARKSM_Core Parameters
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 AP_SC_RESET_MAP_DIST_M     // The state machine has to request a reset of the map after AP_S_RESET_MAP_DIST_M meter is driven.
    type.boolean AP_SC_REV_ASSIST_CODED     // Indicates if reverse assist is enabled (coded).
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM.FC_PARKSM_Core_Params_InterfaceVersion uml:Class
  Members:
    type.uint32 FC_PARKSM_Core_Params_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM.MPState uml:Enumeration
    // State of the Memory Parking.
  Members:
    MP_INACTIVE = 0            
    MP_TRAIN_SCAN = 1            
    MP_TRAIN_AVG = 2            
    MP_TRAIN_FINISHED = 3            
    MP_SILENT_MAP_LOC = 4            
    MP_RECALL_SCAN = 5            
    MP_RECALL_AVG = 6            
    MP_RECALL_FINISHED = 7            
    MP_PAUSE = 8            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM.TPState uml:Enumeration
    // State of the Trained Parking.
  Members:
    TP_INACTIVE = 0            
    TP_TRAIN_SCAN = 1            
    TP_TRAIN_AVG = 2            
    TP_TRAIN_FINISHED = 3            
    TP_SILENT_MAP_LOC = 4            
    TP_RECALL_SCAN = 5            
    TP_RECALL_AVG = 6            
    TP_RECALL_FINISHED = 7            
    TP_PAUSE = 8            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_PSM.RAState uml:Enumeration
    // State of the Reverse Assist.
  Members:
    RA_INACTIVE = 0            
    RA_RECORD = 1            
    RA_AVG_ACTIVE = 2            
    RA_AVG_PAUSE = 3            
    RA_AVG_FINISHED = 4            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.PoseObstacleDist uml:Class
    // 
  Members:
    type.float32 frontObstDist_m     // @range{0,10};@unit{m};Distance to the closest obstacle perpendicular to the vehicle front side. @min: 0 @max: 10.0 @unit: m
    type.boolean frontObstDistValid     // @unit{boolean};@range{0,1};Flag if the distance to front side obstacle is valid. @min: 0 @max: 1 @unit: boolean
    type.float32 rearObstDist_m     // @range{0,10};@unit{m};Distance to the closest obstacle perpendicular to the vehicle rear side. @min: 0 @max: 10.0 @unit: m
    type.boolean rearObstDistValid     // @unit{boolean};@range{0,1};Flag if the distance to rear side obstacle is valid. @min: 0 @max: 1 @unit: boolean
    type.float32 leftObstDist_m     // @range{0,10};@unit{m};Distance to the closest obstacle perpendicular to the vehicle left side. @min: 0 @max: 10.0 @unit: m
    type.boolean leftObstDistValid     // @unit{boolean};@range{0,1};Flag if the distance to left side obstacle is valid. @min: 0 @max: 1 @unit: boolean
    type.float32 rightObstDist_m     // @range{0,10};@unit{m};Distance to the closest obstacle perpendicular to the vehicle right side. @min: 0 @max: 10.0 @unit: m
    type.boolean rightObstDistValid     // @unit{boolean};@range{0,1};Flag if the distance to right side obstacle is valid. @min: 0 @max: 1 @unit: boolean
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.AP_TP_Const uml:Class
    // 
  Members:
    type.uint8 AP_P_MAX_NUM_TRAJ_CTRL_POINTS = 20    
    type.uint8 NUM_MTS_DEBUG_FREESPACE_TP = 10    
    type.uint8 AP_T_MAX_NUM_SMPL_PBOX_VERT_NU = 4    
    type.uint8 AP_T_MAX_NUM_POSE_BOX_DATA_NU = 8    
    type.uint8 NUM_MTS_DEBUG_FREESPACE_TPD = 10    
    type.uint16 AP_P_MAX_NUM_POSES_IN_PATH_NU = 1000    // maximum poses in a parking path
    type.uint8 AP_P_MAX_NUM_SEGMENTS_IN_PATH_NU = 25    // maximum number of segments (changes of turn radius or gear) in a geometric path
    type.uint16 AP_P_MAX_NUM_SAMPLES_IN_DRIVEN_PATH = 1000    
    type.uint8 AP_P_MAX_NUM_SAMPLES_IN_DRIVEN_PATH_BUFFER = 10    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.PosDefApproach uml:Enumeration
    // 
  Members:
    ATTACH_EGO_TO_EDGE = 0            
    CENTERING = 1            
    ROAD_SIDE_CENTERING = 2            
    SHORT_SIDE_CENTERING = 3            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.VLOrientationApproach uml:Enumeration
    // 
  Members:
    FRONT_VL_ORIENTATION = 0            
    REAR_VL_ORIENTATION = 1            
    VL_ORIENTATION_AVERAGE = 2            
    USE_BOTH_VL_PTS_NEAR_SLOT = 3            
    MAX_NUM_VL_ORIENT_APPROACHES = 4            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.VLPositioningApproach uml:Enumeration
    // 
  Members:
    ALIGN_TO_FRONT_VL_PT = 0            
    ALIGN_TO_REAR_VL_PT = 1            
    AVG_BOTH_VL_PTS_NEAR_SLOT = 2            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.ShortSideAngleApproach uml:Enumeration
    // 
  Members:
    SAA_AVERAGE = 0            
    SAA_ROAD_SIDE_ANGLE = 1            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.PlannedTrajType uml:Enumeration
    // 
  Members:
    PERP_PARK_IN_TRAJ = 0            
    PAR_PARK_IN_TRAJ = 1            
    PERP_PARK_OUT_TRAJ = 2            
    PAR_PARK_OUT_TRAJ = 3            
    REMOTE_MAN_TRAJ = 4            
    UNDO_TRAJ = 5            
    MAX_NUM_PLANNED_TRAJ_TYPES = 6            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.DrivingResistanceType uml:Enumeration
    // 
  Members:
    NONE = 0            
    FALLING_LOW = 1            
    FALLING_MEDIUM = 2            
    FALLING_HIGH = 3            
    RISING_LOW = 4            
    RISING_MEDIUM = 5            
    RISING_HIGH = 6            
    WHEEL_STOPPER = 7            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.PoseType uml:Enumeration
    // 
  Members:
    T_PARALLEL_PARKING = 0            
    T_PERP_PARKING_FWD = 1            
    T_PERP_PARKING_BWD = 2            
    T_ANGLED_PARKING_STANDARD = 3            
    T_ANGLED_PARKING_REVERSE = 4            
    T_REM_MAN_FWD = 5            
    T_REM_MAN_BWD = 6            
    T_PERP_PARKING_OUT_FWD = 7            
    T_PERP_PARKING_OUT_BWD = 8            
    T_PAR_PARKING_OUT = 9            
    T_ANGLED_PARKING_STANDARD_OUT = 10            
    T_ANGLED_PARKING_REVERSE_OUT = 11            
    T_UNDO = 12            
    T_GP_FWD = 13            
    T_GP_BWD = 14            
    T_GP_OUT_FWD = 15            
    T_GP_OUT_BWD = 16            
    T_GP_FWD_AXIS = 17            
    T_GP_BWD_AXIS = 18            
    T_GP_OUT_FWD_AXIS = 19            
    T_GP_OUT_BWD_AXIS = 20            
    T_UNDEFINED = 21            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.TargetSide uml:Enumeration
    // 
  Members:
    TS_RIGHT_SIDE = 0            
    TS_LEFT_SIDE = 1            
    TS_IN_FRONT_RIGHT = 2            
    TS_IN_FRONT_CENTER = 3            
    TS_IN_FRONT_LEFT = 4            
    TS_IN_REAR_RIGHT = 5            
    TS_IN_REAR_CENTER = 6            
    TS_IN_REAR_LEFT = 7            
    TS_UNDEFINED_SIDE = 8            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.PoseReachableStatus uml:Enumeration
    // 
  Members:
    TP_NOT_VALID = 0            
    TP_NOT_REACHABLE = 1            
    TP_FULLY_REACHABLE = 2            
    TP_SAFE_ZONE_REACHABLE = 3            
    TP_MANUAL_FWD_REACHABLE = 4            
    TP_MANUAL_BWD_REACHABLE = 5            
    MAX_NUM_POSE_REACHABLE_STATUS = 6            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.PoseFailReason uml:Enumeration
    // 
  Members:
    TAPOSD_PFR_NONE = 0            
    TAPOSD_PFR_PARKING_BOX_WIDTH_TOO_NARROW = 1            
    TAPOSD_PFR_PARKING_BOX_LENGTH_TOO_SHORT = 2            
    TAPOSD_PFR_MAXBOX_EXCEEDED = 3            
    TAPOSD_PFR_WHEEL_COLLISION = 4            
    TAPOSD_PFR_HIGH_OBJECT_COLLISION = 5            
    TAPOSD_PFR_UNKNOWN = 6            
    MAX_NUM_POSE_FAIL_TYPES = 7            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.PlanningFailReason uml:Enumeration
    // 
  Members:
    PFR_NONE = 0            
    PFR_TARGET_POSE_LOST = 1            
    PFR_PARKING_BOX_LOST = 2            
    PFR_INPUT_CORRUPTED = 3            
    PFR_REPLAN_FAIL = 4            
    MAX_NUM_PLANNING_FAIL_TYPES = 5            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.PoseSelectionStatus uml:Enumeration
    // 
  Members:
    PSS_NO_SELECTION = 0            
    PSS_PLANNER_PRESELECTION = 1            
    PSS_DRIVER_SELECTION = 2            
    MAX_NUM_POSE_SEL_STATUS_TYPES = 3            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.PoseReachedStatus uml:Enumeration
    // 
  Members:
    NO_TP_REACHED_STATUS = 0            
    TP_REACHED = 1            
    TP_REACHED_FALLBACK = 2            
    TP_NOT_REACHED = 3            
    MAX_NUM_POSE_REACHED_STATUS_TYPES = 4            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.TargetRoadSide uml:Enumeration
    // 
  Members:
    TRS_RIGHT_SIDE = 0            
    TRS_LEFT_SIDE = 1            
    TRS_IN_FRONT = 2            
    TRS_IN_REAR = 3            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.PlannedTraj uml:Class
    // 
  Members:
    type.float32 xTrajRAReq_m     // @unit{m};Array of relevant x coordinates of the reference trajectory for the middle of the rear wheel axle in frozen/absolut coordinates. (*Trajectory must be driveable: e.g. consindering maximal curvature) @min: 0 @max: 0 @unit: m
    type.float32 yTrajRAReq_m     // @unit{m};Array of relevant y coordinates of the reference trajectory for the middle of the rear wheel axle in frozen/absolut coordinates. (*Trajectory must be driveable: e.g. consindering maximal curvature) @min: 0 @max: 0 @unit: m
    type.float32 yawReq_rad     // @range{-3.1415926,3.1415926};@unit{rad};None @min: -3.1415926 @max: +3.1415926 @unit: rad
    type.float32 crvRAReq_1pm     // @range{-0.24,0.24};@unit{1/m};Curvature of the trajectory at the current point @min: -0.24 @max: 0.24 @unit: 1/m;Range: AP_V_MIN_TURN_RADIUS_M (4.1) + AP_P_MIN_RADIUS_ADD_CIRCLE_M (0.2) = 4.3 m → 1/4.3 = 0.24 [1/m]
    type.float32 distanceToStopReq_m     // @range{0,50};@unit{m};Value for the remaining distance to stop for the current stroke (*used as longitudinal reference value in case of distance control) @min: -2 @max: 30 @unit: m
    type.float32 velocityLimitReq_mps     // @range{0,2.778};@unit{m/s};Array with relevant values for the VelocityLimit/ target vehicle velocity.(*used as longitudinal reference value in case of velocity control; *used as limit for the vehicle velocity in case of distance control) @min: 0 @max: AP_G_MAX_AVG_V_MPS = 10kph = 2.778 m/s @unit: m/s
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.DrivingResistance uml:Class
    // 
  Members:
    type.float32 distance_m     // @range{0,10};@unit{m};Distance to the wheel individual driving resistance based on the movement of the rear axle center. @min: 0 @max: 10 @unit: m
    type.AP_TP.DrivingResistanceType type_nu     // @range{0,7};@unit{enum DrivingResistanceType};Type of the driving resistance. @min: 0 @max: 7 @unit: enum DrivingResistanceType @values: enum { _NONE=0,_FALLING_LOW=1,_FALLING_MEDIUM=2,_FALLING_HIGH=3,_RISING_LOW=4,_RISING_MEDIUM=5,_RISING_HIGH=6,_WHEEL_STOPPER=7 }
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.TargetPose uml:Class
    // 
  Members:
    type.LSM_GEOML.Pose_POD pose     // @unit{nu};A pose defined by an x and y coordinate as well as a yaw angle @min: 0 @max: 0 @unit: nu
    type.uint8 pose_ID     // @range{0,255};@unit{Identifier};ID of target pose @min: 0 @max: 255 @unit: Identifier
    type.uint16 relatedParkingBoxID     // @range{0,65535};@unit{Identifier};To each target pose a related parking box ID is assigned. @min: 0 @max: 65535 @unit: Identifier
    type.AP_TP.PoseType type     // @range{0,21};@unit{enum PoseType};Type of target pose @min: 0 @max: 21 @unit: enum PoseType @values: enum { _T_PARALLEL_PARKING=0,_T_PERP_PARKING_FWD=1,_T_PERP_PARKING_BWD=2,_T_ANGLED_PARKING_STANDARD=3,_T_ANGLED_PARKING_REVERSE=4,_T_REM_MAN_FWD=5,_T_REM_MAN_BWD=6,_T_PERP_PARKING_OUT_FWD=7,_T_PERP_PARKING_OUT_BWD=8,_T_PAR_PARKING_OUT=9,_T_ANGLED_PARKING_STANDARD_OUT=10,_T_ANGLED_PARKING_REVERSE_OUT=11,_T_UNDO=12,_T_GP_FWD=13,_T_GP_BWD=14,_T_GP_OUT_FWD=15,_T_GP_OUT_BWD=16,_T_GP_FWD_AXIS=17,_T_GP_BWD_AXIS=18,_T_GP_OUT_FWD_AXIS=19,_T_GP_OUT_BWD_AXIS=20,_T_UNDEFINED=21 }
    type.AP_TP.TargetSide targetSide     // @range{0,8};@unit{enum TargetSide};Position of target pose relative to ego vehicle @min: 0 @max: 8 @unit: enum TargetSide @values: enum { _TS_RIGHT_SIDE=0,_TS_LEFT_SIDE=1,_TS_IN_FRONT_RIGHT=2,_TS_IN_FRONT_CENTER=3,_TS_IN_FRONT_LEFT=4,_TS_IN_REAR_RIGHT=5,_TS_IN_REAR_CENTER=6,_TS_IN_REAR_LEFT=7,_TS_UNDEFINED_SIDE=8 }
    type.AP_TP.PoseReachableStatus reachableStatus     // @range{0,6};@unit{enum PoseReachableStatus};Classification of reachability @min: 0 @max: 6 @unit: enum PoseReachableStatus @values: enum { _TP_NOT_VALID=0,_TP_NOT_REACHABLE=1,_TP_FULLY_REACHABLE=2,_TP_SAFE_ZONE_REACHABLE=3,_TP_MANUAL_FWD_REACHABLE=4,_TP_MANUAL_BWD_REACHABLE=5,_MAX_NUM_POSE_REACHABLE_STATUS=6 }
    type.AP_TP.PoseFailReason poseFailReason     // @range{0,7};@unit{enum PoseFailReason};Reason to HMI why pose can not be provided @min: 0 @max: 7 @unit: enum PoseFailReason @values: enum { _TAPOSD_PFR_NONE=0,_TAPOSD_PFR_PARKING_BOX_WIDTH_TOO_NARROW=1,_TAPOSD_PFR_PARKING_BOX_LENGTH_TOO_SHORT=2,_TAPOSD_PFR_MAXBOX_EXCEEDED=3,_TAPOSD_PFR_WHEEL_COLLISION=4,_TAPOSD_PFR_HIGH_OBJECT_COLLISION=5,_TAPOSD_PFR_UNKNOWN=6,_MAX_NUM_POSE_FAIL_TYPES=7 }
    type.AP_TP.PoseObstacleDist poseObstacleDist     // Provides the perpendicular distance from the four vehicle sides to the closest obstacles.
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.SelectedPoseData uml:Class
    // 
  Members:
    type.AP_TP.PoseSelectionStatus selectionStatus     // @range{0,3};@unit{enum PoseSelectionStatus};Indicates the preselection of poses done by planner @min: 0 @max: 3 @unit: enum PoseSelectionStatus  @values: enum { _PSS_NO_SELECTION=0,_PSS_PLANNER_PRESELECTION=1,_PSS_DRIVER_SELECTION=2,_MAX_NUM_POSE_SEL_STATUS_TYPES=3 }
    type.AP_TP.PoseReachedStatus reachedStatus     // @range{0,3};@unit{enum PoseReachedStatus};Indicates wether the selected pose is reached by the ego vehicle within the required tolerances @min: 0 @max: 3 @unit: enum PoseReachedStatus @values: enum { _NO_TP_REACHED_STATUS=0,_TP_REACHED=1,_TP_REACHED_FALLBACK=2,_TP_NOT_REACHED=3,_MAX_NUM_POSE_REACHED_STATUS_TYPES=4 }
    type.float32 distanceToStart_m     // @range{0,64};@unit{m};None @min: 0 @max: 64 @unit: m
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.PlannedGeometricPathOutput uml:Class
    // 
  Members:
    type.float32 startPose     // @unit{nu};start pose of the segment @min: 0 @max: 0 @unit: nu
    type.float32 endPose     // @unit{nu};end pose of the segment @min: 0 @max: 0 @unit: nu
    type.float32 turnRadius_m     // @unit{m};number of valid positions in the planned path @min: 0 @max: inf @unit: m
    type.float32 turnRadiusSecond_m     // @unit{m};number of valid positions in the planned path @min: 0 @max: inf @unit: m
    type.uint8 drvDir     // @range{0,3};@unit{---};number of valid positions in the planned path @min: 0 @max: 3 @unit: ---
    type.uint8 steerDir     // @range{0,6};@unit{---};number of valid positions in the planned path @min: 0 @max: 6 @unit: ---
    type.float32 longVel_mps     // @range{0,2.778};@unit{m/s};number of valid positions in the planned path @min: 0 @max: AP_G_MAX_AVG_V_MPS = 10kph = 2.778 m/s @unit: m/s
    type.float32 length_m     // @range{0,20};@unit{m};number of valid positions in the planned path @min: 0 @max: AP_G_MAX_LENGHT_STROKE_M = 20 @unit: m
    type.float32 rotationCenter_m     // @range{-500,500};@unit{m};number of valid positions in the planned path @min: -500 @max: 500 @unit: m
    type.float32 rotationCenterSecond_m     // @range{-500,500};@unit{m};number of valid positions in the planned path @min: -500 @max: 500 @unit: m
    type.uint8 planPhase     // @range{0,26};@unit{---};number of valid positions in the planned path @min: 0 @max: 26 @unit: ---
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.PlannedTrajPort uml:Class
  version: ::ap_tp::PlannedTrajPort_InterfaceVersion::PlannedTrajPort_VERSION
    // From TrajectoryPlanning
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     // @unit{eco.AlgoInterfaceVersionNumber};
    type.eco.SignalHeader sSigHeader     // @unit{eco.SignalHeader};
    type.AP_TP.PlannedTrajType trajType_nu     // @range{0,7};The type of the output trajectory (used to trigger certain behavior in controller).
    type.boolean drivingForwardReq_nu     // Define the requested driving direction.
    type.boolean trajValid_nu     // Flag whether the trajectory data is valid.
    type.boolean newSegmentStarted_nu     // Flag that a new segment (e.g. new stroke) was calculated.
    type.boolean isLastSegment_nu     // Flag that the current segment is the final segment of the current trajectory
    type.uint8 stepInTrajAfterIdx_nu     // @range{0,255};If there is a discontinuity in either x, y or yaw within the trajectory this will indicate the index of the point after which the discontinuity happens
    type.AP_TP.DrivingResistance drivingResistance     // @unit{nu};Driving resistance distance and type information per wheel.
    type.uint8 numValidCtrlPoints_nu     // @range{0,MAX_NUM_TRAJ_CTRL_POINTS};Number of vali d and unique trajectory points in "plannedTraj". (e.g. if this value is 5 only the first 5 trajectory points inside "plannedTraj" are unique; the 6th,7th,... point would be equal to the 5th point)
    type.AP_TP.PlannedTraj plannedTraj     // @unit{nu};Calculated Trajectory from the Trajectory Planning with EM based velocity limit.
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.PlannedTrajPort_InterfaceVersion uml:Class
  Members:
    type.uint32 PlannedTrajPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.TargetPosesPort uml:Class
  version: ::ap_tp::TargetPosesPort_InterfaceVersion::TargetPosesPort_VERSION
    // 
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     // @unit{eco.AlgoInterfaceVersionNumber};
    type.eco.SignalHeader sSigHeader     // @unit{eco.SignalHeader};
    type.uint8 resetCounter     // @unit{Count};indicates number of coordinate resets performed by the planner @min: 0 @max: 0 @unit: Count
    type.AP_TP.PlanningFailReason failReason     // @unit{enum PlanningFailReason};@range{0,4};Reason for failing from TPD @min: 0 @max: 4 @unit: enum PlanningFailReason @values: enum { _PFR_NONE=0,_PFR_TARGET_POSE_LOST=1,_PFR_PARKING_BOX_LOST=2,_PFR_INPUT_CORRUPTED=3,_PFR_REPLAN_FAIL=4,_MAX_NUM_PLANNING_FAIL_TYPES=5 }
    type.boolean anyPathFound     // @unit{boolean};@range{0,1};None @min: 0 @max: 1 @unit: boolean
    type.AP_TP.SelectedPoseData selectedPoseData     // @unit{nu};Information related to the selected Target Pose @min: 0 @max: 0 @unit: nu
    type.uint8 numValidPoses     // @unit{Count};Number of poses that are valid (does not need to be reachable) @min: 0 @max: AP_Common.AP_G_MAX_NUM_TARGET_POSES_NU @unit: Count
    type.AP_TP.TargetPose targetPoses     // @unit{nu};All information related to a possible target pose @min: 0 @max: 0 @unit: nu
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.TargetPosesPort_InterfaceVersion uml:Class
  Members:
    type.uint32 TargetPosesPort_VERSION = 2    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.TrajPlanDebugPort uml:Class
  version: ::ap_tp::TrajPlanDebugPort_InterfaceVersion::TrajPlanDebugPort_VERSION
    // 
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     // @unit{eco.AlgoInterfaceVersionNumber};
    type.eco.SignalHeader sSigHeader     // @unit{eco.SignalHeader};
    type.sint32 debugInt     // @unit{nu};freespace for MTS debug values @min: 0 @max: 0 @unit: nu
    type.float32 debugFloat     // @unit{nu};freespace for MTS debug values @min: 0 @max: 0 @unit: nu
    type.uint8 mNumOfReplanCalls     // @range{0,255};@unit{Count};None @min: 0 @max: 255 @unit: Count
    type.uint8 mTrajPlanState     // @range{0,18};@unit{nu};None @min: 0 @max: 11 @unit: nu
    type.boolean mReplanSuccessful_nu     // @range{0,1};@unit{boolean};None @min: 0 @max: 1 @unit: boolean
    type.boolean mStateEntry_nu     // @range{0,1};@unit{boolean};None @min: 0 @max: 1 @unit: boolean
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.TrajPlanDebugPort_InterfaceVersion uml:Class
  Members:
    type.uint32 TrajPlanDebugPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.TrajPlanVisuPort uml:Class
  version: ::ap_tp::TrajPlanVisuPort_InterfaceVersion::TrajPlanVisuPort_VERSION
    // 
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     // @unit{eco.AlgoInterfaceVersionNumber};
    type.eco.SignalHeader sSigHeader     // @unit{eco.SignalHeader};
    type.uint16 currentPoseIdx_nu     // @range{0,1023};@unit{---};index of current pose along the planned path @min: 0 @max: 1023 @unit: ---
    type.uint16 numValidPoses_nu     // @range{0,1023};@unit{---};number of valid positions in the planned path @min: 0 @max: 1023 @unit: ---
    type.uint8 numValidSegments     // @range{0,255};@unit{---};number of valid positions in the planned path @min: 0 @max: 255 @unit: ---
    type.float32 plannedPathXPos_m     // @range{-1000,1000};@unit{m};x coordinates of the whole path @min: -1000 @max: 1000 @unit: m
    type.float32 plannedPathYPos_m     // @range{-1000,1000};@unit{m};y coordinates of the whole path @min: -1000 @max: 1000 @unit: m
    type.AP_TP.PlannedGeometricPathOutput plannedGeometricPath     // @unit{nu};None @min: 0 @max: 0 @unit: nu
    type.AP_TP.DrivingResistance drivingResistance     // Driving resistance distance and type information per wheel.
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.TrajPlanVisuPort_InterfaceVersion uml:Class
  Members:
    type.uint32 TrajPlanVisuPort_VERSION = 2    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.ReverseAssistAvailabilityPort uml:Class
  version: ::ap_tp::ReverseAssistAvailabilityPort_InterfaceVersion::ReverseAssistAvailabilityPort_VERSION
    // 
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     // @unit{eco.AlgoInterfaceVersionNumber};
    type.eco.SignalHeader sSigHeader     // @unit{eco.SignalHeader};
    type.boolean pathAvailable     // @range{0,1};@unit{nu};indicator signalizing whether a Reverse Assist path is available @min: 0 @max: 1 @unit: nu
    type.float32 pathLength_m     // @range{0,150};@unit{m};length of the currently stored Reverse Assist path @min: 0 @max: 150 @unit: m
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.ReverseAssistAvailabilityPort_InterfaceVersion uml:Class
  Members:
    type.uint32 ReverseAssistAvailabilityPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.SimpleDebugParkingBox uml:Class
    // 
  Members:
    type.uint8 numValidPoints_nu     // @range{0,255};@unit{Count};Number of valid points in parking box @min: 0 @max: 255 @unit: Count
    type.float32 posX_m     // @unit{m};Parking Box vertex: x-Position @min: 0 @max: 0 @unit: m
    type.float32 posY_m     // @unit{m};Parking Box vertex: y-Position @min: 0 @max: 0 @unit: m
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.DelimiterDistances uml:Class
    // 
  Members:
    type.float32 minDistance_m     // @unit{m};Minimal distance to keep from delimiters @min: 0 @max: 0 @unit: m
    type.float32 comfDistance_m     // @unit{m};Comfort distance to keep from delimiters @min: 0 @max: 0 @unit: m
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.PoseBoxData uml:Class
    // 
  Members:
    type.float32 egoYawFirstDetection_rad     // @range{-3.14,3.14};@unit{Radian};Yaw angle of ego vehicle at first detection of parking box with ID parkingBoxID_nu @min: -3.14 @max: 3.14 @unit: Radian
    type.uint16 parkingBoxID_nu     // @range{0,65535};@unit{Identifier};ID of parking box @min: 0 @max: 65535 @unit: Identifier
    type.uint8 poseInID_nu     // @range{0,255};@unit{Identifier};ID of target pose for park in that was created based on parking box with ID parkingBoxID_nu @min: 0 @max: 255 @unit: Identifier
    type.uint8 poseOutID_nu     // @range{0,255};@unit{Identifier};ID of target pose for park out that was created based on parking box with ID parkingBoxID_nu @min: 0 @max: 255 @unit: Identifier
    type.uint8 poseInIdx_nu     // @range{0,8};@unit{Index};Current index of target pose that refers to poseInID @min: 0 @max: 8 @unit: Index
    type.uint8 poseOutIdx_nu     // @range{0,8};@unit{Index};Current index of target pose that refers to poseOutID @min: 0 @max: 8 @unit: Index
    type.AP_TP.TargetRoadSide parkingBoxSide_nu     // @range{0,3};@unit{enum TargetRoadSide};Road side of parking box determined using the driving direction @min: 0 @max: 3 @unit: enum TargetRoadSide @values: enum { _TRS_RIGHT_SIDE=0,_TRS_LEFT_SIDE=1,_TRS_IN_FRONT=2,_TRS_IN_REAR=3 }
    type.boolean leftEdgeIsFront_nu     // @range{0,1};@unit{boolean};Wether the left edge of the parking box is the edge with greater x-value for x pointing in driving direction @min: 0 @max: 1 @unit: boolean
    type.boolean mappingValid_nu     // @range{0,1};@unit{boolean};Wether the mapping values between pose and parking box are valid @min: 0 @max: 1 @unit: boolean
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.ParkingBoxDebugInfo uml:Class
    // 
  Members:
    type.AP_TP.SimpleDebugParkingBox maxParkingBox     // @unit{nu};Max parking box (minimal distance to surroundings) @min: 0 @max: 0 @unit: nu
    type.AP_TP.SimpleDebugParkingBox comfParkingBox     // @unit{nu};Comfort parking box (greater distance to surroundings) @min: 0 @max: 0 @unit: nu
    type.AP_TP.DelimiterDistances delimiterDistances     // @unit{nu};Distances to keep from each delimiter @min: 0 @max: 0 @unit: nu
    type.float32 yawAngle_rad     // @range{-3.14,3.14};@unit{Radian};Yaw angle of the target pose @min: -3.14 @max: 3.14 @unit: Radian
    type.float32 targetPosX_m     // @unit{m};X-position of the target pose @min: 0 @max: 0 @unit: m
    type.float32 targetPosY_m     // @unit{m};Y-position of the target pose @min: 0 @max: 0 @unit: m
    type.boolean leftEdgeIsFront_nu     // @range{0,1};@unit{boolean};Whether the left parking box edge is the front edge (depends on road side, traffic side and wether street is a one-way road) @min: 0 @max: 1 @unit: boolean
    type.boolean isResized_nu     // @range{0,1};@unit{boolean};Whether the parking box was resized successfully into a max- and comfParkingBox @min: 0 @max: 1 @unit: boolean
    type.boolean hasAngle_nu     // @range{0,1};@unit{boolean};Whether the yaw angle was defined successfully @min: 0 @max: 1 @unit: boolean
    type.boolean hasPosition_nu     // @range{0,1};@unit{boolean};Whether the position was defined successfully @min: 0 @max: 1 @unit: boolean
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.PoseBoxDataset uml:Class
    // 
  Members:
    type.AP_TP.PoseBoxData data     // @unit{nu};Linking of Target Pose to Parking Box @min: 0 @max: 0 @unit: nu
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.TAPOSDDebugPort uml:Class
  version: ::ap_tp::TAPOSDDebugPort_InterfaceVersion::TAPOSDDebugPort_VERSION
    // 
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     // @unit{eco.AlgoInterfaceVersionNumber};
    type.eco.SignalHeader sSigHeader     // @unit{eco.SignalHeader};
    type.AP_TP.ParkingBoxDebugInfo pbDebugBackwards     // @unit{nu};Debug information of generation of target poses for backwards parking (Index relates to input data parking box index) @min: 0 @max: 0 @unit: nu
    type.AP_TP.ParkingBoxDebugInfo pbDebugForwards     // @unit{nu};Debug information of generation of target poses for forwards parking (Index relates to input data parking box index) @min: 0 @max: 0 @unit: nu
    type.AP_TP.PoseBoxDataset poseBoxDataset     // @unit{nu};Linking of all Target Poses to their Parking Boxes @min: 0 @max: 0 @unit: nu
    type.float32 latDistToTarget_m     // @range{0,100};@unit{m};(only filled at end of last stroke) Lateral distance between vehicle pose and target pose @min: 0 @max: 100 @unit: m
    type.float32 longDistToTarget_m     // @range{0,100};@unit{m};(only filled at end of last stroke) Longitudinal distance between vehicle pose and target pose @min: 0 @max: 100 @unit: m
    type.float32 yawDiffToTarget_rad     // @range{-3.14,3.14};@unit{Radian};(only filled at end of last stroke) Difference in yaw angle between vehicle pose and target pose @min: -3.14 @max: 3.14 @unit: Radian
    type.sint32 debugInt     // @unit{nu};freespace for MTS debug values @min: 0 @max: 0 @unit: nu
    type.float32 debugFloat     // @unit{nu};freespace for MTS debug values @min: 0 @max: 0 @unit: nu
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.TAPOSDDebugPort_InterfaceVersion uml:Class
  Members:
    type.uint32 TAPOSDDebugPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.FC_TAPOSD_Params uml:Class
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 AP_T_MAX_DEV_UPDATE_LAT_PAR_M     // Maximal deviation between old target position and new target position in lateral direction that still allows sending the old target pose.
    type.float32 AP_T_MAX_DEV_UPDATE_LONG_PAR_M     // Maximal deviation between old target position and new target position in longitudinal direction that still allows sending the old target pose.
    type.float32 AP_T_MAX_DEV_UPD_YAW_PAR_RAD     // Maximal deviation between old target angle and new target angle that still allows sending the old target pose.
    type.uint8 AP_T_MIN_OBJ_EXIST_PERC     // Minimal static object existence percentage to consider object in the calculation of the target pose.
    type.AP_TP.PosDefApproach AP_T_POS_DEF_APPROACH     // Approach of positioning the ego vehicle in the parking box. 0 = ATTACH_EGO_TO_EDGE; 1 = CENTERING; 2 = ROAD_SIDE_CENTERING; 3 = SHORT_SIDE_CENTERING
    type.float32 AP_T_POUT_PAR_POSE_X_M     // X-movement of parking out pose for parallel parking slot relative to current ego vehicle position (in ego coordinates).
    type.float32 AP_T_POUT_PAR_POSE_Y_M     // Y-movement of parking out pose for parallel parking slot relative to current ego vehicle position  (in ego coordinates). The sign of this parameter will be flipped according to side of parking out.
    type.float32 AP_T_POUT_PAR_POSE_YAW_RAD     // Rotation of parking out pose for parallel parking slot relative to current ego vehicle rotation (in ego coordinates).
    type.float32 AP_T_POUT_PERP_POSE_RIGHT_M     // Right movement of parking out pose for perpendicular parking slot relative to road side edge.;Direction is equal to vector from parking box point 1 to point 0 (on road side edge from right edge to left edge).;The sign of this parameter will be flipped according to direction of parking out.
    type.float32 AP_T_POUT_PERP_POSE_Y_LEFT_M     // Gap between parking box and vehicle in target pose for perpendicular parking out with target side left. Y-direction is orthogonal to road side edge pointing into the street.
    type.float32 AP_T_POUT_PERP_POSE_Y_RIGHT_M     // Gap between parking box and vehicle in target pose for perpendicular parking out with target side right. Y-direction is orthogonal to road side edge pointing into the street.
    type.float32 AP_T_POUT_PERP_POSE_Y_R_BWD_M     // Same as AP_T_POUT_PERP_POSE_Y_RIGHT_M but for backward perp. parking out as it is not possible for the planner to hold such  small values.
    type.float32 AP_T_POUT_PERP_DEV_LAT_M     // Allowed lateral deviation from target axis on parking out perpendicular
    type.float32 AP_T_POUT_PERP_DEV_YAW_RAD     // Allowed yaw angle deviation of ego vehicle relative to target axis on parking out perpendicular
    type.float32 AP_T_POUT_PAR_SAFETY_DIST_M     // Safety margin for straight driving corridor used to check if target pose is reached for parallel parking out
    type.float32 AP_T_LEN_IRRELEVANT_AREA_X_M     // Area around vehicle center to exclude for definition of parking out pose based on driven path (x-length)
    type.float32 AP_T_LEN_IRRELEVANT_AREA_Y_M     // Area around vehicle center to exclude for definition of parking out pose based on driven path (y-length)
    type.float32 AP_T_PIN_PAR_POSE_X_M     // Demo 2 relative movement of pose
    type.float32 AP_T_PIN_PAR_POSE_Y_M     // Demo 2 relative movement of pose
    type.float32 AP_T_PIN_PAR_POSE_YAW_RAD     // Demo 2 relative movement of pose
    type.float32 AP_T_PIN_PERP_POSE_X_M     // Demo 2 relative movement of pose
    type.float32 AP_T_PIN_PERP_POSE_Y_M     // Demo 2 relative movement of pose
    type.float32 AP_T_PIN_PERP_POSE_YAW_RAD     // Demo 2 relative movement of pose
    type.uint8 AP_T_MIN_PBOX_EXIST_PERC     // Minimal static object existence percentage to consider object in the calculation of the target pose.
    type.boolean AP_T_UPDATE_POSE_LAST_STROKE_NU     // Defines if it is allowed to update the Target Pose during last stroke
    type.float32 AP_T_MAX_DEV_UPDATE_LAT_PERP_M     // Maximal deviation between old target position and new target position in lateral direction that still allows sending the old target pose.
    type.float32 AP_T_MAX_DEV_UPDATE_LONG_PERP_M     // Maximal deviation between old target position and new target position in longitudinal direction that still allows sending the old target pose.
    type.float32 AP_T_MAX_DEV_UPD_YAW_PERP_RAD     // Maximal deviation between old target angle and new target angle that still allows sending the old target pose.
    type.boolean AP_T_UPDATE_LONG_ONLY_PAR_NU     // For a parallel parking slot update only the longitudinal position of the target pose while in AVG mode. (TODO set back to zero after demo)
    type.float32 AP_T_POUT_PERP_POSE_LEFT_M     // Left movement of parking out pose for perpendicular parking slot relative to road side edge. Direction is equal to vector from parking box;point 1 to point 0 (on road side edge from right edge to left edge). The sign of this parameter will be flipped according to direction of parking out.
    type.float32 AP_T_MAXCOMFPOS_INT_TOL_M     // Tolerance for intersections between 1D projection line and comfort or maximal box for maxComfPositioning approach
    type.float32 AP_T_MAX_LATERAL_DEVIATION_M     // Parameter, limitting the maximum allowed lateral deviation for planner.
    type.boolean AP_T_YAW_ANG_PBOX_ORI_NU     // In case that in PoseDefinerParkingIn::targetYawAngleDefinition no delimiter for orientating the ego vehicle is found, this parametrs determines whether;the orientation shall be determined according the the long side orientation of the parking box (TRUE) or according to the orientation of the ego vehicle;when the parking box has been detected first time (FALSE)
    type.float32 AP_T_POUT_PERP_COR_OFFS_FRONT_M     // For the park out corridor for perpendicular parking out: Distance between the front of the ego vehicle to the Front Side of the park out corridor.
    type.float32 AP_T_POUT_PERP_COR_OFFS_REAR_M     // For the park out corridor for perpendicular parking out: In case of positive value: Reduces the distance between rear axle and Rear Side. In case of;negative value: Increases the distance between rear axle and Rear Side.
    type.float32 AP_T_POUT_PERP_COR_OFFS_SLOTS_M     // For the park out corridor for perpendicular parking out:  Gap between the lateral vehicle border and the "Slot Side"
    type.float32 AP_T_POUT_PERP_COR_OFFS_OPPOS_M     // For the park out corridor for perpendicular parking out: Gap between the lateral vehicle border and the "Opposite Side"
    type.float32 AP_T_MIN_LATERAL_DEVIATION_M     // Parameter, limitting the maximum allowed lateral deviation for planner.
    type.float32 AP_T_MAX_MARKING_ANG_DEV_RAD     // Maximal angular deviation of parking space marking compared Parking Box orientation
    type.float32 AP_T_MIN_LENGTH_MARKING_M     // Minimum length for a marking to be considered
    type.float32 AP_T_EXTENT_MAX_BOX_ROADSIDE_M     // FOR HACK: in case the target pose does not fit in the max box, extend the parking box by this length to the road side edge
    type.float32 AP_T_MAX_DELIM_DIST_TO_BOX_M     // max. distance between a delimiter and the input parking box, to consider this object as relevant delimiter
    type.float32 AP_T_GP_POSE_FREEZE_RADIUS_M     // Garage Parking: Circular area around the target pose that will freeze the target pose if ego vehicle"s rear-axle is inside.
    type.float32 AP_T_MAX_DEV_UPDATE_LAT_GP_M     // Garage Parking: Pose update during garage parking if lateral deviation between new and old pose is higher than this threshold.
    type.float32 AP_T_MAX_DEV_UPDATE_LONG_GP_M     // Garage Parking: Pose update during garage parking if longitudinal deviation between new and old pose is higher than this threshold.
    type.float32 AP_T_MAX_DEV_UPD_YAW_GP_RAD     // Garage Parking: Pose update during garage parking if angular deviation between new and old pose is higher than this threshold.
    type.float32 AP_T_GP_OUT_GAP_ENTRANCE_M     // Garage Parking: Distance of ego vehilce to garage in final parking pose on leaving the garage.
    type.float32 AP_T_GP_OUT_REACHED_M     // Garage Parking: Distance of ego vehilce to garage in final parking pose on leaving the garage.
    type.float32 AP_T_GP_REACHED_DEV_BOX_M     // Garage Parking: Allowed overlap over maxBox to successfully finish maneuver. Set to 1 to "deactivate" check for being inside the maximum box.
    type.float32 AP_T_GP_REACHED_DEV_LONG_M     // Garage Parking: Allowed longitudinal deviation towards garage door to successfully finish parking in maneuver
    type.boolean AP_T_GP_INSIDE_LAT_CENTER_ONLY     // Garage Parking: If this parameter is set to true, only center in the maximum box for entering the garage. This is disregarding any comfort values.
    type.float32 AP_T_AXIS_MIN_OUTSIDE_GARAGE_M     // Garage Parking: If the vehicle dived into the garage that only the length defined by this parameter is still out on the street, switch to pose mode;even though no delimiter was detected at the back wall (curb side edge)
    type.uint8 AP_T_ACTUAL_LENGTH_POSE_HISTORY     // Defines the actual number of poses stored in the pose history that are used to calculate the average target pose. Note that this parameter has to be <= AP_T_MAX_LENGTH_POSE_HISTORY
    type.float32 AP_T_MAX_VIRTUALLINE_ANG_DEV_RAD     // Maximal angular deviation of virtual lines of objects compared Parking Box orientation
    type.float32 AP_T_ORI_CATEG_WEIGHT_SHORTSIDE_PARKMARKING_NU     // Category weight for orientation calculation cost function for short side parking markings.
    type.float32 AP_T_ORI_CATEG_WEIGHT_LONGSIDE_PARKMARKING_NU     // Category weight for orientation calculation cost function for long side parking markings.
    type.float32 AP_T_ORI_CATEG_WEIGHT_SHORTSIDE_CURB_NU     // Category weight for orientation calculation cost function for short side curbs.
    type.float32 AP_T_ORI_CATEG_WEIGHT_LONGSIDE_CURB_NU     // Category weight for orientation calculation cost function for long side curbs.
    type.float32 AP_T_ORI_CATEG_WEIGHT_SHORTSIDE_VIRTUALLINE_NU     // Category weight for orientation calculation cost function for short side virtual lines.
    type.float32 AP_T_ORI_CATEG_WEIGHT_LONGSIDE_VIRTUALLINE_NU     // Category weight for orientation calculation cost function for long side virtual lines.
    type.float32 AP_T_ORI_CATEG_WEIGHT_SHORTSIDE_LANEBOUND_NU     // Category weight for orientation calculation cost function for short side lane boundaries.
    type.float32 AP_T_ORI_CATEG_WEIGHT_LONGSIDE_LANEBOUND_NU     // Category weight for orientation calculation cost function for long side lane boundaries.
    type.float32 AP_T_ORI_CATEG_WEIGHT_FALLBACK_NU     // Category weight for orientation calculation cost function for fallback orientation.
    type.float32 AP_T_ORI_CATEG_SCORE_FALLBACK_M     // Category score for orientation calculation cost function for fallback orientation.
    type.float32 AP_T_DEL_REL_OTHERW_CURB_ASSIGNMENT_RATIO_NU     // How far a curb side edge is allowed to go to the road side when adapting to inside-RELATED_OTHERWISE-delimiter. Given as a multitude of the original distance of Curb- and Road-Side.
    type.float32 AP_T_MIN_ANG_TRESH_CMF_BOX_LIMIT_PA_PER_DEG     // Treshold for the min angle between comfort box edge and estimated road side, allowing to limit the comfort box by the estimated road side for parallel and perpendicular use cases.
    type.float32 AP_T_MAX_ANG_TRESH_CMF_BOX_LIMIT_PA_PER_DEG     // Treshold for the max angle between comfort box edge and estimated road side, allowing to limit the comfort box by the estimated road side for parallel and perpendicular use cases.
    type.float32 AP_T_MIN_ANG_TRESH_CMF_BOX_LIMIT_ANGLED_DEG     // Treshold for the min angle between comfort box edge and estimated road side, allowing to limit the comfort box by the estimated road side for angled use cases.
    type.float32 AP_T_MAX_ANG_TRESH_CMF_BOX_LIMIT_ANGLED_DEG     // Treshold for the max angle between comfort box edge and estimated road side, allowing to limit the comfort box by the estimated road side for angled use cases
    type.float32 AP_T_WHEEL_DIST_TO_WHEELSTOPPER_M     // Distance between the wheels of the ego vehicle to the wheel stopper. Remark: In case that the wheel stopper is not perpendicular to the ego vehicle, this is the minimum distance.
    type.float32 AP_T_URGENT_UPDATE_DEV_FACTOR_PERP_LONG_NU     // Enabling an urgend TAPOSD update for perpendicular parking slots, if this factor multiplied with AP_T_MAX_DEV_UPDATE_LONG_PERP_M;is smaller than the relative difference to the yaw angle of the new calculated pose. The higher the factor, the less likely an urgent update.
    type.float32 AP_T_URGENT_UPDATE_DEV_FACTOR_PERP_LAT_NU     // Enabling an urgend TAPOSD update for perpendicular parking slots, if this factor multiplied with AP_T_MAX_DEV_UPDATE_LAT_PERP_M is smaller than the relative difference to the lateral;position of the new calculated pose. The higher the factor, the less likely an urgent update.
    type.float32 AP_T_URGENT_UPDATE_DEV_FACTOR_PERP_ANG_NU     // Enabling an urgend TAPOSD update for perpendicular parking slots, if this factor multiplied with AP_T_MAX_DEV_UPD_YAW_PERP_RAD is smaller than the relative difference to the longitudinal;position of the new calculated pose. The higher the factor, the less likely an urgent update.
    type.float32 AP_T_URGENT_UPDATE_DEV_FACTOR_PAR_LONG_NU     // Enabling an urgend TAPOSD update for parallel parking slots, if this factor multiplied with AP_T_MAX_DEV_UPDATE_LONG_PAR_M is smaller than the relative difference to the yaw angle of;the new calculated pose. The higher the factor, the less likely an urgent update.
    type.float32 AP_T_URGENT_UPDATE_DEV_FACTOR_PAR_LAT_NU     // Enabling an urgend TAPOSD update for parallel parking slots, if this factor multiplied with AP_T_MAX_DEV_UPDATE_LAT_PAR_M is smaller than the relative difference to the lateral position;of the new calculated pose. The higher the factor, the less likely an urgent update.
    type.float32 AP_T_URGENT_UPDATE_DEV_FACTOR_PAR_ANG_NU     // Enabling an urgend TAPOSD update for parallel parking slots, if this factor multiplied with AP_T_MAX_DEV_UPD_YAW_PAR_RAD is smaller than the relative difference to the longitudinal position;of the new calculated pose. The higher the factor, the less likely an urgent update.
    type.float32 AP_T_MAX_CURB_DEL_DIST_TO_ROADSIDE_PAR_M     // Reflects the maximum allowed distance of a curb delimiter to the (estimated) roadside of a parallel parking box.
    type.float32 AP_T_POUT_PERP_MAX_PULL_OUT_DIST_IN_ROAD_DIR_M     // For perpendicular parking limit max allowed shift in road direction to avoid a collision with obstacles, which potentially block the slot to right or left side.
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.FC_TRJPLA_Params uml:Class
  version: ::ap_tp::FC_TRJPLA_Params_InterfaceVersion::FC_TRJPLA_Params_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 AP_P_MAX_DIST_PERP_WO_BCKT_M     // maximum distance (in target y-direction) between the target pose and the current pose to update geometric perpendicular path
    type.float32 AP_P_MAX_DIST_PARA_WO_BCKT_M     // maximum distance (in target x-direction) between the target pose and the current pose to try geometric parallel path
    type.float32 AP_P_SAFE_PASS_MARGIN_M     // used for the first (i.e., actually the very last) move when parking parallely:if there is enough space the vehicle will;go back so far that it would safely pass the front object even if it was going at a minimal radius that was increased by this;amount (effectively increasing the front clearance during the first stroke of parallel perking)
    type.float32 AP_P_VIRTUAL_OBJ_DIST_TO_TP_M     // distance at which a virtual obstacle is created (i.e., distance to vehicle shape at target pose on the relevant side)
    type.float32 AP_P_MAX_PARKING_VEL_FWD_MPS     // maximum parking velocity used by the trajectory planner when driving forwards
    type.float32 AP_P_MAX_PARKING_VEL_BWD_MPS     // maximum parking velocity used by the trajectory planner when driving backwards
    type.float32 AP_P_MIN_PARKING_VEL_MPS     // the minimum velocity used by the trajectory planner
    type.float32 AP_P_PARKOUT_PAR_SAFTEY_DIST_M     // The safty margin the vehicle will have to the critical point when parking out on the final stroke (with an value of 0.0 the vehicle would touch the obstacle)
    type.float32 AP_P_SAFETY_TURN_RADIUS_AXIS_M     // Preferred safety margin between ego vehicle and an object what needs to be passed without colliding at the rear axle. Typically used for perpendicular parking;where the critical collisian point between e.g., the front left edge of the right object is the right end of the ego vehicle"s rear axle.
    type.float32 AP_P_POUT_PERP_STRAIGHT_EXT_M     // Extension of first straight path segment at the beginning of the perpendicular parking out maneuver.
    type.float32 AP_P_MIN_RADIUS_ADD_CIRCLE_M     // minimum turn radius of ego vehicle for planning arc-circle
    type.float32 AP_P_RADIUS_EXT_PERP_FIN_STRO_M     // extension for the min turn radius for the final stroke of perpendicular parking in. (Note: Extension is added to AP_P_MIN_TURN_RADIUS_EXTENSION_FOR_CIRCLE_M)
    type.float32 AP_P_MIN_RADIUS_ADD_CLOTHOID_M     // minimum turn radius of ego vehicle for planning clothoid
    type.float32 AP_P_MAX_DEVIATION_CLOTHOID_M     // maximum deviation of a common rotation center (base circle of clothoid) having a clothoid-circle-clothoid curve
    type.uint8 AP_P_MAX_NUM_CLOTHOID_LOOP_NU     // maximum number of for-loops for iteratively finding reachable radius on clothoid curve
    type.float32 AP_P_CLOTHOID_VEL_TUNE_STEP_MPS     // Used for iteratively finding the fastest clothoid segment combination that can be used to replace CircleCircle or CircleStraight segment combinations
    type.float32 AP_P_PLAN_DISTANCE_MARGIN_M     // planner always adds this value when moving on a straight to avoid numerical porblems when detecting collisions
    type.float32 AP_P_ROI_EDGE_LENGTH_M     // The edge length of the ROI (equal legged L shape)
    type.float32 AP_P_ROI_CORNER_X_PAR_IN_M     // The corner x-position of the L-Shaped-ROI for parallel parking relative to the target pose
    type.float32 AP_P_ROI_CORNER_Y_PAR_IN_M     // The corner y-position of the L-Shaped-ROI for parallel parking relative to the target pose
    type.float32 AP_P_ROI_CORNER_X_PRPF_IN_M     // The corner x-position of the L-Shaped-ROI for perpendicular forward parking relative to the target pose
    type.float32 AP_P_ROI_CORNER_Y_PRPF_IN_M     // The corner y-position of the L-Shaped-ROI for perpendicular forward parking relative to the target pose
    type.float32 AP_P_ROI_CORNER_X_PRPB_IN_M     // The corner x-position of the L-Shaped-ROI for perpendicular backward parking relative to the target pose
    type.float32 AP_P_ROI_CORNER_Y_PRPB_IN_M     // The corner y-position of the L-Shaped-ROI for perpendicular backward parking relative to the target pose
    type.float32 AP_P_ROI_CORNER_X_ANGS_IN_M     // The corner x-position of the L-Shaped-ROI for angled standard (forward) parking relative to the target pose
    type.float32 AP_P_ROI_CORNER_Y_ANGS_IN_M     // The corner y-position of the L-Shaped-ROI for  angled standard (forward) parking relative to the target pose
    type.float32 AP_P_ROI_CORNER_X_ANGR_IN_M     // The corner x-position of the L-Shaped-ROI for angled reverse (backward) parking relative to the target pose
    type.float32 AP_P_ROI_CORNER_Y_ANGR_IN_M     // The corner y-position of the L-Shaped-ROI for angled reverse (backward) parking relative to the target pose
    type.float32 AP_P_ROI_CORNER_X_PAR_OUT_M     // The corner x-position of the L-Shaped-ROI for parallel parking out relative to the start pose
    type.float32 AP_P_ROI_CORNER_Y_PAR_OUT_M     // The corner y-position of the L-Shaped-ROI for parallel parking out relative to the start pose
    type.float32 AP_P_ROI_CORNER_X_PRPF_OUT_M     // The corner x-position of the L-Shaped-ROI for perpendicular forward parking out relative to the start pose
    type.float32 AP_P_ROI_CORNER_Y_PRPF_OUT_M     // The corner y-position of the L-Shaped-ROI for perpendicular forward parking out relative to the start pose
    type.float32 AP_P_ROI_CORNER_X_PRPB_OUT_M     // The corner x-position of the L-Shaped-ROI for perpendicular backward parking out relative to the start pose
    type.float32 AP_P_ROI_CORNER_Y_PRPB_OUT_M     // The corner y-position of the L-Shaped-ROI for perpendicular backward parking out relative to the start pose
    type.float32 AP_P_ROI_CORNER_X_ANGS_OUT_M     // The corner x-position of the L-Shaped-ROI for angled standard (forward) parking out relative to the start pose
    type.float32 AP_P_ROI_CORNER_Y_ANGS_OUT_M     // The corner y-position of the L-Shaped-ROI for  angled standard (forward) parking out relative to the start pose
    type.float32 AP_P_ROI_CORNER_X_ANGR_OUT_M     // The corner x-position of the L-Shaped-ROI for angled reverse (backward) parking out relative to the start pose
    type.float32 AP_P_ROI_CORNER_Y_ANGR_OUT_M     // The corner y-position of the L-Shaped-ROI for angled reverse (backward) parking out relative to the start pose
    type.float32 AP_P_ROI_CORNER_X_PAR_IN_OP_M     // The corner x-position of the L-Shaped-ROI for parallel parking on the opposite roadside relative to the target pose
    type.float32 AP_P_ROI_CORNER_Y_PAR_IN_OP_M     // The corner y-position of the L-Shaped-ROI for parallel parking on the opposite roadside relative to the target pose
    type.float32 AP_P_ROI_CORNER_X_PRPF_IN_OP_M     // The corner x-position of the L-Shaped-ROI for perpendicular forward parking on the opposite roadside relative to the target pose
    type.float32 AP_P_ROI_CORNER_Y_PRPF_IN_OP_M     // The corner y-position of the L-Shaped-ROI for perpendicular forward parking on the opposite roadside relative to the target pose
    type.float32 AP_P_ROI_CORNER_X_PRPB_IN_OP_M     // The corner x-position of the L-Shaped-ROI for perpendicular backward parking on the opposite roadside relative to the target pose
    type.float32 AP_P_ROI_CORNER_Y_PRPB_IN_OP_M     // The corner y-position of the L-Shaped-ROI for perpendicular backward parking on the opposite roadside relative to the target pose
    type.float32 AP_P_ROI_CORNER_X_ANGS_IN_OP_M     // The corner x-position of the L-Shaped-ROI for angled standard (forward) parking on the opposite roadside relative to the target pose
    type.float32 AP_P_ROI_CORNER_Y_ANGS_IN_OP_M     // The corner y-position of the L-Shaped-ROI for  angled standard (forward) parking on the opposite roadside relative to the target pose
    type.float32 AP_P_ROI_CORNER_X_ANGR_IN_OP_M     // The corner x-position of the L-Shaped-ROI for angled reverse (backward) parking on the opposite roadside relative to the target pose
    type.float32 AP_P_ROI_CORNER_Y_ANGR_IN_OP_M     // The corner y-position of the L-Shaped-ROI for angled reverse (backward) parking on the opposite roadside relative to the target pose
    type.float32 AP_P_ROI_CORNER_X_PAR_OUT_OP_M     // The corner x-position of the L-Shaped-ROI for parallel parking on the opposite roadside out relative to the start pose
    type.float32 AP_P_ROI_CORNER_Y_PAR_OUT_OP_M     // The corner y-position of the L-Shaped-ROI for parallel parking on the opposite roadside out relative to the start pose
    type.float32 AP_P_ROI_CORNER_X_PRPB_OUT_OP_M     // The corner x-position of the L-Shaped-ROI for perpendicular forward parking on the opposite roadside out relative to the start pose
    type.float32 AP_P_ROI_CORNER_Y_PRPB_OUT_OP_M     // The corner y-position of the L-Shaped-ROI for perpendicular forward parking on the opposite roadside out relative to the start pose
    type.float32 AP_P_ROI_CORNER_X_PRPF_OUT_OP_M     // The corner x-position of the L-Shaped-ROI for perpendicular backward parking on the opposite roadside out relative to the start pose
    type.float32 AP_P_ROI_CORNER_Y_PRPF_OUT_OP_M     // The corner y-position of the L-Shaped-ROI for perpendicular backward parking on the opposite roadside out relative to the start pose
    type.float32 AP_P_ROI_CORNER_X_ANGS_OUT_OP_M     // The corner x-position of the L-Shaped-ROI for angled standard (forward) parking on the opposite roadside out relative to the start pose
    type.float32 AP_P_ROI_CORNER_Y_ANGS_OUT_OP_M     // The corner y-position of the L-Shaped-ROI for  angled standard (forward) parking on the opposite roadside out relative to the start pose
    type.float32 AP_P_ROI_CORNER_X_ANGR_OUT_OP_M     // The corner x-position of the L-Shaped-ROI for angled reverse (backward) parking on the opposite roadside out relative to the start pose
    type.float32 AP_P_ROI_CORNER_Y_ANGR_OUT_OP_M     // The corner y-position of the L-Shaped-ROI for angled reverse (backward) parking on the opposite roadside out relative to the start pose
    type.float32 AP_P_PRP_FWD_R_SCALE_1ST_STRK_M     // This rate reduces the difference of first stroke turning yaw angle between circle-planner and clothoid-planner in order to avoid the first stroke collision in forward park-in
    type.boolean AP_P_DISABLE_SCAN_GREAT_DIST_NU     // Disables updating the planned path for parking spaces further away then AP_P_MAX_DIST_PERP_WO_BCKT_M or AP_P_MAX_DIST_PARA_WO_BCKT_M
    type.float32 AP_P_MAX_VEL_CLOSE_OBSTACLE_MPS     // the maximum parking velocity used by the trajectory planner when driving close to an obstacle
    type.float32 AP_P_OUT_VRTX_RADIUS_SCALING_NU     // The scaling factor for the radius that is sufficient to pass an object with the outermost vertex (i.e., the found radius is multiplied by this value to actually pass the obstacle with a greater safety margin)
    type.uint16 AP_P_MIN_NUM_VALID_TO_OUTPUT_NU     // Minimum number of successfull path finding calls for a target pose before it is considered valid
    type.float32 AP_P_MAX_VEL_IN_PAR_SLOT_MPS     // Maximum velocity within parallel parking slot (i.e. during back and forth) used by geometric planner
    type.boolean AP_P_DISABLE_PLANNER_NU     // Disable planner completley (only init and then run proides default outputs)
    type.float32 AP_P_DIRECT_FWD_STEP_DIST_M     // Driving distance per sampling step during direct forward planning
    type.float32 AP_P_DIRECT_FWD_START_ANGLE_TOL_DEG     // Angle tolerance from target pose to try direct forward planning, negative to disable direct forward planning.
    type.uint8 AP_P_MAX_NUM_CLO_VEL_LOOP_NU     // maximum number of for-loops for iteratively finding reachable velocity on clothoid curve
    type.uint8 AP_P_SAVE_EM_REPLANNING_NU     // If 1, save a snapshot of current scene EM in a json file for later analysis
    type.float32 AP_P_ASY_CLO_ST_MIN_LENGTH_M     // Minimum length of start / end clothoid in smooth curve for changing curvature from A to B
    type.float32 AP_P_ADAPT_VEL_X_DISTANCE_M     // Lookuptable x-values (distance) for adaption of velocity to distance to objects
    type.float32 AP_P_ADAPT_VEL_Y_VELOCITY_MPS     // Lookuptable y-values (velocity) for adaption of velocity to distance to objects
    type.uint8 AP_P_ADAPT_VEL_SIZE_NU     // Lookuptable size for adaption of velocity to distance to objects
    type.float32 AP_P_STRAIGHT_PERP_EXTRA_DIST_M     // Straight distance added to the first stroke perpendicular to avoid the an extra step due to EM slot measurement inaccuracy
    type.uint8 AP_P_PARKOUT_PAR_ANGLE_MAX_LOOP     // Max number of loops to calculate the angle for the last stroke for parallel parking out
    type.float32 AP_P_MAX_ALIGNMENT_RADIUS_M     // Maximum senseful radius to align to target axis
    type.float32 AP_P_MIN_ALIGNMENT_ANGLE_RAD     // Minimum senseful angle to align to target axis
    type.float32 AP_P_PLANNING_HORIZON_M     // Planning horizon in Garage Parking / Remote Maneuvering Mode
    type.float32 AP_P_ENDPOSE_NEIGHBOUR_DIST_M     // Distance between neighbouring endPoses of candidate paths in RM Mode
    type.float32 AP_P_DESIRED_DIST_TO_OBST_M     // Desired minimum distance to obstacles in RM mode
    type.float32 AP_P_FACTOR_DETERMINATION_NU     // Factor for determincation in cost function in RM mode
    type.float32 AP_P_FACTOR_SAFETY_NU     // Factor for safety in cost function in RM mode
    type.float32 AP_P_MAX_POLY_VEL_MPS     // max. driving velocity of polynomial paths
    type.float32 AP_P_ENV_FLICKERING_COMP_M     // compensation value for EM flickering, f.e. used for going until collision (EM flickering would lead to collision-check-flickering)
    type.float32 AP_P_DIST_START_END_KEEP_PATH_M     // maximum distance between start and end pose (in x-direction) to keep the old path when using the polynomial planner
    type.float32 AP_P_STRAIGHT_CONV_ALLOWED_ANG_RAD     // maximum angle between start and endPose of an polynomial segment to consider it as a straight
    type.float32 AP_P_MAX_PREP_STROKE_LEN_STRAIGHT_M     // maximum lenght of the straight preparation stroke in garage parking
    type.float32 AP_P_MIN_PREP_STROKE_LEN_STRAIGHT_M     // minimum lenght of the straight preparation stroke in garage parking
    type.float32 AP_P_MIN_ANG_DIFF_INFLECTION_RAD     // minimum angular difference between a curve extremum and inflection point for polynomial paths
    type.boolean AP_P_ENABLE_ONE_STROKE_PARKING     // Enabler for one stroke perpendicular parking
    type.float32 AP_P_MAX_DIST_TP_TO_INTERMP_ONE_STROKE_M     // Distance from target pose to intermediate pose in target poses" x-direction
    type.float32 AP_P_PERP_END_LIMIT_VEL_DIST_M     // The distance on the last stroke for which a lower velocity shall be used
    type.float32 AP_P_PAR_MAX_DIST_FOR_SYM_SCURVE_M     // The distance from end to start pose in which we allow to use a symmetric S-curve. For distances larger than this, minPlanRadius is used for planning
    type.float32 AP_P_PERP_FWD_IN_LAT_DEV_FCTR     // A weight value given for the impact of the parallel lateral deviation in the overall deviation score
    type.float32 AP_P_PERP_FWD_IN_LONG_DEV_FCTR     // A weight value given for the impact of the parallel lateral deviation in the overall deviation score
    type.float32 AP_P_PERP_FWD_IN_YAW_DEV_FCTR     // A weight value given for the impact of the parallel lateral deviation in the overall deviation score
    type.float32 AP_P_PERP_BWD_IN_LAT_DEV_FCTR     // A weight value given for the impact of the parallel lateral deviation in the overall deviation score
    type.float32 AP_P_PERP_BWD_IN_LONG_DEV_FCTR     // A weight value given for the impact of the parallel lateral deviation in the overall deviation score
    type.float32 AP_P_PERP_BWD_IN_YAW_DEV_FCTR     // A weight value given for the impact of the parallel lateral deviation in the overall deviation score
    type.boolean AP_P_ENABLE_DYNAMIC_REPLANNING     // If true planner will try to do dynamic replanning, otherwise replanning only at end of strokes.
    type.float32 AP_P_PAR_NODYNREPLAN_DIST_TO_STOP_M     // Distance to stop where dyn. replanning is not allowed on the 1st scurve of the parallel parking
    type.float32 AP_P_INC_CIR_LENGTH_FOR_SCUR_OUT_M     // extending circle length of lower part of s-curve (i.e., the part within the slot) to avoid the collision if scurve out  stroke (used for dyn replynding) has a collision
    type.uint8 AP_P_NUM_TA_POS_EVAL     // Number of target pose to be evaluated by the planner
    type.uint16 AP_P_NUM_POSE_CHECK_DRVNPATH     // Number of pose checks along the driven path to find the best start pose in intial planning(half before/half behind best found pose during scanning)
    type.uint8 AP_P_BSTPOSE_STEPSZ_DRVNPATH_NU     // in initial planning we check AP_P_NUM_POSE_CHECK_DRVNPATH samples along the driven path (which is sampled equidistantly with AP_P_SAMPLE_POINT_DISTANCE_M);to find the optimal planning start pose. This parameter describes that we only want to check every AP_P_BSTPOSE_STEPSZ_DRVNPATH_NU-th of these driven path;samples (i.e., it is a multiplier for the sample distance).
    type.float32 AP_P_MIN_DIST_START_TGTAXIS_M     // Distance by which the start pose has to pass the target axis to be considered for perpendicular bwd in planning
    type.float32 AP_P_BEST_PATH_GOAL_DEV_THR_NU     // Decision threshold for one path better than an other one. Description how close a path is to the target pose. Weighted sum other the lat, long and yaw deviation of path end pose and target pose
    type.float32 AP_P_BEST_PATH_LENGTH_THR_M     // Decision threshold for one path better than an other one. Overall path length
    type.float32 AP_P_BEST_PATH_TIME_THR_S     // Decision threshold for one path better than an other one. Time needed for the calculated path
    type.boolean AP_P_ENABLE_OUT_IN_STROKES_REPLAN_PARALLEL     // If true planner will do out in strokes in parallel dynamic replanning otherwise planner will subsequently fail and do static replanning
    type.boolean AP_P_ENABLE_LAST_CALL_MANEUVER_PERP     // if true planner will do a last call maneuver for perpendicular parking
    type.float32 AP_P_DISTANCE_FOR_LAST_CALL_MAN_PERP     // Distance to define the area around the target pose in which the last call maneuver for perp parking is triggered
    type.float32 AP_P_MAX_SCURVE_DIST_TO_TGT_POSE_M     // Maximum distance of ego vehicle to target pose for a s-curve align maneuver
    type.boolean AP_P_ENABLE_FIRST_STROKE_FWD_PERP_BWD     // If true planner will allow the driver to stop earlier and try a first stroke fwd maneuver
    type.float32 AP_P_PERP_STRAIGHT_MIN_BWD_DIST_FWD_STRK_M     // Desired minimum straight maneuver distance for perpendicular parking
    type.float32 AP_P_MAX_DISTANCE_SHORTEN_FIRST_STRK_FWD     // max distance between start pose and intersection with target axis that may be shortened;to have an earlier steering and a steeper angle after the first stroke in first stroke fwd maneuvers
    type.float32 AP_P_RATIO_DISTANCE_SHORTING_FIRST_STRK_FWD     // ratio of how much of AP_P_MAX_DISTANCE_SHORTEN_FIRST_STRK_FWD is shortened, required to have a smooth;planning for different start poses
    type.boolean AP_P_ENABLE_EARLY_STEER_FIRST_STRK_FWD     // if true planner tries to early steer and have a steep angle after first stroke in first stroke maneuvers as required for Toyota demo
    type.float32 AP_P_RATIO_ADD_MIN_RADIUS_ANGLED     // ratio of how much of the align radius for angled reverse out is added to the min planning radius
    type.float32 AP_P_RATIO_ADD_ALIGN_RADIUS_ANGLED     // ratio of how much of the align radius for angled reverse out is added to the align radius; has to be greater than 1
    type.boolean AP_P_ENABLE_FIRST_STRK_FWD_PAR     // if true planner tries to early steer and have a steep angle after first stroke in first stroke maneuvers as required for Toyota demo
    type.float32 AP_P_ADD_STRAIGHT_FSF_M     // Additional straight for fsf parallel maneuvers, in same direction as s-curve to improve robustness of the fsf.
    type.float32 AP_P_ADD_STRAIGHT_TO_WHEELSTOPPER_M     // Added straight from target pose to wheelstopper for wheel touching feature of MoCo.
    type.float32 AP_P_LONG_SHIFT_INTERMED_WS_TAPOS_M     // Longitudinal distance for the intermediate tapos shifted from the original wheelstopper target pose to allow for approaching the wheelstopper with non rotated wheels
    type.float32 AP_P_MAX_HEIGHT_OF_TRIANGLE_CLO_COLLISION_M     // Threshold for the max height of triangle to wrap up the most changeable corner-vertex of vehicle shape, in oder to detect vehicle collision when vehicle passes through a clothoid segment
    type.float32 AP_P_SNAP_RATIO_TO_AVOID_COLL     // Distance ratio in a snapshot to avoid collision after calculation of distance to stop of clothoid driving tube
    type.float32 AP_P_STEP_MOVING_RATIO_CLO_COLL     // Ratio of mvoing ego a short step to build wrap-up trianlge for dominated contour in clothoid drving tube
    type.uint32 AP_P_MAX_ATTEMPT_WRAP_UP_TRIANGLE     // Max. attemps to find wrap-up trianlge for dominated contour in clothoid drving tube
    type.uint32 AP_P_KEY_CORNERS_CARBODY     // Indices of key car shape corners to build clothoid driving tube
    type.uint32 AP_P_KEY_CORNERS_WHEEL     // Indices of key wheel shape corners to build clothoid driving tube
    type.float32 AP_P_MIN_LONG_DEV_FOR_STRAIGHT_REPL_PAR_M     // Longitudinal deviation between intermediate pose and new target pose after s-curve has to be bigger than this value to not add an additional straight after the s-curve for better replanning.
    type.float32 AP_P_MAX_LAT_DEV_FOR_STRAIGHT_REPL_PAR_M     // Lateral deviation between intermediate pose and new target pose after s-curve has to be smaller than this value to not add an additional straight after the s-curve for better replanning.
    type.float32 AP_P_MAX_ORIENTATION_DEV_FOR_STRAIGHT_REPL_PAR_RAD     // Orientation deviation between intermediate pose and new target pose after s-curve has to be smaller than this value to not add an additional straight after the s-curve for better replanning.
    type.float32 AP_P_PAR_REPL_MAX_EXTEND_DIST_AFTER_SCURVE_M     // In parallel dynamic replanning, when driving bwd the stroke is extended until collision or if there is no collision until the maximum value set here. This is to get enough space for maneuvering afterwards.
    type.float32 AP_P_RATIO_STRAIGHT_PREPARE_FWD_STROKE_PREP_ALIGN     // For prepare alignment in fwd direction, the distance to intersection of ego and target is partially driven by a straight if the align radius is bigger than min planning radius, to get a proper preparation of the alignment.
    type.float32 AP_P_RATIO_PRE_PREP_ALIGN_AXIS     // In pre-prepare alignment maneuver, a very first alignment preparation of a rotated target pose axis is done to get a better position for the following alignment preparation.;The rotation angle is calculated by the angle between ego and target with the here given ratio.
    type.float32 AP_P_PRE_PREP_ALIGN_RADIUS_EXT_M     // Pre-prepare alignment is done in the first stroke where a bigger maneuver is possible to get a good starting point for preparation of the alignment. Therefore, the radius is enlarged here.
    type.float32 AP_P_RATIO_CONFINED_SPACE     // In PERP BWD situations, where a static obstable is opposite to the parking box, we consider the space to be confined if the available space for maneuvering after the first;stroke is less than this percentage of the ego length. If the parking box is considered confined, a starting position with greater maneuvering space is chosen.
    type.float32 AP_P_MISALIGNED_START_POSE_ANGLE_RAD     // Angle between start pose and target pose which defines the threshold if the maneuver has a misaligned start pose or not.
    type.float32 AP_P_DIST_TO_PB_FOR_MISAL_START_POSE_PLANNING_M     // In misaligned start pose maneuvers, a first stroke to a pose parallel to the road side edge of the parking box is planned. This parameter describes the distance of this pose to the parking box.
    type.float32 AP_P_LENGTH_LONG_CLO_TRANSIT_M     // Length of long transition for clothoidizing
    type.float32 AP_P_LENGTH_SHORT_CLO_TRANSIT_M     // Length of short transition for clothoidizing
    type.boolean AP_P_PARKOUT_ENABLE_FAKE_OBJECTS     // @range{0,1};@unit{none};Enable fake objects sorrounding the ego vehicle after ignition cycle until valid EM is available from SI
    type.float32 AP_P_PARKOUT_EGO_DIST_TO_FAKE_OBJECTS_PAR_M     // @range{0,unbound};@unit{m};Distance between created fake objects (in front and behind ego) and ego vehicle for parallel park out after ignition cycle
    type.float32 AP_P_PARKOUT_EGO_DIST_TO_FAKE_OBJECTS_PERP_M     // @range{0,unbound};@unit{m};Distance between created fake objects (in front and behind ego) and ego vehicle for perpendicular park out after ignition cycle
    type.float32 AP_P_PARKOUT_EGO_DIST_TO_FAKE_OBJECTS_ANG_M     // @range{0,unbound};@unit{m};Distance between created fake objects (in front and behind ego) and ego vehicle for angular park out after ignition cycle
    type.float32 AP_P_PARKOUT_BLIND_DISTANCE_PERP_M     // @range{0,unbound};@unit{m};Maximum distance in park out which will be driven without EM information
    type.float32 AP_P_PARKOUT_BLIND_DISTANCE_ANG_M     // @range{0,unbound};@unit{m};Maximum distance in park out which will be driven without EM information
    type.float32 AP_P_PARKOUT_BLIND_DISTANCE_PAR_M     // @range{0,unbound};@unit{m};Maximum distance in park out which will be driven without EM information
    type.AP_TP.FC_TAPOSD_Params taposdParams     // Nested struct for TAPOSD Params
    type.AP_Common.FC_TRJPLA_Vehicle_Params vehicleParams     // Nested struct for vehicle params section
    type.AP_Common.FC_TRJPLA_Sys_Func_Params sysFuncParams     // Nested struct for sys func params section
    type.float32 MP_P_ROI_EDGE_LENGTH_M     // The edge length of the ROI (equal legged L shape) for memory parking.
    type.float32 MP_P_ROI_CORNER_X_PAR_IN_M     // The corner x-position of the L-Shaped-ROI for parallel parking relative to the target pose for memory parking.
    type.float32 MP_P_ROI_CORNER_Y_PAR_IN_M     // The corner y-position of the L-Shaped-ROI for parallel parking relative to the target pose for memory parking.
    type.float32 MP_P_ROI_CORNER_X_PRPF_IN_M     // The corner x-position of the L-Shaped-ROI for perpendicular forward parking relative to the target pose for memory parking.
    type.float32 MP_P_ROI_CORNER_Y_PRPF_IN_M     // The corner y-position of the L-Shaped-ROI for perpendicular forward parking relative to the target pose for memory parking.
    type.float32 MP_P_ROI_CORNER_X_PRPB_IN_M     // The corner x-position of the L-Shaped-ROI for perpendicular backward parking relative to the target pose for memory parking.
    type.float32 MP_P_ROI_CORNER_Y_PRPB_IN_M     // The corner y-position of the L-Shaped-ROI for perpendicular backward parking relative to the target pose for memory parking.
    type.float32 MP_P_ROI_CORNER_X_ANGS_IN_M     // The corner x-position of the L-Shaped-ROI for angled standard (forward) parking relative to the target pose for memory parking.
    type.float32 MP_P_ROI_CORNER_Y_ANGS_IN_M     // The corner y-position of the L-Shaped-ROI for  angled standard (forward) parking relative to the target pose for memory parking.
    type.float32 MP_P_ROI_CORNER_X_ANGR_IN_M     // The corner x-position of the L-Shaped-ROI for angled reverse (backward) parking relative to the target pose for memory parking.
    type.float32 MP_P_ROI_CORNER_Y_ANGR_IN_M     // The corner y-position of the L-Shaped-ROI for angled reverse (backward) parking relative to the target pose for memory parking.
    type.float32 MP_P_ROI_CORNER_X_PAR_OUT_M     // The corner x-position of the L-Shaped-ROI for parallel parking out relative to the start pose for memory parking.
    type.float32 MP_P_ROI_CORNER_Y_PAR_OUT_M     // The corner y-position of the L-Shaped-ROI for parallel parking out relative to the start pose for memory parking.
    type.float32 MP_P_ROI_CORNER_X_PRPF_OUT_M     // The corner x-position of the L-Shaped-ROI for perpendicular forward parking out relative to the start pose for memory parking.
    type.float32 MP_P_ROI_CORNER_Y_PRPF_OUT_M     // The corner y-position of the L-Shaped-ROI for perpendicular forward parking out relative to the start pose for memory parking.
    type.float32 MP_P_ROI_CORNER_X_PRPB_OUT_M     // The corner x-position of the L-Shaped-ROI for perpendicular backward parking out relative to the start pose for memory parking.
    type.float32 MP_P_ROI_CORNER_Y_PRPB_OUT_M     // The corner y-position of the L-Shaped-ROI for perpendicular backward parking out relative to the start pose for memory parking.
    type.float32 MP_P_ROI_CORNER_X_ANGS_OUT_M     // The corner x-position of the L-Shaped-ROI for angled standard (forward) parking out relative to the start pose for memory parking.
    type.float32 MP_P_ROI_CORNER_Y_ANGS_OUT_M     // The corner y-position of the L-Shaped-ROI for  angled standard (forward) parking out relative to the start pose for memory parking.
    type.float32 MP_P_ROI_CORNER_X_ANGR_OUT_M     // The corner x-position of the L-Shaped-ROI for angled reverse (backward) parking out relative to the start pose for memory parking.
    type.float32 MP_P_ROI_CORNER_Y_ANGR_OUT_M     // The corner y-position of the L-Shaped-ROI for angled reverse (backward) parking out relative to the start pose for memory parking.
    type.float32 MP_P_ROI_CORNER_X_PAR_IN_OP_M     // The corner x-position of the L-Shaped-ROI for parallel parking on the opposite roadside relative to the target pose for memory parking.
    type.float32 MP_P_ROI_CORNER_Y_PAR_IN_OP_M     // The corner y-position of the L-Shaped-ROI for parallel parking on the opposite roadside relative to the target pose for memory parking.
    type.float32 MP_P_ROI_CORNER_X_PRPF_IN_OP_M     // The corner x-position of the L-Shaped-ROI for perpendicular forward parking on the opposite roadside relative to the target pose for memory parking.
    type.float32 MP_P_ROI_CORNER_Y_PRPF_IN_OP_M     // The corner y-position of the L-Shaped-ROI for perpendicular forward parking on the opposite roadside relative to the target pose for memory parking.
    type.float32 MP_P_ROI_CORNER_X_PRPB_IN_OP_M     // The corner x-position of the L-Shaped-ROI for perpendicular backward parking on the opposite roadside relative to the target pose for memory parking.
    type.float32 MP_P_ROI_CORNER_Y_PRPB_IN_OP_M     // The corner y-position of the L-Shaped-ROI for perpendicular backward parking on the opposite roadside relative to the target pose for memory parking.
    type.float32 MP_P_ROI_CORNER_X_ANGS_IN_OP_M     // The corner x-position of the L-Shaped-ROI for angled standard (forward) parking on the opposite roadside relative to the target pose for memory parking.
    type.float32 MP_P_ROI_CORNER_Y_ANGS_IN_OP_M     // The corner y-position of the L-Shaped-ROI for  angled standard (forward) parking on the opposite roadside relative to the target pose for memory parking.
    type.float32 MP_P_ROI_CORNER_X_ANGR_IN_OP_M     // The corner x-position of the L-Shaped-ROI for angled reverse (backward) parking on the opposite roadside relative to the target pose for memory parking.
    type.float32 MP_P_ROI_CORNER_Y_ANGR_IN_OP_M     // The corner y-position of the L-Shaped-ROI for angled reverse (backward) parking on the opposite roadside relative to the target pose for memory parking.
    type.float32 MP_P_ROI_CORNER_X_PAR_OUT_OP_M     // The corner x-position of the L-Shaped-ROI for parallel parking on the opposite roadside out relative to the start pose for memory parking.
    type.float32 MP_P_ROI_CORNER_Y_PAR_OUT_OP_M     // The corner y-position of the L-Shaped-ROI for parallel parking on the opposite roadside out relative to the start pose for memory parking.
    type.float32 MP_P_ROI_CORNER_X_PRPB_OUT_OP_M     // The corner x-position of the L-Shaped-ROI for perpendicular forward parking on the opposite roadside out relative to the start pose for memory parking.
    type.float32 MP_P_ROI_CORNER_Y_PRPB_OUT_OP_M     // The corner y-position of the L-Shaped-ROI for perpendicular forward parking on the opposite roadside out relative to the start pose for memory parking.
    type.float32 MP_P_ROI_CORNER_X_PRPF_OUT_OP_M     // The corner x-position of the L-Shaped-ROI for perpendicular backward parking on the opposite roadside out relative to the start pose for memory parking.
    type.float32 MP_P_ROI_CORNER_Y_PRPF_OUT_OP_M     // The corner y-position of the L-Shaped-ROI for perpendicular backward parking on the opposite roadside out relative to the start pose for memory parking.
    type.float32 MP_P_ROI_CORNER_X_ANGS_OUT_OP_M     // The corner x-position of the L-Shaped-ROI for angled standard (forward) parking on the opposite roadside out relative to the start pose for memory parking.
    type.float32 MP_P_ROI_CORNER_Y_ANGS_OUT_OP_M     // The corner y-position of the L-Shaped-ROI for  angled standard (forward) parking on the opposite roadside out relative to the start pose for memory parking.
    type.float32 MP_P_ROI_CORNER_X_ANGR_OUT_OP_M     // The corner x-position of the L-Shaped-ROI for angled reverse (backward) parking on the opposite roadside out relative to the start pose for memory parking.
    type.float32 MP_P_ROI_CORNER_Y_ANGR_OUT_OP_M     // The corner y-position of the L-Shaped-ROI for angled reverse (backward) parking on the opposite roadside out relative to the start pose for memory parking.
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.FC_TRJPLA_Params_InterfaceVersion uml:Class
  Members:
    type.uint32 FC_TRJPLA_Params_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.StoredWaypointData uml:Class
    // Data structure defining a waypoint to be stored
  Members:
    type.LSM_GEOML.Pose_POD pose     // Pose at the stored waypoint
    type.float32 crv_1pm     // @unit{1pm};@range{-0.275,0.275};Curvature at the stored waypoint (signed), limited by the vehicle kinematics
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.DrivenPathDataPort uml:Class
  version: ::ap_tp::DrivenPathDataPort_InterfaceVersion::DrivenPathDataPort_VERSION
    // Data structure for storing way points of a driven path
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     // @unit{nu};Version number of the driven path data port
    type.eco.SignalHeader sSigHeader     // Signal information
    type.uint8 saveCounter     // @unit{nu};@range{0,255};Counter changes when saving data requested, can overflow
    type.boolean hasValidData     // @unit{nu};@range{0,1};Indicator whether port contains valid data
    type.AP_TP.StoredWaypointData storedDrivenPath     // Actual driven path containing samples in one driving direction
    type.uint16 numElementsInDrivenPath     // @unit{nu};@range{0,AP_TP.AP_TP_Const.AP_P_MAX_NUM_SAMPLES_IN_DRIVEN_PATH};Number of valid elements in driven path
    type.AP_TP.StoredWaypointData buffer     // Buffer for storing way points which are in the opposite direction of the driven path
    type.uint8 numElementsInBuffer     // @unit{nu};@range{0,AP_TP.AP_TP_Const.AP_P_MAX_NUM_SAMPLES_IN_DRIVEN_PATH_BUFFER};Number of valid elements in buffer
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TP.DrivenPathDataPort_InterfaceVersion uml:Class
  Members:
    type.uint32 DrivenPathDataPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.LaDMCCtrlRequestInterfaceType uml:Enumeration
    // Type of the LaDMC control request
  Members:
    LADMC_REQ_TYPE_STEER_WHEEL_ANGLE = 0            
    LADMC_REQ_TYPE_ANGLE_FRONT = 1            
    LADMC_REQ_TYPE_ANGLE_REAR = 2            
    LADMC_REQ_TYPE_ANGLE_FRONT_REAR = 3            
    LADMC_REQ_TYPE_TORQUE_FRONT = 4            
    LADMC_REQ_TYPE_TORQUE_REAR = 5            
    LADMC_REQ_TYPE_TORQUE_FRONT_REAR = 6            
    LADMC_REQ_TYPE_CURVATURE = 7            
    MAX_NUM_LADMC_REQUEST_INTERFACE_TYPE = 8            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.LaDMCCtrlRequestSourceType uml:Enumeration
    // Source of the LaDMC control request
  Members:
    LADMC_REQ_SRC_NO_REQEUESTER = 0            
    LADMC_REQ_SRC_AUP = 1            
    LADMC_REQ_SRC_LSCA = 2            
    LADMC_REQ_SRC_REMOTE = 3            
    LADMC_REQ_SRC_MSP = 4            
    LADMC_REQ_SRC_SAFBAR = 5            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.LoDMCCtrlRequestSourceType uml:Enumeration
    // Source of the LoDMC control request
  Members:
    LODMC_REQ_SRC_NO_REQEUESTER = 0            
    LODMC_REQ_SRC_AUP = 1            
    LODMC_REQ_SRC_LSCA = 2            
    LODMC_REQ_SRC_REMOTE = 3            
    LODMC_REQ_SRC_MSP = 4            
    LODMC_REQ_SRC_SAFBAR = 5            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.LoDMCCtrlRequestType uml:Enumeration
    // Type of the LoDMC control request
  Members:
    LODMC_NO_REQUEST = 0            
    LODMC_NORMAL_REQUEST = 1            
    LODMC_DEGRADATION_REQUEST = 2            
    LODMC_OVERRIDE_REQUEST = 3            
    LODMC_REMOTE_SELFTEST_REQUEST = 4            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.LoDMCCtrlRequestInterfaceType uml:Enumeration
    // Type of the LoDMC control request
  Members:
    LODMC_REQ_TYPE_NO_REQUEST = 0            
    LODMC_REQ_TYPE_DISTANCE = 1            
    LODMC_REQ_TYPE_DISTANCE_VELOCITY = 2            
    LODMC_REQ_TYPE_VELOCITY = 3            
    LODMC_REQ_TYPE_ACCELERATION = 4            
    LODMC_REQ_TYPE_VELOCITY_LIMITATION = 5            
    LODMC_REQ_TYPE_TORQUE_FRONT_REAR = 6            
    MAX_NUM_LODMC_REQUEST_INTERFACE_TYPE = 7            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.LoDMCHoldRequestType uml:Enumeration
    // Type of the LoDMC hold request
  Members:
    LODMC_HOLD_REQ_OFF = 0            
    LODMC_HOLD_REQ_ON = 1            
    LODMC_HOLD_REQ_RAMP_TO_DRIVER = 2            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.MF_CONTROL_te_LongCtrlMode uml:Enumeration
    // Type of the longitudinal control mode (MoCo variant only)
  Members:
    MF_LONG_CTRL_MODE_NO_CONTROL = 0            
    MF_LONG_CTRL_MODE_DIST_VELO_MAX = 1            
    MF_LONG_CTRL_MODE_ACCEL = 2            
    MF_LONG_CTRL_MODE_VELO_LIMITATION = 3            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.DrvResType uml:Enumeration
    // Type of the driving resistance. (e.g. for crossing curbstone or approaching wheel stopper)
  Members:
    DRVRES_NONE = 0            
    DRVRES_FALLING_LOW = 1            
    DRVRES_FALLING_MEDIUM = 2            
    DRVRES_FALLING_HIGH = 3            
    DRVRES_RISING_LOW = 4            
    DRVRES_RISING_MEDIUM = 5            
    DRVRES_RISING_HIGH = 6            
    DRVRES_WHEEL_STOPPER = 7            
    MAX_NUM_DRVRES_TYPE = 8            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.LodmcHandshakeFailedStatus uml:Enumeration
    // Information as to handshake failure.
  Members:
    LODMC_NO_HANDSHAKE_FAILURE = 0            
    LODMC_LSCA_HANDSHAKE_FAILURE = 1            
    LODMC_MSP_HANDSHAKE_FAILURE = 2            
    LODMC_AUP_HANDSHAKE_FAILURE = 3            
    LODMC_REMOTE_HANDSHAKE_FAILURE = 4            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.LadmcHandshakeFailedStatus uml:Enumeration
    // Information as to handshake failure.
  Members:
    LADMC_NO_HANDSHAKE_FAILURE = 0            
    LADMC_LSCA_HANDSHAKE_FAILURE = 1            
    LADMC_AUP_HANDSHAKE_FAILURE = 2            
    LADMC_REMOTE_HANDSHAKE_FAILURE = 3            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.LateralControlSaturationStatus uml:Enumeration
    // Saturation of lateral control (none, to the left, to the right, unknown direction)
  Members:
    LACTRL_NO_SATURATION = 0            
    LACTRL_SATURATION_TO_LEFT = 1            
    LACTRL_SATURATION_TO_RIGHT = 2            
    LACTRL_SATURATION_DIRECTION_UNKNOWN = 3            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.AP_TRJCTL_Consts uml:Class
  Members:
    type.uint8 NUM_MTS_DEBUG_FREESPACE_CTL = 10    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.MF_CONTROL_t_LongCtrlReq uml:Class
  version: ::ap_trjctl::MF_CONTROL_t_LongCtrlReq_InterfaceVersion::MF_CONTROL_t_LongCtrlReq_VERSION
    // Longitudinal control request containing the control mode and control reference values. (MoCo variant only)
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     // Common header for all structured data types (e.g. timestamp)
    type.float32 distanceToStopReq_m     // @unit{m};@range{0,30};Value for the remaining distance to stop the vehicle at rear axle center. (*used as longitudinal reference value in case of distance control)
    type.float32 veloLimMax_mps     // @unit{m/s};@range{0,5};Value for the velocity limit / target vehicle velocity. (*used as longitudinal reference value in case of velocity control; *used as limit for the vehicle velocity in case of distance and velocity control)
    type.float32 accelReq_mps2     // @unit{m/s2};@range{-10,10};Reference / Target value for acceleration control.
    type.AP_TRJCTL.MF_CONTROL_te_LongCtrlMode longCtrlMode_nu     
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.MF_CONTROL_t_LongCtrlReq_InterfaceVersion uml:Class
  Members:
    type.uint32 MF_CONTROL_t_LongCtrlReq_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.DrvRes uml:Class
    // Wheel specific driving resistance to consider for example crossing curbstone and wheel stopper scenarios
  Members:
    type.float32 distance_m     // @unit{m};@range{0,10};Distance to the wheel individual driving resistance based on the intended movement of the rear axle center.
    type.AP_TRJCTL.DrvResType type_nu     // @range{0,8};Type of the driving resistance.
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.MFControlStatusPort uml:Class
  version: ::ap_trjctl::MFControlStatusPort_InterfaceVersion::MFControlStatusPort_VERSION
    // Status signals for handshake / communication with extern /overlaid function components. (e.g. Parking State Machine and Trajectory Planning)
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint8 numUsedCtrlPoints_nu     // @range{0,255};Number of used trajectory points in planned trajectory input for trajectory control. (e.g. if value is 5 the trajectory points with index 0,1,2,3,4 are used) (Changing trajectory points (e.g. dynamic replanning) with an index below might cause issues in the control loop e.g. jerks in steering wheel)
    type.AP_TRJCTL.LodmcHandshakeFailedStatus lodmcHandshakeFailedStatus_nu     // @range{0,4};Information that handshake with LODMC could not be established, specifies which type of function request failed handshake with LODMC.
    type.AP_TRJCTL.LadmcHandshakeFailedStatus ladmcHandshakeFailedStatus_nu     // @range{0,3};Information that handshake with LADMC could not be established, specifies which type of function request failed handshake with LADMC.
    type.boolean correctGearEngaged_nu     // Information that the engaged gear is correct regarding the requested driving direction.
    type.boolean longitudinalControlFinished_nu     // Information that longitudinal control request was finished. (e.g. requested stop point reached in case of distance control)
    type.boolean lateralControlFinished_nu     // Information that lateral control request was finished. (e.g. requested steer angle reached in case of steer angle request)
    type.boolean longitudinalControlFailed_nu     // Information that longitudinal control failed. (e.g. overshoot of requested stop point in case of distance control)
    type.boolean lateralPathControlFailed_nu     // Information that lateral path control failed. (e.g. too high deviation from trajectory)
    type.boolean longitudinalPathControlFailed_nu     // Information that longitudinal path control failed. (e.g. longitudinal outside trajectory in critical distance)
    type.boolean lateralControlFailed_nu     // Information that lateral control failed. (e.g. a requested steer angle can"t be reached because of blockage of wheels)
    type.boolean longitudinalControlSaturated_nu     // Information that longitudinal control is saturated. (e.g. a requested control request can"t be performed in the requested driving direction because of driving resistance blocking the driving path)
    type.AP_TRJCTL.LateralControlSaturationStatus lateralControlSaturationStatus_nu     // @range{0,3};Information about lateral control saturation status: is not in saturation or in saturation to the left or to the right or unknown direction. (e.g. a requested steer angle can"t be reached because of blockage of wheels)
    type.boolean vehStandstillHold_nu     // Information that vehicle standstill is hold by brake system.
    type.boolean vehStandstillSecured_nu     // Information that the vehicle is secured. (e.g. via electric parking brake)
    type.boolean driverSteerIntervDetected_nu     // Information that a driver steer intervention was detected.
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.MFControlStatusPort_InterfaceVersion uml:Class
  Members:
    type.uint32 MFControlStatusPort_VERSION = 4    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.LaDMCCtrlRequestPort uml:Class
  version: ::ap_trjctl::LaDMCCtrlRequestPort_InterfaceVersion::LaDMCCtrlRequestPort_VERSION
    // Request to underlaid Lateral Dynamic Motion Control (LaDMC).
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 steerWheelAngReq_rad     // @unit{rad};@range{-15,15};Requested value for the steering wheel angle.
    type.float32 frontSteerAngReq_rad     // @unit{rad};@range{-0.8,0.8};Requested value for the front steer angle.
    type.float32 rearSteerAngReq_rad     // @unit{rad};@range{-0.8,0.8};Requested value for the rear steer angle.
    type.float32 frontSteerTorqueReq_Nm     // @unit{Nm};@range{-8,8};Requested value for the front steer torque
    type.float32 rearSteerTorqueReq_Nm     // @unit{Nm};@range{-8,8};Requested value for the rear steer torque
    type.float32 curvatureReq_1pm     // @unit{1/m};@range{-0.3,0.3};Requested value for the curvature at the middle of the rear axle. (*used as reference value in case of curvature control)
    type.AP_TRJCTL.LaDMCCtrlRequestInterfaceType laDMCCtrlRequestInterface_nu     // @range{0,8};Type of the LaDMC control request: Used to select control signal of laDMCCtrlRequestPort (torque, steer wheel angle, curvature,...) in receiving component.
    type.AP_TRJCTL.LaDMCCtrlRequestSourceType laDMCCtrlRequestSource_nu     // @range{0,7};Source of the LaDMC control request
    type.boolean laDMCCtrlRequest_nu     // Control command for the activation/deactivation of the Lateral dynamic motion control. (true==activate; false==deactivate)
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.LaDMCCtrlRequestPort_InterfaceVersion uml:Class
  Members:
    type.uint32 LaDMCCtrlRequestPort_VERSION = 2    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.LoDMCCtrlRequestPort uml:Class
  version: ::ap_trjctl::LoDMCCtrlRequestPort_InterfaceVersion::LoDMCCtrlRequestPort_VERSION
    // Request to underlaid Longitudinal Dynamic Motion Control (LoDMC).
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 distanceToStopReq_m     // @unit{m};@range{0,30};Value for the remaining distance to stop the vehicle at rear axle center. (*used as reference value in case of distance control)
    type.float32 velocityLimitReq_mps     // @unit{m/s};@range{0,5};Value for the velocity limit / target vehicle velocity. (*used as reference value in case of velocity control; *used as limit for the vehicle velocity in case of distance and velocity control)
    type.float32 accelerationReq_mps2     // @unit{m/s2};@range{-10,10};Reference / Target value for acceleration control.
    type.AP_TRJCTL.LoDMCCtrlRequestSourceType loDMCCtrlRequestSource_nu     // @range{0,7};Source of the LoDMC control request
    type.AP_TRJCTL.LoDMCCtrlRequestInterfaceType loDMCCtrlRequestInterface_nu     // @range{0,7};Type of the LoDMC control request: Used to select control signal of loDMCCtrlRequestPort (distance, acceleration, torque,...) in receiving component.
    type.AP_TRJCTL.LoDMCCtrlRequestType loDMCCtrlRequest_nu     // @range{0,4};Control command for type of longitudinal dynamic motion control request.
    type.AP_TRJCTL.LoDMCHoldRequestType holdReq_nu     // @range{0,2};Request to comfortably stop the vehicle in case of any vehicle movement and to hold the vehicle in standstill.
    type.boolean emergencyHoldReq_nu     // Request to stop the vehicle in emergency cases.
    type.boolean secureReq_nu     // Request to secure the vehicle standstill. (e.g. close electric parking brake)
    type.boolean drivingForwardReq_nu     // Requested driving direction for longitudinal dynamic motion control. (true == forward; false == backward)
    type.boolean trajectoryReset_nu     // Indicator for a trajectory reset. Will be set "true" in case of a reset.
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.LoDMCCtrlRequestPort_InterfaceVersion uml:Class
  Members:
    type.uint32 LoDMCCtrlRequestPort_VERSION = 3    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.GearboxCtrlRequestPort uml:Class
  version: ::ap_trjctl::GearboxCtrlRequestPort_InterfaceVersion::GearboxCtrlRequestPort_VERSION
    // Request to Gearbox Control.
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.boolean gearboxCtrlRequest_nu     // Control command for the activation/deactivation of the Gearbox Control.
    type.boolean gearSwitchRequest_nu     // Set gear switch request.
    type.AP_CommonVehSigProvider.Gear gearReq_nu     // @range{0,15};Requested gear.
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.GearboxCtrlRequestPort_InterfaceVersion uml:Class
  Members:
    type.uint32 GearboxCtrlRequestPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.DrivingResistancePort uml:Class
  version: ::ap_trjctl::DrivingResistancePort_InterfaceVersion::DrivingResistancePort_VERSION
    // Information about driving resistance (e.g. for driving resistance on planned trajectory)
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.AP_TRJCTL.DrvRes drivingResistance_FL     // @unit{nu};Wheel specific driving resistance to cover curbstone and wheel stopper scenarios
    type.AP_TRJCTL.DrvRes drivingResistance_RL     // @unit{nu};Wheel specific driving resistance to cover curbstone and wheel stopper scenarios
    type.AP_TRJCTL.DrvRes drivingResistance_RR     // @unit{nu};Wheel specific driving resistance to cover curbstone and wheel stopper scenarios
    type.AP_TRJCTL.DrvRes drivingResistance_FR     // @unit{nu};Wheel specific driving resistance to cover curbstone and wheel stopper scenarios
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.DrivingResistancePort_InterfaceVersion uml:Class
  Members:
    type.uint32 DrivingResistancePort_VERSION = 2    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.TrajCtrlDebugPort uml:Class
  version: ::ap_trjctl::TrajCtrlDebugPort_InterfaceVersion::TrajCtrlDebugPort_VERSION
    // Debug port of component
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.sint32 debugInt     // @unit{nu};Freespace for MTS debug values
    type.float32 debugFloat     // @unit{nu};Freespace for MTS debug values
    type.float32 currentDeviation_m     // @unit{m};@range{-2,2};Current lateral deviation of vehicle from interpolated trajectory at rear axle. (based on orthogonal projection)
    type.float32 orientationError_rad     // @unit{rad};@range{-3142,3142};Current orientation error of vehicle from interpolated trajectory at rear axle. (based on orthogonal projection)
    type.float32 velocityLimitReqInterpolTraj_mps     // @unit{mps};@range{0,5};Interpolated velocity limit of trajectory.
    type.float32 distanceToStopReqInterExtrapolTraj_m     // @unit{m};@range{-2,30};Interpolated/Extrapolated distance to stop of trajectory.
    type.float32 xInterpolTraj_m     // @unit{m};Interpolated x value of trajectory.
    type.float32 yInterpolTraj_m     // @unit{m};Interpolated y value of trajectory.
    type.float32 curvatureInterpolTraj_1pm     // @unit{1/m};@range{-0.3,0.3};Interpolated curvature value of trajectory.
    type.float32 curvaturePreviewed_1pm     // @unit{1/m};Previewed curvature of the trajectory
    type.float32 filteredSteerIntervention_nu     // @range{0,1};Filtered value of steer intervention flag.
    type.float32 trajIntermediateValueRaw_perc     // Intermediate value on trajectory.
    type.float32 steerAngReqRaw_rad     // @unit{rad};Steer angle raw value.
    type.float32 steerAngReqYawDeviation_rad     // @unit{rad};Steer angle request based on yaw deviation.
    type.float32 steerAngReqLateralDeviation_rad     // @unit{rad};Steer angle request based on lateral deviation.
    type.float32 steerAngReqCurvature_rad     // @unit{rad};Steer angle request based on curvature.
    type.uint8 currentTrajectoryIndex_nu     // @range{0,128};Current index on trajectory.
    type.uint8 pathControlRequestMode_nu     // @range{0,128};Path control request mode.
    type.uint8 rawSteerAngleRequestMode_nu     // @range{0,128};Raw steer angle request mode
    type.uint8 rateAndAccelerationLimitatMode     // @range{0,128};Rate and Acceleration limitation mode.
    type.uint8 comfStandStillSteeringMode_nu     // @range{0,128};Comfortable standstill steering mode. (comfortable steering)
    type.uint8 comfStandstillSteeringExtrapolationMode_nu     // @range{0,128};Comfortable standstill steering extrapolation mode at end of standstill steering
    type.boolean driverInterventionProcessed_nu     // Processed information that driver intervention was detected. (Schmitt-Trigger based evaluation output)
    type.boolean driverInterventionDetected_nu     // Information that driver intervention was detected.
    type.boolean free1     // @unit{Boolean};free
    type.boolean free2     // @unit{Boolean};free
    type.boolean gearCorrect_nu     // Information whether current gear compared to target gear is correct.
    type.boolean standstillSteeringDesired_nu     // Information whether standstill steering is desired.
    type.boolean distanceControlFinished_nu     // Information whether distance control is finished.
    type.boolean standstillHoldCur_nu     // Information whether standstillHoldCur is reached.
    type.boolean finalVehicleStateReached_nu     // Information whether finalVehicleStateReached is reached.
    type.boolean outsideTrajectoryStart_nu     // Flag for outside trajectory (start).
    type.boolean outsideTrajectoryEnd_nu     // Flag for outside trajectory (end).
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.TrajCtrlDebugPort_InterfaceVersion uml:Class
  Members:
    type.uint32 TrajCtrlDebugPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.MOCO_t_LongLims uml:Class
    // Longitudinal limitations of acceleration and jerk (MoCo variant only)
  Members:
    type.float32 longAccelMax     // Upper limit for longitudinal acceleration
    type.float32 longAccelMin     // Lower limit for longitudinal acceleration
    type.float32 longJerkMax     // Upper limit for longitudinal jerk
    type.float32 longJerkMin     // Lower limit for longitudinal jerk
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.MOCO_te_FunMode uml:Enumeration
    // Function request identifier (MoCo variant only)
  Members:
    MOCO_FUN_MODE_INVALID = 0            
    MOCO_FUN_MODE_CRUISE = 1            
    MOCO_FUN_MODE_CRUISE_LAT = 2            
    MOCO_FUN_MODE_CRUISE_LONG = 3            
    MOCO_FUN_MODE_SPEED_LIM = 4            
    MOCO_FUN_MODE_SAFETY_LONG_AEB = 5            
    MOCO_FUN_MODE_SAFETY_LONG_VDS = 6            
    MOCO_FUN_MODE_SAFETY_LAT = 7            
    MOCO_FUN_MODE_MANEUVER = 8            
    MOCO_FUN_MODE_OFFROAD = 9            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.MOCO_te_DynMode uml:Enumeration
    // Dynamic mode. (MoCo variant only)
  Members:
    MOCO_DYN_MODE_LOW = 0            
    MOCO_DYN_MODE_MEDIUM = 1            
    MOCO_DYN_MODE_HIGH = 2            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.MOCO_te_MotReq uml:Enumeration
    // Motion request (MoCo variant only)
  Members:
    MOCO_MOT_REQ_SUPPRESS_STANDSTILL = 0            
    MOCO_MOT_REQ_STANDSTILL = 1            
    MOCO_MOT_REQ_DRIVEOFF = 2            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.MOCO_te_DrivingDirReq uml:Enumeration
    // Driving direction request (MoCo variant only)
  Members:
    MOCO_DRIVING_DIR_REQ_NO_REQ = 0            
    MOCO_DRIVING_DIR_REQ_FORWARD = 1            
    MOCO_DRIVING_DIR_REQ_BACKWARD = 2            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.MF_CONTROL_t_LongManeuverRequestPort uml:Class
  version: ::ap_trjctl::MF_CONTROL_t_LongManeuverRequestPort_InterfaceVersion::MF_CONTROL_t_LongManeuverRequestPort_VERSION
    // Longitudinal request to underlaid Trajectory Tracking Controller (TRATCO). (MoCo variant only)
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.AP_TRJCTL.MF_CONTROL_t_LongCtrlReq ctrlReq     // Longitudinal control request (control mode and control reference values)
    type.AP_TRJCTL.MOCO_t_LongLims lims     // Longitudinal limitations of acceleration and jerk.
    type.AP_TRJCTL.MOCO_te_FunMode funMode     // @range{0,9};Function request identifier. (e.g. MOCO_FUN_MODE_MANEUVER)
    type.AP_TRJCTL.MOCO_te_DynMode dynMode     // @range{0,2};Dynamic mode. (Low, Medium, High)
    type.AP_TRJCTL.MOCO_te_MotReq motReq     // @range{0,2};Motion request (SUPPRESS_STANDSTILL=0, STANDSTILL=1, DRIVEOF = 2)
    type.uint8 activateCtrl     // Control command to activate/deactivate the longitudinal control. (true==activate; false==deactivate)
    type.uint8 fullBrakingReq     // Request to stop the vehicle in emergency cases.
    type.AP_TRJCTL.MOCO_te_DrivingDirReq drivingDirReq     // @range{0,2};Requested driving direction for longitudinal control. (NO_REQ = 0, FORWARD = 1, BACKWARD = 2)
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.MF_CONTROL_t_LongManeuverRequestPort_InterfaceVersion uml:Class
  Members:
    type.uint32 MF_CONTROL_t_LongManeuverRequestPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.FC_TRJCTL_Vehicle_Params uml:Class
  Members:
    type.float32 AP_V_MAX_STEER_ANG_RAD     // Maximum steering angle (at wheels).
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.FC_TRJCTL_Sys_Func_Params uml:Class
  Members:
    type.float32 AP_G_MAX_AVG_LAT_ACCEL_MPS2     // Maximum lateral acceleration of the ego-vehicle
    type.float32 AP_G_MAX_AVG_YAW_RATE_RADPS     // Maximum yaw rate of the ego-vehicle
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.FC_TRJCTL_Params uml:Class
  version: ::ap_trjctl::FC_TRJCTL_Params_InterfaceVersion::FC_TRJCTL_Params_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 AP_C_MANEUV_FINISHED_TIME_DRV_S     // SEMI_AP: Limit for passed time without any movement. If this value is reached the current stroke will be finished.
    type.float32 AP_C_MANEUV_FINISHED_LIMIT_M     // FULL_AP: Define distance limit for intern calculation of "maneuveringFinished_nu".;Remark: - Value needs to be aligned with LoDMC threshold to start maneuvering. This value needs to be higher than all LoDMC thresholds (e.g. normal/curb scenario).
    type.float32 AP_C_MANEUV_V_THRESH_RESET_MPS     // SEMI_AP: Lower velocity threshold for reset of maneuvering finished timer.
    type.float32 AP_C_MANEUV_D_THRESH_RESET_M     // SEMI_AP: Lower distance threshold for reset of maneuvering finished timer.
    type.float32 AP_C_MANEUV_FINISHED_TIME_S     // FULL_AP: Limit for passed time without any movement. If this value is reached the current stroke will be finished.
    type.uint8 AP_C_PC_NUM_CONTROL_GAINS_NU     // Number of control gains. (Describes the number of elements in velocity depending control gains.)
    type.float32 AP_C_PC_GAIN_VELVEC_MPS     // Velocity vector for velocity depending control gains.
    type.float32 AP_C_PC_GAIN_YAW_DEVIATION_NU     // Velocity depending control gains for yaw deviation.
    type.float32 AP_C_PC_GAIN_LAT_DEVIATION_NU     // Velocity depending control gains for lateral deviation.
    type.float32 AP_C_PC_FAIL_MAX_LAT_ERROR_M     // Maximum allowed absolute deviation from calculated trajectory. Trajectory control stops car and requests replanning of trajectory inputs if this value is reached.
    type.float32 AP_C_PC_FAIL_MAX_YAW_ERROR_RAD     // Maximum allowed absolute yaw angle (orientation) error from calculated trajectory. Trajectory control stops car and requests replanning of trajectory input if this value is reached.
    type.float32 AP_C_PC_STEER_ANGLE_T_FILT_S     // Filter coefficients for steer angle request output.
    type.float32 AP_C_PI_YAW_DEVIATION_T_FILT_S     // Filter coefficients for calculated yaw deviation.
    type.float32 AP_C_PC_FILT_FACTOR_MAX_NU     // Factor for steer angle filter coefficient. Used for low speed maneuvering to compensate discrete steps in;odometry estimation. Between velocity 0 and AP_C_PC_FILT_FAC_VEL_LIMIT_MPS the resulting filter factor will be interpolated between this value and 1.
    type.float32 AP_C_PC_FILT_FAC_VEL_LIMIT_MPS     // Upper velocity limit for filter factor. (see AP_C_PC_FILT_FACTOR_MAX_NU)
    type.float32 AP_C_PC_ORIENT_CTRL_END_DIST_M     // Threshold for parallel parking in. If the current distance to stop of the last stroke is smaller than this value the end maneuver for parking in parallel will be activated. (end maneuver is orientation correction)
    type.float32 AP_C_PC_ORIENT_CTRL_INIT_DIST_M     // Threshold for parallel parking in. If the initial distance to stop of the last stroke is smaller than this value the end maneuver for parking in parallel will be activated. (end maneuver is orientation correction)
    type.float32 AP_C_PC_FIRST_STEER_ACCUR_RAD     // Interval limits (positive and negative) to reach end of first steering. (at steering wheel)
    type.float32 AP_C_PC_FIRST_STEER_VEL_RADPS     // Maximum steer angle velocity for ramping steer angle to desired value for first steering (at wheels)
    type.float32 AP_C_PC_FIRST_STEER_ACC_RADPS2     // Maximum steer angle acceleration for ramping steer angle to desired value for first steering (at wheels)
    type.float32 AP_C_PC_CURV_PREVIEW_FACTOR_NU     // Factor to manipulate preview length (factor*v*T_Filter)
    type.float32 AP_C_PC_CURV_PREVIEW_MIN_M     // Minimum preview length to cover drive off situations (velocity changes from zero to non-zero value).
    type.float32 AP_C_SECURE_FINISHED_TIME_S     // Limit for passed time with secured vehicle standstill until information will be forwarded to status port
    type.boolean AP_C_STEER_INTERV_ACTIVE_NU     // Activation/Deactivation of steer intervention detection.
    type.float32 AP_C_STEER_INTERV_FILT_TIME_S     // Filter coefficient for steering intervention by driver.
    type.float32 AP_C_STEER_INTERV_RISE_NU     // Schmitt-Trigger rising threshold for steering intervention by driver.
    type.float32 AP_C_STEER_INTERV_FALL_NU     // Schmitt-Trigger falling threshold for steering intervention by driver.
    type.float32 AP_C_PC_FILT_FAC_TRAJ_STEP_NU     // Factor for steer angle filter coefficient. Used for path control to compensate discontinuities in trajectory. Factor will be multiplied if discontinuity at current trajectory point occurs.
    type.float32 AP_C_ACTIVE_CONTROL_MIN_TIME_S     // Minimum time interval for active control. Maneuvering finished flag will be accepted when this time was passed.
    type.boolean AP_C_VL_RAMP_UP_VEL_NU     // Flag to activate or deactivate the ramp up feature for limitation of velocity limit request. In case of rising unsteadiness in velocity limit request during maneuvering the acceleration will be limited by a ramped velocity limit.
    type.float32 AP_C_VL_VEL_RAMP_LIMIT_MPS2     // Maximum acceleration used to ramp up the velocity limit request.
    type.boolean AP_C_VL_RAMP_UP_DIST_NU     // Flag to activate or deactivate the ramp up feature for limitation of distance to stop. In case of rising unsteadiness in distance to stop request during maneuvering the acceleration will be limited by a ramped distance to stop based on current ego velocity and AP_C_MIN_PARKING_VEL_MPS.
    type.float32 AP_C_MIN_PARKING_VEL_MPS     // Minimum parking velocity.
    type.float32 AP_C_PC_MIN_STEER_VEL_RADPS     // Minimum steer angle velocity for ramping steer angle to desired value for first steering (at wheels)
    type.boolean AP_C_DRV_RESIST_FAKE_DATA_NU     // Temp: Boolean to set test data in driving resistance interface.
    type.uint8 AP_C_DRV_RESIST_FL_TYPE_NU     // Temp: Type of driving resistance send to LoDMC. (00: NONE, 01: FALLING_LOW, 02: FALLING_MEDIUM, 03: FALLING_HIGH, 04: RISING_LOW, 05: RISING_MEDIUM, 06: RISING_HIGH, 07: WHEEL_STOPPER)
    type.float32 AP_C_DRV_RESIST_FL_RED_DIST_M     // Temp: Reduce distance to stop for driving resistance information sent to LoDMC by this parameter. (only for first stroke) (e.g. if distance to stop of first stroke is 5 meter and distance to this curb is 4 meter set value to 1m)
    type.uint8 AP_C_DRV_RESIST_RL_TYPE_NU     // Temp: Type of driving resistance send to LoDMC. (00: NONE, 01: FALLING_LOW, 02: FALLING_MEDIUM, 03: FALLING_HIGH, 04: RISING_LOW, 05: RISING_MEDIUM, 06: RISING_HIGH, 07: WHEEL_STOPPER)
    type.float32 AP_C_DRV_RESIST_RL_RED_DIST_M     // Temp: Reduce distance to stop for driving resistance information sent to LoDMC by this parameter. (only for first stroke) (e.g. if distance to stop of first stroke is 5 meter and distance to this curb is 4 meter set value to 1m)
    type.uint8 AP_C_DRV_RESIST_RR_TYPE_NU     // Temp: Type of driving resistance send to LoDMC. (00: NONE, 01: FALLING_LOW, 02: FALLING_MEDIUM, 03: FALLING_HIGH, 04: RISING_LOW, 05: RISING_MEDIUM, 06: RISING_HIGH, 07: WHEEL_STOPPER)
    type.float32 AP_C_DRV_RESIST_RR_RED_DIST_M     // Temp: Reduce distance to stop for driving resistance information sent to LoDMC by this parameter. (only for first stroke) (e.g. if distance to stop of first stroke is 5 meter and distance to this curb is 4 meter set value to 1m)
    type.uint8 AP_C_DRV_RESIST_FR_TYPE_NU     // Temp: Type of driving resistance send to LoDMC. (00: NONE, 01: FALLING_LOW, 02: FALLING_MEDIUM, 03: FALLING_HIGH, 04: RISING_LOW, 05: RISING_MEDIUM, 06: RISING_HIGH, 07: WHEEL_STOPPER)
    type.float32 AP_C_DRV_RESIST_FR_RED_DIST_M     // Temp: Reduce distance to stop for driving resistance information sent to LoDMC by this parameter. (only for first stroke) (e.g. if distance to stop of first stroke is 5 meter and distance to this curb is 4 meter set value to 1m)
    type.float32 AP_C_STEER_SATURATE_THRESH_RAD     // Threshold to maximum steer angle to use a reduced steer angle rate. This will reduce slope of ramp into steer angle limitation. (Warning: This might influence the control performance.)
    type.float32 AP_C_STEER_SATURATE_RATE_RADPS     // Maximum steering angle velocity (at wheels) in case of steer angle request in saturation threshold.  (Warning: This might influence the control performance.)
    type.float32 AP_C_PC_MAX_STEER_ACC_RADPS2     // Maximum steer angle acceleration for increasing steer angle velocity (at wheels)
    type.float32 AP_C_PC_RATE_LIMIT_FACTOR_NU     // Factor to reduce rate limit in path control output. Ensures overall rate limit and acceleration limitation.
    type.float32 AP_C_PC_CURV_PREV_FACTOR_ADD_NU     // Add this value to AP_C_PC_CURV_PREVIEW_FACTOR_NU to compensate curvature step in trajectory. (factor*v*T_Filter)
    type.boolean AP_C_PC_CURV_PREV_REDUCED_NU     // Flag to activate the reduced preview for curvature. If activated a check will be performed whether an curvature step is in planned trajectory. If yes, the extended preview length will be used. If no curvature step was detected the preview length will be reduced.
    type.float32 AP_C_FAIL_MAX_LONG_OVERSHOOT_M     // Maximum allowed longitudinal overshoot from calculated trajectory. Trajectory control demands replanning in Trajectory Planning if value is reached. Trajectory control demands emergency hold if value is reached.
    type.float32 AP_C_COMP_TIRE_DEF_FACTOR_NU     // Factor that is used to compensate tire contact area deformation during lateral control mode LACTRL_COMF_ANGLE_ADJUSTMENT. (e.g. if standstill steering starts at -400deg and should steer to 0deg the difference 400deg is multiplied with this factor to steer to a;positive value to avoid that the tire contact area deformation causes and a value smaller than 0deg when the steering system is deactivated)
    type.float32 AP_C_MANEUV_FINISHED_HYST_M     // Additional hysteresis distance threshold to keep maneuvering finished indicated. (e.g. to cover odometry drift)
    type.float32 AP_C_MIN_DIST_REQ_M     // Minimum length in case of distance control (distance, distance-velocity, trajectory) to start/request maneuvering
    type.boolean AP_C_FEAT_WAIT_FOR_CONTACT_NU     // Activate feature to wait for contact (e.g. in case of a wheel stopper detected). Force an overshoot of the stop point and wait for contact to finish the control.
    type.float32 AP_C_WFC_OVERSHOOT_LENGTH_M     // Overshoot length for wait for contact feature.
    type.float32 AP_C_WFC_OVERSHOOT_DIST_THRES_M     // Distance threshold for wait for contact feature distance overshoot. (Below this threshold the distance to stop send to LoDMC will be manipulated; AP_C_WFC_OVERSHOOT_LENGTH_M will be added)
    type.float32 AP_C_WFC_VDY_DIST_THRES_M     // Distance threshold for wait for contact feature VDY based detection of wheelstopper. (If the smallest distance to any wheel stopper is smaller than this threshold the VDY based wheel stopper detection will be activated.)
    type.float32 AP_C_WFC_VDY_DRIVE_OFF_THRES_M     // Distance threshold for wait for contact feature VDY based detection of wheelstopper. (To ensure no false positive detection during drive off the feature is deactivated at the beginning of a stroke.)
    type.boolean AP_C_FEAT_WS_VEL_REDUCED_NU     // Activate feature to reduce target velocity in case of a close wheel stopper information.
    type.float32 AP_C_WFC_WS_VEL_DIST_THRESH_M     // Distance threshold for reduced target velocity in case of a close wheelstopper. (below this distance threshold of the minimum distance to any wheel stopper the velocity limit will be reduced)
    type.float32 AP_C_WFC_WS_VEL_LIMIT_MPS     // Velocity limit for the reduced target velocity in case of a close wheelstopper.
    type.boolean AP_C_HACK_WS_LAST_STROKE_NU     // HACK: Allow that for last stroke mf_control internally sets a wheel stopper information and distance equal to the distance to stop.
    type.float32 AP_C_NO_COMF_STEER_WS_THRES_M     // Longitudinal distance threshold between front wheels and wheel stopper to deactivate comfortable standstill steering (e.g. after parking maneuver).
    type.float32 AP_C_DRIVE_OFF_DISTANCE_THRES_M     // Distance threshold to detect drive off was realized by underlaid control. If the passed distance gets above this limit the drive off is interpreted as done.
    type.float32 AP_C_DRIVE_OFF_VELO_THRES_MPS     // Velocity threshold to detect drive off was realized by underlaid control. If the passed velocity gets above this limit the drive off is interpreted as done.
    type.float32 AP_C_HANDSHAKE_WAIT_THRES_TIME_S     // Handshake wait time to detect failure in handshake . If the wait time gets above this limit it should be reported as failure.
    type.float32 AP_C_PC_VELO_PREVIEW_TIME_S     // Preview time of velocity request to compensate delay in underlaid longitudinal control chain.
    type.float32 AP_C_LEAVING_PATH_BEFORE_M     // Threshold to detect that the ego position is in a critical distance outside the planned path (before first path point).
    type.float32 AP_C_LEAVING_PATH_BEHIND_M     // Threshold to detect that the ego position is in a critical distance outside the planned path (behind last path point).
    type.AP_TRJCTL.FC_TRJCTL_Vehicle_Params vehicleParams     // Nested struct for vehicle params section
    type.AP_TRJCTL.FC_TRJCTL_Sys_Func_Params sysFuncParams     // Nested struct for sys func params section
----------------------------------------------------------------------------------------------------------

  # ID:  type.AP_TRJCTL.FC_TRJCTL_Params_InterfaceVersion uml:Class
  Members:
    type.uint32 FC_TRJCTL_Params_VERSION = 2    
----------------------------------------------------------------------------------------------------------

  # ID:  type.LSM_VEDODO.LSM_VEDODO_Constants uml:Class
    // 
  Members:
    type.uint8 ODO_BUFFER_SIZE = 15    
    type.uint8 ODO_PREDICTION_SIZE = 4    
    type.uint8 DBG_NUM_OF_WHEELS_NU = 4    
    type.uint8 NUM_MTS_DEBUG_FREESPACE_ODO = 10    
    type.float32 COORDINATE_OVERFLOW_POSITION_M = 1000.0    
    type.float32 COORDINATE_OVERFLOW_YAW_ANGLE_RAD = 3.141592653589    
----------------------------------------------------------------------------------------------------------

  # ID:  type.LSM_VEDODO.Direction uml:Enumeration
    // The vehicle current driving direction.
  Members:
    DIRECTION_REVERSE = -1            
    DIRECTION_UNDEFINED = 0            
    DIRECTION_FORWARD = 1            
----------------------------------------------------------------------------------------------------------

  # ID:  type.LSM_VEDODO.MotionState uml:Enumeration
    // determined motion state
  Members:
    ODO_STANDSTILL = 0            
    ODO_NO_STANDSTILL = 1            
----------------------------------------------------------------------------------------------------------

  # ID:  type.LSM_VEDODO.OdoDebugPort uml:Class
  version: ::lsm_vedodo::OdoDebugPort_InterfaceVersion::OdoDebugPort_VERSION
    // Debug Signals to be provided for debugging purposes.
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 steeringWheelAngleOffset_rad     // @unit{rad};estimated steering wheel angle offset
    type.float32 steeringWheelAngleOffsetCalibrated_rad     // @unit{rad};steer angle with applied estimated offset
    type.float32 yawRateOffset_radps     // @unit{rad/s};estimated yaw rate offset
    type.float32 yawRateOffsetCalibrated_radps     // @unit{rad/s};yaw rate with applied estimated offset
    type.float32 distancePerStepRA_m     // @unit{m};estimated distance driven on front axle per loop
    type.float32 distancePerStepFA_m     // @unit{m};estimated distance driven on rear axle per loop
    type.float32 distanceFA_m     // @unit{m};driven distance on front axle
    type.float32 distanceRA_m     // @unit{m};driven distance on rear axle
    type.float32 yawAnglePerStepAckermann_rad     // @unit{rad};estimated yaw angle per loop, derived from ackermann model
    type.float32 yawAnglePerStepYawRate_rad     // @unit{rad};estimated yaw angle per loop, derived from yaw rate model
    type.float32 yawAnglePerStepStandstillSteer_rad     // @unit{rad};estimated yaw angle per loop during standstill steering
    type.float32 yawAnglePerStepWhlDistFront_rad     // @unit{rad};estimated yaw angle per loop, derived from difference between left and right wheel distance on front axle
    type.float32 yawAnglePerStepWhlDistRear_rad     // @unit{rad};estimated yaw angle per loop, derived from difference between left and right wheel distance on rear axle
    type.float32 odoAccelByWheel_mps2     // @unit{m/ss};vehicle acceleration derived from wheel speeds
    type.float32 distancePerStepWheel_m     // @unit{m};estimated distance made by each wheel
    type.sint32 debugInt     // @unit{nu};freespace for MTS debug values
    type.float32 debugFloat     // @unit{nu};freespace for MTS debug values
    type.float32 steerAngleFL     // @unit{rad};calculated angle of wheel front left
    type.float32 steerAngleFR     // @unit{rad};calculated angle of wheel front right
    type.float32 steerAngleCTR     // @unit{rad};calculated angle of "virtual" wheel front center.
    type.float32 estimYawRateFLRR_radps     // @unit{rad/s};estimated yaw rate by wheel combination FL RR
    type.float32 estimVelRearAxleFLRR_mps     // @unit{m/s};estimated velocity by wheel combination FL RR
    type.float32 estimYawRateFRRL_radps     // @unit{rad/s};estimated yaw rate by wheel combination FR RL
    type.float32 estimVelRearAxleFRRL_mps     // @unit{m/s};estimated velocity by wheel combination FR RL
    type.float32 estimYawRateFRRR_radps     // @unit{rad/s};estimated yaw rate by wheel combination FR RR
    type.float32 estimVelRearAxleFRRR_mps     // @unit{m/s};estimated velocity by wheel combination FR RR
    type.float32 estimYawRateFLFR_radps     // @unit{rad/s};estimated yaw rate by wheel combination FL RR
    type.float32 estimVelRearAxleFLFR_mps     // @unit{m/s};estimated velocity by wheel combination FL RR
    type.float32 estimYawRateRLRR_radps     // @unit{rad/s};estimated yaw rate by wheel combination RL RR
    type.float32 estimVelRearAxleRLRR_mps     // @unit{m/s};estimated velocity by wheel combination RL RR
    type.float32 estimYawRateFLRL_radps     // @unit{rad/s};estimated yaw rate by wheel combination FL RL
    type.float32 estimVelRearAxleFLRL_mps     // @unit{m/s};estimated velocity by wheel combination FL RL
    type.sint8 ticsIncrement_4_nu     // counted wheel tics per loop
    type.sint8 drivingDirectionGear_nu     // determined vehicle driving direction based on gearbox information (-1 reverse, 0 default, 1 forward)
----------------------------------------------------------------------------------------------------------

  # ID:  type.LSM_VEDODO.OdoDebugPort_InterfaceVersion uml:Class
  Members:
    type.uint32 OdoDebugPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------



  # ID:  type.LSM_VEDODO.OdoEstimationOutputPort uml:Class
  version: ::lsm_vedodo::OdoEstimationOutputPort_InterfaceVersion::OdoEstimationOutputPort_VERSION
    // A buffer/array storing the odometry estimation samples throught a certain time-window
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.LSM_VEDODO.OdoEstimation odoEstimationBuffer     // @unit{nu};A buffer/array storing the odometry estimation samples throught a certain time-window.
    type.LSM_VEDODO.OdoEstimationPrediction odoPredictionBuffer     // @unit{nu};A buffer/array storing the odometry estimation prediction samples.
    type.LSM_VEDODO.OdoEstimation odoEstimation     // @unit{nu};Current Estimation.
----------------------------------------------------------------------------------------------------------

  # ID:  type.LSM_VEDODO.OdoEstimationOutputPort_InterfaceVersion uml:Class
  Members:
    type.uint32 OdoEstimationOutputPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.LSM_VEDODO.OdoEstimation uml:Class
  version: ::lsm_vedodo::OdoEstimation_InterfaceVersion::OdoEstimation_VERSION
    // Result of odometry estimation.
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 longiVelocity_mps     // @unit{m/s};@range{-100,100};Value for the current vehicle velocity; positive: driving forward, negative: driving backward
    type.float32 lateralVelocity_mps     // @unit{m/s};@range{-100,100};Value for the current vehicle lateral velocity
    type.float32 longiAcceleration_mps2     // @unit{m/s2};@range{-12,12};Value for the current vehicle calibrated and slope compensated longitudinal acceleration; forward direction positive: accelerating, negative: decelerating.
    type.float32 lateralAcceleration_mps2     // @unit{m/s2};@range{-12,12};Value for the current vehicle calibrated and slope compensated lateral acceleration; forward direction positive: left curve, negative: right curve.;Vertical Acceleration to be zero on horizontal plane.
    type.float32 verticalAcceleration_mps2     // @unit{m/s2};@range{-12,12};Value for the current vehicle calibrated and slope compensated vertical acceleration; positive: when driving uphill, negative: when driving downhill.
    type.float32 xPosition_m     // @unit{m};@range{-COORDINATE_OVERFLOW_POSITION_M,+COORDINATE_OVERFLOW_POSITION_M};current x position of the middle of the rear axle in frozen/absolute coordinates.
    type.float32 yPosition_m     // @unit{m};@range{-COORDINATE_OVERFLOW_POSITION_M,+COORDINATE_OVERFLOW_POSITION_M};Current y position of the middle of the rear axle in frozen/absolute coordinates.
    type.float32 xVelocity_mps     // @unit{m/s};@range{-100,100};Value for the current vehicle velocitiy in the local coordinate system; positive: vehicle is moving towards greater position in x direction, negative towards smaller in x direction.
    type.float32 yVelocity_mps     // @unit{m/s};@range{-100,100};Value for the current vehicle velocitiy in the local coordinate system; positive: vehicle is moving towards greater position in y direction, negative towards smaller in y direction.
    type.float32 xPositionVar_m2     // @unit{m2};@range{0,COORDINATE_OVERFLOW_POSITION_M * COORDINATE_OVERFLOW_POSITION_M};Uncertainty of x position estimation
    type.float32 yPositionVar_m2     // @unit{m2};@range{0,COORDINATE_OVERFLOW_POSITION_M * COORDINATE_OVERFLOW_POSITION_M};Uncertainty of y position estimation
    type.float32 xyPositionVar_m2     // @unit{m2};@range{0,COORDINATE_OVERFLOW_POSITION_M * COORDINATE_OVERFLOW_POSITION_M};Correlation of x-y position estimation.
    type.float32 yxPositionVar_m2     // @unit{m2};@range{0,COORDINATE_OVERFLOW_POSITION_M * COORDINATE_OVERFLOW_POSITION_M};Correlation of y-x position estimation.
    type.float32 rollAngle_rad     // @unit{rad};@range{-PI_RAD,+PI_RAD};current vehicle roll angle in absolute coordinates.
    type.float32 pitchAngle_rad     // @unit{rad};@range{-PI_RAD,+PI_RAD};current vehicle pitch angle in absolute coordinates.
    type.float32 yawAngle_rad     // @unit{rad};@range{-COORDINATE_OVERFLOW_YAW_ANGLE_RAD,+COORDINATE_OVERFLOW_YAW_ANGLE_RAD};Current vehicle yaw angle in absolute coordinates.
    type.float32 rollAngleVar_rad2     // @unit{rad2};@range{0,COORDINATE_OVERFLOW_YAW_ANGLE_RAD * COORDINATE_OVERFLOW_YAW_ANGLE_RAD};Uncertainty of roll angle Estimation
    type.float32 pitchAngleVar_rad2     // @unit{rad2};@range{0,COORDINATE_OVERFLOW_YAW_ANGLE_RAD * COORDINATE_OVERFLOW_YAW_ANGLE_RAD};Uncertainty of Pitch Angle Estimation
    type.float32 yawAngleVar_rad2     // @unit{rad2};@range{0,COORDINATE_OVERFLOW_YAW_ANGLE_RAD * COORDINATE_OVERFLOW_YAW_ANGLE_RAD};Uncertainty of yaw angle estimation
    type.float32 rollRate_radps     // @unit{rad/s};@range{-PI_RAD,+PI_RAD};current vehicle roll rate in absolute coordinates.
    type.float32 pitchRate_radps     // @unit{rad/s};@range{-PI_RAD,+PI_RAD};current vehicle pitch rate in absolute coordinates.
    type.float32 yawRate_radps     // @unit{rad/s};@range{-PI_RAD,+PI_RAD};Current vehicle yaw rate
    type.float32 steerAngFrontAxle_rad     // @unit{rad};@range{-0.8,0.8};virtual centre steer angle at front axle.
    type.float32 steerAngRearAxle_rad     // @unit{rad};@range{-0.8,0.8};virtual centre steer angle at rear axle.
    type.float32 wheelAngleFR_rad     // @unit{rad};Wheel Angle Front Right
    type.float32 wheelAngleFL_rad     // @unit{rad};Wheel Angle Front Left
    type.float32 sideSlipAngle_rad     // @unit{rad};Side Slip Angle.
    type.float32 suspPitch_rad     // @unit{rad};Suspension Pitch angle.
    type.float32 suspRoll_rad     // @unit{rad};Suspension Roll angle.
    type.float32 suspHeight_m     // @unit{m};Suspension Height.
    type.float32 drivenDistance_m     // @unit{m};@range{TBD,TBD};The accumulation of the absolute vehicle displacements.
    type.LSM_VEDODO.MotionState motionStatus_nu     // @range{0,1};determined motion state
    type.LSM_VEDODO.Direction drivingDirection_nu     // @range{-1,1};The vehicle current driving direction.
----------------------------------------------------------------------------------------------------------

  # ID:  type.LSM_VEDODO.OdoEstimation_InterfaceVersion uml:Class
  Members:
    type.uint32 OdoEstimation_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.LSM_VEDODO.OdoEstimationPrediction uml:Class
    // Result of odometry estimation prediction.
  Members:
    type.float32 longiVelocity_mps     // @unit{m/s};@range{-100,100};Value for the current vehicle velocity; positive: driving forward, negative: driving backward
    type.float32 xPosition_m     // @unit{m};@range{-COORDINATE_OVERFLOW_POSITION_M,+COORDINATE_OVERFLOW_POSITION_M};current x position of the middle of the rear axle in frozen/absolute coordinates.
    type.float32 yPosition_m     // @unit{m};@range{-COORDINATE_OVERFLOW_POSITION_M,+COORDINATE_OVERFLOW_POSITION_M};Current y position of the middle of the rear axle in frozen/absolute coordinates.
    type.float32 yawAngle_rad     // @unit{rad};@range{-COORDINATE_OVERFLOW_YAW_ANGLE_RAD,+COORDINATE_OVERFLOW_YAW_ANGLE_RAD};Current vehicle yaw angle in frozen/absolute coordinates.
----------------------------------------------------------------------------------------------------------

  # ID:  type.LSM_VEDODO.OdoNvStWhlAngCal_t uml:Class
  Members:
    type.float32 ZeroAngle     
    type.uint32 CalStatus     
----------------------------------------------------------------------------------------------------------

  # ID:  type.LSM_VEDODO.YwRate_t uml:Class
  Members:
    type.float32 ZeroRate     
    type.float32 ZeroRateMin     
    type.float32 ZeroRateMax     
    type.uint32 CalStatus     
----------------------------------------------------------------------------------------------------------

  # ID:  type.LSM_VEDODO.LatAcc_t uml:Class
  Members:
    type.float32 ZeroAccel     
    type.uint32 CalStatus     
----------------------------------------------------------------------------------------------------------

  # ID:  type.LSM_VEDODO.OdoNvmState_t uml:Class
  Members:
    type.uint32 State     
----------------------------------------------------------------------------------------------------------

  # ID:  type.LSM_VEDODO.OdoNVMData uml:Class
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.LSM_VEDODO.OdoNvStWhlAngCal_t StWhlAng     
    type.LSM_VEDODO.YwRate_t YwRate     
    type.LSM_VEDODO.LatAcc_t LatAcc     
    type.LSM_VEDODO.OdoNvmState_t State     
----------------------------------------------------------------------------------------------------------

  # ID:  type.LSM_VEDODO.OdoPersDataPort uml:Class
  version: ::lsm_vedodo::OdoPersDataPort_InterfaceVersion::OdoPersDataPort_VERSION
    // Persistent Data to be stored in the Non-volatile Memory (NVM)
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 SensorOffsetLateralAcceleration_mps     // @unit{m/s};Learn value of lateral acceleration sensor offset to be stored in NVRAM
    type.float32 SensorOffsetLongitudinalAcceleration_mps     // @unit{m/s};Learn value of longitudinal acceleration sensor offset to be stored in NVRAM
    type.float32 SensorOffsetYawRate_radps     // @unit{rad/s};Learn value of yaw rate sensor offset to be stored in NVRAM
    type.float32 SensorOffsetSteeringWheelAngle_rad     // @unit{rad};Learn value of steer angle sensor offset to be stored in NVRAM
----------------------------------------------------------------------------------------------------------

  # ID:  type.LSM_VEDODO.OdoPersDataPort_InterfaceVersion uml:Class
  Members:
    type.uint32 OdoPersDataPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.LSM_VEDODO.FC_VEDODO_Params uml:Class
  version: ::lsm_vedodo::FC_VEDODO_Params_InterfaceVersion::FC_VEDODO_Params_VERSION
    // parameters of VEDODO function
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 LSM_O_SAMPLE_TIME_S     // Odometry calling update time rate.
    type.float32 LSM_O_MSE_MOTION_STATUS_TIMER_S     // Time for standstill detection (motionStatus_nu changes from 1 to 0 if no wheel ticks are measured for the defined time
    type.float32 LSM_O_YOC_MAX_STRWHL_VEL_RADPS     // Maximum steering wheel angle velocity for yaw rate offset estimation
    type.uint16 LSM_O_MAX_WHEEL_PULSE_VALUE_NU     // Define maximum wheel counter value (e.g. Passat B8: 1000).
    type.float32 LSM_O_MAX_POS_X_VALUE_M     // Limitation of x position (min: -LSM_O_MAX_POS_X_VALUE_M max: LSM_O_MAX_POS_X_VALUE_M)
    type.float32 LSM_O_MAX_POS_Y_VALUE_M     // Limitation of y position (min: -LSM_O_MAX_POS_Y_VALUE_M max: LSM_O_MAX_POS_Y_VALUE_M)
    type.float32 LSM_O_MAX_POS_PHI_VALUE_RAD     // Limitation of heading angle (min: -LSM_O_MAX_POS_PHI_VALUE_RAD max: LSM_O_MAX_POS_PHI_VALUE_RAD)
    type.uint8 LSM_O_USE_STEER_WHEEL_ANG_FILT     // Filter steer wheel angle: 0 = Filter ON 1 = Filter OFF
    type.uint8 LSM_O_USE_YAW_STANDSTILL_CORR     // Determine influence of steering on yaw angle at standstill (using calculated Motion Status) 0 = Only use yaw rate input 1 = Use steering wheel angle in standstill situation to adapt yaw angle change
    type.uint8 LSM_O_DRIVING_DIRECTION_METHOD     // Select source of driving direction: 0 = Use driving direction from gear 1 = Use driving direction from rear axis 2 = Use driving direction from front axis 3 = USE_DRIVING_DIRECTION_FROM_FOUR_WHEELS, 4 = Use driving direction from accelerometer 5 = Use driving direction from last pulse 6 = Use driving direction from encoder, accelerometer and yawrate-steerangle
    type.float32 LSM_O_SIGNAL_LATENCY_S     // Latency of input signals. Zero turns the latency compensation of
    type.float32 LSM_O_WS_STRAIGHT_THRESH_MPS     // This is a threshold for the deviation between the rear wheel speeds which is used as an indication of moving straight forward. The more this value increased the more tolerance we have for indicating straight motion.
    type.float32 LSM_O_GYRO_LINEAR_ERR_FACTOR     // Correction factor for gyro yaw rate linear error.
    type.float32 LSM_O_YAW_RATE_SIGNAL_LATENCY_S     // Additional signal latency of yaw rate sensor
    type.float32 LSM_O_WHEELBASE_M     // Vehicle wheelbase. "copied from vehicle params"
    type.uint8 LSM_O_WHEEL_NUMBER_OF_TEETH_NU     // Number of teeth used to measure wheel pulses. "copied from vehicle params"
    type.float32 LSM_O_TYRE_CIRCUMFERENCE_FR_M     // Circumference of front wheels. "copied from vehicle params"
    type.float32 LSM_O_TYRE_CIRCUMFERENCE_RE_M     // Circumference of rear wheels. "copied from vehicle params"
    type.float32 LSM_O_TRACK_FRONT_M     // Track at front axle. "copied from vehicle params"
    type.float32 LSM_O_TRACK_REAR_M     // Track at rear axle. "copied from vehicle params"
    type.float32 LSM_O_STEER_ANG_TO_YAW_ANG_NU     // Influence of front steering angle on yaw angle (vehicle at standstill). "copied from vehicle params"
    type.float32 LSM_O_STEER_POLY_CTR_WHL_RAD     // 3nd deg polynomial coefficients of characteristic curve steergain "virtual" center wheel. "copied from vehicle params"
    type.float32 LSM_O_TYRE_CIRCUMFERENCE_IN_ESC     // tire circumference value used by the brake system to compute the wheel velocity from angular to translation value
    type.float32 LSM_O_REAR_AXLE_TO_COR_1_M     // Rear Axle to center of rotation option 1
    type.float32 LSM_O_REAR_AXLE_TO_COR_2_M     // Rear Axle to center of rotation option 2
    type.float32 LSM_O_REAR_AXLE_TO_COR_3_M     // Rear Axle to center of rotation option 3
    type.float32 LSM_O_SIDE_SLIP_ANGLE_GRADIENT_RADS2PM     // Side slip angle gradient (unit:rad*s^2/m)
    type.float32 LSM_O_IMU_MOUNTING_POS_TRANSLATION_M     // Imu mounting position translation from center to rear along x,y,z
    type.float32 LSM_O_IMU_MOUNTING_POS_ROTATION_RAD     // Imu mounting position rotation around the vehicle coordinate system axis x,y,z
----------------------------------------------------------------------------------------------------------

  # ID:  type.LSM_VEDODO.FC_VEDODO_Params_InterfaceVersion uml:Class
  Members:
    type.uint32 FC_VEDODO_Params_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.tce.DebugConstants uml:Class
    // 
  Members:
    type.uint8 NUM_MTS_DEBUG_FREESPACE_TCE = 10    
----------------------------------------------------------------------------------------------------------

  # ID:  type.tce.TceDebugPort uml:Class
  version: ::tce::TceDebugPort_InterfaceVersion::TceDebugPort_VERSION
    // None
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     // timestamp_us
    type.sint32 debugInt     // @unit{nu};freespace for MTS debug values
    type.float32 debugFloat     // @unit{nu};freespace for MTS debug values
    type.float32 tireCircInternalFL_m     // @unit{m};internal learned circumference for front right tire.
    type.float32 tireCircInternalFR_m     // @unit{m};internal learned circumference for rear left tire.
    type.float32 tireCircInternalRL_m     // @unit{m};internal learned circumference for rear right tire.
    type.float32 tireCircInternalRR_m     // @unit{m};internal learned circumference for front left tire.
    type.float32 gpsDisplacements_m     // @unit{m};calculated vehicle displacement from the GPS measurements.
    type.float32 vehDrivenDistanceWhl_m     // @unit{m};vehicle driven distance calculated by wheel sensor displacements.
    type.float32 vehDrivenDistanceGPS_m     // @unit{m};vehicle driven distance calculated by GPS displacements.
    type.float32 sumWheelRotations_rad     // @unit{rad};summation of the average of rear wheel angular rotations
    type.float32 earthEllipsoidRadius_m     // @unit{m};earth radius calculated from the ellipsoid model and latitude angle measured by GPS
    type.boolean gpsUpdateFlag     // @unit{bool};flag indicating reception of new GPS measurement.
----------------------------------------------------------------------------------------------------------

  # ID:  type.tce.TceDebugPort_InterfaceVersion uml:Class
  Members:
    type.uint32 TceDebugPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.tce.TceEstimationPort uml:Class
  version: ::tce::TceEstimationPort_InterfaceVersion::TceEstimationPort_VERSION
    // Result of tce estimation.
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     // timestamp_us
    type.float32 tireCircFL_m     // @unit{m};@range{1.5,3.0}
    type.float32 tireCircStdvFL_m     // @unit{m};@range{0.0,0.1};Standard deviation of estimated tyre circumference of front left wheel
    type.float32 tireCircFR_m     // @unit{m};@range{1.5,3.0};Estimated tyre circumference of front right wheel
    type.float32 tireCircStdvFR_m     // @unit{m};@range{0.0,0.1};Standard deviation of estimated tyre circumference of front right wheel
    type.float32 tireCircRL_m     // @unit{m};@range{1.5,3.0};Estimated tyre circumference of rear left wheel
    type.float32 tireCircStdvRL_m     // @unit{m};@range{0.0,0.1};Standard deviation of estimated tyre circumference of rear left wheel
    type.float32 tireCircRR_m     // @unit{m};@range{1.5,3.0};Estimated tyre circumference of rear right wheel
    type.float32 tireCircStdvRR_m     // @unit{m};@range{0.0,0.1};Standard deviation of estimated tyre circumference of rear right wheel
    type.float32 rearTrackWidth_m     // @unit{m};@range{na,na};Estimated rear axle trackwidth
    type.float32 rearTrackWidthStdv_m     // @unit{m};@range{na,na};Standard deviation of estimated rear axle track width
    type.boolean tireCircFLValid     // @unit{bool};Validity flag for the estimated tyre circumference of front left wheel
    type.boolean tireCircFRValid     // @unit{bool};Validity flag for the estimated tyre circumference of front right wheel
    type.boolean tireCircRLValid     // @unit{bool};Validity flag for the estimated tyre circumference of rear left wheel
    type.boolean tireCircRRValid     // @unit{bool};Validity flag for the estimated tyre circumference of rear right wheel
    type.boolean rearTrackWidthValid     // @unit{bool};Validity flag for the estimated rear axle track width
----------------------------------------------------------------------------------------------------------

  # ID:  type.tce.TceEstimationPort_InterfaceVersion uml:Class
  Members:
    type.uint32 TceEstimationPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.tce.TcePersDataPort uml:Class
  version: ::tce::TcePersDataPort_InterfaceVersion::TcePersDataPort_VERSION
    // None
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     // timestamp_us
    type.uint16 tireCircFL_0p1mm     // @unit{mm / 10};Stored learned value of front left wheel tire circumference in 0.1 mm unit.
    type.uint16 tireCircFR_0p1mm     // @unit{mm / 10};Stored learned value of front right wheel tire circumference in 0.1 mm unit.
    type.uint16 tireCircRL_0p1mm     // @unit{mm / 10};Stored learned value of rear left wheel tire circumference in 0.1 mm unit.
    type.uint16 tireCircRR_0p1mm     // @unit{mm / 10};Stored learned value of rear right wheel tire circumference in 0.1 mm unit.
    type.uint16 tireCircStdvFL_0p1mm     // @unit{mm / 10};Standard deviation of the last stored front left tire circumference in 0.1 mm unit.
    type.uint16 tireCircStdvFR_0p1mm     // @unit{mm / 10};Standard deviation of the last stored front right tire circumference in 0.1 mm unit.
    type.uint16 tireCircStdvRL_0p1mm     // @unit{mm / 10};Standard deviation of the last stored rear left tire circumference in 0.1 mm unit.
    type.uint16 tireCircStdvRR_0p1mm     // @unit{mm / 10};Standard deviation of the last stored rear right tire circumference in 0.1 mm unit.
    type.uint16 rearTrackWidth_0p1mm     // @unit{mm / 10};Stored learned value of rear track width in 0.1 mm unit.
    type.uint16 rearTrackWidthStdv_0p1mm     // @unit{mm / 10};Standard deviation of the last stored rear trackwidth in 0.1 mm unit.
    type.boolean dataChanged     // @unit{nu};flag to indicate if persistent data is changed.
----------------------------------------------------------------------------------------------------------

  # ID:  type.tce.TcePersDataPort_InterfaceVersion uml:Class
  Members:
    type.uint32 TcePersDataPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.tce.FC_TCE_Params uml:Class
  version: ::tce::FC_TCE_Params_InterfaceVersion::FC_TCE_Params_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint64 TCE_SYNC_BUFFER_DELAY_US     // GPS synchronization buffer time delay
    type.float32 TCE_WHEELBASE_M     // Vehicle wheel base (distance between center of rear axle and front axle)
    type.float32 TCE_TYRE_CIRCUMFERENCE_FL_M     // Vehicle configured tire circumference for front left wheel
    type.float32 TCE_TYRE_CIRCUMFERENCE_FR_M     // Vehicle configured tire circumference for front right wheel
    type.float32 TCE_TYRE_CIRCUMFERENCE_RL_M     // Vehicle configured tire circumference for rear left wheel
    type.float32 TCE_TYRE_CIRCUMFERENCE_RR_M     // Vehicle configured tire circumference for rear right wheel
    type.float32 TCE_TRACK_FRONT_M     // Vehicle track width, distance between center of wheels on the front axle.
    type.float32 TCE_TRACK_REAR_M     // Vehicle track width, distance between center of wheels on the rear axle.
    type.float32 TCE_MIN_TYRE_CIRCUMFERENCE_M     // lower limitation of tyre circumference
    type.float32 TCE_MAX_TYRE_CIRCUMFERENCE_M     // upper limitation of tyre circumference
    type.float32 TCE_SPEED_DEPENDANCY     // generic factor to tune the speed dependancy of the circumference
    type.uint8 TCE_WHEEL_NUMBER_OF_TEETH_NU     // Number of teeth in the wheel pulse/tick encoder
----------------------------------------------------------------------------------------------------------

  # ID:  type.tce.FC_TCE_Params_InterfaceVersion uml:Class
  Members:
    type.uint32 FC_TCE_Params_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MANAGER.MF_MANAGER_Consts uml:Class
  Members:
    type.uint8 AP_M_MAX_NUM_TRAJ_CTRL_POINTS = 20    
    type.uint8 AP_M_MAX_NUM_DRIVING_RESISTANCE = 4    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MANAGER.ActiveManeuveringFunction uml:Enumeration
  Members:
    FUNCTION_NONE = 0            
    FUNCTION_COLLISION_WARNING = 1            
    FUNCTION_LSCA = 2            
    FUNCTION_AUTOMATED_PARKING = 3            
    FUNCTION_TRAILER_HITCH_ASSIST = 4            
    FUNCTION_TRAILER_REVERSE_ASSIST = 5            
    FUNCTION_TRAINED_PARKING = 6            
    MAX_NUM_MANEUVERING_FUNCTIONS = 7            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MANAGER.LoCtrlRequestType uml:Enumeration
  Members:
    LOCTRL_OFF = 0            
    LOCTRL_BY_TRAJECTORY = 1            
    LOCTRL_BY_DISTANCE = 2            
    LOCTRL_BY_VELOCITY = 3            
    LOCTRL_BY_DISTANCE_VELOCITY = 4            
    LOCTRL_BY_ACCELERATION = 5            
    LOCTRL_VELOCITY_LIMITATION = 6            
    LOCTRL_EMERGENCY_STOP = 7            
    MAX_NUM_LO_REQUEST_TYPES = 8            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MANAGER.ActivateLoCtrlType uml:Enumeration
  Members:
    LOCTRL_NORMAL = 0            
    LOCTRL_OVERRIDE = 1            
    LOCTRL_DEGRADATION = 2            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MANAGER.RequestedDrivingDir uml:Enumeration
  Members:
    NOT_RELEVANT = 0            
    FORWARD = 1            
    BACKWARD = 2            
    MAX_NUM_DRIV_DIRECTIONS = 3            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MANAGER.LaCtrlRequestType uml:Enumeration
  Members:
    LACTRL_OFF = 0            
    LACTRL_BY_TRAJECTORY = 1            
    LACTRL_BY_ANGLE_FRONT = 2            
    LACTRL_BY_ANGLE_REAR = 3            
    LACTRL_BY_ANGLE_FRONT_REAR = 4            
    LACTRL_BY_TORQUE = 5            
    LACTRL_BY_CURVATURE = 6            
    LACTRL_COMF_ANGLE_ADJUSTMENT = 7            
    MAX_NUM_LA_REQUEST_TYPES = 8            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MANAGER.MFMPlannedTrajType uml:Enumeration
  Members:
    MFM_PERP_PARK_IN_TRAJ = 0            
    MFM_PAR_PARK_IN_TRAJ = 1            
    MFM_PERP_PARK_OUT_TRAJ = 2            
    MFM_PAR_PARK_OUT_TRAJ = 3            
    MFM_REMOTE_MAN_TRAJ = 4            
    MFM_UNDO_TRAJ = 5            
    MFM_MAX_NUM_PLANNED_TRAJ_TYPES = 6            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MANAGER.MFMDrivingResistanceType uml:Enumeration
  Members:
    MFM_NONE = 0            
    MFM_FALLING_LOW = 1            
    MFM_FALLING_MEDIUM = 2            
    MFM_FALLING_HIGH = 3            
    MFM_RISING_LOW = 4            
    MFM_RISING_MEDIUM = 5            
    MFM_RISING_HIGH = 6            
    MFM_WHEEL_STOPPER = 7            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MANAGER.DirectedLoCtrlRequest uml:Class
  Members:
    type.float32 requestedValue     
    type.MF_MANAGER.RequestedDrivingDir drivingDirRequest     
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MANAGER.MFMPlannedTraj uml:Class
  Members:
    type.float32 xTrajRAReq_m     
    type.float32 yTrajRAReq_m     
    type.float32 yawReq_rad     
    type.float32 crvRAReq_1pm     
    type.float32 distanceToStopReq_m     
    type.float32 velocityLimitReq_mps     
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MANAGER.MFMDrivingResistance uml:Class
  Members:
    type.float32 distance_m     
    type.MF_MANAGER.MFMDrivingResistanceType type_nu     
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MANAGER.ActiveManeuveringFunctionPort uml:Class
  version: ::mf_manager::ActiveManeuveringFunctionPort_InterfaceVersion::ActiveManeuveringFunctionPort_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.MF_MANAGER.ActiveManeuveringFunction loCtrlRequestOrigin     
    type.MF_MANAGER.ActiveManeuveringFunction laCtrlRequestOrigin     
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MANAGER.ActiveManeuveringFunctionPort_InterfaceVersion uml:Class
  Members:
    type.uint32 ActiveManeuveringFunctionPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------


  # ID:  type.MF_MANAGER.LoCtrlRequestPort uml:Class
  version: ::mf_manager::LoCtrlRequestPort_InterfaceVersion::LoCtrlRequestPort_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.boolean activateLoCtrl     
    type.MF_MANAGER.LoCtrlRequestType loCtrlRequestType     
    type.MF_MANAGER.ActivateLoCtrlType activateLoCtrlType_nu     
    type.boolean secureInStandstill     
    type.boolean comfortStopRequest     
    type.MF_MANAGER.DirectedLoCtrlRequest distanceToStopReq_m     
    type.MF_MANAGER.DirectedLoCtrlRequest velocityReq_mps     
    type.MF_MANAGER.DirectedLoCtrlRequest accelerationReq_mps2     
    type.MF_MANAGER.loCtrlRequestSource loCtrlRequestSource_nu     
    type.boolean remoteSelfTest     
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MANAGER.LoCtrlRequestPort_InterfaceVersion uml:Class
  Members:
    type.uint32 LoCtrlRequestPort_VERSION = 5    
----------------------------------------------------------------------------------------------------------


  # ID:  type.MF_MANAGER.LaCtrlRequestPort uml:Class
  version: ::mf_manager::LaCtrlRequestPort_InterfaceVersion::LaCtrlRequestPort_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.boolean activateLaCtrl     
    type.MF_MANAGER.LaCtrlRequestType laCtrlRequestType     
    type.float32 steerAngReqFront_rad     
    type.float32 steerAngReqRear_rad     
    type.float32 steerTorqueReq_Nm     
    type.float32 curvatureReq_1pm     
    type.MF_MANAGER.laCtrlRequestSource laCtrlRequestSource_nu     
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MANAGER.LaCtrlRequestPort_InterfaceVersion uml:Class
  Members:
    type.uint32 LaCtrlRequestPort_VERSION = 3    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MANAGER.laCtrlRequestSource uml:Enumeration
  Members:
    LADMC_REQ_SRC_NO_REQEUESTER = 0            
    LADMC_REQ_SRC_AUP = 1            
    LADMC_REQ_SRC_AUP_WITH_REMOTE = 6            
    LADMC_REQ_SRC_LSCA = 2            
    LADMC_REQ_SRC_REMOTE_MANEUVERING = 3            
    LADMC_REQ_SRC_MSP = 4            
    LADMC_REQ_SRC_SAFBAR = 5            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MANAGER.loCtrlRequestSource uml:Enumeration
  Members:
    LODMC_REQ_SRC_NO_REQEUESTER = 0            
    LODMC_REQ_SRC_AUP = 1            
    LODMC_REQ_SRC_AUP_WITH_REMOTE = 6            
    LODMC_REQ_SRC_LSCA = 2            
    LODMC_REQ_SRC_REMOTE_MANEUVERING = 3            
    LODMC_REQ_SRC_MSP = 4            
    LODMC_REQ_SRC_SAFBAR = 5            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MANAGER.TrajRequestPort uml:Class
  version: ::mf_manager::TrajRequestPort_InterfaceVersion::TrajRequestPort_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.MF_MANAGER.MFMPlannedTraj plannedTraj     
    type.MF_MANAGER.MFMPlannedTrajType trajType_nu     
    type.uint8 numValidCtrlPoints_nu     
    type.boolean drivingForwardReq_nu     
    type.boolean trajValid_nu     
    type.boolean newSegmentStarted_nu     
    type.boolean isLastSegment_nu     
    type.uint8 stepInTrajAfterIdx_nu     
    type.MF_MANAGER.MFMDrivingResistance drivingResistance     
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MANAGER.TrajRequestPort_InterfaceVersion uml:Class
  Members:
    type.uint32 TrajRequestPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MANAGER.FC_MF_Manager_Params uml:Class
  version: ::mf_manager::FC_MF_Manager_Params_InterfaceVersion::FC_MF_Manager_Params_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint8 AP_M_DUMMY_FOR_PDO     
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MANAGER.FC_MF_Manager_Params_InterfaceVersion uml:Class
  Members:
    type.uint32 FC_MF_Manager_Params_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.MF_LSCA_Consts uml:Class
  Members:
    type.uint8 MAX_BODY_SHAPE_SIZE = 16U    
    type.uint8 MAX_BODY_INDICES_SIZE = 17U    
    type.uint8 MAX_WHEEL_SHAPE_SIZE = 4U    
    type.uint8 MAX_HITCH_SHAPE_SIZE = 4U    
    type.uint8 MAX_MIRROR_SHAPE_SIZE = 4U    
    type.uint8 MAX_SIMPLE_BODY_SHAPE_SIZE = 4U    
    type.uint8 MAX_STATIC_OBJECT_SIZE_NU = 16U    
    type.uint8 MAX_DYNAMIC_OBJECT_SIZE_NU = 4U    
    type.uint8 MAX_DYNAMIC_OBJECT_PREDICTION_SIZE_NU = 6U    
    type.uint8 MAX_STATIC_OBJECTS_BRAKE_NU = 32U    
    type.uint8 MAX_DYNAMIC_OBJECTS_BRAKE_NU = 10U    
    type.uint8 STATIC_BRAKE_ROI_SIZE = 4U    
    type.uint8 DYNAMIC_BRAKE_ROI_SIZE = 4U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.LSCA_BRAKE_MODE uml:Enumeration
  Members:
    BRAKE_MODE_NONE = 0            
    BRAKE_MODE_COMFORT = 1            
    BRAKE_MODE_EMERGENCY = 2            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.LSCA_MODE uml:Enumeration
  Members:
    LSCA_MODE_OFF = 0            
    LSCA_MODE_FREEDRIVE = 1            
    LSCA_MODE_BACKUP = 2            
    LSCA_MODE_SECURE = 3            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.LSCA_STEER_MODE uml:Enumeration
  Members:
    STEER_MODE_NONE = 0            
    STEER_MODE_ANGLE_FRONT = 1            
    STEER_MODE_ANGLE_REAR = 2            
    STEER_MODE_ANGLE_FRONT_AND_REAR = 3            
    STEER_MODE_TORQUE = 4            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.LSCA_WARNING_STATUS uml:Enumeration
  Members:
    LSCA_WARNING_NONE = 0            
    LSCA_WARNING_ALL = 1            
    LSCA_WARNING_FRONT = 2            
    LSCA_WARNING_LEFT = 3            
    LSCA_WARNING_RIGHT = 4            
    LSCA_WARNING_REAR = 5            
    LSCA_WARNING_FRONT_LEFT = 6            
    LSCA_WARNING_FRONT_RIGHT = 7            
    LSCA_WARNING_REAR_LEFT = 8            
    LSCA_WARNING_REAR_RIGHT = 9            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.LSCA_STATE uml:Enumeration
  Members:
    LSCA_STATE_DEACTIVATED = 0            
    LSCA_STATE_ACTIVATED = 1            
    LSCA_STATE_INTERVENTION = 2            
    LSCA_STATE_ERROR = 3            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.LSCA_TUBE_MARKING uml:Enumeration
  Members:
    LSCA_TUBE_OFF = 0            
    LSCA_TUBE_LEFT = 1            
    LSCA_TUBE_RIGHT = 2            
    LSCA_TUBE_LEFT_AND_RIGHT = 3            
    LSCA_TUBE_FULL = 4            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.LscaStatusPort uml:Class
  version: ::mf_lsca::LscaStatusPort_InterfaceVersion::LscaStatusPort_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.MF_LSCA.LSCA_STATE brakingModuleState_nu     // Braking state machine state
    type.MF_LSCA.LSCA_STATE doorProtectionModuleState_nu     // Door protection state machine state
    type.MF_LSCA.LSCA_STATE rctaModuleState_nu     // Rear cross traffic assist state machine state
    type.MF_LSCA.LSCA_STATE PmpModuleState_nu     // Pedal misapplication protection state machine state
    type.MF_LSCA.LSCA_STATE steeringResistanceModuleState_nu     // Steering (Steering Resistance) state machine state
    type.MF_LSCA.LSCA_STATE steeringProposalModuleState_nu     // Steering (Steering Proposal) state machine state
    type.MF_LSCA.LSCA_MODE lscaOverallMode_nu     // General LSCA state machine state
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.LscaStatusPort_InterfaceVersion uml:Class
  Members:
    type.uint32 LscaStatusPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.LscaHMIPort uml:Class
  version: ::mf_lsca::LscaHMIPort_InterfaceVersion::LscaHMIPort_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.boolean activateBrakeInterventionScreen_nu     // Indicate to HMI that LSCA requests an information screen for the driver
    type.boolean enforceHMIConfirmation_nu     // Demand HMI confirmation after LSCA braking to release brake in standstill
    type.MF_LSCA.LSCA_WARNING_STATUS warningBody     // On what part of the ego body will the collision be
    type.MF_LSCA.LSCA_WARNING_STATUS warningWheel     // What wheel is affected by a collision with a low object
    type.MF_LSCA.LSCA_WARNING_STATUS warningObject     // Where is the object relative to the ego vehicle
    type.MF_LSCA.LSCA_TUBE_MARKING warningTube     // Indicate what side of the driving tube shall be marked as critical in HMI
    type.uint16 criticalObjectBrakeID_nu     // ID of most critical object
    type.boolean RctraAlertLeft_nu     // RCTRA test module output - left
    type.boolean RctraAlertRight_nu     // RCTRA test module output - right
    type.boolean DoorProtFL_nu     // Door protection module output - front left door
    type.boolean DoorProtFR_nu     // Door protection module output - front right door
    type.boolean DoorProtBL_nu     // Door protection module output - rear left door
    type.boolean DoorProtBR_nu     // Door protection module output - rear right door
    type.boolean DoorProtTrunk_nu     // Door protection module output - trunk
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.LscaHMIPort_InterfaceVersion uml:Class
  Members:
    type.uint32 LscaHMIPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.LscaBrakePort uml:Class
  version: ::mf_lsca::LscaBrakePort_InterfaceVersion::LscaBrakePort_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 distanceToStop_m     // Maximum allowed driving distance in current direction until a standstill must be reached
    type.boolean holdInStandstill_nu     // Request braking also in standstill e.g. via epb
    type.MF_LSCA.LSCA_BRAKE_MODE requestMode     // The current request mode
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.LscaBrakePort_InterfaceVersion uml:Class
  Members:
    type.uint32 LscaBrakePort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.LscaSteerPort uml:Class
  version: ::mf_lsca::LscaSteerPort_InterfaceVersion::LscaSteerPort_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 requestTorque_Nm     // Requested torque, if torque is requested
    type.float32 requestAngleFront_rad     // Requested front steering angle, if angle is requested
    type.float32 requestAngleRear_rad     // Requested rear steering angle, if angle is requested
    type.MF_LSCA.LSCA_STEER_MODE requestMode     // The current request mode
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.LscaSteerPort_InterfaceVersion uml:Class
  Members:
    type.uint32 LscaSteerPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.configGeneral_t uml:Class
    // Struct that contains all relevant general parameter data
  Members:
    type.uint32 simpleShapeActualSize_nu     
    type.cml.Vec2Df_POD simpleShape     // 4-point-shape for dynamic object calculations
    type.uint32 forwardLeftIndicesActualSize_nu     
    type.uint8 forwardLeftIndices     // Indices of points for forward left motion      #maybe this can be removed, using the bodyIndexLocationInfo?
    type.uint32 forwardRightIndicesActualSize_nu     
    type.uint8 forwardRightIndices     // Indices of points for forward right motion     #maybe this can be removed, using the bodyIndexLocationInfo?
    type.uint32 backwardLeftIndicesActualSize_nu     
    type.uint8 backwardLeftIndices     // Indices of points for backward left motion     #maybe this can be removed, using the bodyIndexLocationInfo?
    type.uint32 backwardRightIndicesActualSize_nu     
    type.uint8 backwardRightIndices     // Indices of points for backward right motion    #maybe this can be removed, using the bodyIndexLocationInfo?
    type.uint32 bodyIndexLocationFrontActualSize_nu     
    type.uint8 bodyIndexLocationFront     // Indices of points on the body front
    type.uint32 bodyIndexLocationBackActualSize_nu     
    type.uint8 bodyIndexLocationBack     // Indices of points on the body back
    type.uint32 bodyIndexLocationLeftActualSize_nu     
    type.uint8 bodyIndexLocationLeft     // Indices of points on the body left
    type.uint32 bodyIndexLocationRightActualSize_nu     
    type.uint8 bodyIndexLocationRight     // Indices of points on the body right
    type.uint32 bodyIndexLocationFrontLeftActualSize_nu     
    type.uint8 bodyIndexLocationFrontLeft     // Indices of points on the body left-front corner
    type.uint32 bodyIndexLocationFrontRightActualSize_nu     
    type.uint8 bodyIndexLocationFrontRight     // Indices of points on the body right-front corner
    type.uint32 bodyIndexLocationBackLeftActualSize_nu     
    type.uint8 bodyIndexLocationBackLeft     // Indices of points on the body left-back corner
    type.uint32 bodyIndexLocationBackRightActualSize_nu     
    type.uint8 bodyIndexLocationBackRight     // Indices of points on the body right-back corner
    type.boolean LscaActive_nu     // Overall function switch
    type.boolean onlyBackupMode     // Force lsca off when AP is off
    type.boolean proposalActive_nu     // Steering proposal switch
    type.boolean resistanceActive_nu     // Steering resistance switch
    type.boolean brakeStaticActive_nu     // Static brake switch
    type.boolean brakeDynamicActive_nu     // Dynamic brake switch
    type.boolean doorProtectionStatActive_nu     // Door opening protection switch for static objects
    type.boolean doorProtectionDynActive_nu     // Door opening protection switch for dynamic objects
    type.boolean reverseAssistDynActive_nu     // Reverse crossing traffic switch for dynamic objects
    type.boolean pedalMisapplicationActive_nu     // Pmp static switch
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.brakeModelParams_t uml:Class
    // Struct that contains all relevant brake model parameter data for one specific brake exeuction scenario
  Members:
    type.float32 deadtime_s     // Deadtime from lsca braking flag to first increase of brake pressure
    type.float32 rampTime_s     // Ramp time for linear increase until maximum brake pressure
    type.float32 maxDeceleration_ms     // Maximum brake deceleration
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.brakeModel_t uml:Class
    // Struct that contains all relevant brake model parameter data for one braking category (e.g. emergency or comfort)
  Members:
    type.MF_LSCA.brakeModelParams_t forwards     // Brake model for forward driving
    type.MF_LSCA.brakeModelParams_t backwards     // Brake model for backward driving
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.brakeSet_t uml:Class
    // Struct that contains all relevant brake model parameter data for both braking categories (emergency and comfort)
  Members:
    type.MF_LSCA.brakeModel_t emergencyBraking     // Brake model for emergency braking
    type.MF_LSCA.brakeModel_t comfortBraking     // Brake model for comfort braking
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.configBrake_t uml:Class
    // Struct that contains all relevant braking function parameter data
  Members:
    type.MF_LSCA.brakeSet_t brakeModel     // Brake model related parameters
    type.uint32 bodyAdjustmentActualSize_nu     
    type.cml.Vec2Df_POD bodyAdjustment     // Each body point can be adjusted for the brake function
    type.uint32 protectedMirrorShapeLeftActualSize_nu     
    type.cml.Vec2Df_POD protectedMirrorShapeLeft     // Per-point adjustment of the left mirror
    type.uint32 protectedMirrorShapeRightActualSize_nu     
    type.cml.Vec2Df_POD protectedMirrorShapeRight     // Per-point adjustment of the right mirror
    type.float32 enlargementFront_m     // Enlargement of ego shape to the front
    type.float32 enlargementRear_m     // Enlargement of ego shape to the rear
    type.float32 enlargementSide_m     // Enlargement of ego shape to the side
    type.float32 warnTime_s     // Time that the warning shall occur before the braking at constant speed
    type.float32 lowerActivationSpeedForwards_ms     // Minimum speed that needs to be exceeded for function activation - forwards
    type.float32 upperActivationSpeedForwards_ms     // Maximum speed that must not be exceeded for function activation - forwards
    type.float32 lowerDeactivationSpeedForwards_ms     // Minimum speed that needs to be exceeded for function to not become deactivated - forwards
    type.float32 upperDeactivationSpeedForwards_ms     // Minimum speed that must notbe exceeded for function to not become deactivated - forwards
    type.float32 lowerActivationSpeedBackwards_ms     // Minimum (unsigned consideration) speed that needs to be exceeded for function activation - backwards
    type.float32 upperActivationSpeedBackwards_ms     // Maximum (unsigned consideration) speed that must not be exceeded for function activation - backwards
    type.float32 lowerDeactivationSpeedBackwards_ms     // Minimum (unsigned consideration) speed that needs to be exceeded for function to not become deactivated - backwards
    type.float32 upperDeactivationSpeedBackwards_ms     // Minimum (unsigned consideration) speed that must not be exceeded for function to not become deactivated - backwards
    type.float32 maximumOverrideDistance_m     // Maximum distance that is ignored after override command
    type.float32 autoContinueOverrideDistance_m     // Maximum distance that is ignored after override command
    type.float32 marginDelay_s     // Safety margin in seconds (safety distance along trajectory = current speed * this parameter)
    type.float32 standStillTime_s     // Time that the vehicle must stay in standstill until it allows to proceed/releases the brake
    type.float32 ignoreTrailerX_m     // Every object behind this point is ignored if a trailer is attached
    type.float32 rimProtectionSpeed_mps     // Maximum allowed speed for driving over wheel traversanble objects - e.g. curbstone
    type.float32 rimProtectionAngle_deg     // Defines under what angle objects are considered as obstacles
    type.uint16 minTriggerCountStaticBrake_nu     // Minimum number of collision triggering before an EBA flag is propagated to the output
    type.uint8 minHeightConfidence_perc     // Minimum required height confidence to trust the classification
    type.uint8 minClassConfidence_perc     // Minimum required class confidence to trust the classification
    type.uint8 minStaticObjectProbability_perc     // Minimum required static detection probability for the function to react on an object
    type.boolean protectBody_nu     // Switch to determine if the body shall be protected against high objects
    type.boolean protectWheel_nu     // Switch to determine if the wheels shall be protected against body traversable objects
    type.boolean protectRim_nu     // Switch to determine if the rim shall be protected against speed-profile-dependent-wheel-traversable-objects
    type.boolean protectMirror_nu     // Switch to determine if the mirrors shall be protected against high objects
    type.boolean protectHitch_nu     // Switch to determine if the hitch shall be protected against high objects
    type.boolean comfortInManual_nu     // Switch to determine if there shall be used a comfort braking in manual mode (AP mode is always EBA)
    type.boolean autoContinue_nu     // Switch to determine if an automatic continue or a continue screen shall be used in manual mode
    type.boolean warnHigh_nu     // Warn before braking at high objects that are on collision course
    type.boolean warnLow_nu     // Warn before braking at low objects that are on collision course
    type.boolean enableBrakeLow_nu     // Disable the final brake triggering for low objects but don"t disable the calculation process - used for warning only
    type.boolean enableBrakeHigh_nu     // Disable the final brake triggering for high objects but don"t disable the calculation process - used for warning only
    type.boolean drivingTubeEnabled_nu     // Use a complete driving tube calculation for collision detection (instead of just the AAB of the driving tube)
    type.boolean checkDoors_nu     // Disable braking function if a door is openedisable the final brake triggering for high objects but not disable the calculation process - used for warning only
    type.boolean checkDriverSeatbelt_nu     // Disable braking function if the driver"s seatbelt is not fastened
    type.boolean checkTrunk_nu     // Disable braking function if the trunk is opened
    type.boolean forwardBrakeEnabledIfTrunkOpen_nu     // If trunk is open keep braking function active in front of the car, only disable if moving backwards
    type.boolean checkHood_nu     // Check if the engine hood is open or not
    type.boolean checkPedalOverrideGas_nu     // Check if stepping on the gas pedal deactivates LSCA
    type.boolean checkPedalOverrideBrake_nu     // Check if stepping on the brake pedal deactivates LSCA
    type.boolean checkDriverSeatOccupied_nu     // Check if the drivier is in the vehicle
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.configPmp_t uml:Class
  Members:
    type.float32 jerkModel_mps     // Modeled jerk for pedal kickdown. Assumption: dddot(x) = const = this value while kickdown
    type.float32 safetyMargin_m     // Safety margin in meters: will be added to the required distance to stop
    type.float32 safetyMargin_s     // Safety margin in seconds: will be converted into distance by multiplication with current speed
    type.float32 maxSpeedBackwards_mps     // If the speed is higher than this value while driving backwards, the function will not intervene
    type.float32 maxSpeedForwards_mps     // If the speed is higher than this value while driving forwards, the function will not intervene
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.configSteerPropose_t uml:Class
    // Struct that contains all relevant general parameter data for the objects inside the driving tube
  Members:
    type.float32 minDiffToAct_deg     // The minimum difference between the required left and right steering to avoid an obstacle to make the decision in which direction to steer
    type.float32 maxEffort_deg     // Maximum required steering effort for a steering proposal interference
    type.float32 debounceDist_m     // Required driven distance before a steering proposal request is processed
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.configSteerResist_t uml:Class
    // Struct that contains all relevant general parameter data for objects outside the driving tube
  Members:
    type.float32 steerMargin_deg     // The minimum difference between the required left and right steering to avoid an obstacle to make the decision in which direction to steer
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.configSteer_t uml:Class
    // Struct that contains all relevant general parameter data for the steering functions
  Members:
    type.MF_LSCA.configSteerPropose_t proposalData     // Steering Proposal specific data
    type.cml.Vec2Df_POD maxSteeringAngleLimits_deg     // Maximum allowed steering angles in degree
    type.cml.Vec2Df_POD maxSteeringAngleDelta_deg     // Maximum allowed steering angle rate in degree per second
    type.cml.Vec2Df_POD maxSteeringTorqueLimits_Nm     // Maximum allowed steering torques in NM
    type.cml.Vec2Df_POD maxSteeringTorqueDelta_Nms     // Maximum allowed steering torque rate in Nm per second
    type.MF_LSCA.configSteerResist_t resistanceData     // Steering resistance specific data
    type.float32 roiLengthOffset_m     // Minimum length of the ROI
    type.float32 roiLength_s     // Multiplier for velicity dependent ROI length
    type.float32 roiWidthOffset_m     // Minimum width of the ROI
    type.float32 roiWidth_s     // Multiplier for velicity dependent ROI length
    type.float32 roiTravelledDistOffsetProp_m     // Minimum travelled distance within the ROI for steering proposal
    type.float32 roiTravelledDistOffsetResist_m     // Minimum travelled distance within the ROI for steering resistance
    type.float32 roiTravelledDistProp_s     // Multiplier for travelled distance within the ROI for steering proposal
    type.float32 roiTravelledDistResist_s     // Multiplier for travelled distance within the ROI for steering resistance
    type.float32 enlargementFront_m     // Enlargement of ego shape to the front
    type.float32 enlargementRear_m     // Enlargement of ego shape to the rear
    type.float32 enlargementSide_m     // Enlargement of ego shape to the side
    type.float32 immediateOverrideTorque_Nm     // Driver hand torque which when exceeded immediatly disables steering functionalities
    type.float32 timeBasedOverrideTorque_Nm     // Driver hand torque which when exceeded for more than overrideTime_s seconds disables steering functionalities
    type.float32 overrideTime_s     // Time during which the driver hand torque needs to exceed timeBasedOverrideTorque_Nm before disabling steering functionalities
    type.float32 overrideDistance_m     // Distance to drive before steering is re-enabled after override by driver hand torque
    type.float32 lowerActivationSpeedForwards_ms     // Minimum speed that needs to be exceeded for function activation - forwards
    type.float32 upperActivationSpeedForwards_ms     // Maximum speed that must not be exceeded for function activation - forwards
    type.float32 lowerDeactivationSpeedForwards_ms     // Minimum speed that needs to be exceeded for function to become deactivated - forwards
    type.float32 upperDeactivationSpeedForwards_ms     // Maximum speed that must not be exceeded for function to become deactivated - forwards
    type.float32 lowerActivationSpeedBackwards_ms     // Minimum (unsigned consideration) speed that needs to be exceeded for function activation - backwards
    type.float32 upperActivationSpeedBackwards_ms     // Maximum (unsigned consideration) speed that must not be exceeded for function activation - backwards
    type.float32 lowerDeactivationSpeedBackwards_ms     // Minimum (unsigned consideration) speed that needs to be exceeded for function to become deactivated - backwards
    type.float32 upperDeactivationSpeedBackwards_ms     // Maximum (unsigned consideration) speed that must not be exceeded for function to become deactivated - backwards
    type.float32 rearSteerAngleRatio     // Ratio between rear and front steering angle
    type.uint8 minStaticObjectProbability_perc     // Minimum required static detection probability for the function to react on an object
    type.uint8 minDynamicObjectProbability_perc     // Minimum required dynamic detection probability for the function to switch off if a dynamic onject is within the ROI
    type.boolean stopInterventionIfError_nu     // Stop the steering intervention if a sensor error is detected
    type.boolean checkDoors_nu     // Disable steering function if a door is opened
    type.boolean checkDriverSeatbelt_nu     // Disable steering function if the driver"s seatbelt is not fastened
    type.boolean checkTrunk_nu     // Disable steering function if the trunk is opened
    type.boolean checkSeat_nu     // Disable steering function if the driver"s seat is not occupied
    type.boolean torqueInterfaceActive_nu     // Indicates true if steering by torque is active, otherwise steering by angle is active
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.configApplicationFeatures_t uml:Class
  Members:
    type.boolean treatCrowdedPedestriansAsStatic_nu     // If more than "numberOfPedestriansUntilCrowded_nu" pedestrians (including on bicycle) are in front of the car, treat them as static objects
    type.uint8 numberOfPedestriansUntilCrowded_nu     // Number of pedestrians (including on bicycle) until all pedestrians are considered static objects (until fewer than this number of pedestrians are detected)
    type.boolean ignoreDynamicVehicles_nu     // Ignore dynamic objects of type "vehicle"
    type.boolean checkLoDmcHandshake_nu     // Check if loDMC handshake was successful or not (and disable lsca or put it in error if not)
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.configDeveloper_t uml:Class
    // Struct that contains all relevant develop parameter data
  Members:
    type.boolean useFakeInputStatic_nu     // Fake Input switch for static object
    type.boolean useFakeInputDynamic_nu     // Fake Input switch for dynamic object
    type.boolean genericDeveloperSwitch_nu     // Can be used for quick testing of whatever
    type.uint8 messageLevel_nu     // Determine from what level on, messages shall be active
    type.uint8 messageComponent_nu     // Determine for what component, messages shall be active
    type.uint8 fakeEmScenario_nu     // Select from differnt FakeEmScenarios
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.LscaFunctionConfig uml:Class
  version: ::mf_lsca::LscaFunctionConfig_InterfaceVersion::LscaFunctionConfig_VERSION
    // LscaFunctionConfig Struct that contains all relevant function configuration data for lsca
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.MF_LSCA.configGeneral_t general     // General parameters
    type.MF_LSCA.configBrake_t functionBrake     // Brake function specific parameters
    type.MF_LSCA.configPmp_t functionPmp     // PMP function specific parameters
    type.MF_LSCA.configSteer_t functionSteer     // Steer function specific parameters
    type.MF_LSCA.configApplicationFeatures_t applicationFeatures     // Parameters and switches for application/client specific features
    type.MF_LSCA.configDeveloper_t developer     // Developer parameters (tests, fake objects)
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.LscaFunctionConfig_InterfaceVersion uml:Class
  Members:
    type.uint32 LscaFunctionConfig_VERSION = 3U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.LscaStaticObjectShapeSerializable uml:Class
    // Polygon that contains all coordinates for a static object shape description
  Members:
    type.uint32 actualSize     // Number of points in this object shape
    type.cml.Vec2Df_POD points     // Points in this shape
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.LscaStaticObjectListBrakeSerializable uml:Class
    // Collection of static objects for debug purposes
  Members:
    type.uint32 actualSize     // Number of objects in this list
    type.MF_LSCA.LscaStaticObjectShapeSerializable objects     // Objects in this list
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.LscaEgoBodyShapeSerializable uml:Class
    // Polygon that contains all coordinates for the body shape description
  Members:
    type.uint32 actualSize     // Number of points in this shape
    type.cml.Vec2Df_POD points     // Points in this shape
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.LscaEgoWheelShapeSerializable uml:Class
    // Polygon that contains all coordinates for the wheel shape description
  Members:
    type.uint32 actualSize     // Number of points in this shape
    type.cml.Vec2Df_POD points     // Points in this shape
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.LscaEgoHitchShapeSerializable uml:Class
    // Polygon that contains all coordinates for the hitch shape description
  Members:
    type.uint32 actualSize     // Number of points in this shape
    type.cml.Vec2Df_POD points     // Points in this shape
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.LscaEgoMirrorShapeSerializable uml:Class
    // Polygon that contains all coordinates for a mirror shape description
  Members:
    type.uint32 actualSize     // Number of points in this shape
    type.cml.Vec2Df_POD points     // Points in this shape
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.LscaEgoStaticRoiShapeSerializable uml:Class
    // Polygon that contains all coordinates for the static braking RoI shape description
  Members:
    type.uint32 actualSize     // Number of points in this shape
    type.cml.Vec2Df_POD points     // Points in this shape
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.LscaDynamicObjectShapeSerializable uml:Class
    // Polygon that contains all coordinates for a dynamic object shape description
  Members:
    type.uint32 actualSize     // Number of points in this object shape
    type.cml.Vec2Df_POD points     // Points in this shape
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.LscaDynamicObjectListBrakeSerializable uml:Class
    // Collection of dynamic objects for debug purposes
  Members:
    type.uint32 actualSize     // Number of objects in this list
    type.MF_LSCA.LscaDynamicObjectShapeSerializable objects     // Objects in this list
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.LscaDynamicObjectPredictionShapeSerializable uml:Class
    // Polygon that contains all coordinates for a dynamic object shape prediction description
  Members:
    type.uint32 actualSize     // Number of points in this object shape
    type.cml.Vec2Df_POD points     // Points in this shape
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.LscaDynamicObjectPredictionListBrakeSerializable uml:Class
    // Collection of dynamic object predictions (RoIs - regions of interest) for debug purposes
  Members:
    type.uint32 actualSize     // Number of object predictions in this list
    type.MF_LSCA.LscaDynamicObjectPredictionShapeSerializable rois     // Object predictions in this list
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.LscaEgoDynamicRoiShapeSerializable uml:Class
    // Polygon that contains all coordinates for the dynamic braking RoI shape description
  Members:
    type.uint32 actualSize     // Number of points in this shape
    type.cml.Vec2Df_POD points     // Points in this shape
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.LscaEgoSimpleBodyShapeSerializable uml:Class
    // Polygon that contains all coordinates for a simple 4-point body shape description
  Members:
    type.uint32 actualSize     // Number of points in this shape
    type.cml.Vec2Df_POD points     // Points in this shape
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.LscaStaticBrakeDebugDataSerializable uml:Class
    // Debug data regarding static breaking
  Members:
    type.MF_LSCA.LscaStaticObjectListBrakeSerializable objectsInRoi     // The objects that LSCA detected inside its Region of Interest
    type.MF_LSCA.LscaEgoBodyShapeSerializable bodyShape     // Polygon that contains all coordinates for the body shape description
    type.MF_LSCA.LscaEgoWheelShapeSerializable wheelFrontLeftShape     // Polygon that contains all coordinates for the front left wheel shape description
    type.MF_LSCA.LscaEgoWheelShapeSerializable wheelFrontRightShape     // Polygon that contains all coordinates for the front right wheel shape description
    type.MF_LSCA.LscaEgoWheelShapeSerializable wheelRearLeftShape     // Polygon that contains all coordinates for the rear left wheel shape description
    type.MF_LSCA.LscaEgoWheelShapeSerializable wheelRearRightShape     // Polygon that contains all coordinates for the rear right wheel shape description
    type.MF_LSCA.LscaEgoHitchShapeSerializable hitchShape     // Polygon that contains all coordinates for the hitch shape description
    type.MF_LSCA.LscaEgoMirrorShapeSerializable mirrorLeftShape     // Polygon that contains all coordinates for the left mirror shape description
    type.MF_LSCA.LscaEgoMirrorShapeSerializable mirrorRightShape     // Polygon that contains all coordinates for the right mirror shape description
    type.float32 rotationAngleToBrake     // Signed rotation until vehicle standstill
    type.cml.Vec2Df_POD icr     // Center or rotation for rotationAngleToBrake
    type.MF_LSCA.LscaEgoStaticRoiShapeSerializable roi     // Region of interest for static braking
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.LscaDynamicBrakeDebugDataSerializable uml:Class
    // Debug data regarding dynamic breaking
  Members:
    type.MF_LSCA.LscaDynamicObjectListBrakeSerializable currentPositions     // The current positions of the surrounding dynamic objects
    type.MF_LSCA.LscaDynamicObjectListBrakeSerializable endPositions     // The end positions of the surrounding dynamic objects (predicted positions after time-to-stop seconds)
    type.MF_LSCA.LscaDynamicObjectListBrakeSerializable collisionMomentPositions     // The collision moment positions of the surrounding dynamic objects (predicted positions in the last cycle before impact, if impact is imminent)
    type.MF_LSCA.LscaDynamicObjectPredictionListBrakeSerializable objectRois     // Polygons describing the trajectory of the surrounding dynamic objects until ego standstill (assuming ego starts braking right now)
    type.MF_LSCA.LscaEgoDynamicRoiShapeSerializable roi     // Region of interest for dynamic braking
    type.MF_LSCA.LscaEgoSimpleBodyShapeSerializable bodyShape     // Ego body shape (4 point approximation)
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.LscaGeneralDebugDataSerializable uml:Class
    // Debug data regarding general information
  Members:
    type.boolean gasPedalOverride     // Indicates if an override action by gas pedal is detected
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.LscaDebugDataPort uml:Class
  version: ::mf_lsca::LscaDebugDataPort_InterfaceVersion::LscaDebugDataPort_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.MF_LSCA.LscaStaticBrakeDebugDataSerializable staticBraking     // Debug data regarding static breaking
    type.MF_LSCA.LscaDynamicBrakeDebugDataSerializable dynamicBraking     // Debug data regarding dynamic breaking
    type.MF_LSCA.LscaGeneralDebugDataSerializable generalInfo     // Debug data regarding general information
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LSCA.LscaDebugDataPort_InterfaceVersion uml:Class
  Members:
    type.uint32 LscaDebugDataPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LVMD.LVMDSystemStatus uml:Enumeration
    // lvmd system status
  Members:
    LVMD_Status_OFF = 0            
    LVMD_Status_IDLE = 1            
    LVMD_Status_INACTIVE = 2            
    LVMD_Status_ACTIVE = 3            
    LVMD_Status_FAILURE = 4            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LVMD.LVMDWarningTrigger uml:Enumeration
    // lvmd warning trigger
  Members:
    LVMD_Trigger_NONE = 0            
    LVMD_Trigger_VISUAL = 1            
    LVMD_Trigger_VISUAL_AUDIO = 2            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LVMD.LVMDWarningStatus uml:Enumeration
    // lvmd warning status
  Members:
    LVMD_Warning_NONE = 0            
    LVMD_Warning_EGO_VEH_NOT_STANDSTILL = 1            
    LVMD_Warning_LEAD_VEHICLE_NOT_SELECTED = 2            
    LVMD_Warning_LEAD_VEH_NOT_MOVED_AWAY = 3            
    LVMD_Warning_LEAD_VEH_NOT_IN_RANGE = 4            
    LVMD_Warning_EGO_VEH_GEAR_POSITION_IN_REVERSE = 5            
    LVMD_Warning_LEAD_VEH_CUT_OUT = 6            
    LVMD_Warning_VRU_In_ROI = 7            
    LVMD_Warning_VEHICLE_APPEARED_FRONT_OF_EGO_VEHICLE = 8            
    LVMD_Warning_LEAD_VEHICLE_MOVEMENT_DETECTION_SYSTEM_FAULT = 9            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LVMD.LVMDLeadVehicleStatus uml:Class
    // lvmd lead Vehicle status
  Members:
    type.float32 leadvehicle_standstilltime     // @unit{s};@range{0,1000};Lead Vehicle Standstill Time
    type.float32 leadvehicle_forward_distance     // @unit{m};@range{0,10};Lead Vehicle Forward Distance
    type.boolean leadvehicle_valid     // Boolean to indicate whether Lead Vehicle is in ROI or Not
    type.float32 leadvehicle_driven_distance     // @unit{m};@range{0,15};Lead Vehicle Driven Distance from Standstill Condition
    type.uint16 leadvehicle_object_id     // Lead Vehicle Object Reference Id
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LVMD.LvmdStatusPort uml:Class
  version: ::mf_lvmd::LvmdStatusPort_InterfaceVersion::LvmdStatusPort_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.MF_LVMD.LVMDSystemStatus lvmdSystemStatus_nu     // lvmd status of the system
    type.MF_LVMD.LVMDWarningTrigger lvmdWarningTrigger_nu     // lvmd warning trigger
    type.MF_LVMD.LVMDWarningStatus lvmdWarningStatus_nu     // lvmd warning status
    type.MF_LVMD.LVMDLeadVehicleStatus lvmdLeadVehicleStatus_nu     // lvmd lead vehicle status
    type.uint8 numVehiclesinROI_nu     // Number of Dynamic Objects in ROI
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LVMD.LvmdStatusPort_InterfaceVersion uml:Class
  Members:
    type.uint32 LvmdStatusPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LVMD.LvmdParams uml:Class
  version: ::mf_lvmd::LvmdParams_InterfaceVersion::LvmdParams_VERSION
    // parameters of the lvmd function
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.boolean lvmdAct_nu     // activation of the lvmd
    type.uint8 lvmdWarningInput_nu     // warning input from the driver
    type.float32 lvmdMinForwardSelectionDistance_nu     // Minimum Forward Selection Distance
    type.float32 lvmdMaxForwardSelectionDistance_nu     // Maximum Forward Selection Distance
    type.float32 lvmdMinForwardAlertDistance_nu     // Minimum Forward Alert Distance
    type.float32 lvmdMaxDetectionRange_nu     // Maximum Detection Range
    type.float32 lvmdAlertMinDriveoffDistance_nu     // Minimum Lead Vehicle Driver Off Distance from Standstill for Providing alert
    type.float32 lvmdMinEgoStandstill_nu     // Minimum Ego vehicle Standstill time required for Selection in seconds
    type.float32 lvmdMinLeadStandstill_nu     // Minimum Lead vehicle Standstill time required for Selection in seconds
    type.float32 lvmdVisualWarningTime_nu     // Duration for which the Visual Warning should be shown
    type.float32 lvmdAudioWarningTime_nu     // Duration for which the Audio Warning should be shown
    type.float32 lvmdAlertMinLeadVelocity_nu     // Minimum Lead Vehicle Velocity to provide alert
    type.float32 lvmdAlertMaxLeadVelocity_nu     // Maximum Lead Vehicle Velocity to provide alert
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_LVMD.LvmdParams_InterfaceVersion uml:Class
  Members:
    type.uint32 LvmdParams_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_WhlProtectProc.WHPConstants uml:Class
    // 
  Members:
    type.uint8 NUM_MTS_DEBUG_FREESPACE_WHPP = 10    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_WhlProtectProc.WHPProcDebugPort uml:Class
  version: ::mf_whlprotectproc::WHPProcDebugPort_InterfaceVersion::WHPProcDebugPort_VERSION
    // WHPP internal signals for debug
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.sint32 debugInt     // @unit{nu};freespace for MTS debug values
    type.float32 debugFloat     // @unit{nu};freespace for MTS debug values
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_WhlProtectProc.WHPProcDebugPort_InterfaceVersion uml:Class
  Members:
    type.uint32 WHPProcDebugPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------


  # ID:  type.MF_WhlProtectProc.WHPProcOutputPort uml:Class
  version: ::mf_whlprotectproc::WHPProcOutputPort_InterfaceVersion::WHPProcOutputPort_VERSION
    // Signals from Wheel Protection Processing containg the information about wheels warning levels
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.MF_WhlProtectProc.WhlWarningLevel whlWarningLevel_nu     // @range{0,4};Wheel warning level for each wheel. Positions front left: 0, front right: 1, rear left: 2; rear right: 3, to be defined in an enum.
    type.boolean whlWarningPresent_nu     // true - warnings (any wheel) present; false - warnings (any wheel) not present
    type.boolean processingError_nu     // PDCP error information
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_WhlProtectProc.WHPProcOutputPort_InterfaceVersion uml:Class
  Members:
    type.uint32 WHPProcOutputPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_WhlProtectProc.WhlWarningLevel uml:Enumeration
    // Wheel warning level for each wheel. Positions front left: 0, front right: 1, rear left: 2; rear right: 3, to be defined in an enum.
  Members:
    WHP_NOT_AVAILABLE = 0            
    WHP_NO_WARNING = 1            
    WHP_LOW_WARNING = 2            
    WHP_HIGH_WARNING = 3            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_WhlProtectProc.FC_MF_WhlProtectProc_Params uml:Class
  version: ::mf_whlprotectproc::FC_MF_WhlProtectProc_Params_InterfaceVersion::FC_MF_WhlProtectProc_Params_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 WHP_P_BOX_WIDTH_EXT_OFF_M     // Box width exterior OFF (wheel size VW Passat B8 215 60 R16)
    type.float32 WHP_P_BOX_WIDTH_EXT_ON_M     // Box width exterior ON
    type.float32 WHP_P_BOX_WIDTH_EXT_CRIT_OFF_M     // Box width exterior OFF critical (3 stage warning)
    type.float32 WHP_P_BOX_WIDTH_EXT_CRIT_ON_M     // Box width exterior ON critical (3 stage warning)
    type.float32 WHP_P_BOX_WIDTH_INT_OFF_M     // Box width exterior OFF (wheel size VW Passat B8 215 60 R16)
    type.float32 WHP_P_BOX_WIDTH_INT_ON_M     // Box width exterior ON
    type.float32 WHP_P_BOX_WIDTH_INT_CRIT_OFF_M     // Box width exterior OFF critical (3 stage warning)
    type.float32 WHP_P_BOX_WIDTH_INT_CRIT_ON_M     // Box width exterior ON critical (3 stage warning)
    type.float32 WHP_P_BOX_LEN_AGDDIR_OFF_M     // Box length against the driving direction OFF
    type.float32 WHP_P_BOX_LEN_AGDDIR_ON_M     // Box length against the driving direction ON
    type.float32 WHP_P_BOX_LEN_AGDDIR_CRIT_OFF_M     // Box length against the driving direction OFF  critical (3 stage warning)
    type.float32 WHP_P_BOX_LEN_AGDDIR_CRIT_ON_M     // Box length in the driving direction OFF critical (3 stage warning)
    type.float32 WHP_P_BOX_LEN_DDIR_OFF_M     // Box length in the driving direction OFF
    type.float32 WHP_P_BOX_LEN_DDIR_ON_M     // Box length in the driving direction ON
    type.float32 WHP_P_BOX_LEN_DDIR_CRIT_OFF_M     // Box length against the driving direction OFF critical (3 stage warning)
    type.float32 WHP_P_BOX_LEN_DDIR_CRIT_ON_M     // Box length in the driving direction ON critical (3 stage warning)
    type.float32 WHP_P_OBJ_MAX_STATIC_HEIGHT_M     // Maximum object height for Wheel Protection relevance
    type.float32 WHP_P_OBJ_MIN_EXISTANCE_PROB_NU     // Minimum object existance probability for Wheel Protection relevance
    type.float32 WHP_P_OBJ_MIN_HEIGHT_CONF_PROB_NU     // Minimum height confidence for the height class of an obstacle
    type.uint8 WHP_P_CIRC_SECT_POLY_NU     // Number of polygons circular sectors will be divided in
    type.boolean WHP_P_ALREADY_RUN_OVER_NU     // Switch between calculating boxes around the wheel/only in the driving direction
    type.boolean WHP_P_CURVED_BOXES_NU     // Parameter to enable/disable curved boxes
    type.boolean WHP_P_3_STAGE_WARN_NU     // Parameter to toggle between 2 and 3 stage warning
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_WhlProtectProc.FC_MF_WhlProtectProc_Params_InterfaceVersion uml:Class
  Members:
    type.uint32 FC_MF_WhlProtectProc_Params_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.MF_MemPark_Consts uml:Class
  Members:
    type.uint8 MAX_NUM_PARKING_SLOTS_NU = 10    
    type.uint8 MAX_NUM_TRAJECTORY_NU = 5    
    type.uint8 MAX_NUM_REQUESTABLE_RELOCALIZATION_SLOTS = 10    
    type.uint8 MAX_SIZE_OF_POINTS = 200    // Parameter that determines the max size of points to;use for trajectory representation, each point takes place every meter.;E.g. for MAX_SIZE_OF_POINTS=100 we can represent 100 meters distance;due to 1 point per meter
    type.uint8 MAX_SLOTS_PER_METAMAP_NU = 1    
    type.uint8 MAX_TRAJECTORIES_PER_METAMAP_NU = 1    
    type.uint8 MAX_NUM_MAPS_STORED_NU = 10    
    type.uint8 INVALID_MAP_ID = 255    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.SystemDefinedPoseSide uml:Enumeration
    // Pose side of the target pose provided by the system
  Members:
    SYSTEM_DEF_POSE_CURRENT_EGO = 0            
    SYSTEM_DEF_POSE_RIGHT = 1            
    SYSTEM_DEF_POSE_LEFT = 2            
    SYSTEM_DEF_POSE_FRONT = 3            
    SYSTEM_DEF_POSE_REAR = 4            
    MAX_NUM_SYSTEM_DEF_POSE_SIDES = 5            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.SystemDefinedPoseType uml:Enumeration
    // Pose type of the target pose provided by the system
  Members:
    SYSTEM_DEF_TYPE_CURRENT_EGO = 0            
    SYSTEM_DEF_TYPE_PAR = 1            
    SYSTEM_DEF_TYPE_PERP_BWD = 2            
    SYSTEM_DEF_TYPE_PERP_FWD = 3            
    SYSTEM_DEF_TYPE_FOLLOW_BWD = 4            
    SYSTEM_DEF_TYPE_FOLLOW_FWD = 5            
    SYSTEM_DEF_TYPE_ANGLE_BWD = 6            
    MAX_NUM_SYSTEM_DEF_POSE_TYPES = 7            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.MemorizedParkingStatus uml:Enumeration
    // General status of memory parking indicating its current state.
  Members:
    MEMORIZED_PARKING_STATE_INIT = 0            
    MEMORIZED_PARKING_STATE_IDLE = 1            
    MEMORIZED_PARKING_STATE_ACTIVE = 2            
    MEMORIZED_PARKING_STATE_ERROR = 3            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.RedetectionRequest uml:Class
    // Elements of the localization request slots
  Members:
    type.uint8 mapID     
    type.LSM_GEOML.Pose_POD startPose     
    type.boolean initialLocalizationRequest     
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.RelocalizationRequestPort uml:Class
  version: ::mf_mempark::RelocalizationRequestPort_InterfaceVersion::RelocalizationRequestPort_VERSION
    // Request localization
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint8 numValidRequests_nu     // indicating the number of valid relocalization requests
    type.MF_MemPark.RedetectionRequest requestedRelocalizationSlots     // array indicating for which map a relocalization is requested
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.RelocalizationRequestPort_InterfaceVersion uml:Class
  Members:
    type.uint32 RelocalizationRequestPort_VERSION = 2    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.RecordingRequestPort uml:Class
  version: ::mf_mempark::RecordingRequestPort_InterfaceVersion::RecordingRequestPort_VERSION
    // Indicates the mapID and the storeRequest
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint8 mapID     
    type.boolean storeRequest     
    type.uint64 timestamp_us     
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.RecordingRequestPort_InterfaceVersion uml:Class
  Members:
    type.uint32 RecordingRequestPort_VERSION = 2    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.DeletionRequestPort uml:Class
  version: ::mf_mempark::DeletionRequestPort_InterfaceVersion::DeletionRequestPort_VERSION
    // Indicates the mapID and the deleteRequest
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint8 mapID     // ID of map that will be deleted; if INVALID_MAP_ID, no processing takes place
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.DeletionRequestPort_InterfaceVersion uml:Class
  Members:
    type.uint32 DeletionRequestPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.SystemDefinedPosePort uml:Class
  version: ::mf_mempark::SystemDefinedPosePort_InterfaceVersion::SystemDefinedPosePort_VERSION
    // Port containing the information of a pose with was saved by the system. This information can be used to park to this pose using the AUP stack
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.boolean valid     
    type.uint8 targetPoseID     
    type.LSM_GEOML.Pose_POD currentEgoInReference     
    type.LSM_GEOML.Pose_POD targetPoseInReference     
    type.float32 curvature_1pm     
    type.MF_MemPark.SystemDefinedPoseSide side     
    type.MF_MemPark.SystemDefinedPoseType type     
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.SystemDefinedPosePort_InterfaceVersion uml:Class
  Members:
    type.uint32 SystemDefinedPosePort_VERSION = 2    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.GPSSilentOfferingEnabled uml:Enumeration
    // Silent GPS offer condition
  Members:
    GPS_OFFER_ENABLED = 0            
    GPS_OFFER_DISABLED = 1            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.StoringStatus uml:Enumeration
    // The status of the storing, loading and deleting data.
  Members:
    STORING_STATUS_INIT = 0            
    STORING_STATUS_IDLE = 1            
    STORING_STATUS_ONGOING = 2            
    STORING_STATUS_SUCCESS = 3            
    STORING_STATUS_FAILED = 4            
    STORING_STATUS_NOT_VALID = 5            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.TrainingStatus uml:Enumeration
    // Current status of the training phase whether memory parking is
    // reasdy to save recorded data and reason for failure if it
    // cannot save data.
  Members:
    TRAIN_STATUS_INIT = 0            
    TRAIN_STATUS_ACTIVE_NOT_READY_TO_SAVE = 1            
    TRAIN_STATUS_ACTIVE_READY_TO_SAVE = 2            
    TRAIN_STATUS_FAILED_MAX_LIMIT_EXCEEDED = 3            
    TRAIN_STATUS_SAVE_FAILED_TIMEOUT = 4            
    TRAIN_STATUS_SAVE_FAILED_NO_SPACE = 5            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.LocalizationStatus uml:Enumeration
    // Current status of the localization request if the memory parking
    // is ready to offer slot or path or reason for failure if it
    // cannot localize path or slot.
  Members:
    LOCALIZE_STATUS_INIT = 0            
    LOCALIZE_STATUS_UNAVAIL_OUTSIDE_ZONE = 1            
    LOCALIZE_STATUS_ACTIVE = 2            
    LOCALIZE_STATUS_SUCCESS_OFFER = 3            
    LOCALIZE_STATUS_SUCCESS_NO_OFFER = 4            
    LOCALIZE_STATUS_FAILED = 5            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.UserUpdateRequestStatus uml:Enumeration
    // Current status of the slot update request from user. It indicates
    // if the update request was successful and if the last request
    // pushed the update to maximum allowed limits.
  Members:
    UPDATE_STATUS_INIT = 0            
    UPDATE_STATUS_SUCCESS = 1            
    UPDATE_STATUS_SUCCESS_MAX_C_ANGLE = 2            
    UPDATE_STATUS_SUCCESS_MAX_CC_ANGLE = 3            
    UPDATE_STATUS_SUCCESS_MAX_LONG_POS_UP = 4            
    UPDATE_STATUS_SUCCESS_MAX_LONG_POS_DOWN = 5            
    UPDATE_STATUS_SUCCESS_MAX_LAT_POS_LEFT = 6            
    UPDATE_STATUS_SUCCESS_MAX_LAT_POS_RIGHT = 7            
    UPDATE_STATUS_SUCCESS_MAX_ATTEMPTS = 8            
    UPDATE_STATUS_FAILED = 9            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.MemorizedSlot uml:Class
    // The features of the list of slots
  Members:
    type.uint8 ID     
    type.MF_MemPark.LocalizationStatus relocalizationStatus     
    type.MF_MemPark.GPSSilentOfferingEnabled gpsSilentOfferingEnabled     
    type.uint8 roadWidth     
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.TrainedPath uml:Class
    // The features of the list of paths.
  Members:
    type.uint8 ID     
    type.MF_MemPark.LocalizationStatus relocalizationStatus     
    type.MF_MemPark.GPSSilentOfferingEnabled gpsSilentOfferingEnabled     
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.MPStatus uml:Class
    // The elements for Memory Parking Status
  Members:
    type.uint8 numStoredMemoryParkingSlots_nu     
    type.MF_MemPark.MemorizedSlot memorizedParkingSlots     
    type.MF_MemPark.MemorizedParkingStatus memoryParkingState     
    type.MF_MemPark.TrainingStatus trainingStatus     
    type.MF_MemPark.LocalizationStatus localizationStatus     
    type.MF_MemPark.UserUpdateRequestStatus userUpdateRequestStatus     
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.TPStatus uml:Class
    // The elements for Trained Parking Status
  Members:
    type.uint8 numberStoredTrainedParkingPaths     
    type.MF_MemPark.TrainedPath trainedPaths     
    type.MF_MemPark.MemorizedParkingStatus memoryParkingState     
    type.MF_MemPark.TrainingStatus trainingStatus     
    type.MF_MemPark.LocalizationStatus localizationStatus     
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.MemoryParkingStatusPort uml:Class
  version: ::mf_mempark::MemoryParkingStatusPort_InterfaceVersion::MemoryParkingStatusPort_VERSION
    // The elements for MemoryParkingStatusPort
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.MF_MemPark.MPStatus mpStatus     
    type.MF_MemPark.TPStatus tpStatus     
    type.MF_MemPark.StoringStatus storingStatus     
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.MemoryParkingStatusPort_InterfaceVersion uml:Class
  Members:
    type.uint32 MemoryParkingStatusPort_VERSION = 3    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.MemParkParams uml:Class
  version: ::mf_mempark::MemParkParams_InterfaceVersion::MemParkParams_VERSION
    // Memory Parking Parameters
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.boolean functionActive_nu     // None
    type.float32 scanRoiHalfWidth_m     // Scan roi
    type.float32 scanRoiBackExtension_m     // Scan roi
    type.float32 scanRoiFrontExtension_m     // Scan roi
    type.uint8 minRequiredRelocalizationProb_perc     // ICP covergence criterion
    type.float32 memParkDeveloperParam_0     // developer parameters
    type.float32 memParkDeveloperParam_1     // Generic member for developer parameter.
    type.float32 memParkDeveloperParam_2     // Generic member for developer parameter.
    type.float32 memParkDeveloperParam_3     // Generic member for developer parameter.
    type.float32 memParkDeveloperParam_4     // Generic member for developer parameter.
    type.float32 memParkDeveloperParam_5     // Generic member for developer parameter.
    type.float32 memParkDeveloperParam_6     // Generic member for developer parameter.
    type.float32 memParkDeveloperParam_7     // Generic member for developer parameter.
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.MemParkParams_InterfaceVersion uml:Class
  Members:
    type.uint32 MemParkParams_VERSION = 2    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.MemParkDate_t uml:Class
    // Date structure for memory parking.
  Members:
    type.uint16 year     
    type.uint8 month     
    type.uint8 day     
    type.uint8 hour     
    type.uint8 minute     
    type.uint8 second     
    type.sint16 timeZone     
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.SlotMetaData uml:Class
    // Meta data of the parking slot
  Members:
    type.MF_MemPark.MemParkDate_t slotSaveDate     // Date containing mainly day/month/year,;as auxiliary info could include time (hour, minute, second);and time zone of the moment where the slot was saved.
    type.MF_MemPark.MemParkDate_t slotLastModification     // Date containing mainly day/month/year,;as auxiliary info could include time (hour, minute, second);and time zone of the moment where the slot was last modified.
    type.uint8 slotName     // Name of the slot for user reference.;See https://confluence.auto.continental.cloud/display/PLP/Slots+and+Trajectories+Interfaces;Temporary datatype set, should be string!
    type.uint8 egoVehicle     // [Optional] Identifies the vehicle that saved the slot.;See https://confluence.auto.continental.cloud/display/PLP/Slots+and+Trajectories+Interfaces;Temporary datatype set, should be string!
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.ParkingSlot uml:Class
  version: ::mf_mempark::ParkingSlot_InterfaceVersion::ParkingSlot_VERSION
    // Describes parking slots where the vehicle parked before.
    // This data will be saved to have further reference on new parking attempts on a location already visited.
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint8 slotID     
    type.AP_CommonVehSigProvider.GPSData slotGNSS     
    type.LSM_GEOML.Pose_POD slotPose     
    type.LSM_GEOML.Pose_POD startPose     // [Optional]
    type.MF_MemPark.SystemDefinedPoseSide slotSide     
    type.MF_MemPark.SystemDefinedPoseType slotType     
    type.uint8 correspondingMapId     
    type.MF_MemPark.SlotMetaData metaData     
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.ParkingSlot_InterfaceVersion uml:Class
  Members:
    type.uint32 ParkingSlot_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.TrajectoryMetaData uml:Class
    // Meta data of the parking trajectory
  Members:
    type.MF_MemPark.MemParkDate_t trajectorySaveDate     // Date containing mainly day/month/year,;as auxiliary info could include time (hour, minute, second);and time zone of the moment where the trajectory was saved.
    type.uint8 egoVehicle     // [Optional] Identifies the vehicle that saved the trajectory.;See https://confluence.auto.continental.cloud/display/PLP/Slots+and+Trajectories+Interfaces;Temporary datatype set, should be string!
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.TrajectoryPoint uml:Class
  Members:
    type.uint8 pointID     
    type.LSM_GEOML.Pose_POD pointPose     
    type.AP_CommonVehSigProvider.GPSData pointGNSS     
    type.boolean isGNSSValid     
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.ParkingTrajectory uml:Class
  version: ::mf_mempark::ParkingTrajectory_InterfaceVersion::ParkingTrajectory_VERSION
    // Describes the 2D path followed by the vehicle during a parking maneuver.
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint8 trajectoryID     
    type.LSM_GEOML.Pose_POD startPose     
    type.LSM_GEOML.Pose_POD endPose     
    type.uint8 numValidTrajPoints     
    type.MF_MemPark.TrajectoryPoint listOfPoints     
    type.uint8 slotID     
    type.MF_MemPark.TrajectoryMetaData metaData     
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.ParkingTrajectory_InterfaceVersion uml:Class
  Members:
    type.uint32 ParkingTrajectory_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.MetaMap uml:Class
  version: ::mf_mempark::MetaMap_InterfaceVersion::MetaMap_VERSION
    // This structure represents a MetaMap which contains arrays of Parking Slots and Parking Trajectories
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint8 correspondingMapID     
    type.uint8 numValidParkingSlots     
    type.MF_MemPark.ParkingSlot parkingSlots     
    type.uint8 numValidTrajectories     
    type.MF_MemPark.ParkingTrajectory parkingTrajectories     
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.MetaMap_InterfaceVersion uml:Class
  Members:
    type.uint32 MetaMap_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.MapIDToMetaMap uml:Class
    // This structure represents a key-value pair where a Map ID is associated with its corresponding MetaMap.
  Members:
    type.uint8 mapID     
    type.MF_MemPark.MetaMap metaMap     
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.MapsToMetaMaps uml:Class
  version: ::mf_mempark::MapsToMetaMaps_InterfaceVersion::MapsToMetaMaps_VERSION
    // This structure represents a collection of map IDs each associated with its corresponding MetaMap.
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint8 numValidMaps     
    type.MF_MemPark.MapIDToMetaMap mapsMetaMaps     
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.MapsToMetaMaps_InterfaceVersion uml:Class
  Members:
    type.uint32 MapsToMetaMaps_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.MemoryParkingDataRequestPort uml:Class
  version: ::mf_mempark::MemoryParkingDataRequestPort_InterfaceVersion::MemoryParkingDataRequestPort_VERSION
    // The request for an operation issued to the Parameter Handler when map data is ready (event-based triggering).
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint8 metaMapIdSave     // The unique identifier map to save.
    type.boolean isValidToSave     // The Id of the map is valid.
    type.uint8 metaMapIdLoad     // The unique identifier map to load.
    type.boolean isValidToLoad     // The Id of the map is valid.
    type.uint8 metaMapIdDelete     // The unique identifier map to delete.
    type.boolean isValidToDelete     // The Id of the map is valid.
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.MemoryParkingDataRequestPort_InterfaceVersion uml:Class
  Members:
    type.uint32 MemoryParkingDataRequestPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.MemoryParkingDataResultPort uml:Class
  version: ::mf_mempark::MemoryParkingDataResultPort_InterfaceVersion::MemoryParkingDataResultPort_VERSION
    // The response from the Parameter Handler to a request to perform an operation.
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint8 metaMapIdSaving     // The unique identifier of the saved map.
    type.MF_MemPark.StoringStatus savingStatus     // The state of the save request.
    type.uint8 metaMapIdLoading     // The unique identifier of the loaded map.
    type.MF_MemPark.StoringStatus loadingStatus     // The state of the load request.
    type.uint8 metaMapIdDeleting     // The unique identifier of the deleted map.
    type.MF_MemPark.StoringStatus deletingStatus     // The state of the delete request.
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_MemPark.MemoryParkingDataResultPort_InterfaceVersion uml:Class
  Members:
    type.uint32 MemoryParkingDataResultPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.MF_HMIH_Consts uml:Class
    // MHMIH consts
  Members:
    type.uint8 AP_H_MAX_NUM_SLOTS_SIDE_NU = 4    
    type.uint8 AP_H_MAX_NUM_PLACEHOLDER_NU = 9    
    type.uint8 AP_H_MAX_NUM_PARKING_POSES_NU = 20    
    type.uint8 AP_H_MAX_NUM_VISIBILITY_TAGS_NU = 35    
    type.uint8 MAX_NUM_SECTORS_PER_SIDE_HMIH = 4    
    type.uint8 NUM_MTS_DEBUG_FREESPACE_HMIH = 10    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.BlindSpotViewStatus uml:Enumeration
    // HMI screen content depending on system status
  Members:
    LEFT_BLIND_SPOT_VIEW = 0            
    RIGHT_BLIND_SPOT_VIEW = 1            
    BLIND_SPOT_VIEW_OFF = 2            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.ScreenHeadUnit uml:Enumeration
    // HMI screen content depending on system status
  Members:
    SYSTEM_NOT_ACTIVE = 0            
    SCANNING_ACTIVE = 1            
    PARKING_SPACE_SELECTION = 2            
    REMOTE_APP_ACTIVE = 3            
    MANEUVER_ACTIVE = 4            
    MANEUVER_FINISHED = 5            
    MANEUVER_INTERRUPTED = 6            
    UNDO_MANEUVER_ACTIVE = 7            
    START_REMOTE_APP = 8            
    PARK_OUT_INIT = 9            
    PARK_OUT_SIDE = 10            
    MENU_REM = 11            
    MANEUVER_ACTIVE_LONG_MAN = 12            
    REM_SV = 13            
    REM_MAN = 14            
    REM_MAN_ACTIVE = 15            
    KEY_CONTROL_ACTIVE = 16            
    PDC_ACTIVE = 17            
    GP_START = 18            
    GP_MANEUVER_ACTIVE = 19            
    DIAG_ERROR = 20            
    MP_SPACE_SELECTION = 21            
    MP_USER_ADJUSMENTS = 22            
    MP_WAITING_FOR_FINISH = 23            
    REVERSE_ASSIST = 24            
    PRE_CONDITIONS_NOT_MET = 25            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.AVGType uml:Enumeration
    // Shows the simplified situation during Automated Vehicle Guidance to enable HMI to inform driver of maneuver.
  Members:
    NO_AVG_TYPE = 0            
    PARK_IN_PARALLEL_LEFT = 1            
    PARK_IN_PARALLEL_RIGHT = 2            
    PARK_IN_PERPENDICULAR_LEFT_FWD = 3            
    PARK_IN_PERPENDICULAR_LEFT_BWD = 4            
    PARK_IN_PERPENDICULAR_RIGHT_FWD = 5            
    PARK_IN_PERPENDICULAR_RIGHT_BWD = 6            
    PARK_OUT_PARALLEL_LEFT = 7            
    PARK_OUT_PARALLEL_RIGHT = 8            
    PARK_OUT_PERPENDICULAR_LEFT_FWD = 9            
    PARK_OUT_PERPENDICULAR_LEFT_BWD = 10            
    PARK_OUT_PERPENDICULAR_RIGHT_FWD = 11            
    PARK_OUT_PERPENDICULAR_RIGHT_BWD = 12            
    REM_MAN_FWD = 13            
    REM_MAN_BWD = 14            
    GP_GENREAL = 15            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.SlotUnreachReason uml:Enumeration
    // Keeps information about why a slot is not reachable
  Members:
    NO_UNREACHABLE_REASON = 0            
    SLOT_TOO_NARROW = 1            
    SLOT_TOO_SHORT = 2            
    SLOT_EXCEEDS_MAX_PB = 3            
    SLOT_WHEEL_COLLISION = 4            
    SLOT_HIGH_OBJECT_COLLISION = 5            
    UNKNOWN_REASON = 6            
    NO_PATH_FOUND = 7            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.PossOrientation uml:Enumeration
    // Indicates wether orientation of parking space is certain or not.
  Members:
    POSS_ORI_CERTAIN = 0            
    POSS_ORI_UNCERTAIN = 1            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.SelectedOrientation uml:Enumeration
    // If orientation is uncertain, the user can choose the final orientation by a button. The result of the user selection is indicated in this signal. If selection of orientation is not possible, the signal value indicates the only possible parking orientation.
  Members:
    SEL_ORI_PARALLEL = 0            
    SEL_ORI_PERPENDICULAR = 1            
    SEL_ORI_ANGLED_STANDARD = 2            
    SEL_ORI_ANGLED_REVERSE = 3            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.PossibleDirection uml:Enumeration
    // Indicates wether it is possible to select direction of vehicle in parking space (BOTH_DIRECTIONS) or not.
  Members:
    POSS_DIR_ONE_DIRECTION = 0            
    POSS_DIR_BOTH_DIRECTIONS = 1            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.SelectedDirection uml:Enumeration
    // If selection of direction is possible, the user can choose the direction of the vehicle in the parking space. The result of the user selection is indicated in this signal. If selection of direction is not possible, the signal value indicates the only possible parking direction.
  Members:
    SEL_DIR_FORWARDS = 0            
    SEL_DIR_BACKWARDS = 1            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.DrvTubeDirection uml:Enumeration
    // Driving tube direction
  Members:
    PDC_DRV_TUBE_RIGHT = 0            
    PDC_DRV_TUBE_LEFT = 1            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.WheelAngDirection uml:Enumeration
    // Direction of the wheel angle for each wheel. Positions front left: 0, front right: 1, rear left: 2; rear right: 3.
  Members:
    WHL_ANG_DIR_LEFT = 0            
    WHL_ANG_DIR_RIGHT = 1            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.APUserActionRemoteDevice uml:Enumeration
    // User interaction with HMI handler (Remote Parking App)
  Members:
    AP_REM_NO_USER_ACTION = 0            
    AP_REM_APP_STARTED = 1            
    AP_REM_APP_CLOSED = 2            
    AP_REM_TAP_ON_START_PARKING = 3            
    AP_REM_TAP_ON_INTERRUPT = 4            
    AP_REM_TAP_ON_CONTINUE = 5            
    AP_REM_TAP_ON_UNDO = 6            
    AP_REM_TAP_ON_CANCEL = 7            
    AP_REM_TAP_ON_REDO = 8            
    AP_REM_TAP_ON_PARK_IN = 9            
    AP_REM_TAP_ON_PARK_OUT = 10            
    AP_REM_TAP_ON_REM_MAN = 11            
    AP_REM_TAP_ON_REM_SV = 12            
    AP_REM_TAP_ON_REM_FWD = 13            
    AP_REM_TAP_ON_REM_BWD = 14            
    AP_REM_TAP_ON_PREVIOUS_SCREEN = 15            
    AP_REM_TAP_ON_GP = 16            
    AP_REM_TAP_SWITCH_TO_HEAD_UNIT = 17            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.APUserActionHeadUnit uml:Enumeration
    // User interaction with HMI handler (Head Unit)
  Members:
    AP_NO_USER_ACTION = 0            
    AP_TAP_ON_START_SELECTION = 1            
    AP_TAP_ON_START_PARKING = 2            
    AP_TAP_ON_INTERRUPT = 3            
    AP_TAP_ON_CONTINUE = 4            
    AP_TAP_ON_UNDO = 5            
    AP_TAP_ON_CANCEL = 6            
    AP_TAP_ON_REDO = 7            
    AP_TAP_ON_START_REMOTE_PARKING = 8            
    AP_TAP_ON_SWITCH_DIRECTION = 9            
    AP_TAP_ON_SWITCH_ORIENTATION = 10            
    AP_TAP_ON_PREVIOUS_SCREEN = 11            
    AP_TOGGLE_AP_ACTIVE = 12            
    AP_TAP_ON_FULLY_AUTOMATED_PARKING = 13            
    AP_TAP_ON_SEMI_AUTOMATED_PARKING = 14            
    AP_TAP_ON_START_KEY_PARKING = 15            
    AP_TAP_ON_GP = 16            
    AP_TAP_ON_RM = 17            
    AP_TAP_SWITCH_TO_REMOTE_APP = 18            
    AP_TAP_SWITCH_TO_REMOTE_KEY = 19            
    AP_TAP_ON_EXPLICIT_SCANNING = 20            
    AP_TAP_ON_REVERSE_ASSIST = 21            
    AP_TAP_ON_USER_SLOT_DEFINE = 22            
    AP_TAP_ON_MEMORY_PARKING = 23            
    AP_TAP_ON_USER_SLOT_REFINE = 24            
    AP_TAP_ON_USER_SLOT_SAVE = 25            
    AP_TAP_ON_USER_SLOT_CLOSE = 26            
    AP_TAP_ON_USER_SLOT_DELETE = 27            
    AP_TAP_ON_CROSS = 28            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.PDCUserActionHeadUnit uml:Enumeration
    // User interaction with HMI handler (Head Unit) regarding PDW
  Members:
    PDC_NO_USER_ACTION = 0            
    PDC_TAP_ON_ACT_DEACT_BTN = 1            
    PDC_TAP_ON_MUTE_BTN = 2            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.WHPUserActionHeadUnit uml:Enumeration
    // User interaction with HMI handler (Head Unit) regarding WHP
  Members:
    WHP_NO_USER_ACTION = 0            
    WHP_TAP_ON_ACT_DEACT_BTN = 1            
    WHP_TAP_ON_MUTE_BTN = 2            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.LVMDUserActionHeadUnit uml:Enumeration
    // User interaction with HMI handler (Head Unit) regarding LVMD
  Members:
    LVMD_NO_USER_ACTION = 0            
    LVMD_TAP_ON_ACT_DEACT_BTN = 1            
    LVMD_TAP_ON_MUTE_BTN = 2            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.UserDefinedSlotSide uml:Enumeration
    // Keeps information abut the side where the user has defined a new slot.
  Members:
    USER_DEF_SIDE_CURRENT_EGO = 0            
    USER_DEF_SIDE_RIGHT = 1            
    USER_DEF_SIDE_LEFT = 2            
    USER_DEF_SIDE_FRONT = 3            
    USER_DEF_SIDE_REAR = 4            
    USER_DEF_SIDE_NOT_VALID = 5            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.UserDefinedSlotType uml:Enumeration
    // Keeps information about the type of slot defined by the user.
  Members:
    USER_DEF_TYPE_CURRENT_EGO = 0            
    USER_DEF_TYPE_PAR = 1            
    USER_DEF_TYPE_PERP_BWD = 2            
    USER_DEF_TYPE_PERP_FWD = 3            
    USER_DEF_TYPE_ANGL_BWD = 4            
    USER_DEF_TYPE_NOT_VALID = 5            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.HMIState uml:Enumeration
    // State of the  hmi communication state machine
  Members:
    HMI_INACTIVE = 0            
    HMI_SCANNING_IN = 1            
    HMI_SCANNING_OUT = 2            
    HMI_DIRECTION_SELECTION = 3            
    HMI_PARKING_SLOT_SELECTION = 4            
    HMI_LOW_ENERGY_LEVEL = 5            
    HMI_USER_MANEUVERING_REQ = 6            
    HMI_VIS_MANEUVERING_CONDITIONS = 7            
    HMI_VIS_PARKING_MANEUVER = 8            
    HMI_MANEUVERING_PAUSE = 9            
    HMI_PARKING_FAILED = 10            
    HMI_REQUEST_HANDOVER = 11            
    HMI_CONNECT_REMOTE_DEVICE = 12            
    HMI_SHOW_CIRCLE_SCREEN = 13            
    HMI_REM_DIRECTION_SELECTION = 14            
    HMI_REM_SLOT_SELECTION = 15            
    HMI_REM_RM_CHOSEN = 16            
    HMI_CANCELED_BY_USER = 17            
    HMI_CANCELED_BY_BEHAVIOR = 18            
    HMI_PARKING_SUCCESS = 19            
    HMI_REVERSIBLE_ERROR = 20            
    HMI_IRREVERSIBLE_ERROR = 21            
    HMI_INACTIVE_NO_ERROR_MODE = 22            
    HMI_SUB_HMI_INACTIVE = 23            
    HMI_PDC_COMM = 24            
    HMI_SCANNING_GP_IN = 25            
    HMI_SCANNING_GP_OUT = 26            
    HMI_REM_GP_SLOT_SELECTION = 27            
    HMI_MP_SLOT_ADJUSTMENT = 28            
    HMI_MP_SLOT_SELECTION = 29            
    HMI_REVERSE_ASSIST = 30            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.ParkingSpace uml:Class
    // Status of the left parking spaces
  Members:
    type.boolean scanned_nu     // Parking space [1,...,4] scanned
    type.boolean free_nu     // Parking space [1,...,4] free
    type.boolean selected_nu     // Selected parking space for parking maneuver
    type.MF_HMIH.PossOrientation possOrientation_nu     // @range{0,1};Indicates wether orientation of parking space is certain or not.
    type.MF_HMIH.SelectedOrientation selectedOrientation_nu     // @range{0,3};If orientation is uncertain, the user can choose the final orientation by a button. The result of the user selection is indicated in this signal. If selection of orientation is not possible, the signal value indicates the only possible parking orientation.
    type.MF_HMIH.PossibleDirection possDirection_nu     // @range{0,1};Indicates wether it is possible to select direction of vehicle in parking space (BOTH_DIRECTIONS) or not.
    type.MF_HMIH.SelectedDirection selectedDirection_nu     // @range{0,1};If selection of direction is possible, the user can choose the direction of the vehicle in the parking space. The result of the user selection is indicated in this signal. If selection of direction is not possible, the signal value indicates the only possible parking direction.
    type.uint8 poseID_nu     // @range{0,255};ID of corresponding target pose.
    type.float32 memorizedPoseYaw_rad     // @range{-3.14159265359,3.14159265359};Yaw angle for the redetected memorized pose(currently only one pose can be redetected).;It is filled only in case a memorized pose is redetected on that side and used by the HMI to correctly display the rotation angle of the pose.
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.ParkingSituationSides uml:Class
    // Situation on left side of parking vehicle
  Members:
    type.uint8 egoRelativePos_nu     // Ego vehicle rear axle position, relative to the parking slots. 0 = rear axle is one slot width ahead of the top most slot. Increment 1 means half a slot width further down.
    type.boolean perpendicularParkingSpaces_nu     // Perpendicular parking spaces on left side of vehicle
    type.boolean parallelParkingSpaces_nu     // Parallel parking spaces on left side of vehicle
    type.boolean angledStandardSpaces_nu     // Angled parking spaces on left side of vehicle in orientation for forwards parking in
    type.boolean angledReverseSpaces_nu     // Angled parking spaces on left side of vehicle in orientation for backwards parking in
    type.boolean street_nu     // Street on left side of vehicle
    type.boolean uncertainSituation_nu     // If perpendicular and parallel parking is possible on this side (note: One situation will be preselected).
    type.boolean parallelParkingOut_nu     // Parking out "parking spaces" (target direction) on the left side of vehicle in a parallel parking situation.
    type.boolean notAvailable_nu     // Especially for parking out or if no parking spaces are available
    type.boolean placeholder_nu     // Reserved for additional situations
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.ParkingSituationFrontRear uml:Class
    // Situation in front of parking vehicle
  Members:
    type.boolean perpendicularParkingOut_nu     // Parking out "parking spaces" (target direction) in front of vehicle in a perpendicular parking situation.
    type.boolean notAvailable_nu     // Usually set for parking in
    type.boolean placeholder_nu     // Reserved for additional situations
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.PDCSectorInfo uml:Class
    // Information for the PDC sectors from the left side (numbering is done from front towards rear)
  Members:
    type.float32 smallestDistance_m     // Distance between the car contour and the closest obstacle for this sector
    type.PDCP.CriticalityLevel criticalityLevel_nu     // @range{0,3};Criticality level for this sector (the higher the criticality, the closer the obstacle is to the car).
    type.uint8 slice_nu     // @range{0,3};Slice number within the criticality level (numbering is done from the outside of the car towards the car contour).
    type.uint8 sectorID_nu     // Sector ID (unique over all sectors)
    type.boolean intersectsDrvTube_nu     // Indicates if the obstacle that determines the criticality of this sector intersects the driving tube
    type.boolean scanned_nu     // Indicates if this sector was scanned (scanned means that PDCP had valid information to calculate the smallestDistance_nu).
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.HMIGeneral uml:Class
    // Major screen control of HMI.
  Members:
    type.AP_PSM_APP.RemoteMode remoteModeActive_nu     // @range{0,4};[Optional] Indicates the currently active mode of AP related to remote functionality
    type.boolean remoteAppActive_nu     // Indicates, whether the Remote App is active (= is opened and is not breaked down)
    type.boolean remoteAppAuthorized_nu     // True if the usage of the remote Control is authorized
    type.boolean remoteAppCoded_nu     // Indicates, that the remote control is coded for the current sw
    type.boolean remoteKeySelected_nu     // True if user selected Remote Key instead of in vehicle parking
    type.boolean remoteKeyPoss_nu     // Indicates, whether Remote Key is possible in the vehicle
    type.boolean continuePoss_nu     // Indicates wether it is possible to continue the interrupted maneuver
    type.boolean parkInPoss_nu     // True, if remote parking in is possible. (In this case, user can select it via menu of the remote parking app)
    type.boolean parkOutPoss_nu     // True, if remote parking out is possible. (In this case, user can select it via menu of the remote parking app)
    type.boolean remManPoss_nu     // True, if remote maneuvering is possible. (In this case, user can select it via menu of the remote parking app)
    type.boolean undoPoss_nu     // Indicates wether it is possible to undo the interrupted maneuver
    type.boolean svPoss_nu     // True, if the vehicle can stream a surround-view view to the remote parking app
    type.boolean btnForwardPoss_nu     // [for Remote Maneuvering]True, if it is possible to move the ego vehicle in forward direction -> Forward-Button is active in REM_MAN-Screen
    type.boolean btnBackwardPoss_nu     // [for Remote Maneuvering]True, if it is possible to move the ego vehicle in backward direction -> Backward-Button is active in REM_MAN-Screen
    type.boolean btnFullyAutomParkingPoss_nu     // Indicates, if Fully Automated Parking is possible (shows the corespondent button)
    type.boolean btnSemiAutomParkingPoss_nu     // Indicates, if Semi Automated Parking is possible (shows the corespondent button)
    type.boolean garageOpenerAvail_nu     // Indicates, that a remote garage door opener is available and configured (true, when GARAGE_OPENER_STATUS_NU == GOS_CONFIGURED)
    type.uint8 distanceToStop_perc     // @unit{Percent};@range{0,102};Distance to next stopping point during parking maneuver (could be shown in a filling bar)
    type.AP_PSM_APP.MaxSpeed10KPHwarning maxSpeed10KPHwarning_nu     // @range{0,2};For manual gear box, warning maximum speed limit : 10KPH
    type.AP_Common.DrivingDirection drivingDirection_nu     // @range{0,3};Vehicle direction for the current maneuver part
    type.AP_PSM_APP.GarageParking garageParking_nu     // @range{0,6};0: set, when either GARAGE_PARKING_CODED_NU==false or when no garage has been detected 1: set, when a garage scan is active in the front of the vehicle 2: set, when a garage scan is active in the back of the vehicle 3: set, wehen a forward parking out situation has been detected . 4: set, when a backward parking out situation has been detected 5: set, when a garage has been detected in the front of the vehicle 6: set, when a garage has been detected in the back of the vehicle
    type.MF_HMIH.AVGType avgType_nu     // @range{0,15};Shows the simplified situation during Automated Vehicle Guidance to enable HMI to inform driver of maneuver.
    type.AP_CommonVehSigProvider.Gear currentGear_nu     // @range{0,15};Current gear of the ego vehicle. This information is displayed in automated vehicle guidance (AVG) mode.
    type.AP_PSM_APP.HMIMessage generalUserInformation_nu     // @range{0,58};general information for the User. Extended User interface. Currently NOT used. Can be used to get rid of the visualization ports
    type.AP_PSM_APP.PPCParkingMode ppcParkingMode_nu     // @range{0,11};current active parking mode. Extended User interface. Currently NOT used. Can be used to get rid of the visualization ports
    type.AP_PSM_APP.APFinishType finishType_nu     // @range{0,3};type of the finish event. Extended User interface. Currently NOT used. Can be used to get rid of the visualization ports
    type.MF_DRVWARNSM.PDWSystemState pdcSystemState_nu     // @range{0,5};PDw system state info
    type.MF_LVMD.LVMDSystemStatus lvmdSystemState_nu     // @range{0,4};LVMD system state info
    type.MF_LVMD.LVMDWarningTrigger lvmdWarningType_nu     // @range{0,2};LVMD warning type
    type.MF_DRVWARNSM.PDWShutdownCause pdcShutdownCause_nu     // @range{0,3};PDW shutdown cause in case PDW system state is off
    type.MF_DRVWARNSM.WHPState whpState_nu     // @range{0,4};WHP state info
    type.boolean whpDisplayReq_nu     // WHP display request
    type.boolean apSwitchInputDevicePoss_nu     // describs if it is possible to switch the input device
    type.boolean memoryParkingPoss_nu     
    type.boolean displayBackButton_nu     // Indicates if the back button should be displayed. Purpose of the button is to allow the user to go back to top view;from the 3D view that is displayed once a slot was selected by the user.
    type.MF_HMIH.BlindSpotViewStatus blindSpotViewStatus_nu     
    type.MF_HMIH.SlotUnreachReason slotUnreachReason_nu     // Describes the reason why the selected slot is not reachable, when the user selects an unreachable pose.
    type.uint8 memorySlotsStatus_nu     // Bitarray indicating which of the memory slots are occupied (bit X = 1 means slot X contains a memorized user defined pose).
    type.uint8 redetectedPoseMemSlotId_nu     // Indicates the memory slot id that stores the redetected memorized pose.
    type.uint8 adjustmentButtons_nu     // Indicates if the adjustments buttons are disabled or if movement in the selected direction is still possibile(Bit X=1 means movement button X is available)
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.ParkingSpaces uml:Class
    // Status of the parking spaces
  Members:
    type.MF_HMIH.ParkingSpace left     // @unit{0};Status of the left parking spaces
    type.MF_HMIH.ParkingSpace right     // @unit{0};Status of the right parking spaces
    type.MF_HMIH.ParkingSpace front     // @unit{0};Status of the front parking spaces
    type.MF_HMIH.ParkingSpace rear     // @unit{0};Status of the rear parking spaces
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.ParkingSituation uml:Class
    // Parking space situation around the vehicle
  Members:
    type.MF_HMIH.ParkingSituationSides left     // @unit{0};Situation on left side of parking vehicle
    type.MF_HMIH.ParkingSituationSides right     // @unit{0};Situation on right side of parking vehicle
    type.MF_HMIH.ParkingSituationFrontRear front     // @unit{0};Situation in front of parking vehicle
    type.MF_HMIH.ParkingSituationFrontRear rear     // @unit{0};Situation at back of parking vehicle
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.PDCSectors uml:Class
    // Information for the PDC sectors
  Members:
    type.MF_HMIH.PDCSectorInfo left     // @unit{0};Information for the PDC sectors from the left side (numbering is done from front towards rear)
    type.MF_HMIH.PDCSectorInfo right     // @unit{0};Information for the PDC sectors from the right side (numbering is done from front towards rear)
    type.MF_HMIH.PDCSectorInfo front     // @unit{0};Information for the PDC sectors from the front (numbering is done from left side towards right side)
    type.MF_HMIH.PDCSectorInfo rear     // @unit{0};Information for the PDC sectors from the rear (numbering is done from left side towards right side)
    type.float32 PDC_P_SECTOR_INNER_COORDS_X_M     // The x coordinates of the inner sectors contour
    type.float32 PDC_P_SECTOR_INNER_COORDS_Y_M     // The y coordinates of the inner sectors contour
    type.float32 PDC_P_SECTOR_OUTER_COORDS_X_M     // The x coordinates of the outer sectors contour
    type.float32 PDC_P_SECTOR_OUTER_COORDS_Y_M     // The y coordinates of the outer sectors contour
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.DrivingTube uml:Class
    // Driving tube
  Members:
    type.uint16 frontRadius_cm     // @unit{cm};Radius of the driving tube on the front.  0..65533: valid radius values 65534: infinite (straight driving tube should be displayed) 65535: unknown (driving tube cannot be determined)
    type.uint16 rearRadius_cm     // @unit{cm};Radius of the driving tube on the rear.  0..65533: valid radius values 65534: infinite (straight driving tube should be displayed) 65535: unknown (driving tube cannot be determined)
    type.PDCP.DrvTubeDisplay drvTubeDisplay_nu     // @range{0,2};Side where the driving tube should be displayed
    type.MF_HMIH.DrvTubeDirection drvTubeDirection_nu     // @range{0,1};Driving tube direction
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.WheelWarnings uml:Class
    // Wheel warning information coming from WHP
  Members:
    type.MF_WhlProtectProc.WhlWarningLevel warningLevel_nu     // @range{0,3};Wheel warning level for each wheel. Positions front left: 0, front right: 1, rear left: 2; rear right: 3.
    type.uint8 whlAngleAbs_deg     // @unit{deg};Absolute value of wheel angles for each wheel. Positions front left: 0, front right: 1, rear left: 2; rear right: 3.
    type.MF_HMIH.WheelAngDirection whlAngDirection_nu     // @range{0,1};Direction of the wheel angle for each wheel. Positions front left: 0, front right: 1, rear left: 2; rear right: 3.
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.DayTime uml:Enumeration
    // Possible values of the DayTimeDefinition class
    // @range{0,2}
  Members:
    DT_DAY = 0            
    DT_NIGHT = 1            
    DT_TWILIGHT = 2            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.GeneralWarnings uml:Class
    // General warning and attention information which can be displayed on the HMI.
  Members:
    type.MF_HMIH.DayTime visualDaytimeEstimation_nu     // @range{0,2};Estimated daytime recognition. Can be used to inform the driver about low-light situation.
    type.boolean narrowParkingSpaceWarning_nu     // Parking situation is a narrow slot. Driver should consider to use remote parking.
    type.boolean parkingSpaceObstaclesAreMoving_nu     // Vehicles next to the parking in slot are non stationairy. Driver attention should be raised.
    type.boolean visualSensorSystemWarning_nu     // Warn the driver that the camera system might be visually impaired.;Shall be based on the information from PerceptionAvailabilityPort and similar sources.
    type.boolean ultrasoundSensorSystemWarning_nu     // Warn the driver that the USS system might be impaired.;Shall be based on the information from PerceptionAvailabilityPort and similar sources.
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.LscaWarnings uml:Class
    // LSCA warning information
  Members:
    type.MF_LSCA.LSCA_WARNING_STATUS warningBody_nu     // Indicate on what part of the ego body will the collision be.
    type.MF_LSCA.LSCA_WARNING_STATUS warningWheel_nu     // Indicate what wheel is affected by a collision with a low object.
    type.MF_LSCA.LSCA_WARNING_STATUS warningObject_nu     // Indicate where is the object relative to the ego vehicle.
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.ParkingTargetPose uml:Class
    // Target pose information to be displayed by Visu
  Members:
    type.uint8 id_nu     // ID of target pose
    type.boolean isFree_nu     // Parking slot free
    type.boolean isScanned_nu     // Parking slot was scanned
    type.boolean isSelected_nu     // Parking slot selected
    type.boolean isSwitchable_nu     // Parking slot switchable
    type.LSM_GEOML.Pose_POD pose_nu     // A pose defined by an x and y coordinate as well as a yaw angle
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.ParkingTargetPoses uml:Class
    // Target poses information for Visu
  Members:
    type.uint8 numValidParkingPoses_nu     // Number of poses that are valid (does not need to be reachable)
    type.MF_HMIH.ParkingTargetPose parkingPoses     // All information related to a possible target pose
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.VisuHMIData uml:Class
    // HMI data for Visu
  Members:
    type.float32 firstClickEventX_px     // X finger position on HMI Screen display in pixels (13 bits range signal) for main finger.
    type.float32 firstClickEventY_px     // Y finger position on HMI Screen display in pixels (13 bits range signal) for main finger.
    type.float32 secondClickEventX_px     // X finger position on HMI Screen display in pixels (13 bits range signal) for second finger.
    type.float32 secondClickEventY_px     // Y finger position on HMI Screen display in pixels (13 bits range signal) for second finger.
    type.uint8 gestureFinger_nu     // Number of Fingers used on the touch screen.
    type.uint8 gestureCounter_nu     // Wrap-around counter for gesture signal (will be increased with every gesture event).
    type.AP_HMIToAP.GestureCode gestureCode_nu_u8     // Gesture Code identifies which kind of gesture was done by the user.
    type.AP_HMIToAP.BlindSpotView blindSpotViewType_nu     // To enable the blind spot view activation from HMI/IC.
    type.boolean visibilityTags_nu     // Enabling or disabling of overlay. Overlay supported according to ScreenTypes.
    type.boolean isSequence_nu     // Identifies whether a gesture sequence is being performed.
    type.boolean startVideoRecorderReq_nu     // User starts video recording.
    type.boolean startScreenCaptureReq_nu     // User starts screen capture recording.
    type.boolean rimProtectionStatus_nu     // Wheel protection activation from HMI
    type.boolean reverseAssistStatus_nu     // To enable the reverse assist activation from HMI.
    type.AP_HMIToAP.ParkingAugmentationType parkingAugmentationType_nu     // Parking Augmentation status
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.RemoteDeviceInteraction uml:Class
    // Keep information about the status of the remote device.
  Members:
    type.float32 batteryLevel_perc     // @unit{Percent};@range{0,102};Returns battery level of remote device to ensure enough usage time of the remote device for parking function
    type.uint16 fingerPositionX_px     // @unit{Pixel};@range{0,4095};x Finger position on Smartphone display 4095: Finger is not on screen (0xfff)
    type.uint16 fingerPositionY_px     // @unit{Pixel};@range{0,4095};y Finger position on Smartphone display 4095: Finger is not on screen (0xfff)
    type.MF_HMIH.APUserActionRemoteDevice apUserActionRemoteDevice_nu     // @range{0,17};User interaction with HMI handler (Remote Parking App)
    type.uint8 aliveCounter_nu     // @range{0,255};Increases on each cycle by one (to clarify: needed here)
    type.boolean deadMansSwitchBtn_nu     // Dead Man"s Switch pressed (test purpose only)
    type.boolean paired_nu     // true if bluetooth device is paried
    type.boolean connected_nu     // true if bluetooth device is connected
    type.uint16 screenResolutionX_px     // @unit{Pixel};@range{0,16383};resolution screen curent device in X
    type.uint16 screenResolutionY_px     // @unit{Pixel};@range{0,16383};resolution screen curent device in Y
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.HeadUnitInteraction uml:Class
    // User interaction with HMI handler
  Members:
    type.MF_HMIH.APUserActionHeadUnit apUserActionHeadUnit_nu     // @range{0,28};User interaction with HMI handler (Head Unit)
----------------------------------------------------------------------------------------------------------











  # ID:  type.MF_HMIH.HeadUnitVisualizationPort uml:Class
  version: ::mf_hmih::HeadUnitVisualizationPort_InterfaceVersion::HeadUnitVisualizationPort_VERSION
    // Store the current Head Unit screen.
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     // Provide a upconversion for old measurement to newer interfaces(needed as the first field in a port).
    type.eco.SignalHeader sSigHeader     // Common header for all structured data types (e.g. timestamp)
    type.MF_HMIH.ScreenHeadUnit screen_nu     // @range{0,25};HMI screen content depending on system status 0: AP is deactivated or AP is activated but is not performing scanning or maneuvering 10 = Screen, where the user can choose the parking out direction 11 = Menu-Screen of Remote Parking App, where user can choose the wished mode (e.g. "remote parking out" or "remote maneuvering") 12 = Adaption of maneuver active screen (no. 4) when driver does longitudinal control (differences might be only visible in instrument cluster)  13 = Screen, where the user can see the SurroundView-Stream in the Smartphone App (in case of the US-only-Variant: the ParkDistanceView) 14 = Screen for Remote Maneuvering with two arrows, allowing the user to navigate the vehicle in forward or backward direction  15 = [for Remote Maneuvering] Shows a screen with the Dead-Man-Switch / circle during remote maneuvering with an arrow, indicating the current driving direction 16 = Shows the current Garage Parking Situation, based on garageParking_nu
    type.AP_PSM_APP.HMIMessage message_nu     // @range{0,63};Message to show on screen 1: If the vehicle is very close to objects, an additional warning could appear, that the driver has to observe the maneuver very carefully. 2: Shows message that driver is responsible and has to observe the environment. Additionally it shows the "Start Parking" button. 6,7,8,21,22: Specific messages if the longitudinal control has to be done by driver 12: Shows message that driver is responsible and has to observe the environment. 13: Shows a message, that the Remote Maneuvering-Maneuver was stopped because of an obstacle in the driving path 14: Message shows up while using remote app and distance of remote device (key or smartphone) to vehicle is too high 15: If door opens during maneuver vehicle will stop and offer driver start remote parking. In case of remote parking driver will be asked to close the door. 17: If remote parking is active passengers will be asked to leave the vehicle before maneuver starts 18: If in-vehicle parking is active driver will be asked to return to his seat 19: AP system error 21: For manual gear box (driver has to control longitudinal movement) 22: For manual gear box, when driver has to stop the vehicle 23: Shows a warning, when the user has not moved the vehicle for a certain waiting time during a remote controlled AP maneuver. If the user does not react within a further waiting time, the vehicle will stop the engine. 26: Shows  message that diriver has to select the type of parking . Aditionally it shows the  fully "Automated Parking" button and the "Semi-Automated Parking" button 31: Indicates that a detected garage is still closed
    type.boolean apaFeatureActiveHU     // This Signal is used to Eshtablish connection/Disconnection with  HU.
    type.MF_HMIH.HUCurrentFeatureState apaHuCurrentFeaturesState     // This signal indicates current status of the APA features
    type.uint16 apaPreconditionsCheck     // This signal indicates the pre-condition check for APA
    type.MF_HMIH.HuCrossButton apaHuCrossButton     // This signal indicates presence of cross button in different screens
    type.uint8 apaHuParkingIconsGrayOut     // This signal indicates the icons to be displayed on APA screen right strip
    type.uint8 apaHuSVsIconsGrayOut     // This signal indicates the icons to be displayed on SVS screen right strip
    type.MF_HMIH.HuParkInSubFeatureState apaHuParkInSubFeatureState     // This signal indicates current status of the APA Park In sub features.
    type.MF_HMIH.HuParkOutSubFeatureState apaHuParkOutSubFeatureState     // This signal indicates current status of the APA Park Out sub-features
    type.MF_HMIH.HuGarageParkSubFeatureState apaHuGarageParkSubFeatureState     // This signal indicates current status of the garage parking sub-features
    type.MF_HMIH.HuSVsSubFeatureState apaHuSVsSubFeatureState     // This signal indicates current status of the SVS sub-features
    type.MF_HMIH.HuRaSubFeatureState apaHuRaSubFeatureState     // This signal indicates current status of the Reverse Assist sub-features
    type.MF_HMIH.HuBvmSubFeatureState apaHuBvmSubFeatureState     // This signal indicates the current status of the BVM  features
    type.uint8 apaInteractionPauseCount     
    type.uint8 apaInteractionTimer     
    type.MF_HMIH.HUIntrPauseCondition apaIntrPauseCondition     // This signal Indicates the Pause condition messages
    type.MF_HMIH.HuFrameReady apaHuFrameReady     // This signal Indicates the readiness of the frames
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.HeadUnitVisualizationPort_InterfaceVersion uml:Class
  Members:
    type.uint32 HeadUnitVisualizationPort_VERSION = 3    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.RemoteVisualizationPort uml:Class
  version: ::mf_hmih::RemoteVisualizationPort_InterfaceVersion::RemoteVisualizationPort_VERSION
    // Store screen and message which is send to the remote app.
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     // Provide a upconversion for old measurement to newer interfaces(needed as the first field in a port).
    type.eco.SignalHeader sSigHeader     // Common header for all structured data types (e.g. timestamp)
    type.MF_HMIH.ScreenHeadUnit screen_nu     // @range{0,25};HMI screen content depending on system status 0: AP is deactivated or AP is activated but is not performing scanning or maneuvering 10 = Screen, where the user can choose the parking out direction 11 = Menu-Screen of Remote Parking App, where user can choose the wished mode (e.g. "remote parking out" or "remote maneuvering") 12 = Adaption of maneuver active screen (no. 4) when driver does longitudinal control (differences might be only visible in instrument cluster)  13 = Screen, where the user can see the SurroundView-Stream in the Smartphone App (in case of the US-only-Variant: the ParkDistanceView) 14 = Screen for Remote Maneuvering with two arrows, allowing the user to navigate the vehicle in forward or backward direction  15 = [for Remote Maneuvering] Shows a screen with the Dead-Man-Switch / circle during remote maneuvering with an arrow, indicating the current driving direction
    type.AP_PSM_APP.HMIMessage message_nu     // @range{0,58};Message to show on screen 1: If the vehicle is very close to objects, an additional warning could appear, that the driver has to observe the maneuver very carefully. 2: Shows message that driver is responsible and has to observe the environment. Additionally it shows the "Start Parking" button. 6,7,8,21,22: Specific messages if the longitudinal control has to be done by driver 12: Shows message that driver is responsible and has to observe the environment. 13: Shows a message, that the Remote Maneuvering-Maneuver was stopped because of an obstacle in the driving path 14: Message shows up while using remote app and distance of remote device (key or smartphone) to vehicle is too high 15: If door opens during maneuver vehicle will stop and offer driver start remote parking. In case of remote parking driver will be asked to close the door. 17: If remote parking is active passengers will be asked to leave the vehicle before maneuver starts 18: If in-vehicle parking is active driver will be asked to return to his seat 19: AP system error 21: For manual gear box (driver has to control longitudinal movement) 22: For manual gear box, when driver has to stop the vehicle 23: Shows a warning, when the user has not moved the vehicle for a certain waiting time during a remote controlled AP maneuver. If the user does not react within a further waiting time, the vehicle will stop the engine.
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.RemoteVisualizationPort_InterfaceVersion uml:Class
  Members:
    type.uint32 RemoteVisualizationPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.MpSlotList uml:Class
    // 0
  Members:
    type.uint8 slotId     // unique identifier for slot
    type.float32 roadWidth     // to limit the maneuvering space towards the road
    type.boolean relocalizationStatus     // signal to indicate relocalization status
    type.boolean gpsSilentOfferingEnabled     // signal to indicate automatic relocalization is enabled
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.MemoryParkingInfo uml:Class
    // List of memorized slots
  Members:
    type.MF_HMIH.MpSlotList memorySlots     
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.HMIGeneralInputPort uml:Class
  version: ::mf_hmih::HMIGeneralInputPort_InterfaceVersion::HMIGeneralInputPort_VERSION
    // Store all information which is send to the HMI.
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     // Provide a upconversion for old measurement to newer interfaces(needed as the first field in a port).
    type.eco.SignalHeader sSigHeader     // Common header for all structured data types (e.g. timestamp)
    type.MF_HMIH.HMIGeneral general     // @unit{0};Major screen control of HMI.
    type.MF_HMIH.ParkingSpaces parkingSpaces     // @unit{0};Status of the parking spaces
    type.MF_HMIH.ParkingSituation parkingSituation     // @unit{0};Parking space situation around the vehicle
    type.MF_HMIH.PDCSectors pdcSectors     // @unit{0};Information for the PDC sectors
    type.MF_HMIH.DrivingTube drivingTube     // @unit{0};Driving tube
    type.MF_HMIH.GeneralWarnings generalWarnings     // @unit{0};General warnings from the driving function system.
    type.MF_HMIH.WheelWarnings wheelWarnings     // @unit{0};Wheel warning information coming from WHP
    type.MF_HMIH.LscaWarnings lscaWarnings     // @unit{0};LSCA warning information
    type.MF_HMIH.MemoryParkingInfo memoryParkingInfo     
    type.AP_TP.ReverseAssistAvailabilityPort reverseAssistAvailabilityPort_nu     // Availability of the reverse assist function and information about the saved reverse path.
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.HMIGeneralInputPort_InterfaceVersion uml:Class
  Members:
    type.uint32 HMIGeneralInputPort_VERSION = 8    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.APUserInteractionPort uml:Class
  version: ::mf_hmih::APUserInteractionPort_InterfaceVersion::APUserInteractionPort_VERSION
    // Store information about user action.
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     // Provide a upconversion for old measurement to newer interfaces(needed as the first field in a port).
    type.eco.SignalHeader sSigHeader     // Common header for all structured data types (e.g. timestamp)
    type.MF_HMIH.RemoteDeviceInteraction remoteDeviceInteraction     // @unit{nu};Keep information about the status of the remote device.
    type.MF_HMIH.HeadUnitInteraction headUnitInteraction     // @unit{nu};User interaction with HMI handler
    type.uint8 selectedTPID_nu     // @range{0,255};Unique identifier of selected target pose. If no pose is selected, ID is 255.
    type.uint8 selectedMemorySlotID_nu     // @range{0,255};Unique identifier of selected memory slot
    type.boolean saveRequest_nu     // save request from user
    type.boolean deleteRequest_nu     // delete request from user
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.APUserInteractionPort_InterfaceVersion uml:Class
  Members:
    type.uint32 APUserInteractionPort_VERSION = 2    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.PDCUserInteractionPort uml:Class
  version: ::mf_hmih::PDCUserInteractionPort_InterfaceVersion::PDCUserInteractionPort_VERSION
    // User interaction with HMI hander regarding PDW and WHP.
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     // Provide a upconversion for old measurement to newer interfaces(needed as the first field in a port).
    type.eco.SignalHeader sSigHeader     // Common header for all structured data types (e.g. timestamp)
    type.MF_HMIH.PDCUserActionHeadUnit pdcUserActionHeadUnit_nu     // @range{0,2};User interaction with HMI handler (Head Unit) regarding PDW
    type.MF_HMIH.WHPUserActionHeadUnit whpUserActionHeadUnit_nu     // @range{0,2};User interaction with HMI handler (Head Unit) regarding WHP
    type.boolean pdwAutoActivate_nu     // True if PDW should auto activate.
    type.boolean lscaAutoActivate_nu     // True if LSCA should auto activate.
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.PDCUserInteractionPort_InterfaceVersion uml:Class
  Members:
    type.uint32 PDCUserInteractionPort_VERSION = 2    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.LVMDUserInteractionPort uml:Class
  version: ::mf_hmih::LVMDUserInteractionPort_InterfaceVersion::LVMDUserInteractionPort_VERSION
    // Driver input signals for the Lead Vehicle Movement Detection function.
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     // Provide a upconversion for old measurement to newer interfaces(needed as the first field in a port).
    type.eco.SignalHeader sSigHeader     // Common header for all structured data types (e.g. timestamp)
    type.MF_HMIH.LVMDUserActionHeadUnit lvmdUserActionHeadUnit_nu     // @range{0,1};User interaction with HMI handler (Head Unit) regarding LVMD.
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.LVMDUserInteractionPort_InterfaceVersion uml:Class
  Members:
    type.uint32 LVMDUserInteractionPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.UserDefinedSlotPort uml:Class
  version: ::mf_hmih::UserDefinedSlotPort_InterfaceVersion::UserDefinedSlotPort_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     // Provide a upconversion for old measurement to newer interfaces(needed as the first field in a port).
    type.eco.SignalHeader sSigHeader     // Common header for all structured data types (e.g. timestamp)
    type.LSM_GEOML.Pose_POD pose     
    type.MF_HMIH.UserDefinedSlotSide slotSide_nu     // Selected side for the user defined slot
    type.MF_HMIH.UserDefinedSlotType slotType_nu     // Selected type for the user defined slot
    type.boolean userDefined_nu     // Indicates whether we are in the case that the user defined a slot via HMI and the port contains the corresponding information
    type.boolean valid     // Indicates that the data in the memory slot selected by user is valid.
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.UserDefinedSlotPort_InterfaceVersion uml:Class
  Members:
    type.uint32 UserDefinedSlotPort_VERSION = 2    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.VisuInputPort uml:Class
  version: ::mf_hmih::VisuInputPort_InterfaceVersion::VisuInputPort_VERSION
    // Keep information for Visu: e.g.trajectory planner, target poses, distance to stop.
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     // Provide a upconversion for old measurement to newer interfaces(needed as the first field in a port).
    type.eco.SignalHeader sSigHeader     // Common header for all structured data types (e.g. timestamp)
    type.float32 distanceToStopReq_m     // Value for the remaining distance to stop the vehicle.
    type.MF_HMIH.VisuHMIData visuHMIData     // HMI data for Visu
    type.AP_HMIToAP.ScreenTypes HmiOutUserActScreenReq_u8     // Description which stream should be renderd
    type.AP_TP.TrajPlanVisuPort trajPlanVisuPort_nu     // Trajectory planner data for Visu
    type.MF_HMIH.ParkingTargetPoses parkingPosesVisu     // Target poses information for Visu
    type.boolean driverSelection_nu     // Indicates if the slot was selected by the user or if the selected slot was preselected by HMIHandler.
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.VisuInputPort_InterfaceVersion uml:Class
  Members:
    type.uint32 VisuInputPort_VERSION = 7    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.SurroundViewRequestPort uml:Class
  version: ::mf_hmih::SurroundViewRequestPort_InterfaceVersion::SurroundViewRequestPort_VERSION
    // Store information about the type of the requested stream.
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     // Provide a upconversion for old measurement to newer interfaces(needed as the first field in a port).
    type.eco.SignalHeader sSigHeader     // Common header for all structured data types (e.g. timestamp)
    type.AP_HMIToAP.ScreenTypes streamRequest_nu     // Description which stream should be renderd
    type.boolean driverSelection_nu     // Indicates if the slot was selected by the user or if the selected slot was preselected by HMIHandler.
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.SurroundViewRequestPort_InterfaceVersion uml:Class
  Members:
    type.uint32 SurroundViewRequestPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.MFHmiHDebugPort uml:Class
  version: ::mf_hmih::MFHmiHDebugPort_InterfaceVersion::MFHmiHDebugPort_VERSION
    // Debug port for HMIH.
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     // Provide a upconversion for old measurement to newer interfaces(needed as the first field in a port).
    type.eco.SignalHeader sSigHeader     // Common header for all structured data types (e.g. timestamp)
    type.sint32 debugInt     // @unit{nu};freespace for MTS debug values
    type.float32 debugFloat     // @unit{nu};freespace for MTS debug values
    type.MF_HMIH.HMIState stateVarHMI_nu     // State of the  hmi communication state machine
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.MFHmiHDebugPort_InterfaceVersion uml:Class
  Members:
    type.uint32 MFHmiHDebugPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.FC_MFHMIH_Params uml:Class
  version: ::mf_hmih::FC_MFHMIH_Params_InterfaceVersion::FC_MFHMIH_Params_VERSION
    // HMIH parameters
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 AP_H_OBSTACLE_MIN_PAR_HMI_M     // Parallel parking: Minimum dist between two parking slots in order to visualize an obstacle in HMI.
    type.float32 AP_H_OBSTACLE_MAX_PAR_HMI_M     // Parallel parking: Maximum dist between two parking slots in order to visualize one obstacle in HMI.
    type.float32 AP_H_OBSTACLE_MIN_PERP_HMI_M     // Perpendicular parking: Minimum dist between two parking slots in order to visualize an obstacle in HMI.
    type.float32 AP_H_OBSTACLE_MAX_PERP_HMI_M     // Perpendicular parking: Maximum dist between two parking slots in order to visualize one obstacle in HMI.
    type.float32 AP_H_FWDBWD_SLOT_DIST_MAX_HMI_M     // Distance threshold between to parking slots. If the distance is smaller than the threshold the two slots;are seen as the same in the HMI. This param is used to assign a perpendicular fwd and perpendicular bwd slot to the same position
    type.float32 AP_H_FIRST_SLOT_HMI_OFFSET_M     // Offest added to the distance between the first parking slot showen in HMI to the ego vehicle position
    type.float32 AP_H_PERPAR_SLOT_DIST_MAX_HMI_M     // Distance threshold between to parking slots. If the distance is smaller than the threshold the two slots are seen as the same in the HMI.;This param is used to assign a perpendicular and a parallel slot to the same position
    type.float32 AP_H_USER_DEF_INIT_LAT_OFFSET_M     // Lateral offset from the ego vehicle to the initial slot proposed for user defined slot feature
    type.float32 AP_H_USER_DEF_SLIDE_OFFSET_M     // Lateral / longitudinal offset applied at each step of sliding the user defined slot
    type.float32 AP_H_USER_DEF_ANG_OFFSET_RAD     // Angle offset applied at each step of rotating the user defined slot
    type.float32 AP_H_USER_DEF_MAX_ANG_PAR_RAD     // Angle threshold for user defined slot when selected type is parallel
    type.float32 AP_H_USER_DEF_MAX_ANG_PERP_RAD     // Angle threshold for user defined slot when selected type is perpendicular
    type.boolean AP_H_USER_DEF_ENABLED_NU     // Indicates if the user defined feature is enabled.
    type.boolean AP_H_CUSTOMER_DEMO_NU     // Indicates if the HMIH is configured for a customer demo.
    type.boolean AP_H_EXPLICIT_SCAN_ENABLED_NU     // Indicates if the explicit scanning feature is enabled (only for Entry ART for now).
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.FC_MFHMIH_Params_InterfaceVersion uml:Class
  Members:
    type.uint32 FC_MFHMIH_Params_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.HUCurrentFeatureState uml:Enumeration
    // Current status of the APA features
  Members:
    NOT_USED = 0            
    NO_FEATURE_ACTIVE = 1            
    PRE_CONDITION_NOT_MET = 2            
    PARK_IN = 3            
    PARK_OUT = 4            
    GARAGE_PARK_IN = 5            
    SVS = 6            
    REVERSE_ASSIST = 7            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.HuCrossButton uml:Enumeration
    // This signal indicates presence of cross button in different screens
  Members:
    NOT_USED = 0            
    NO_CROSS_BUTTON = 1            
    CROSS_BUTTON_PRESENT = 2            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.HuParkInSubFeatureState uml:Enumeration
    // current status of the park in  features
  Members:
    UNUSED = 0            
    PARK_IN_ACTIVE = 1            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.HuParkOutSubFeatureState uml:Enumeration
    // current status of the park out  features
  Members:
    UNUSED = 0            
    PARK_OUT_ACTIVE = 1            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.HuGarageParkSubFeatureState uml:Enumeration
    // current status of the Garage parking  features
  Members:
    UNUSED = 0            
    GARAGE_PARKING = 1            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.HuSVsSubFeatureState uml:Enumeration
    // current status of the SVS  features
  Members:
    UNUSED = 0            
    SVS_2D_ACTIVE = 1            
    SVS_3D_ACTIVE = 2            
    SVS_FRONT_CORNER_ACTIVE = 3            
    SVS_REAR_CORNER_ACTIVE = 4            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.HuRaSubFeatureState uml:Enumeration
    // current status of the RA  features
  Members:
    UNUSED = 0            
    REVERSE_ASSIST_ACTIVE = 1            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.HuBvmSubFeatureState uml:Enumeration
    // current status of the BVM  features
  Members:
    UNUSED = 0            
    BVM_IN_IVI_LEFT_ACTIVE = 1            
    BVM_IN_IVI_RIGHT_ACTIVE = 2            
    BVM_IN_IC_LEFT_ACTIVE = 3            
    BVM_IN_IC_RIGHT_ACTIVE = 4            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.HUIntrPauseCondition uml:Enumeration
    // This signal Indicates the Pause condition messages
  Members:
    NO_MSG = 0            
    DRIVER_DOOR_NOT_CLOSED = 1            
    CO_DRIVER_DOOR_NOT_CLOSED = 2            
    DYN_STAT_OBJ_TRAJ = 3            
    SEAT_BELT_OPEN = 4            
    ORVM_CLOSE = 5            
    BRAKE_PRESS_TH = 6            
    APA_HOLD_RELEASED = 7            
    REAR_PSG_DOOR_NOT_CLOSED = 8            
    TRUNK_LID_NOT_CLOSED = 9            
    TAIL_GATE_NOT_CLOSED = 10            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_HMIH.HuFrameReady uml:Enumeration
    // Indicates the readiness of the frames
  Members:
    NO_USED = 0            
    IN_TRANSITION = 1            
    FRAME_ERROR = 2            
    EARLY_RVC_READY = 3            
    APA_READY = 4            
    PARK_IN_READY = 5            
    PARK_OUT_READY = 6            
    GARAGE_PARKING_READY = 7            
    REVERSE_ASSIST_READY = 8            
    RCTA_READY = 9            
    BVM_READY = 10            
    TWO_D_VIEW_READY = 11            
    THREE_D_VIEW_READY = 12            
    FRONT_CORNER_VIEW_READY = 13            
    REAR_CORNER_VIEW_READY = 14            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_DRVWARNSM_CORE.MF_DRVWARNSM_CORE_Consts uml:Class
  Members:
    type.uint8 NUM_MTS_DEBUG_FREESPACE_CORE_DWF = 10    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_DRVWARNSM_CORE.DWFCoreState uml:Enumeration
    // Internal state of the PDW core state machine
  Members:
    DWF_CORE_INIT = 0            
    DWF_CORE_OFF = 1            
    DWF_CORE_INACTIVE = 2            
    DWF_CORE_ACTIVE = 3            
    DWF_CORE_FAILURE = 4            
    DWF_CORE_NUM_STATES = 5            
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_DRVWARNSM_CORE.DrvWarnCoreDebugPort uml:Class
  version: ::mf_drvwarnsm_core::DrvWarnCoreDebugPort_InterfaceVersion::DrvWarnCoreDebugPort_VERSION
    // Driver Warning Core state machines internal signals for debug
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.sint32 debugInt     // freespace for MTS debug values
    type.float32 debugFloat     // freespace for MTS debug values
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_DRVWARNSM_CORE.DrvWarnCoreDebugPort_InterfaceVersion uml:Class
  Members:
    type.uint32 DrvWarnCoreDebugPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_DRVWARNSM_CORE.DrvWarnCoreStatusPort uml:Class
  version: ::mf_drvwarnsm_core::DrvWarnCoreStatusPort_InterfaceVersion::DrvWarnCoreStatusPort_VERSION
    // Main output of Driver Warning State Machines Core (status information)
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.boolean pdwActiveState_nu     // PDW core activation state flag
    type.MF_DRVWARNSM_CORE.DWFCoreState pdwCoreState_nu     // @range{0,5};Internal state of the PDW core state machine
    type.MF_DRVWARNSM_CORE.DWFCoreState whpCoreState_nu     // @range{0,5};Internal state of the WHP core state machine
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_DRVWARNSM_CORE.DrvWarnCoreStatusPort_InterfaceVersion uml:Class
  Members:
    type.uint32 DrvWarnCoreStatusPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_DRVWARNSM_CORE.FC_DrvWarnSMCore_Params uml:Class
  version: ::mf_drvwarnsm_core::FC_DrvWarnSMCore_Params_InterfaceVersion::FC_DrvWarnSMCore_Params_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.float32 DWF_C_CRUISING_SPEED_OFF_MPS     // Maximum allowed cruising speed. Below this threshold  the processing component does the calculations
    type.boolean DWF_C_WHP_ENABLED_NU     // Coding parameter to enable WHP function
    type.boolean DWF_C_PDW_ENABLED_NU     // Coding parameter to enable PDW function
    type.float32 DWF_C_SPEED_HYSTERESIS_MPS     // The hysteresis to be applied to the speed threshold when switching to off state
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_DRVWARNSM_CORE.FC_DrvWarnSMCore_Params_InterfaceVersion uml:Class
  Members:
    type.uint32 FC_DrvWarnSMCore_Params_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_TONH.MF_TONH_Consts uml:Class
    // TONH consts
  Members:
    type.uint8 NUM_SPEAKERS = 2    
    type.uint8 NUM_MTS_DEBUG_FREESPACE_TONH = 10    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_TONH.SpeakerOutput uml:Class
    // Sound description for each speaker
  Members:
    type.uint8 pitch_nu     // @range{0,255};Sound pitch
    type.uint8 volume_nu     // @range{0,7};Sound volume (0 = silence)
    type.boolean soundOn_nu     // Sound output on/off
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_TONH.ToneHandlerDebugPort uml:Class
  version: ::mf_tonh::ToneHandlerDebugPort_InterfaceVersion::ToneHandlerDebugPort_VERSION
    // Tone handler internal signals for debug
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     // Provide a upconversion for old measurement to newer interfaces(needed as the first field in a port).
    type.eco.SignalHeader sSigHeader     // Common header for all structured data types (e.g. timestamp)
    type.sint32 debugInt     // freespace for MTS debug values
    type.float32 debugFloat     // freespace for MTS debug values
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_TONH.ToneHandlerDebugPort_InterfaceVersion uml:Class
  Members:
    type.uint32 ToneHandlerDebugPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_TONH.ToneOutputPort uml:Class
  version: ::mf_tonh::ToneOutputPort_InterfaceVersion::ToneOutputPort_VERSION
    // Port describing the tone output from tone handler
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     // Provide a upconversion for old measurement to newer interfaces(needed as the first field in a port).
    type.eco.SignalHeader sSigHeader     // Common header for all structured data types (e.g. timestamp)
    type.MF_TONH.SpeakerOutput speakerOutput     // Sound description for each speaker
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_TONH.ToneOutputPort_InterfaceVersion uml:Class
  Members:
    type.uint32 ToneOutputPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_TONH.FC_MF_ToneHandler_Params uml:Class
  version: ::mf_tonh::FC_MF_ToneHandler_Params_InterfaceVersion::FC_MF_ToneHandler_Params_VERSION
    // Tone handler parameters
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     // Common header for all structured data types (e.g. timestamp)
    type.float32 TH_ACK_TONE_DELAY_S     // Delay for acknowledgement tone
    type.float32 TH_ACK_TONE_DURATION_S     // Duration for acknowledgement tone
    type.float32 TH_ERR_TONE_DURATION_S     // Duration for error tone
    type.float32 TH_SOUND_PULSE_LEN_S     // Duration of sound ON for warning tone
    type.float32 TH_LVMD_SOUND_PULSE_LEN_S     // Duration of sound ON for warning tone - LVMD only
    type.float32 TH_CONT_TONE_LIMIT_M     // Distance to determine the continuous tone limit
    type.float32 TH_PING_PONG_DURATION_S     // Duration of ping pong sound
    type.float32 TH_MIN_PAUSE_DURATION_S     // Minimum duration of a pause
    type.float32 TH_MAX_PAUSE_DURATION_S     // Maximum duration of a pause
    type.float32 TH_ACK_PAUSE_DURATION_S     // Duration of acknowledgement pause
    type.float32 TH_WHP_LOW_PAUSE_DURATION_S     // Duration of pause when WHP has a low criticality
    type.float32 TH_WHP_HIGH_PAUSE_DURATION_S     // Duration of pause when WHP has a high criticality
    type.float32 TH_LSCA_PAUSE_DURATION_S     // Duration of pause for LSCA
    type.float32 TH_ACTIVE_DISTANCE_M     // Maximum distance for object to be reported
    type.uint8 TH_VOLUME_FRONT_NU     // Front volume level
    type.uint8 TH_VOLUME_REAR_NU     // Rear volume level
    type.uint8 TH_ACK_TONE_PITCH_NU     // Pitch for acknowledgement tone
    type.uint8 TH_ACK_TONE_VOL_OFFSET_NU     // Volume offset to the reference volume for acknowledgement tone
    type.uint8 TH_ERR_TONE_VOL_OFFSET_NU     // Volume offset to the reference volume for error tone
    type.uint8 TH_ERR_TONE_PITCH_NU     // Pitch for error tone
    type.uint8 TH_WARN_TONE_PITCH_NU     // Pitch for warning tone - general value
    type.uint8 TH_WHP_WARN_TONE_PITCH_NU     // Pitch for warning tone - WHP only;Pitch for warning tone - LVMD only
    type.uint8 TH_LSCA_LOW_TONE_PITCH_NU     // Pitch for low warning tone - LSCA only
    type.uint8 TH_LSCA_HIGH_TONE_PITCH_NU     
    type.uint8 TH_LVMD_TONE_PITCH_NU     // Pitch for high warning tone - LSCA only
    type.uint8 TH_WEAK_VOLUME_NU     // Rear volume level
    type.boolean TH_ENABLED_NU     // Parameter to enable/disable the tone handler entirely
    type.boolean TH_DIFFERENT_TONES_NU     // Parameter to enable/disable usage of different pitch for different plugins
    type.boolean TH_ACK_ACTIVE_NU     // Parameter to enalbe/disable the acknowledge sounds for PDW and WHP.
----------------------------------------------------------------------------------------------------------

  # ID:  type.MF_TONH.FC_MF_ToneHandler_Params_InterfaceVersion uml:Class
  Members:
    type.uint32 FC_MF_ToneHandler_Params_VERSION = 2    
----------------------------------------------------------------------------------------------------------

  # ID:  type.vc.OverlayTag uml:Enumeration
  Members:
    OVERLAY_TAG_UNDEFINED = 0            
    OVERLAY_TAG_DISTANCE_BAR_LEFT = 1            
    OVERLAY_TAG_DISTANCE_BAR_RIGHT = 2            
    OVERLAY_TAG_CARPET = 3            
    OVERLAY_TAG_FRONT_TRAJECTORY = 4            
    OVERLAY_TAG_TRAILER_TARGET = 5            
    OVERLAY_TAG_REAR_TRAJECTORY = 6            
    OVERLAY_TAG_HITCH_TRAJECTORY = 7            
    OVERLAY_TAG_TAILGATE = 8            
    OVERLAY_TAG_DETECTED_PARK_SLOT = 9            
    OVERLAY_TAG_GLOW_FRONT_LEFT = 10            
    OVERLAY_TAG_GLOW_FRONT_RIGHT = 11            
    OVERLAY_TAG_GLOW_REAR_LEFT = 12            
    OVERLAY_TAG_GLOW_REAR_RIGHT = 13            
    OVERLAY_TAG_CAM_LEFT_N_A = 14            
    OVERLAY_TAG_CAM_RIGHT_N_A = 15            
    OVERLAY_TAG_CAM_FRONT_N_A = 16            
    OVERLAY_TAG_CAM_REAR_N_A = 17            
    OVERLAY_TAG_CAM_LEFT_DEFECT = 18            
    OVERLAY_TAG_CAM_RIGHT_DEFECT = 19            
    OVERLAY_TAG_CAM_FRONT_DEFECT = 20            
    OVERLAY_TAG_CAM_REAR_DEFECT = 21            
    OVERLAY_TAG_DOOR_LEFT = 22            
    OVERLAY_TAG_DOOR_RIGHT = 23            
    OVERLAY_TAG_MIRROR_LEFT = 24            
    OVERLAY_TAG_MIRROR_RIGHT = 25            
    OVERLAY_TAG_PARK_MARKER_DETECTION = 26            
    OVERLAY_TAG_TRUNK = 27            
    OVERLAY_TAG_EARLY_BITMAP = 28            
    OVERLAY_TAG_POLYGON = 29            
    OVERLAY_TAG_PLANNED_TRAJECTORY = 30            
    OVERLAY_TAG_AUP_FRONT_TRAJECTORY = 31            
    OVERLAY_TAG_AUP_REAR_TRAJECTORY = 32            
    OVERLAY_TAG_CLIPPING_MASK = 33            
    OVERLAY_TAG_PDW = 34            
    OVERLAY_TAG_GHOST_CAR = 35            
    OVERLAY_TAG_REVERSE_ASSIST_ICON = 36            
    OVERLAY_TAG_REVERSE_ASSIST_DISTANCE_MARKER = 37            
    OVERLAY_TAG_RIM_WARNING = 38            
    NUM_OVERLAY_TAGS = 39            
----------------------------------------------------------------------------------------------------------

  # ID:  type.vc.SelectedTargetPose uml:Class
    // This struct will be provided by visu component actually. But because visu is
    // not yet a GitHub component, we made a compromise and declared it here.
    // The port will be consumed by HMI Handler.
  Members:
    type.uint8 selected_pose_id     
    type.boolean switch_pose_orientation     
----------------------------------------------------------------------------------------------------------

  # ID:  type.vc.BlindSpotViewStatus uml:Enumeration
  Members:
    LEFT_BLIND_SPOT_VIEW = 0            
    RIGHT_BLIND_SPOT_VIEW = 1            
    BLIND_SPOT_VIEW_OFF = 2            
----------------------------------------------------------------------------------------------------------

  # ID:  type.vc.TransparencyPreset uml:Enumeration
  Members:
    OPAQUE = 0            
    TRANSPARENT = 1            
    WHEELS_FRAME = 2            
----------------------------------------------------------------------------------------------------------

  # ID:  type.vc.ErrorCode uml:Enumeration
  Members:
    VIS_ERRORCODE_OK = 0            
    VIS_ERRORCODE_FAILURE = 1            
    VIS_ERRORCODE_OVERLAY_FAILED = 2            
    VIS_ERRORCODE_VIDEO_FAILED = 3            
    VIS_ERRORCODE_INVALID_VIEW = 4            
    VIS_ERRORCODE_NOTACTIVATED = 5            
    VIS_ERRORCODE_DEACTIVATED = 6            
    VIS_ERRORCODE_COUNT = 7            
    VIS_ERRORCODE_MAX = 256            
----------------------------------------------------------------------------------------------------------

  # ID:  type.vc.HMIWarningMessage uml:Enumeration
  Members:
    COLLISION_WARNING = 0            
    CAMERA_UNAVAILABLE = 1            
    CAMERA_CLEANING_REQUIRED = 2            
    DOOR_OPEN = 3            
    TRUNK_OPEN = 4            
    ORVM_FOLDED = 5            
    BSV_NOT_ACTIVE_HAZARD_LIGHT_ON = 6            
    CAMERA_NOT_CALIBRATED = 7            
    PLEASE_DRIVE_CAREFULLY = 8            
    PART_OF_IMAGE_IS_NOT_LIVE = 9            
    SPEED_WARNING = 10            
    TRAILER_CONNECTED = 11            
    NO_WARNING = 12            
----------------------------------------------------------------------------------------------------------

  # ID:  type.vc.HMIInstructionSet uml:Enumeration
  Members:
    PLEASE_DRIVE_FORWARD = 0            
    PLEASE_DRIVE_BACKWARD = 1            
    SWIPE_OR_DRAG_TO_VISUALIZE_360_VIEW = 2            
    SELECT_PRESET_OF_YOUR_CHOICE = 3            
    NO_INSTRUCTION = 4            
----------------------------------------------------------------------------------------------------------

  # ID:  type.vc.ComSignalState uml:Enumeration
  Members:
    COMSIGSTATE_INIT = 0            
    COMSIGSTATE_VALID = 1            
    COMSIGSTATE_INVALID = 2            
    COMSIGSTATE_TIMEOUT = 3            
    COMSIGSTATE_ERROR = 0            
----------------------------------------------------------------------------------------------------------

  # ID:  type.vc.UserEventDataPort uml:Class
  version: ::vc::UserEventDataPort_InterfaceVersion::UserEventDataPort_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint16 deltaPolarAngle_u16     
    type.uint16 deltaAzimuthAngle_u16     
    type.uint8 GestureFinger_nu_u8     
    type.uint8 gestureCounter     
    type.uint16 deltaZoom     
    type.boolean isSequence     
    type.float32 clickEventX_px     
    type.float32 clickEventY_px     
----------------------------------------------------------------------------------------------------------

  # ID:  type.vc.UserEventDataPort_InterfaceVersion uml:Class
  Members:
    type.uint32 UserEventDataPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.vc.ScreenSwitchDataPort uml:Class
  version: ::vc::ScreenSwitchDataPort_InterfaceVersion::ScreenSwitchDataPort_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.AP_HMIToAP.ScreenTypes HmiOutUserActScreenReq_u8     
    type.vc.BlindSpotViewStatus blindSpotViewType     
    type.AP_HMIToAP.ScreenTypes currentViewMode     
    type.uint8 ClusterScreenResponse_nu_u8     
    type.boolean deactivateView     
    type.vc.TransparencyPreset transparencyPreset     
----------------------------------------------------------------------------------------------------------

  # ID:  type.vc.ScreenSwitchDataPort_InterfaceVersion uml:Class
  Members:
    type.uint32 ScreenSwitchDataPort_VERSION = 2    
----------------------------------------------------------------------------------------------------------

  # ID:  type.vc.OverlayRequestDataPort uml:Class
  version: ::vc::OverlayRequestDataPort_InterfaceVersion::OverlayRequestDataPort_VERSION
    // Requested overlay by name according to hmi_overlay_config.json
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.boolean visibilityTags     
    type.MF_HMIH.ParkingTargetPoses parkingPosesVC     
    type.vc.SelectedTargetPose selectedParkingPose     
----------------------------------------------------------------------------------------------------------

  # ID:  type.vc.OverlayRequestDataPort_InterfaceVersion uml:Class
  Members:
    type.uint32 OverlayRequestDataPort_VERSION = 2    
----------------------------------------------------------------------------------------------------------

  # ID:  type.vc.ErrorCodeDataPort uml:Class
  version: ::vc::ErrorCodeDataPort_InterfaceVersion::ErrorCodeDataPort_VERSION
    // Error code has been displayed on HMI
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.vc.ErrorCode errorCode     
    type.vc.HMIWarningMessage hmiWarningMessage     
    type.vc.HMIInstructionSet hmiInstructionSet     
----------------------------------------------------------------------------------------------------------

  # ID:  type.vc.ErrorCodeDataPort_InterfaceVersion uml:Class
  Members:
    type.uint32 ErrorCodeDataPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.US_DRV_Consts uml:Class
  Members:
    type.uint8 US_DRV_MAX_NUM_ASIC_COMMAND_WORDS = 64    
    type.uint8 US_DRV_MAX_NUM_SENSORS = 18    
    type.uint8 US_DRV_MAX_NUM_STOCHASTIC_CODES = 8    
    type.uint16 US_DRV_MAX_NUM_DETECTIONS = 1800    
    type.uint8 US_DRV_MAX_NUM_SIGNAL_PATHS = 32    
    type.uint32 US_DRV_MAX_NUM_SAMPLES = 4096    
    type.uint8 US_DRV_MAX_NUM_ASICS = 2    
    type.uint8 US_DRV_MAX_NUM_DSI_CHANNELS = 2    
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvCRMData uml:Class
  Members:
    type.uint8 commandStatus     
    type.uint8 physicalAddress     
    type.uint8 extendedData     
    type.uint8 registerData     
    type.uint8 crc     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvCRMInput uml:Class
  version: ::us_drv::UsDrvCRMInput_InterfaceVersion::UsDrvCRMInput_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint8 sequence     
    type.uint8 flags     
    type.uint16 busBitmask     
    type.US_DRV.UsDrvCRMData crmCommand     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvCRMInput_InterfaceVersion uml:Class
  Members:
    type.uint32 UsDrvCRMInput_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvCRMOutput uml:Class
  version: ::us_drv::UsDrvCRMOutput_InterfaceVersion::UsDrvCRMOutput_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint8 sequence     
    type.uint8 flags     
    type.uint8 bus     // DSI3 bus this CRM response was received on
    type.US_DRV.UsDrvCRMData crmResponse     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvCRMOutput_InterfaceVersion uml:Class
  Members:
    type.uint32 UsDrvCRMOutput_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvAsicCmd uml:Class
  version: ::us_drv::UsDrvAsicCmd_InterfaceVersion::UsDrvAsicCmd_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint8 sequence     // request sequence counter
    type.uint8 flags     
    type.uint8 length     
    type.uint16 rawSpiData     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvAsicCmd_InterfaceVersion uml:Class
  Members:
    type.uint32 UsDrvAsicCmd_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvDetectionType uml:Enumeration
  Members:
    US_DETECTION_CONFIDENCE_MASK = 15            // Bits 0..3 of this enum specify confidence level of detection
    US_DETECTION_NORMAL_PATH = 0            // Detection on standard path
    US_DETECTION_ADVANCED_PATH_1 = 16            // Detection on advanced path 1 (chirp up)
    US_DETECTION_ADVANCED_PATH_2 = 32            // Detection on advanced path 2 (chirp down)
    US_DETECTION_NFD_PATH = 48            
    US_DETECTION_AATG_THRESHOLD_1 = 64            // Detected AATG threshold from first channel
    US_DETECTION_AATG_THRESHOLD_2 = 80            // Detected AATG threshold from second channel
    US_DETECTION_FIRING_TIMESTAMP = 128            // Sensor timestamp of firing event
    US_DETECTION_TYPE_MASK = 240            // Bitmask for detection type
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvDetection uml:Class
  Members:
    type.uint8 sensorId     // @range{1,US_DRV_MAX_NUM_SENSORS};logical sensor id of receiving sensor
    type.US_DRV.UsDrvDetectionType detectionType     
    type.uint16 amplitude     // @range{0,65535};@unit{nu};measured amplitude of detection
    type.sint8 phaseDerivative     // @range{-128,127};@unit{nu};phase derivative value
    type.uint8 syncCnt_ms     // @unit{ms};@range{0,255};PDCM sync pulse count when this measurement was transmitted to ECU
    type.uint16 sensorTimestamp_us     // @range{0,65535};@unit{us};sensor internal timer timestamp of detection
    type.sint32 relEcuTimestamp_us     // @range{-2147483648,2147483647};@unit{us};relative timestamp when measurement was received by ECU. this is a negative offset to UsDrvDetectionList.sSigHeader.uiTimestamp
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvSensorState uml:Enumeration
  Members:
    US_SENSOR_STATE_OFF = 0            // Sensor is off.
    US_SENSOR_STATE_STANDBY = 1            // Sensor is ready but not firing.
    US_SENSOR_STATE_INITIALISING = 2            // Sensor is in process of initialization
    US_SENSOR_STATE_RUNNING_NORMAL_MODE = 3            // Sensor is in normal/active operation mode.
    US_SENSOR_STATE_RUNNING_DIAGNOSTIC_MODE = 4            // Sensor is in diagnostic RX/TX mode.
    US_SENSOR_STATE_ERROR = 128            // Sensor is in error state.
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvSensorMode uml:Enumeration
  Members:
    US_SENSOR_MODE_OFF = 0            // sensor inactive/off mode
    US_SENSOR_MODE_STANDBY = 1            // standy mode
    US_SENSOR_MODE_NORMAL = 2            // normal operation mode
    US_SENSOR_MODE_DIAGNOSTIC = 3            // diagnostic RX/TX mode.
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvDetectionList uml:Class
  version: ::us_drv::UsDrvDetectionList_InterfaceVersion::UsDrvDetectionList_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.US_DRV.UsDrvSensorState sensorState     // @range{0,128};@unit{nu};State Information for each Sensor
    type.uint16 numDetections     // @range{0,US_DRV_MAX_NUM_DETECTIONS};@unit{nu};Number of elements in detections array
    type.US_DRV.UsDrvDetection detections     // Array of raw ultrasonic detections
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvDetectionList_InterfaceVersion uml:Class
  Members:
    type.uint32 UsDrvDetectionList_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvEnvelopeSignalPath uml:Class
  Members:
    type.uint8 rxSensorId     // receiving logical sensor id
    type.uint8 txSensorId     // transmitting logical sensor id
    type.uint8 signalPathId     // signal path id
    type.uint32 startSample     // index of first sample in sample data array
    type.uint32 numSamples     // count of samples in sample data array
    type.uint64 timestamp_us     // timestamp when the first sample was captured
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvEnvelopeData uml:Class
  version: ::us_drv::UsDrvEnvelopeData_InterfaceVersion::UsDrvEnvelopeData_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.US_DRV.UsDrvSensorState sensorState     // state of each sensor
    type.uint8 numSignalPaths     // number of signal paths carrying data in this envelope data set
    type.US_DRV.UsDrvEnvelopeSignalPath signalPaths     // array of signal path information for this data set of evelope data
    type.uint32 numSamples     // total number of samples in samples array
    type.uint16 samples     // bulk array of sample data, signal path assignment according to signalPaths memeber
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvEnvelopeData_InterfaceVersion uml:Class
  Members:
    type.uint32 UsDrvEnvelopeData_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvRuntimeSensorConfiguration uml:Class
  Members:
    type.US_DRV.UsDrvSensorMode state     // current/requested sensor operation mode and state
    type.uint8 stochasticCodes_ms     // delay values between burst pulses which form a stochastic code
    type.uint8 firingSchemes     // @range{0,7};burst configuration for specific pulse
    type.uint8 firCoefficientSet     // @range{0,1};selection of FIR coefficient set
    type.uint8 receiveGain     // @range{0,1};reception gain
    type.uint8 advancedPathConfiguration     // @range{0,1}
    type.uint8 aatgConfiguration1     // @range{0,3};Advanced Threshold Generator Path 1 configuration
    type.uint8 aatgConfiguration2     // @range{0,3};Advanced Threshold Generator Path 2 configuration
    type.uint8 adaptiveThresholdGeneration     // @range{0,1};select adaptive threshold generation
    type.uint8 swVersionMajor     // major version of sensor firmware; read-only, can not be set
    type.uint8 swVersionMinor     // minor version of sensor firmware; read-only, can not be set
    type.uint8 hwVersionMajor     // major version of sensor hardware; read-only, can not be set
    type.uint8 hwVersionMinor     // minor version of sensor hardware; read-only, can not be set
    type.uint32 serialNumber     // sensor serial number; read-only, can not be set
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvRequestState uml:Enumeration
  Members:
    US_REQUEST_NONE = 0            // No active sensor request
    US_REQUEST_INITIATED = 1            // State of the active request : request initiated
    US_REQUEST_PENDING = 2            // State of the active request : Pending, request was accpeted by UsDrv
    US_REQUEST_SUCCESS = 3            // State of the active request : Ended with success
    US_REQUEST_FAILED = 4            // State of the active request : Ended with failure, can be due to impossibility of the request, incompatible sensor state
    US_REQUEST_CANCELLED = 5            // State of the active request : Request cancelled, can be due to already existing active request
    US_REQUEST_TIMEOUT = 6            // State of the active request : Ended due to timeout, timeout cancel from USP side
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvRequestType uml:Enumeration
  Members:
    US_REQUEST_TYPE_NONE = 0            // No active sensor request
    US_REQUEST_TYPE_RESET = 1            // Active request of type : Sensor Reset
    US_REQUEST_TYPE_SLEEP = 2            // Active request of type : Sensor Sleep Mode
    US_REQUEST_TYPE_SAFESTATE = 3            // Active request of type : Sensor Safe State
    US_REQUEST_TYPE_UMM = 4            // Active request of type : Update Measurement Mode
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvRequestMeasMode uml:Enumeration
  Members:
    US_MEASMODE_NEAR_RANGE_FIELD = 0            // Measurement mode set to near range echoes
    US_MEASMODE_FAR_RANGE_FIELD = 1            // Measurement mode set to far range echoes
    US_MEASMODE_COUNT = 2            // Number of measurement modes and value used when no mode is available due to sensor not running
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvSensorRequest uml:Class
  Members:
    type.US_DRV.UsDrvRequestType type     // Type of the sensor request
    type.US_DRV.UsDrvRequestState state     // State of the last active sensor request
    type.US_DRV.UsDrvRequestMeasMode measMode     // Measurement Mode of the last active sensor request
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvRuntimeConfiguration uml:Class
  version: ::us_drv::UsDrvRuntimeConfiguration_InterfaceVersion::UsDrvRuntimeConfiguration_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint8 sequence     // @range{0,255};@unit{nu};Request sequence number
    type.US_DRV.UsDrvSensorRequest sensorRequests     // Request for each sensor
    type.uint8 responseSubState     // @range{0,255};@unit{nu};Sub state of response
    type.uint8 numConfigurations     // @range{0,US_DRV_MAX_NUM_SENSORS};@unit{nu};Number of configurations
    type.US_DRV.UsDrvRuntimeSensorConfiguration sensorConfigurations     // Array of sensor configurations
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvRuntimeConfiguration_InterfaceVersion uml:Class
  Members:
    type.uint32 UsDrvRuntimeConfiguration_VERSION = 2U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvPdcmStatusCounts uml:Class
  Members:
    type.uint32 dataOk     
    type.uint32 overWritten     
    type.uint32 pdcmError     
    type.uint32 outOfSync     
    type.uint32 lengthError     
    type.uint32 timingError     
    type.uint32 timeoutError     
    type.uint32 crcError     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvCrmStatusCounts uml:Class
  Members:
    type.uint32 dataOk     
    type.uint32 overWritten     
    type.uint32 crmError     
    type.uint32 reserved     
    type.uint32 lengthError     
    type.uint32 timingError     
    type.uint32 timeoutError     
    type.uint32 crcError     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvDebugPortBus uml:Class
  Members:
    type.uint32 crmCount     
    type.uint32 crmErrorCount     
    type.US_DRV.UsDrvCrmStatusCounts crmStatusCounts     
    type.uint32 pdcmCount     
    type.uint32 pdcmErrorCount     
    type.uint32 pdcmSizeMismatchCount     
    type.US_DRV.UsDrvPdcmStatusCounts pdcmStatusCounts     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvDebugPort uml:Class
  version: ::us_drv::UsDrvDebugPort_InterfaceVersion::UsDrvDebugPort_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint32 spiCommandCount     
    type.uint32 spiCrcErrorCount     // number of CRC errors appearing
    type.uint32 errorStatusCount     // number of CRC errors appearing
    type.uint32 pdcmCountTotalFront     
    type.uint32 pdcmErrorCountTotalFront     
    type.uint32 crmCountTotalFront     
    type.uint32 crmErrorCountTotalFront     
    type.uint32 pdcmCountTotalRear     
    type.uint32 pdcmErrorCountTotalRear     
    type.uint32 crmCountTotalRear     
    type.uint32 crmErrorCountTotalRear     
    type.US_DRV.UsDrvDebugPortBus busDebug     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvDebugPort_InterfaceVersion uml:Class
  Members:
    type.uint32 UsDrvDebugPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvErrorStatus uml:Enumeration
  Members:
    ERROR_NOT_TESTED = 0            // Initial status of error, before any check
    ERROR_INACTIVE = 1            // Error is inactive and no occurance in the current ignition cycle
    ERROR_PENDING = 2            // Error is detected, but occurances are under qualifying debounce level
    ERROR_SPORADIC = 3            // Error is inactive and there are occurances in the current ignition cycle or yet not cleared
    ERROR_ACTIVE = 4            // Error detected and number of occurences more than qualifying debounce level
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvVariantData uml:Class
  Members:
    type.uint8 usedAsicNum     
    type.uint8 usedSensorNum     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvSwErrors uml:Class
  Members:
    type.US_DRV.UsDrvErrorStatus usDriverNotReady     
    type.uint8 usDriverNotReadyDetails     
    type.US_DRV.UsDrvErrorStatus spiDriverOpFailure     
    type.uint8 spiDriverOpFailureDetails     
    type.US_DRV.UsDrvErrorStatus inputPortNull     
    type.uint8 inputPortNullDetails     
    type.US_DRV.UsDrvErrorStatus inputPortInvalid     
    type.uint8 inputPortInvalidDetails     
    type.US_DRV.UsDrvErrorStatus inputPortFreezed     
    type.uint8 inputPortFreezedDetails     
    type.US_DRV.UsDrvErrorStatus outputPortNull     
    type.uint8 outputPortNullDetails     
    type.US_DRV.UsDrvErrorStatus configurationNull     
    type.US_DRV.UsDrvErrorStatus configurationInvalid     
    type.uint8 configurationInvalidDetails     
    type.US_DRV.UsDrvErrorStatus spiDriverNull     
    type.US_DRV.UsDrvErrorStatus spiDriverIncompatible     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvAsicStatusErrors uml:Class
  Members:
    type.US_DRV.UsDrvErrorStatus asicInitFailure     
    type.uint8 asicInitFailureDetails     
    type.US_DRV.UsDrvErrorStatus asicInitTimeout     
    type.US_DRV.UsDrvErrorStatus asicCommTimeout     
    type.US_DRV.UsDrvErrorStatus ramBistFailed     
    type.US_DRV.UsDrvErrorStatus overTemperature     
    type.US_DRV.UsDrvErrorStatus vccUnderVoltage     
    type.US_DRV.UsDrvErrorStatus clkrefFailure     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvDsiCommErrors uml:Class
  Members:
    type.US_DRV.UsDrvErrorStatus discoverWrongSensorCount     
    type.uint8 discoverWrongSensorCountDetails     
    type.US_DRV.UsDrvErrorStatus dsiUndervoltage     
    type.US_DRV.UsDrvErrorStatus dsiCmdOverrun     
    type.US_DRV.UsDrvErrorStatus dsiCrcBistFailed     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvSpiCommErrors uml:Class
  Members:
    type.US_DRV.UsDrvErrorStatus prevCmdIncomplete     
    type.US_DRV.UsDrvErrorStatus crcErrorDetected     
    type.US_DRV.UsDrvErrorStatus undefinedCmd     
    type.US_DRV.UsDrvErrorStatus spiCrcBistFailed     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvCrmModeErrors uml:Class
  Members:
    type.US_DRV.UsDrvErrorStatus crmCmdFailure     
    type.uint8 crmCmdFailureDetails     
    type.US_DRV.UsDrvErrorStatus dataEarlyError     
    type.US_DRV.UsDrvErrorStatus dataLateError     
    type.US_DRV.UsDrvErrorStatus symbolCountError     
    type.US_DRV.UsDrvErrorStatus crcError     
    type.US_DRV.UsDrvErrorStatus spiCmdSequenceFailure     
    type.US_DRV.UsDrvErrorStatus symbolCodingError     
    type.US_DRV.UsDrvErrorStatus dsiPinUndervoltage     
    type.US_DRV.UsDrvErrorStatus clkRefError     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvPdcmFrameFormatErrors uml:Class
  Members:
    type.US_DRV.UsDrvErrorStatus noNewData     
    type.US_DRV.UsDrvErrorStatus dataBufferOvr     
    type.US_DRV.UsDrvErrorStatus dataLoss     
    type.US_DRV.UsDrvErrorStatus packetCountErr     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvDsiPacketErrors uml:Class
  Members:
    type.US_DRV.UsDrvErrorStatus symbolCountErr     
    type.US_DRV.UsDrvErrorStatus symbolCodingErr     
    type.US_DRV.UsDrvErrorStatus dsiPinUndervoltage     
    type.US_DRV.UsDrvErrorStatus clkrefErr     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvAsicErrors uml:Class
  Members:
    type.US_DRV.UsDrvAsicStatusErrors asicStatusErrors     
    type.US_DRV.UsDrvSpiCommErrors spiCommErrors     
    type.US_DRV.UsDrvDsiCommErrors dsiCommErrors     
    type.US_DRV.UsDrvCrmModeErrors crmModeErrors     
    type.US_DRV.UsDrvPdcmFrameFormatErrors pdcmFrameFormatErrors     
    type.US_DRV.UsDrvDsiPacketErrors dsiPacketErrors     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvSensorDiagErrors uml:Class
  Members:
    type.US_DRV.UsDrvErrorStatus burstErrorFlag     
    type.US_DRV.UsDrvErrorStatus burstPeakVoltageError     
    type.US_DRV.UsDrvErrorStatus ringingTimeTooShort     
    type.US_DRV.UsDrvErrorStatus ringingFreqMeasNotPossible     
    type.US_DRV.UsDrvErrorStatus ringingFreqOutOfRange     
    type.US_DRV.UsDrvErrorStatus tempOutOfRange     
    type.US_DRV.UsDrvErrorStatus transducerVoltageLimitation     
    type.US_DRV.UsDrvErrorStatus rawSignalOutOfToleranceDuringBurst     
    type.US_DRV.UsDrvErrorStatus burstLengthOutOfTolerance     
    type.US_DRV.UsDrvErrorStatus driverVoltageMonitoringError     
    type.US_DRV.UsDrvErrorStatus overvoltageErrorDrv1Drv2DrvsVdrv     
    type.US_DRV.UsDrvErrorStatus vdrvUndervoltageError     
    type.US_DRV.UsDrvErrorStatus vdrvOvervoltageError     
    type.US_DRV.UsDrvErrorStatus vsupUndervoltageError     
    type.US_DRV.UsDrvErrorStatus vsupOvervoltageError     
    type.US_DRV.UsDrvErrorStatus scodeIncorrect     
    type.US_DRV.UsDrvErrorStatus firingSchemeMissing     
    type.US_DRV.UsDrvErrorStatus receivingSchemeMissing     
    type.US_DRV.UsDrvErrorStatus pdcmPulseInvalid     
    type.US_DRV.UsDrvErrorStatus commandCrcError     
    type.US_DRV.UsDrvErrorStatus dsi3FccFailed     
    type.US_DRV.UsDrvErrorStatus dsi3SyncCrmCommandFailed     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvSensorSwErrors uml:Class
  Members:
    type.US_DRV.UsDrvErrorStatus sensorInitTimeout     
    type.uint8 sensorInitTimeoutDetails     
    type.US_DRV.UsDrvErrorStatus sensorFwVersionIncompatible     
    type.US_DRV.UsDrvErrorStatus sensorHwVersionIncompatible     
    type.US_DRV.UsDrvErrorStatus sensorInitFiringSchemeSendFail     
    type.US_DRV.UsDrvErrorStatus sensorInitReceivingSchemeSendFail     
    type.US_DRV.UsDrvErrorStatus sensorInitBusError     
    type.US_DRV.UsDrvErrorStatus sensorInitScodeIncorrect     
    type.US_DRV.UsDrvErrorStatus sensorPdcmCommTimeout     
    type.US_DRV.UsDrvErrorStatus unexpectedMeasModeRecfg     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvSensorErrors uml:Class
  Members:
    type.US_DRV.UsDrvSensorDiagErrors sensorDiagErrors     
    type.US_DRV.UsDrvSensorSwErrors sensorSwErrors     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvDiagPort uml:Class
  version: ::us_drv::UsDrvDiagPort_InterfaceVersion::UsDrvDiagPort_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.US_DRV.UsDrvVariantData usDriverVariantData     
    type.US_DRV.UsDrvSwErrors usDriverSwErrors     
    type.US_DRV.UsDrvAsicErrors asicErrors     
    type.US_DRV.UsDrvSensorErrors sensorErrors     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvDiagPort_InterfaceVersion uml:Class
  Members:
    type.uint32 UsDrvDiagPort_VERSION = 2U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvSensorConfiguration uml:Class
  Members:
    type.uint8 bus     
    type.uint8 physicalAddress     
    type.US_DRV.UsDrvRuntimeSensorConfiguration initialConfiguration     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvConfiguration uml:Class
  version: ::us_drv::UsDrvConfiguration_InterfaceVersion::UsDrvConfiguration_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.US_DRV.UsDrvSensorConfiguration sensorConfiguration     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_DRV.UsDrvConfiguration_InterfaceVersion uml:Class
  Members:
    type.uint32 UsDrvConfiguration_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.US_PROCESSING_Consts uml:Class
  Members:
    type.uint8 US_PROCESSING_MAX_NUM_SENSORS = 12    
    type.uint8 US_PROCESSING_MAX_NUM_SENSOR_GROUPS = 4    
    type.uint32 US_PROCESSING_MAX_NUM_POINTS = 255    
    type.uint32 US_PROCESSING_MAX_NUM_OBJECTS = 20    
    type.uint16 US_PROCESSING_MAX_NUM_VERTICES = 240    
    type.uint16 US_PROCESSING_MAX_NUM_ECHOES = 720    
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingSnsMode uml:Enumeration
  Members:
    US_PROCESSING_SNS_MODE_ACTIVATE = 0            
    US_PROCESSING_SNS_MODE_DEACTIVATE = 1            
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingMeasurementStatus uml:Enumeration
  Members:
    US_PROCESSING_MEASUREMENT_INVALID = 0            // Point measurement status is invalid
    US_PROCESSING_MEASUREMENT_NEW = 1            // This indicates the first detection of this particular point in the point list
    US_PROCESSING_MEASUREMENT_UPDATED = 2            // The point was updated with new information from a sensor in the last update cycle of this point list
    US_PROCESSING_MEASUREMENT_PREDICTED = 3            // The point was predicted without any new information with respect to the previous update cycle
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingSensorStatus uml:Enumeration
  Members:
    US_PROCESSING_SNS_INIT = 0            
    US_PROCESSING_SNS_ERROR = 1            
    US_PROCESSING_SNS_FAILSAFE = 2            
    US_PROCESSING_SNS_RUNNING = 3            
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingSensorRunningMode uml:Enumeration
  Members:
    US_SENSOR_MODE_NONE = 0            
    US_SENSOR_MODE_ERROR = 1            
    US_SENSOR_MODE_NFD = 2            
    US_SENSOR_MODE_FFD = 3            
    US_SENSOR_MODE_SWITCHING = 4            
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingObjectType uml:Enumeration
  Members:
    US_PROCESSING_OBJ_TYPE_UNKOWN = 0            
    US_PROCESSING_OBJ_TYPE_POLE = 1            
    US_PROCESSING_OBJ_TYPE_WALL = 2            
    US_PROCESSING_OBJ_TYPE_CURB = 3            
    US_PROCESSING_OBJ_TYPE_VEHICLE = 4            
    US_PROCESSING_OBJ_TYPE_PEDESTRIAN = 5            
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingObjectHeight uml:Enumeration
  Members:
    US_PROCESSING_OBJ_HT_UNKOWN = 0            
    US_PROCESSING_OBJ_HT_LOWER_BUMPER_HEIGHT = 1            
    US_PROCESSING_OBJ_HT_AT_BUMPER_HEIGHT = 2            
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingObjectKinematic uml:Enumeration
  Members:
    US_PROCESSING_OBJ_KN_UNKOWN = 0            
    US_PROCESSING_OBJ_KN_STATIC = 1            
    US_PROCESSING_OBJ_KN_DYNAMIC = 2            
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingFuncInput uml:Class
  version: ::us_processing::UsProcessingFuncInput_InterfaceVersion::UsProcessingFuncInput_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.boolean errorSuppressionReq     // Request to suppress error reporting or not
    type.uint8 usSnsGrpModeReq     // Sensor Mode request for each sensor group
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingFuncInput_InterfaceVersion uml:Class
  Members:
    type.uint32 UsProcessingFuncInput_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingVertex uml:Class
  Members:
    type.float32 posX_m     // @unit{m};X position of point, cartesian coordinate system DIN 8855 middle of rear axix XY plane parallel to road surface
    type.float32 posY_m     // @unit{m};Y position of point, cartesian coordinate system DIN 8855 middle of rear axix XY plane parallel to road surface
    type.float32 posXVar_m     // @unit{m};Variance of X position
    type.float32 posYVar_m     // @unit{m};Variance of Y position
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingPoint uml:Class
  Members:
    type.uint64 timestamp_us     // @range{0,18446744073709551615};@unit{us};Timestamp when point detection occurred
    type.float32 xPosition_m     // @range{-20,20};@unit{m};X position of point, cartesian coordinate system DIN 8855 middle of rear axix XY plane parallel to road surface
    type.float32 yPosition_m     // @range{-20,20};@unit{m};Y position of point, cartesian coordinate system DIN 8855 middle of rear axix XY plane parallel to road surface
    type.float32 varXposition_m     // @range{0.0F,400.0F};@unit{m^2};Variance of X position
    type.float32 varYposition_m     // @range{0.0F,400.0F};@unit{m^2};Variance of Y position
    type.float32 xyPositionCovar_m     // @range{-400.0F,400.0F};@unit{m^2};cross-covariance XY
    type.float32 direction     // @range{0,2Π};@unit{Radians};Reflection Point Direction
    type.float32 directionVariance     // @range{0,2Π^2};@unit{Radians^2};Variance of Reflection Point Direction
    type.float32 trackCurvature     // @range{0.0,1000.0};@unit{nu};Track curvature at this point
    type.float32 varTrackCurvature     // @range{0.0,1000.0};@unit{nu};Track curvature variance
    type.float32 probabilityHigh     // @range{0.0F,1.0F};@unit{nu};Probability that this point belongs to High Object
    type.float32 heightConfidence     // @range{0.0F,1.0F};@unit{nu};Confidence about the height information
    type.float32 xSensorPos_m     // @range{-20,20};@unit{m};Sensor location in X Direction in world coordindefines at the measurement time
    type.float32 ySensorPos_m     // @range{-20,20};@unit{m};Sensor location in Y Direction in world coordindefines at the measurement time
    type.float32 sensorDirection     // @range{0,2Π};@unit{rad};Sensor direction in world coordinates defined at the measurement time
    type.uint32 sensorMask     // @range{0,3584};@unit{nu};Sensor Mask defines which sensors contributed in this point generation
    type.float32 rawMeasRange_m     // @range{0,14};@unit{m};Raw measured range from sensor to the reflection point of the object
    type.uint32 trackId     // @range{0,4294967295};@unit{nu};Track id which this point associated to
    type.uint32 pointCountInTrack     // @range{0,4294967295};@unit{nu};Point iterator in tracker
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingObject uml:Class
  Members:
    type.uint64 timestamp_us     // @unit{us};timestamp when object detection occured
    type.float32 xRelativeVelocity_mps     // @unit{mps};X velocity of point relative to car
    type.float32 yRelativeVelocity_mps     // @unit{mps};Y velocity of point relative to car
    type.float32 heightEstimation_m     // @unit{m};Object Height estimation
    type.float32 confidenceLevel     // @range{0.0,1.0};@unit{nu};confidence level of detection, normalized in the range  0.0f to 1.0f
    type.uint16 objectId     // @range{0,65535};@unit{nu};Unique ID number of object this point originated from
    type.uint16 vertexStartIndex     // Vertex Start Index gives reference to the location of the first vertex describes this object in VertexStartIndex
    type.uint16 numOfVertices     // Number of Verticies describes this object within Verticies List starts from VertexStartIndex
    type.US_PROCESSING.UsProcessingMeasurementStatus measurementStatus     // @range{0,3};see definition of UsProcessingMeasurementStatus
    type.US_PROCESSING.UsProcessingObjectType objectType     // @range{0,5};see definition of UsProcessingObjectType
    type.US_PROCESSING.UsProcessingObjectHeight heightClassificationLevel     // @range{0,2};see definition of UsProcessingObjectHeight
    type.US_PROCESSING.UsProcessingObjectKinematic kinematicClassificationGuess     // @range{0,2};see definition of UsProcessingObjectKinematic
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingPointList uml:Class
  version: ::us_processing::UsProcessingPointList_InterfaceVersion::UsProcessingPointList_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint32 numberOfPoints     // @range{0,US_PROCESSING_MAX_NUM_POINTS};@unit{nu};used data buffers to optimize processing
    type.US_PROCESSING.UsProcessingPoint points     // Array of Points
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingPointList_InterfaceVersion uml:Class
  Members:
    type.uint32 UsProcessingPointList_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingObjectList uml:Class
  version: ::us_processing::UsProcessingObjectList_InterfaceVersion::UsProcessingObjectList_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint32 numberOfObjects     // @range{0,US_PROCESSING_MAX_NUM_OBJECTS};@unit{nu};used data buffers to optimize processing
    type.US_PROCESSING.UsProcessingObject objects     // Array of Objects
    type.US_PROCESSING.UsProcessingVertex vertices     // Array of Vertices referenced in the Object definition to describe the objects shape
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingObjectList_InterfaceVersion uml:Class
  Members:
    type.uint32 UsProcessingObjectList_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingDiagOutput uml:Class
  version: ::us_processing::UsProcessingDiagOutput_InterfaceVersion::UsProcessingDiagOutput_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.US_PROCESSING.UsProcessingSensorStatus sensorStatus     // @range{0,3};@unit{nu};Sensors Status , see definition of SensorStatus
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingDiagOutput_InterfaceVersion uml:Class
  Members:
    type.uint32 UsProcessingDiagOutput_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingDistanceList uml:Class
  version: ::us_processing::UsProcessingDistanceList_InterfaceVersion::UsProcessingDistanceList_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint8 numberOfSensors     // @range{0,US_PROCESSING_MAX_NUM_SENSORS};@unit{nu};used data buffers to optimize processing
    type.float32 distdir_m     // @range{0.0F,14.0F};@unit{m};Sensor direct signal way (Traveled distance) measured
    type.float32 distDirQuality     // @range{-3.4x10^38,3.4x10^38};@unit{nu};Quality of Sensor direct signal way (Traveled distance) measured
    type.float32 distCross_m     // @range{0.0F,14.0F};@unit{m};Sensor cross signal way (Traveled distance) measured
    type.float32 distCrossQuality     // @range{-3.4x10^38,3.4x10^38};@unit{nu};Quality of Sensor cross signal way (Traveled distance) measured
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingDistanceList_InterfaceVersion uml:Class
  Members:
    type.uint32 UsProcessingDistanceList_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingDataIntegrity uml:Class
  version: ::us_processing::UsProcessingDataIntegrity_InterfaceVersion::UsProcessingDataIntegrity_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.US_PROCESSING.UsProcessingSensorRunningMode sensorRunningMode     // @range{0,4};@unit{nu};Sensor running mode in current cycle
    type.float32 filtDist_m     // @range{0.0F,7.0F};@unit{m};Filtered sensor distance in current cycle
    type.uint16 rawEchoCount     // @range{0,500};@unit{nu};Sensor raw echo count in current cycle
    type.float32 inputVelocity     // @range{-100.0F,100.0F};@unit{mps};Input velocity to UDP in current cycle
    type.float32 inputTemperature     // @range{-40.0F,95.0F};@unit{°C};Input temperature to UDP in current cycle
    type.boolean isInputCommOk     // @range{0,1};@unit{nu};Input data to UDP in SW Devkit is ok
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingDataIntegrity_InterfaceVersion uml:Class
  Members:
    type.uint32 UsProcessingDataIntegrity_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingFilteredEcho uml:Class
  Members:
    type.uint8 rxSensorId     // @range{1,US_PROCESSING_MAX_NUM_SENSORS};@unit{nu};Sensor Id of receiving sensor
    type.uint8 txSensorId     // @range{1,US_PROCESSING_MAX_NUM_SENSORS};@unit{nu};Sensor Id of transmitting sensor
    type.uint16 trackId     // @range{1,65535};@unit{nu};Track Id of correlation track
    type.sint32 relEcuTimestamp_us     // @range{-2147483648,2147483647};@unit{us};Relative timestamp when measurement was received by ECU. This is a negative offset to UsProcessingFilteredEcho.sSigHeader.uiTimestamp
    type.uint16 tof_us     // @range{0,65535};@unit{us};Echo time of flight
    type.uint16 amplitude     
    type.sint8 phaseDerivative     // @range{-128,127};@unit{nu};Phase derivative value
    type.uint8 codingConfidence     // @range{0,15};@unit{nu};Frequency domain coding cofidence level
    type.uint8 timeConfidence     // @range{0,15};@unit{nu};Time domain tracking confidence level
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingFilteredEchoList uml:Class
  version: ::us_processing::UsProcessingFilteredEchoList_InterfaceVersion::UsProcessingFilteredEchoList_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint16 numEchoes     // @range{0,US_PROCESSING_MAX_NUM_ECHOES};@unit{nu};Number of elements in echoes array
    type.US_PROCESSING.UsProcessingFilteredEcho echoes     // Array of filtered ultrasonic echoes
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingFilteredEchoList_InterfaceVersion uml:Class
  Members:
    type.uint32 UsProcessingFilteredEchoList_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingErrorsAndWarnings uml:Class
  Members:
    type.uint64 lastErrorsTs     
    type.uint64 lastWarnsTs     
    type.uint32 uspErrors     
    type.uint32 stCfgWarnings     
    type.uint32 dynCfgWarnings     
    type.uint32 inputWarnings     
    type.uint32 sensorWarnings     
    type.uint32 outputWarnings     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingDebugPort uml:Class
  version: ::us_processing::UsProcessingDebugPort_InterfaceVersion::UsProcessingDebugPort_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.US_PROCESSING.UsProcessingErrorsAndWarnings uspErrAndWarnings     // field with information regarding last erros and warnings occured in USP operations
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingDebugPort_InterfaceVersion uml:Class
  Members:
    type.uint32 UsProcessingDebugPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingSensorParameter uml:Class
  Members:
    type.float32 posX_m     
    type.float32 posY_m     
    type.float32 posZ_m     
    type.float32 horRot_deg     
    type.float32 verRot_deg     
    type.uint8 associatedTx     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingSensorParameters uml:Class
  Members:
    type.uint32 sensorParameterCount     
    type.US_PROCESSING.UsProcessingSensorParameter sensorParameter     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingStateManagementCfg uml:Class
  Members:
    type.float32 check1     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingEchoPreProcessingCfg uml:Class
  Members:
    type.uint64 echo_filter_timestamp_threshold     
    type.float32 echo_filter_distance_threshold     
    type.float32 echo_filter_distance_correction_factor     
    type.float32 echo_filter_distance_correction_weight_1     
    type.float32 echo_filter_distance_correction_weight_2     
    type.float32 echo_filter_delta_min     
    type.float32 echo_filter_delta_max     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingReflectionTrackingCfg uml:Class
  Members:
    type.float32 fovDeviation     
    type.float32 measurementDistanceUncertainty     
    type.uint64 halfLifeTime     
    type.float32 halfLifeTimeMultiplicator     
    type.float32 unbindTrackerDirSigmaIncrease     
    type.float32 singleTrackerRelaxDirSigma     
    type.float32 singleTrackerRelaxRange     
    type.float32 dualTrackerRelaxDirSigma     
    type.float32 tripleTrackerRelaxDirSigma     
    type.float32 convexSaturationMaximumIncrement     
    type.float32 convexSaturationIncrementLimit     
    type.float32 convexSaturationIncrementMultiplicator     
    type.float32 rftrOutputFilterRadiusAngleSigma     
    type.float32 rftrOutputFilterConvexSaturationLimit     
    type.float32 multiplicityFractionAcceptance     
    type.float32 multiplicityMaxRadius     
    type.float32 multiplicityMaxDistanceSinceLastUpdate     
    type.float32 measurementUncertaintyModifierMax     
    type.float32 measurementUncertaintyMultplier     
    type.uint8 stableTrackerMeasurementNumber     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingNeighboringFilterCfg uml:Class
  Members:
    type.float32 keep_distance     
    type.float32 neighbor_region_threshold     
    type.uint16 keep_cycles     
    type.uint16 keep_buffer_length     
    type.uint16 neighbor_count_threshold     
    type.uint16 neighbor_count_threshold_only     
    type.uint8 neighbor_paths_threshold     
    type.boolean use_absolute_distance     
    type.boolean weight_neighbors_by_dir_variance     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingObjectTrackingCfg uml:Class
  Members:
    type.US_PROCESSING.UsProcessingNeighboringFilterCfg usnfParams     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingDynParams uml:Class
  Members:
    type.uint8 dynParDummyChar     
    type.uint32 dynParDummyInt     
    type.float32 dynParDummyFloat     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingParams uml:Class
  version: ::us_processing::UsProcessingParams_InterfaceVersion::UsProcessingParams_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.US_PROCESSING.UsProcessingStateManagementCfg ustmParams     
    type.US_PROCESSING.UsProcessingEchoPreProcessingCfg usepParams     
    type.US_PROCESSING.UsProcessingReflectionTrackingCfg rftrParams     
    type.US_PROCESSING.UsProcessingObjectTrackingCfg objtParams     
    type.US_PROCESSING.UsProcessingSensorParameters sensParams     
    type.US_PROCESSING.UsProcessingDynParams dynParams     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_PROCESSING.UsProcessingParams_InterfaceVersion uml:Class
  Members:
    type.uint32 UsProcessingParams_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_EM.US_EM_Consts uml:Class
  Members:
    type.uint16 US_EM_MAX_NUM_MAP_POINTS = 500U    
    type.uint8 US_EM_MAX_NUM_STATIC_OBJ_PTS = 10U    
    type.uint8 US_EM_MAX_NUM_STATIC_OBJ = 32U    
    type.uint8 US_EM_MAX_NUM_DYN_OBJ_PTS = 4U    
    type.uint8 US_EM_MAX_NUM_DYN_OBJ = 1U    
    type.uint8 US_EM_MAX_NUM_OF_SENSORS = 12U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_EM.ObjectTrend uml:Enumeration
    // Trend information about the object.
  Members:
    OBJ_TRND_UNKNOWN = 0            
    OBJ_TRND_APPROACHING = 1            
    OBJ_TRND_MOVING_AWAY = 2            
    OBJ_TRND_FIXED_DIST = 3            
    OBJ_TRND_MAX_NUM = 4            
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_EM.StaticObjHeightType uml:Enumeration
    // Height class of the object
  Members:
    SO_HI_UNKNOWN = 0            
    SO_HI_WHEEL_TRAVERSABLE = 1            
    SO_HI_BODY_TRAVERSABLE = 2            
    SO_HI_DOOR_OPENABLE = 3            
    SO_HI_HIGH_OBSTACLE = 4            
    SO_HI_HANGING_OBJECT = 5            
    SO_HI_LOWER_BUMPER_HEIGHT = 6            
    MAX_NUM_HEIGHT_TYPES = 7            
----------------------------------------------------------------------------------------------------------


  # ID:  type.US_EM.StaticObjectSerializable uml:Class
    // Structures comprising the static environment of the vehicle
  Members:
    type.uint16 refObjID_nu     // @range{0,65535};@unit{nu};Unique identifier of the object of which this structure is part (a physical object maybe split into multiple structures to comply to the convexity requirement)
    type.uint8 existenceProb_perc     // @unit{Percent};@range{0,100};Existence probability of the structure
    type.uint16 objAgeInCycles_nu     // @range{0,65535};@unit{nu};Algo cycles since the first corresponding detection
    type.uint16 objMeasLastUpdateInCycles_nu     // @range{0,65535};@unit{nu};Algo cycles since of the last object update
    type.uint16 objTrendLastUpdateInCycles_nu     // @range{0,65535};@unit{nu};Algo cycles since last cycle where the object-trend has been updated
    type.US_EM.ObjectTrend objTrend_nu     // @range{0,4};@unit{nu};Movement trend of object
    type.boolean readFromNVRAM_nu     // @range{0,1};@unit{nu};True if structure has only been read from non volatile RAM and not (yet) been confirmed by new measurements
    type.US_EM.StaticObjShapeSerializable objShape_m     // @unit{m};Positions of the object polygon vertices
    type.US_EM.StaticObjHeightType objHeightClass_nu     // @range{0,5};@unit{nu};Height class of the object
    type.uint8 objHeightClassConfidence_perc     // @unit{Percent};@range{0,100};Confidence in the object height classification
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_EM.StaticObjShapeSerializable uml:Class
  Members:
    type.LSM_GEOML.size_type actualSize     // @unit{nu};@range{0,US_EM_MAX_NUM_STATIC_OBJ_PTS};Number of vertices in object polygon
    type.cml.Vec2Df_POD array     // @unit{Vector2D};@range{-1000.0F,1000.0F};Vertex x,y coordinates
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_EM.ObjMeasurementState uml:Enumeration
    // Measurement state of the dynamic object
  Members:
    MEAS_STATE_NEW = 0            
    MEAS_STATE_MEASURED = 1            
    MEAS_STATE_PREDICTED = 2            
    MEAS_STATE_DELETED = 3            
    MAX_NUM_MEASUREMENT_STATES = 4            
----------------------------------------------------------------------------------------------------------


  # ID:  type.US_EM.DynamicObjectSerializable uml:Class
    // Dynamic objects list
  Members:
    type.uint8 existenceProb_perc     // @unit{Percent};@range{0,100};Existence probability of the dynamic object
    type.US_EM.DynamicObjShapeSerializable objShape_m     
    type.cml.Vec2Df_POD vel_mps     // @unit{m / s};Velocity vector of the object
    type.cml.Vec2Df_POD accel_mps2     // @unit{m / s^2}
    type.float32 headingAngle_rad     // @unit{Radian};@range{-3.14159265359,3.14159265359}
    type.US_EM.ObjMeasurementState measurementState_nu     // @range{0,4};Measurement state of the dynamic object
    type.uint16 refObjID_nu     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_EM.DynamicObjShapeSerializable uml:Class
  Members:
    type.LSM_GEOML.size_type actualSize     
    type.cml.Vec2Df_POD array     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_EM.UsEnvModelPort uml:Class
  version: ::us_em::UsEnvModelPort_InterfaceVersion::UsEnvModelPort_VERSION
    // None
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint8 numberOfStaticObjects_u8     // @range{0,255};@unit{nu};Number of valid static objects
    type.uint8 numberOfDynamicObjects_u8     // @range{0,255};@unit{nu};Number of valid dynamic objects
    type.uint8 firstStatObjOutDetZoneIdx_u8     // @range{0,255};@unit{nu};Idx of first static object out of the detection zone
    type.uint8 firstDynObjOutDetZoneIdx_u8     // @range{0,255};@unit{nu};Idx of first dynamic object out of the detection zone
    type.US_EM.DynamicObjectSerializable dynamicObjects     // @unit{nu};dynamic objects
    type.US_EM.StaticObjectSerializable staticObjects     // @unit{nu};structures comprising the static environment of the vehicle
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_EM.UsEnvModelPort_InterfaceVersion uml:Class
  Members:
    type.uint32 UsEnvModelPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_EM.AvailabilityStatus uml:Enumeration
    // Availability state of the USS sensors
  Members:
    ITEM_IN_DEGRADED_STATE = 0            
    ITEM_IN_ERROR_STATE = 1            
    ITEM_NOT_AVAILABLE = 2            
    ITEM_AVAILABLE = 3            
    MAX_NUM_AVAILABILITY_STATES = 4            
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_EM.PerceptionAvailabilityPort uml:Class
  version: ::us_em::PerceptionAvailabilityPort_InterfaceVersion::PerceptionAvailabilityPort_VERSION
    // Availability states of the sensors and EM
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.US_EM.AvailabilityStatus statusUSSensors_nu     // @range{0,5};Availability state of the USS sensors
    type.US_EM.AvailabilityStatus statusEnvModel_nu     // @range{0,5};Availability state of the environment model
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_EM.PerceptionAvailabilityPort_InterfaceVersion uml:Class
  Members:
    type.uint32 PerceptionAvailabilityPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_EM.UsEmPosition uml:Class
  Members:
    type.float32 xPosition_m     
    type.float32 yPosition_m     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_EM.UsEmPoint uml:Class
  Members:
    type.US_EM.UsEmPosition pointPos     
    type.float32 probHigh     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_EM.UsEmDecData uml:Class
  Members:
    type.float32 objDecFactors     
    type.float32 alphaProj     
    type.uint8 sensorId     
    type.uint8 visibility     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_EM.UsEmDebugOutputPort uml:Class
  version: ::us_em::UsEmDebugOutputPort_InterfaceVersion::UsEmDebugOutputPort_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint16 numberOfPoints     
    type.US_EM.UsEmPoint usEmPointList     
    type.US_EM.UsEmPosition vehicleBoundingBoxes     
    type.US_EM.UsEmDecData decayData     
    type.uint32 cycleCounter     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_EM.UsEmDebugOutputPort_InterfaceVersion uml:Class
  Members:
    type.uint32 UsEmDebugOutputPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_EM.UsEmNeighboringFilterCfgIntern uml:Class
  Members:
    type.float32 keep_distance     // maximum distance away from the car to keep the points  = 5.0f
    type.float32 neighbor_region_threshold     // parameters to define the region around the point where we look for neighbors = 0.18f
    type.uint16 keep_cycles     // maximum number of cycles after which the points will be deleted
    type.uint16 keep_buffer_length     // the maximum length of the object tracker buffer
    type.uint16 neighbor_count_threshold     // filter threshold based on the count of neighbors
    type.uint16 neighbor_count_threshold_only     // filter threshold based on the count of neighbors used only without the paths
    type.uint8 neighbor_paths_threshold     // filter threshold based on the number of pathes
    type.boolean use_absolute_distance     // use abs(x)+abs(y) as distance and not sqrt(x^2+y^2) = false
    type.boolean weight_neighbors_by_dir_variance     // weight the each neighbor by its own variance of the direction = true
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_EM.UsEmDetectionZoneCfg uml:Class
  Members:
    type.float32 detZoneFrontLeftX     
    type.float32 detZoneFrontLeftY     
    type.float32 detZoneFrontRightX     
    type.float32 detZoneFrontRightY     
    type.float32 detZoneBackLeftX     
    type.float32 detZoneBackLeftY     
    type.float32 detZoneBackRightX     
    type.float32 detZoneBackRightY     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_EM.UsEmSensorParameterIntern uml:Class
  Members:
    type.float32 posX_m     // @range{-10.0,10.0};@unit{m};Orig rear axis mid.
    type.float32 posY_m     // @range{-10.0,10.0};@unit{m};Orig rear axis mid.
    type.float32 posZ_m     // @range{0.0,2.0};@unit{m};Orig rear axis mid.
    type.float32 horRot_deg     // @range{-360.0,360.0};Hor. rot.
    type.float32 verRot_deg     // @range{-360.0,360.0};Vert. rot.
    type.uint8 associatedTx     // phys. ind. echo sensor ID
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_EM.UsEmSensorParametersIntern uml:Class
  Members:
    type.uint32 sensorParameterCount     
    type.US_EM.UsEmSensorParameterIntern sensorParameter     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_EM.UsEmParams uml:Class
  version: ::us_em::UsEmParams_InterfaceVersion::UsEmParams_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.US_EM.UsEmNeighboringFilterCfgIntern uspcParams     
    type.US_EM.UsEmDetectionZoneCfg detZoneParams     
    type.US_EM.UsEmSensorParametersIntern usSensorParams     
----------------------------------------------------------------------------------------------------------

  # ID:  type.US_EM.UsEmParams_InterfaceVersion uml:Class
  Members:
    type.uint32 UsEmParams_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.HPSD.HealthVector uml:Class
    // A list of booleans containing the consolidated health information of the system. Functions shall use this information for deciding on degradation.
  Members:
    type.boolean hostTemperatureWarning     // Platform (ADCU) is near but has not reached critical over/undertemperature. CAT3 degradation can happen any time.
    type.boolean vehicleCommunicationError     // Platform (ADCU) has detected communication issues over the vehicle CAN. (Idea: this is a severe communication error, i.e. not only a single message missing)
    type.boolean onlySafetyCoreAvailable     // Platform (ADCU) has shut off the performance domain of the ECU. Only MoCo is available.
    type.boolean hmiCommunicationError     // Theres a problem with LVDS or I2C
    type.boolean odometryUnreliable     // Odometry information can"t be trusted (derived from VDP or Vedodo or EMO DEMs)
    type.boolean frontCamPreProcUnreliable     // Front camera chain has pre-processing problems (i.e. no image, wrong image, camera reliablility signal, data stream/crc errors.)
    type.boolean rearCamPreProcUnreliable     // Rear camera chain has pre-processing problems (i.e. no image, wrong image, camera reliablility signal, data stream/crc errors.)
    type.boolean leftCamPreProcUnreliable     // Left camera chain has pre-processing problems (i.e. no image, wrong image, camera reliablility signal, data stream/crc errors.)
    type.boolean rightCamPreProcUnreliable     // Right camera chain has pre-processing problems (i.e. no image, wrong image, camera reliablility signal, data stream/crc errors.)
    type.boolean frontCamVisionUnreliable     // Front CV chain has issues. Environment Model does not incude new data from the front camera.
    type.boolean rearCamVisionUnreliable     // Rear CV chain has issues. Environment Model does not incude new data from the rear camera.
    type.boolean leftCamVisionUnreliable     // Left CV chain has issues. Environment Model does not incude new data from the left camera.
    type.boolean rightCamVisionUnreliable     // Right CV chain has issues. Environment Model does not incude new data from the right camera.
    type.boolean frontCamBlockage     // Front camera has significant blockage. Input from this camera is not used.
    type.boolean rearCamBlockage     // Rear camera has significant blockage. Input from this camera is not used.
    type.boolean leftCamBlockage     // Left camera has significant blockage. Input from this camera is not used.
    type.boolean rightCamBlockage     // Right camera has significant blockage. Input from this camera is not used.
    type.boolean frontUltrasonicsUnreliable     
    type.boolean rearUltrasonicsUnreliable     
    type.boolean envModelStaticObjsUnreliable     // Internal issues happened in the Environment Model"s static object processing. Static object list can"t be trusted.
    type.boolean envModelTrafficParticiapantsUnreliable     // Internal issues happened in the Environment Model"s traffic participant processing. Traffic participant list can"t be trusted.
    type.boolean envModelParkingFeaturesUnreliable     // Internal issues happened in the Environment Model"s parking feature processing. Parking slots, wheel stoppers/lockers, markers can"t be trusted.
    type.boolean localizationUnreliable     // Localization has some issues.
    type.boolean pdwUnreliable     // An internal issue in PDW was detected, and we can"t trust that it can fulfill its purpose.
    type.boolean lscaUnreliable     // An internal issue in LSCA was detected, and we can"t trust that it can fulfill its purpose.
    type.boolean aupUnreliable     // An internal issue in AUP was detected, and we can"t trust that it can fulfill its purpose.
    type.boolean raUnreliable     // An internal issue in RA was detected, and we can"t trust that it can fulfill its purpose.
    type.boolean hvUnreliable     // An internal issue in HV was detected, and we can"t trust that it can fulfill its purpose.
    type.boolean avgaFailed     // Automated Vehicle Guidance Arbiter has failed, it is not possible for maneuvering functions to control the vehicle.
    type.boolean mfManagerFailed     // It is not possible to prioritize between LSCA and other MFs control requests.
    type.boolean mocoFailed     // It is not possible to control the vehicle.
----------------------------------------------------------------------------------------------------------

  # ID:  type.HPSD.HealthVectorPort uml:Class
  version: ::hpsd::HealthVectorPort_InterfaceVersion::HealthVectorPort_VERSION
    // A port structure containing the health vector.
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     // Version number of interface.
    type.eco.SignalHeader sSigHeader     // Signal header with common signal information.
    type.HPSD.HealthVector healthVector     // A list of booleans describing the consolidated health status of the system.
----------------------------------------------------------------------------------------------------------

  # ID:  type.HPSD.HealthVectorPort_InterfaceVersion uml:Class
  Members:
    type.uint32 HealthVectorPort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.HPSD.PlatformState uml:Enumeration
    // An enumeration describing the possible states of the platform running the application
  Members:
    STARTUP = 0            // Platform (ADCU) during startup. It is not possible to run the application yet.
    FAST_PDW = 1            // The platform can already support fastPDW functionality. IU, SPU and CVU are operational.
    NORMAL_OPERATION = 2            // All the cores have been started.
    SHUTDOWN = 3            // The system has initiated shutdown.
----------------------------------------------------------------------------------------------------------

  # ID:  type.HPSD.PlatformStatePort uml:Class
  version: ::hpsd::PlatformStatePort_InterfaceVersion::PlatformStatePort_VERSION
    // Port structure that describes the state of the platform
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     // Version number of interface.
    type.eco.SignalHeader sSigHeader     // Signal header with common signal information.
    type.HPSD.PlatformState platformState     // Enumeration describing the current state of the platform.
----------------------------------------------------------------------------------------------------------

  # ID:  type.HPSD.PlatformStatePort_InterfaceVersion uml:Class
  Members:
    type.uint32 PlatformStatePort_VERSION = 1U    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AVGA_SWC.AVGA_consts uml:Class
  Members:
    type.uint8 DUMMY_CONST = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AVGA_SWC.AVGA_params uml:Class
  version: ::avga_swc::AVGA_params_InterfaceVersion::AVGA_params_VERSION
  Members:
    type.eco.AlgoInterfaceVersionNumber uiVersionNumber     
    type.eco.SignalHeader sSigHeader     
    type.uint8 AP_M_DUMMY_FOR_PDO     
----------------------------------------------------------------------------------------------------------

  # ID:  type.AVGA_SWC.AVGA_params_InterfaceVersion uml:Class
  Members:
    type.uint32 AVGA_params_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AVGA_SWC.SupervisionStateType uml:Enumeration
  Members:
    AVGA_OFF = 0            
    AVGA_ON = 1            
    AVGA_INTERVENING = 2            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AVGA_SWC.AVGA_SupervisionState uml:Class
  version: ::avga_swc::AVGA_SupervisionState_InterfaceVersion::AVGA_SupervisionState_VERSION
  Members:
    type.AVGA_SWC.SupervisionStateType supervisionState     
----------------------------------------------------------------------------------------------------------

  # ID:  type.AVGA_SWC.AVGA_SupervisionState_InterfaceVersion uml:Class
  Members:
    type.uint32 AVGA_SupervisionState_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AVGA_SWC.AVGA_SupervisionRequest uml:Class
  version: ::avga_swc::AVGA_SupervisionRequest_InterfaceVersion::AVGA_SupervisionRequest_VERSION
  Members:
    type.boolean supervisionRequest     
----------------------------------------------------------------------------------------------------------

  # ID:  type.AVGA_SWC.AVGA_SupervisionRequest_InterfaceVersion uml:Class
  Members:
    type.uint32 AVGA_SupervisionRequest_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AVGA_SWC.AVGA_SupervisionAvailability uml:Class
  version: ::avga_swc::AVGA_SupervisionAvailability_InterfaceVersion::AVGA_SupervisionAvailability_VERSION
  Members:
    type.boolean supervisionAvailability     
----------------------------------------------------------------------------------------------------------

  # ID:  type.AVGA_SWC.AVGA_SupervisionAvailability_InterfaceVersion uml:Class
  Members:
    type.uint32 AVGA_SupervisionAvailability_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AVGA_SWC.AutomatedVehicleGuidanceRequestType uml:Enumeration
  Members:
    AVGA_NONE = 0            
    AVGA_BRAKING_SUPERVISION = 1            
    AVGA_BRAKING_VISUAL_SUPERVISION = 2            
    AVGA_BRAKING_AUDIO_VISUAL_SUPERVISION = 3            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AVGA_SWC.AVGA_AutomatedVehicleGuidanceRequest uml:Class
  version: ::avga_swc::AVGA_AutomatedVehicleGuidanceRequest_InterfaceVersion::AVGA_AutomatedVehicleGuidanceRequest_VERSION
  Members:
    type.AVGA_SWC.AutomatedVehicleGuidanceRequestType automatedVehicleGuidanceRequest     
----------------------------------------------------------------------------------------------------------

  # ID:  type.AVGA_SWC.AVGA_AutomatedVehicleGuidanceRequest_InterfaceVersion uml:Class
  Members:
    type.uint32 AVGA_AutomatedVehicleGuidanceRequest_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AVGA_SWC.AutomatedVehicleGuidanceStateType uml:Enumeration
  Members:
    AVGA_UNAVAILABLE = 0            
    AVGA_AVAILABLE = 1            
    AVGA_USED = 2            
    AVGA_GRANTED = 3            
    AVGA_OVERRULED = 4            
----------------------------------------------------------------------------------------------------------

  # ID:  type.AVGA_SWC.AVGA_AutomatedVehicleGuidanceState uml:Class
  version: ::avga_swc::AVGA_AutomatedVehicleGuidanceState_InterfaceVersion::AVGA_AutomatedVehicleGuidanceState_VERSION
  Members:
    type.AVGA_SWC.AutomatedVehicleGuidanceStateType selfDrivingState     
----------------------------------------------------------------------------------------------------------

  # ID:  type.AVGA_SWC.AVGA_AutomatedVehicleGuidanceState_InterfaceVersion uml:Class
  Members:
    type.uint32 AVGA_AutomatedVehicleGuidanceState_VERSION = 1    
----------------------------------------------------------------------------------------------------------

  # ID:  type.AVGA_SWC.AvgaStopRequestPort uml:Class
  version: ::avga_swc::AvgaStopRequestPort_InterfaceVersion::AvgaStopRequestPort_VERSION
  Members:
    type.boolean stopRequest     
----------------------------------------------------------------------------------------------------------

  # ID:  type.AVGA_SWC.AvgaStopRequestPort_InterfaceVersion uml:Class
  Members:
    type.uint32 AvgaStopRequestPort_VERSION = 1    
----------------------------------------------------------------------------------------------------------
