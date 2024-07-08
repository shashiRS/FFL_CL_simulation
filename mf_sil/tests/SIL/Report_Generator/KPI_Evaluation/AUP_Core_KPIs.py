from KPI_Evaluation.AUP_KPI_cfg import  *
from KPI_Evaluation.AUP_KPI_cfg import AUPParameters_Import, AUPQuantities_Import_SW_Arch, \
    TRJPLA_PLANNED_TRAJ, PATH_NoOfStrokes_TRJPLA, TAPOSD_STATIC_STRUCT

# ==================================================================================
# calculate virtual signals for TRJPLA
# ==================================================================================
def TRJPLA_Signals(curQuantDict, time, curName):
    found        = False
    '''position in the tuple TRJPLA_PlannedTraj[index] : (x, y, crv, velocity, distToStop, yaw)'''
    pos_x        = 0
    pos_y        = 1
    pos_crv      = 2
    pos_velocity = 3
    pos_distToStop = 4
    pos_yaw        = 5

    JUMP         = 1
    NO_JUMP      = 0
    NO_ACCEL     = 0
    NO_DECEL     = 0
    NO_YAW_UPDATE = 0
    INVALID_TRAJ  = 0
    NO_CALCULATION = 0
    DIST_EXCEED = 1
    DIST_NOT_EXCEED = 0
    NOT_CALCULATED = -1
    
    AP_V_COMF_STEER_ANG_VEL_RADPS = AUPParameters_Import['AP_V_COMF_STEER_ANG_VEL_RADPS'] #0.4267#
    AP_G_MAX_YAW_RATE_PLANNED_RADPS = AUPParameters_Import['AP_G_MAX_YAW_RATE_PLANNED_RADPS'] #0.9#
    AP_G_MAX_DECEL_COMFORTABLE_MPS2 = AUPParameters_Import['AP_G_MAX_DECEL_COMFORTABLE_MPS2'] #0.17 mps#
    AP_G_MIN_AVG_PLANNED_V_MPS = AUPParameters_Import['AP_G_MIN_AVG_PLANNED_V_MPS'] #0.25 mps#
    AP_G_MIN_SAF_DIST_HIGH_OBST_PLANNED_M = AUPParameters_Import['AP_G_MIN_SAF_DIST_HIGH_OBST_PLANNED_M'] #0.1m
    AP_G_MIN_SAF_DIST_TRAV_OBST_PLANNED_M = AUPParameters_Import['AP_G_MIN_SAF_DIST_TRAV_OBST_PLANNED_M'] #0.05m
    virtualSignals          = []
    TRJPLA_PlannedTraj      = []
    TRJPLA_numberOfCrvSteps = []
    TRJPLA_numberOfDistSteps = []
    TRJPLA_absVel = []
    TRJPLA_absMaxVelLimit_mps = []
    TRJPLA_absMinVelLimit_mps = []
    TRJPLA_crv    = []
    TRJPLA_crv_max = []
    TRJPLA_crv_maxStep = []
    TRJPLA_Min_Decel = [] #min value of deceleration of trajectory points calculated
    TRJPLA_Max_Accel = [] #max value of acceleration of trajectory points calculated
    TRJPLA_YawAng_CR = []
    TRJPLA_NrOfStrokes = []
    newSegment = 0
    TRJPLA_StrokesNumber = []
    MinDistToHightObjExceed = []
    MinDistToWheelTravObjExceed = []
    MinDistToBodyTravObjExceed = []
    vel_val = []
    AverageAbsoluteVelocity = []
    Average_AbsoluteVelocity = []
    NewSegmentStartedList = []
    

    '''maximum number of strokes read from the measurement file'''
    numberOfStrokes = curQuantDict[AUPQuantities_Import_SW_Arch["AP.numberOfStrokes"]][-1] #take the last value of the signal
    '''Signal "AP.evaluationPort.n_strokes_max_nu" contains the value from the Scenario Catalogue'''
    n_strokes_max   = curQuantDict[AUPQuantities_Import_SW_Arch["AP.evaluationPort.n_strokes_max_nu"]][-1]

    for index in range(len(time)):
        '''list which will contain the result of the check if there is a jump or not in the
        trajectories in the list TRJPLA_PlannedTraj[index]'''
        curvatureSteps   = []
        distToStopSteps = []
        numberOfCrvSteps = 0
        numberOfDistSteps = 0
        yawChangeRate    = []
        CRDecelAccel = []
        valCrvSteps = []
        TRJPLA_PlannedTraj.append([])
        ValidPoints = curQuantDict[AUPQuantities_Import_SW_Arch["AP.plannedTrajPort.numValidCtrlPoints_nu"]][index] #nr of valid trajectory points in every timestamp
        ValidTrajectory = curQuantDict[AUPQuantities_Import_SW_Arch["AP.trajValid"]][index] 
        NewSegmentStarted = curQuantDict[AUPQuantities_Import_SW_Arch["AP.NewSegmentStarted"]][index]
        NewSegmentStartedList.append(NewSegmentStarted) 
        DistanceToHighObject = curQuantDict[AUPQuantities_Import_SW_Arch["AP.shortestDistanceToHighObject_m"]][index]
        DistanceToWheelTravObject = curQuantDict[AUPQuantities_Import_SW_Arch["AP.shortestDistanceToWheelTravObject_m"]][index]
        DistanceToBodyTravObject = curQuantDict[AUPQuantities_Import_SW_Arch["AP.shortestDistanceToBodyTravObject_m"]][index]

        x_driven   = curQuantDict[AUPQuantities_Import_SW_Arch["AP.plannedTrajDrivenPoint.xTrajRAReq_m"]][index]
        y_driven   = curQuantDict[AUPQuantities_Import_SW_Arch["AP.plannedTrajDrivenPoint.yTrajRAReq_m"]][index]
        crv_driven = curQuantDict[AUPQuantities_Import_SW_Arch["AP.plannedTrajDrivenPoint.crvRAReq_1pm"]][index]
        velocity_driven = curQuantDict[AUPQuantities_Import_SW_Arch["AP.plannedTrajDrivenPoint.velocityLimitReq_mps"]][index]
        distToStop_driven = curQuantDict[AUPQuantities_Import_SW_Arch["AP.plannedTrajDrivenPoint.distanceToStopReq_m"]][index]
        yaw_driven  = curQuantDict[AUPQuantities_Import_SW_Arch["AP.plannedTrajDrivenPoint.yawReq_rad"]][index]
        if ValidTrajectory == INVALID_TRAJ:  #skip the calculations if trajectory is invalid
            TRJPLA_numberOfCrvSteps.append(NO_CALCULATION)
            TRJPLA_numberOfDistSteps.append(NO_CALCULATION)
            TRJPLA_absMinVelLimit_mps.append(NO_CALCULATION)
            TRJPLA_absMaxVelLimit_mps.append(NO_CALCULATION)
            TRJPLA_crv_max.append(NO_CALCULATION)
            TRJPLA_crv_maxStep.append(NO_CALCULATION)
            TRJPLA_Min_Decel.append(NO_CALCULATION)
            TRJPLA_YawAng_CR.append(NO_CALCULATION)
            TRJPLA_Max_Accel.append(NO_CALCULATION)
            MinDistToHightObjExceed.append(DIST_NOT_EXCEED)
            MinDistToWheelTravObjExceed.append(DIST_NOT_EXCEED)
            MinDistToBodyTravObjExceed.append(DIST_NOT_EXCEED)
        else:
            ''' each calculated trajectory points from 0 to 9 (TRJPLA_PLANNED_TRAJ)
            TRJPLA_PlannedTraj[index] -  list with the information of the 10 trajectory points calculated'''
            for i in range(ValidPoints):
                x         = curQuantDict[AUPQuantities_Import_SW_Arch["AP_TRJPLA_" + str(i) + "_xTrajRAReq_m"]][index]
                y         = curQuantDict[AUPQuantities_Import_SW_Arch["AP_TRJPLA_" + str(i) + "_yTrajRAReq_m"]][index]
                crv       = curQuantDict[AUPQuantities_Import_SW_Arch["AP_TRJPLA_" + str(i) + "_crvRARReq_1pm"]][index]
                velocity  = curQuantDict[AUPQuantities_Import_SW_Arch["AP_TRJPLA_" + str(i) + "_velocityLimitReq_mps"]][index]
                distToStop = curQuantDict[AUPQuantities_Import_SW_Arch["AP_TRJPLA_" + str(i) + "_distanceToStopReq_m"]][index]
                yawCR      = curQuantDict[AUPQuantities_Import_SW_Arch["AP_TRJPLA_" + str(i) + "_yawReq_rad"]][index]
                TRJPLA_PlannedTraj[index].append((x, y, crv, velocity, distToStop, yawCR))
                '''if there are at least 2 entries, check if there is a jump in the last 2 trajectory points calculated'''
                if i>0:
                    distToNextSample = np.absolute(TRJPLA_PlannedTraj[index][i][pos_distToStop] - TRJPLA_PlannedTraj[index][i-1][pos_distToStop])
                    if max(TRJPLA_PlannedTraj[index][i][pos_velocity], TRJPLA_PlannedTraj[index][i - 1][pos_velocity]) == 0:
                        curvatureSteps.append(NO_JUMP)
                    else:
                        if np.absolute(TRJPLA_PlannedTraj[index][i][pos_crv] - TRJPLA_PlannedTraj[index][i-1][pos_crv]) >\
                                    AP_V_COMF_STEER_ANG_VEL_RADPS * distToNextSample/\
                                    max(TRJPLA_PlannedTraj[index][i][pos_velocity], TRJPLA_PlannedTraj[index][i-1][pos_velocity]):
                            curvatureSteps.append(JUMP)
                        else:
                            curvatureSteps.append(NO_JUMP)
                    l = len(curvatureSteps)
                    if l>1: #if there are at least 2 entries in the list
                        '''Eg: l = 2: curvatureSteps[0] == 0 and curvatureSteps[1] == 1 => increment numberOfCrvSteps'''
                        '''count the transitions 0 to 1 in curvatureSteps and save it in TRJPLA_numberOfCrvSteps'''
                        if curvatureSteps[l - 2] == 0 and curvatureSteps[l - 1] == 1:
                            numberOfCrvSteps = numberOfCrvSteps + 1
                    # req 204        
                    distanceChange = (TRJPLA_PlannedTraj[index][i][pos_distToStop] - TRJPLA_PlannedTraj[index][i-1][pos_distToStop])
                    if distanceChange > 0: #check if there are 
                        if distanceChange > math.sqrt(pow((TRJPLA_PlannedTraj[index][i][pos_y] - TRJPLA_PlannedTraj[index][i-1][pos_y]),2) + pow((TRJPLA_PlannedTraj[index][i][pos_x] - TRJPLA_PlannedTraj[index][i-1][pos_x]),2)) *\
                                min(1,(np.mean([TRJPLA_PlannedTraj[index][i][pos_velocity], TRJPLA_PlannedTraj[index][i - 1][pos_velocity]]) / AP_G_MIN_AVG_PLANNED_V_MPS)):
                            distToStopSteps.append(JUMP)
                        else:
                            distToStopSteps.append(NO_JUMP)
                    else:
                        distToStopSteps.append(NO_JUMP)
                    r = len(distToStopSteps)
                    if r>1:
                        if distToStopSteps[r - 2] == 0 and distToStopSteps[r - 1] == 1:
                            numberOfDistSteps = numberOfDistSteps + 1

                    if distToNextSample == 0:
                        #req _162 -> long decel/accel
                        CRDecelAccel.append(NO_ACCEL)
                        #req _156 -> calculate the yaw change rate
                        yawChangeRate.append(NO_YAW_UPDATE)
                    else:
                        CRDecelAccel.append((TRJPLA_PlannedTraj[index][i][pos_velocity] - TRJPLA_PlannedTraj[index][i-1][pos_velocity]) * max(TRJPLA_PlannedTraj[index][i][pos_velocity], TRJPLA_PlannedTraj[index][i-1][pos_velocity])/ distToNextSample)
                        yawChangeRate.append(np.absolute(TRJPLA_PlannedTraj[index][i-1][pos_yaw] - TRJPLA_PlannedTraj[index][i][pos_yaw]) * max(TRJPLA_PlannedTraj[index][i][pos_velocity], TRJPLA_PlannedTraj[index][i-1][pos_velocity]) / distToNextSample)
                    crvStep = np.absolute(TRJPLA_PlannedTraj[index][i][pos_crv] - TRJPLA_PlannedTraj[index][i-1][pos_crv])
                    valCrvSteps.append(crvStep)
                TRJPLA_absVel.append(abs(velocity))
                TRJPLA_crv.append(abs(crv))
            TRJPLA_absMinVelLimit_mps.append(min(TRJPLA_absVel)) #min value of velocity limit of trajectory points calculated in every timestamp
            TRJPLA_absMaxVelLimit_mps.append(max(TRJPLA_absVel)) #max value of velocity limit of trajectory points calculated in every timestamp
            TRJPLA_Min_Decel.append(min(CRDecelAccel)) #min value of deceleration of trajectory points calculated in every timestamp
            TRJPLA_Max_Accel.append(max(CRDecelAccel)) #max value of acceleration of trajectory points calculated in every timestamp
            TRJPLA_crv_max.append(max(TRJPLA_crv)) #max value of absolute curvature of trajectory points calculated in every timpestamp
            TRJPLA_YawAng_CR.append(max(yawChangeRate)) #max value of yaw change rate of trajectory points calculated in every timestamp
            TRJPLA_crv_maxStep.append(max(valCrvSteps)) 
            TRJPLA_numberOfCrvSteps.append(numberOfCrvSteps)
            TRJPLA_numberOfDistSteps.append(numberOfDistSteps)
                        
            if DistanceToHighObject == NOT_CALCULATED:
                MinDistToHightObjExceed.append(DIST_NOT_EXCEED)
            elif DistanceToHighObject >= AP_G_MIN_SAF_DIST_HIGH_OBST_PLANNED_M:
                MinDistToHightObjExceed.append(DIST_NOT_EXCEED)
            else:
                MinDistToHightObjExceed.append(DIST_EXCEED)
                
            if DistanceToWheelTravObject == NOT_CALCULATED:
                MinDistToWheelTravObjExceed.append(DIST_NOT_EXCEED)
            elif DistanceToWheelTravObject >= AP_G_MIN_SAF_DIST_TRAV_OBST_PLANNED_M:
                MinDistToWheelTravObjExceed.append(DIST_NOT_EXCEED)
            else:
                MinDistToWheelTravObjExceed.append(DIST_EXCEED)
                
            if DistanceToBodyTravObject == NOT_CALCULATED:
                MinDistToBodyTravObjExceed.append(DIST_NOT_EXCEED)
            elif DistanceToBodyTravObject >= AP_G_MIN_SAF_DIST_TRAV_OBST_PLANNED_M:
                MinDistToBodyTravObjExceed.append(DIST_NOT_EXCEED)
            else:
                MinDistToBodyTravObjExceed.append(DIST_EXCEED)
                
        if NewSegmentStarted:
            if NewSegmentStartedList[-2] == 1:
                newSegment = newSegment
            else:
                newSegment = newSegment + 1
        TRJPLA_StrokesNumber.append(newSegment)

        if TRJPLA_StrokesNumber[index] == 1 and velocity_driven != -1:
            vel_val.append(velocity_driven)
    if len(vel_val) ==0:
        AverageAbsoluteVelocity = -1
    else:
        AverageAbsoluteVelocity = sum(vel_val)/len(vel_val)
    for a in TRJPLA_StrokesNumber:
        if a == 1:
            Average_AbsoluteVelocity.append(AverageAbsoluteVelocity)
        else:
            Average_AbsoluteVelocity.append(NOT_CALCULATED)

    TRJPLA_Min_Decel = [z if z <= 0 else NO_DECEL for z in TRJPLA_Min_Decel] #check if in the min list is a positive value and replace with 0 to have only deceleration value on VS  
    TRJPLA_Max_Accel = [x if x >= 0 else NO_ACCEL for x in TRJPLA_Max_Accel] #check if in the max list is a negative value and replace with 0 to have only acceleration value on VS
    '''virtual signal for average absolute velocity limit for the first stroke'''
    Average_AbsoluteVelocity_VS = Signal(samples=Average_AbsoluteVelocity,
                               timestamps=time,
                               name='Average_AbsoluteVelocity_VS',
                               unit='')
    virtualSignals.append(Average_AbsoluteVelocity_VS)
    '''create the virtual signal to be used for the evaluation of curvature jumps'''
    TRJPLA_numberOfCrvSteps_VS = Signal(samples=TRJPLA_numberOfCrvSteps,
                               timestamps=time,
                               name='TRJPLA_numberOfCrvSteps_VS',
                               unit='')
    virtualSignals.append(TRJPLA_numberOfCrvSteps_VS)
    '''v s dist to stop jumps'''
    TRJPLA_numberOfDistSteps_VS = Signal(samples=TRJPLA_numberOfDistSteps,
                               timestamps=time,
                               name='TRJPLA_numberOfDistSteps_VS',
                               unit='')
    virtualSignals.append(TRJPLA_numberOfDistSteps_VS)
    '''virtual signal for max velocity limit of valid trajectory points calculated'''
    TRJPLA_absMaxVelLimit_mps_VS = Signal(samples=TRJPLA_absMaxVelLimit_mps,
                               timestamps=time,
                               name='TRJPLA_absMaxVelLimit_mps_VS',
                               unit='')
    virtualSignals.append(TRJPLA_absMaxVelLimit_mps_VS)
    '''virtual signal for min upper abs velocity limit of valid trajectory points calculated'''
    TRJPLA_absMinVelLimit_mps_VS = Signal(samples=TRJPLA_absMinVelLimit_mps,
                               timestamps=time,
                               name='TRJPLA_absMinVelLimit_mps_VS',
                               unit='')
    virtualSignals.append(TRJPLA_absMinVelLimit_mps_VS)
    '''virtual signal for max absolute curvature of valid trajectory points calculated'''
    TRJPLA_abs_crv_max_VS = Signal(samples=TRJPLA_crv_max,
                               timestamps=time,
                               name='TRJPLA_abs_crv_max_VS',
                               unit='')
    virtualSignals.append(TRJPLA_abs_crv_max_VS)
    '''virtual signal for min long deceleration of valid trajectory points calculated'''
    TRJPLA_Min_Decel_VS = Signal(samples=TRJPLA_Min_Decel,
                               timestamps=time,
                               name='TRJPLA_Min_Decel_VS',
                               unit='')
    virtualSignals.append(TRJPLA_Min_Decel_VS)
    '''virtual signal for max yaw change rate of valid trajectory points calculated'''
    TRJPLA_YawAng_CR_VS = Signal(samples=TRJPLA_YawAng_CR,
                               timestamps=time,
                               name='TRJPLA_YawAng_CR_VS',
                               unit='')
    virtualSignals.append(TRJPLA_YawAng_CR_VS)
    '''virtual signal for max long accel change rate of valid trajectory points calculated'''
    TRJPLA_Max_Accel_VS = Signal(samples=TRJPLA_Max_Accel,
                               timestamps=time,
                               name='TRJPLA_Max_Accel_VS',
                               unit='')
    virtualSignals.append(TRJPLA_Max_Accel_VS)
    '''virtual signal to check if distance to high object exceed'''
    MinDistToHightObjExceed_VS = Signal(samples=MinDistToHightObjExceed,
                               timestamps=time,
                               name='MinDistToHightObjExceed_VS',
                               unit='')
    virtualSignals.append(MinDistToHightObjExceed_VS)
    '''virtual signal to check if distance to body traversable object exceed'''
    MinDistToBodyTravObjExceed_VS = Signal(samples=MinDistToBodyTravObjExceed,
                               timestamps=time,
                               name='MinDistToBodyTravObjExceed_VS',
                               unit='')
    virtualSignals.append(MinDistToBodyTravObjExceed_VS)
    '''virtual signal to check if distance to wheel traversable objects exceed'''
    MinDistToWheelTravObjExceed_VS = Signal(samples=MinDistToWheelTravObjExceed,
                               timestamps=time,
                               name='MinDistToWheelTravObjExceed_VS',
                               unit='')
    virtualSignals.append(MinDistToWheelTravObjExceed_VS)
    '''virtual signal for number of strokes based on new segment started'''
    TRJPLA_StrokeNumber_VS = Signal(samples=TRJPLA_StrokesNumber,
                               timestamps=time,
                               name='TRJPLA_StrokeNumber_VS',
                               unit='')
    virtualSignals.append(TRJPLA_StrokeNumber_VS)
    '''virtual signal for value of curvature step'''
    TRJPLA_CrvSteps_VS = Signal(samples=TRJPLA_crv_maxStep,
                               timestamps=time,
                               name='TRJPLA_CrvSteps_VS',
                               unit='')
    virtualSignals.append(TRJPLA_CrvSteps_VS)
    

    ''' save the maximum number of strokes in the file from PATH_NoOfStrokes_TRJPLA
        If there is already an entry in the file for the scenario 'curName', do not save the info'''
    new_line = str(curName) + ' numberOfStrokes = ' + str(numberOfStrokes) + '\n'
    
    try:
        with open(PATH_NoOfStrokes_TRJPLA, 'r') as file:
            content = file.readlines()
            file.close()
    except:
        content = ''
    for line in content:
        if curName in line:
            found = True
            n_strokes_max = int(line.replace(str(curName) + ' numberOfStrokes = ', ''))
            #print("name", curName, 'n_strokes_max', n_strokes_max)
    '''create the virtual signal to be used for the evaluation of the number of strokes'''
    n_strokes_max_VS = Signal(samples=np.array([n_strokes_max] * len(time)),
                               timestamps=time,
                               name='n_strokes_max_VS',
                               unit='')
    virtualSignals.append(n_strokes_max_VS)
    
    if not found:
        with open(PATH_NoOfStrokes_TRJPLA, 'a+') as file:
            file.write(new_line)
    return virtualSignals

# ==================================================================================
# calculate virtual signals for TAPOSD
# ==================================================================================
def TAPOSD_Signals(curQuantDict, time, curName, veh_points):
    # search for the time when Screen HeadUnit = ManeuverActive and Screen HeadUnit = ManeuverFunished
    t_idx_func_activated = None
    t_idx_func_stopped   = None
    NOT_CALCULATED       = -1
    TRUE                 = 1
    FALSE                = 0
    EXISTENCE_PROB_PERC  = 80
    NO_STATIC_STRUCTURES = TAPOSD_STATIC_STRUCT # 8
    NO_POINTS_IN_STRUCT  = 16
    IGNORED_SAMPLES      = 3
    PARKSM_SHU_ManeuverActive = 4

    index = 0
    virtualSignals = []
    if len(curQuantDict[AUPQuantities_Import_SW_Arch['AP_HMI_userActionHeadUnit_nu']]) > IGNORED_SAMPLES:
        startSample = IGNORED_SAMPLES  # ignore first samples, might not be relevant
    else:
        startSample = None
    for value in list(curQuantDict[AUPQuantities_Import_SW_Arch['AP_VISU_screen_nu']][startSample:]):
        if value == PARKSM_SHU_ManeuverActive:
            t_idx_func_activated = index + startSample
            break
        index = index + 1
    PB_coordinates = {}
    if t_idx_func_activated:
        stop_init = t_idx_func_activated
    else:
        stop_init = len(time)

    '''Initialiaze the values of the virtual signals with value -1 = NOT_CALCULATED, from 0 to the t_idx_func_activated'''
    car_outside_PB = np.array([NOT_CALCULATED]*stop_init)
    Static_Struct_Colides_Target_Pose = []
    for i in range(NO_STATIC_STRUCTURES):
        Static_Struct_Colides_Target_Pose.append(np.array([NOT_CALCULATED]*stop_init))

    if t_idx_func_activated:
        for index in range(t_idx_func_activated, len(time)):
            '''Save the info of the static structures in a dictionary with following information:
                            key - number of the static structure: i = 0 to 7
                            value - list of points (x,y)'''
            TAPOSD_StaticStructure = {}
            for i in range(NO_STATIC_STRUCTURES):
                TAPOSD_StaticStructure[i] = []
                if curQuantDict[AUPQuantities_Import_SW_Arch["AP_envModelPort_" + str(i) + "_existenceProb_perc"]][
                    index] > EXISTENCE_PROB_PERC:
                    for j in range(NO_POINTS_IN_STRUCT):
                        x = \
                        curQuantDict[AUPQuantities_Import_SW_Arch["AP_envModelPort_" + str(i) + "_objShape_m_" + str(j) + ".x"]][
                            index]
                        y = \
                        curQuantDict[AUPQuantities_Import_SW_Arch["AP_envModelPort_" + str(i) + "_objShape_m_" + str(j) + ".y"]][
                            index]
                        if x or y:
                            TAPOSD_StaticStructure[i].append((x, y))
                        else:
                            break

            '''calculate the coordinates of the PB
            Save the info of the PB in a dict with following information:
                        key - 'FrontLeft'/'FrontRight'/'RearLeft'/'RearRight'
                        value - point (x,y)'''
            PB_coordinates['FrontLeft'] = [curQuantDict[AUPQuantities_Import_SW_Arch["AP_PB_0_FrontLeft_x"]][index],
                                           curQuantDict[AUPQuantities_Import_SW_Arch["AP_PB_0_FrontLeft_y"]][index]]  # coordinates x, y
            PB_coordinates['FrontRight'] = [curQuantDict[AUPQuantities_Import_SW_Arch["AP_PB_0_FrontRight_x"]][index],
                                            curQuantDict[AUPQuantities_Import_SW_Arch["AP_PB_0_FrontRight_y"]][index]]  # coordinates x, y
            PB_coordinates['RearLeft'] = [curQuantDict[AUPQuantities_Import_SW_Arch["AP_PB_0_RearLeft_x"]][index],
                                          curQuantDict[AUPQuantities_Import_SW_Arch["AP_PB_0_RearLeft_y"]][index]]  # coordinates x, y
            PB_coordinates['RearRight'] = [curQuantDict[AUPQuantities_Import_SW_Arch["AP_PB_0_RearRight_x"]][index],
                                           curQuantDict[AUPQuantities_Import_SW_Arch["AP_PB_0_RearRight_y"]][index]]  # coordinates x, y
            pos_x = curQuantDict[AUPQuantities_Import_SW_Arch["AP_TAPOSD_0_pos.x"]][index]
            pos_y = curQuantDict[AUPQuantities_Import_SW_Arch["AP_TAPOSD_0_pos.y"]][index]
            pos_yaw = curQuantDict[AUPQuantities_Import_SW_Arch["AP_TAPOSD_0_yaw_rad"]][index]

            '''translate the coordinates of the Parking Box in the coordinate system with pos as origin
            Save the info in a list with following information: point (x,y)'''

            veh_points_translated = []
            for point in veh_points:
                x_translated = pos_x + point[0] * cos(pos_yaw) - point[1] * sin(pos_yaw)
                y_translated = pos_y + point[0] * sin(pos_yaw) + point[1] * cos(pos_yaw)
                veh_points_translated.append((x_translated, y_translated))

            if 'parkout' not in curName.lower():
                PB_area = PolygonArea(list(PB_coordinates.values()))
                PB_translated_ordered = orderClockwise(list(PB_coordinates.values()))
                car_outside_PB_ = NOT_CALCULATED
                '''check if the vehicle points are inside the PB
                check for each vehicle point if the area of the PB is equal to the sum of the area of the triangles
                of each to 2 closed points from PB and the vehicle point'''
                for point in veh_points_translated:
                    car_outside_PB_ = pointOutsidePoligon(PB_area, PB_translated_ordered, point)
                    #second version of the check, same results
                    '''Point = shapely.geometry.Point(point)
                    temp = Polygon(PB_translated_ordered).contains(Point)
                    if temp == True:
                        car_outside_PB_ = FALSE
                    else:
                        car_outside_PB_ = TRUE
                        break'''
                    if car_outside_PB_ == TRUE:
                        break
                car_outside_PB = np.append(car_outside_PB, car_outside_PB_)
            else:
                car_outside_PB = np.append(car_outside_PB, NOT_CALCULATED)

            '''calculate for each static structure, if any of the points inside the structure collides with the vehicle 
            positioned inside the Parking Box'''
            for staticStructure in TAPOSD_StaticStructure.keys():
                collision         = NOT_CALCULATED
                if TAPOSD_StaticStructure[staticStructure] != []:
                    veh_points_ordered = orderClockwise(veh_points_translated)
                    points_static_struct_ordered = orderClockwise(TAPOSD_StaticStructure[staticStructure])
                    if Polygon(points_static_struct_ordered).intersection(Polygon(veh_points_ordered)):
                        collision = TRUE
                    else:
                        collision = FALSE
                Static_Struct_Colides_Target_Pose[staticStructure] = np.append(Static_Struct_Colides_Target_Pose[staticStructure], collision)
    #create the virtual signals
    car_outside_PB_VS = Signal(samples=car_outside_PB,
                               timestamps=time,
                               name='car_outside_PB_VS',
                               unit='')
    virtualSignals.append(car_outside_PB_VS)

    for i in range(NO_STATIC_STRUCTURES):
        Static_Struct_Colides_Target_Pose_VS = Signal(samples=Static_Struct_Colides_Target_Pose[i],
                                                      timestamps=time,
                                                      name='staticStructColidesTarget_Pose_' + str(i) + '_VS',
                                                      unit='')
        virtualSignals.append(Static_Struct_Colides_Target_Pose_VS)
    return virtualSignals


#======================================================================================
#Check if a certain point is outside/inside the polygon
#Inputs:
#           Poligon_area - calculated area of the poligon
#           Poligon_coordinates - dictionary with: key - FrontLeft/FrontRight/RearRight/RearLeft
#                                              value - (x,y)coordinates of the point
#Output: 1 - if point is outside the Poligon
#        0 - if point is inside the Poligon
#======================================================================================
def pointOutsidePoligon (Poligon_area, Poligon_coordinates, point):
    '''Calculate the sum of the areas of the triangles of the point, to each 2 close points of the PB'''
    CAR_INSIDE_POLIGON  = 0
    CAR_OUTSIDE_POLIGON = 1
    point_outside_poligon = CAR_INSIDE_POLIGON
    area_pointToPoligon = PolygonArea([point, Poligon_coordinates[0], Poligon_coordinates[1]]) +\
                     PolygonArea([point, Poligon_coordinates[1], Poligon_coordinates[2]]) +\
                     PolygonArea([point, Poligon_coordinates[2], Poligon_coordinates[3]]) + \
                     PolygonArea([point, Poligon_coordinates[3], Poligon_coordinates[0]])
    if round(Poligon_area,4) != round(area_pointToPoligon,4):
        point_outside_poligon = CAR_OUTSIDE_POLIGON
    return point_outside_poligon

#======================================================================================
#Return a list of points ordered clockwise
#======================================================================================
def orderClockwise(corners):
    if corners != list(corners):
        return 0
    cornersOrdered=corners
    x_avg = float(0)
    y_avg = float(0)
    arctan = []
    for i in range(len(cornersOrdered)):
        x_avg += cornersOrdered[i][0]
        y_avg += cornersOrdered[i][1]
    x_avg = x_avg/len(cornersOrdered)
    y_avg = y_avg/len(cornersOrdered)
    for i in range(len(cornersOrdered)):
        arctan.insert(i,math.atan2(cornersOrdered[i][1]-y_avg,cornersOrdered[i][0]-x_avg))
        j = i
        while j>0 and (arctan[j]<arctan[j-1]):
            aux1 = cornersOrdered[j]
            cornersOrdered[j] = cornersOrdered[j-1]
            cornersOrdered[j-1] = aux1
            aux2 = arctan[j]
            arctan[j] = arctan[j-1]
            arctan[j-1] = aux2
            j -= 1
    return cornersOrdered
#======================================================================================
#Calculate the area of a polygon
#Input: list with the coordinates (x,y) of the points
#======================================================================================
def PolygonArea(corners):
    cornersOrdered = orderClockwise(corners)
    area = float(0)
    i = 0
    j = 1
    for i in range(len(cornersOrdered)-1):
        area += (cornersOrdered[i][0]*cornersOrdered[i+1][1])
    for i in range(len(cornersOrdered)-1):
        area -= (cornersOrdered[i+1][0]*cornersOrdered[i][1])
    area += cornersOrdered[len(cornersOrdered)-1][0]*cornersOrdered[0][1]
    area -= cornersOrdered[len(cornersOrdered)-1][1]*cornersOrdered[0][0]
    return abs(area)/2
#corners = [(2.0, 1.0), (4.0, 5.0), (7.0, 8.0)]
