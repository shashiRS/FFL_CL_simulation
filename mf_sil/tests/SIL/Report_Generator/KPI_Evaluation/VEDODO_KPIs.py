
from KPI_Evaluation.AUP_KPI_cfg import  *
#from ErrorCodes_cfg import EVALUATION_DROPPED
from os import listdir
from os.path import isfile, join

requiredQuantsVEDODO   = ['AP.headUnitVisualizationPort.screen_nu',
                        'AP.trajCtrlDebugPort.currentDeviation_m',
                        'AP.trajCtrlDebugPort.orientationError_rad',
                        'Sensor.Collision.Vhcl.Fr1.Count',
                        'AP.odoDebugPort.odoCmRefxPosition_m',
                        'AP.odoDebugPort.odoCmRefyPosition_m',
                        'AP.odoDebugPort.odoCmRefyawAngEgoRaCur_rad',
                        'AP.odoEstimationPort.yawAngle_rad',
                        'AP.odoEstimationPort.xPosition_m',
                        'AP.odoEstimationPort.yPosition_m',
                        "Car.Yaw",
                        "Car.v",
                        "AP.odoEstimationPort.longiVelocity_mps",
                        "AP.odoEstimationPortCM.steerAngFrontAxle_rad",
                        "AP.odoEstimationPort.motionStatus_nu",
                        "AP.odoEstimationPort.yawRate_radps",
                        "AP.odoEstimationPort.pitchAngle_rad",
                        "AP.odoInputPort.odoExtCtrlPort.resetPoseEstimation_nu",
                        'AP.odoInputPort.odoSigFcanPort.vehDynamics.timestamp_us',
                        "PT.GearBox.GearNo",
                        "AP.odoInputPort.odoSigFcanPort.steerCtrlStatus.steeringWheelAngle_rad",
                        "AP.odoInputPort.odoSigFcanPort.steerCtrlStatus.steeringWheelAngleVelocity_radps",
                        "AP.odoInputPort.odoSigFcanPort.vehDynamics.lateralAcceleration_mps2",
                        "AP.odoInputPort.odoSigFcanPort.vehDynamics.longitudinalAcceleration_mps2",
                        "AP.odoInputPort.odoSigFcanPort.vehDynamics.yawRate_radps",
                        "Car.YawRate",
                        "Car.Road.Route.LatSlope",
                        "Car.Road.Route.LongSlope",
                        "AP.odoInputPort.odoSigFcanPort.wheelMeasurements.wheelSpeed.wheelSpeedFL_radps",
                        "Vhcl.FL.rot",
                        "AP.planningCtrlPort.apStates",
                        "AP.steeringWheelAngleAcceleration",
                        "Car.ay",
                        "Car.ax",
                        "Car.vx",
                        "Vhcl.v",
                        "AP.taposdDebugPort.yawDiffToTarget_rad",
                        "AP.taposdDebugPort.longDistToTarget_m",
                        "AP.taposdDebugPort.latDistToTarget_m",
                        "AP.targetPosesPort.selectedPoseData.reachedStatus",
                        "Vhcl.YawRate",
                        "Time"]

requiredQuantsVEDODO_MTS = {
                        'CAN_RangeHunterPosLocalX':'Reference_CAN.RT-Range_2017_02_28_SystemC.RangeHunterPosLocal.RangeHunterPosLocalX',
                        'CAN_RangeHunterPosLocalY':'Reference_CAN.RT-Range_2017_02_28_SystemC.RangeHunterPosLocal.RangeHunterPosLocalY',
                        'CAN_AngleHeading':'Reference_CAN.NAVconfig_AP_Passat_154_28-02-2017.HeadingPitchRoll.AngleHeading',
                        'CAN_VelForward': 'Reference_CAN.NAVconfig_AP_Passat_154_28-02-2017.VelocityLevel.VelForward',
                        'CAN_yawRateEgoCur_rad':'Reference_CAN.NAVconfig_AP_Passat_154_28-02-2017.RateVehicle.AngRateZ',
                        'xPosition_m':'SIM VFB.VEDODO.OdoEstimationPort.xPosition_m',
                        'yPosition_m':'SIM VFB.VEDODO.OdoEstimationPort.yPosition_m',
                        'yawAngle_rad':'SIM VFB.VEDODO.OdoEstimationPort.yawAngle_rad',
                        'vehVelocityCur_mps':'SIM VFB.VEDODO.OdoEstimationPort.vehVelocityCur_mps',
                        'steerAngCur_rad':'SIM VFB.VEDODO.OdoEstimationPort.steerAngCur_rad',
                        'motionStatus_nu':'SIM VFB.VEDODO.OdoEstimationPort.motionStatus_nu',
                        'timestamp_us':'SIM VFB.VEDODO.OdoEstimationPort.timestamp_us',
                        'HMI_HeadUnit': 'ADC426_AlgoData.TestView.HMIOutputPort_t.userActionHeadUnit_nu',
                        'yawRateEgoCur_rad':'SIM VFB.VEDODO.OdoEstimationPort.yawRateEgoCur_rad'
                        }

requiredQuantsVEDODO_HIL_MTS = {
                        'CAN_RangeHunterPosLocalX':'Reference_CAN.RT-Range_2017_02_28_SystemC.RangeHunterPosLocal.RangeHunterPosLocalX',
                        'CAN_RangeHunterPosLocalY':'Reference_CAN.RT-Range_2017_02_28_SystemC.RangeHunterPosLocal.RangeHunterPosLocalY',
                        'CAN_AngleHeading':'Reference_CAN.NAVconfig_AP_Passat_154_28-02-2017.HeadingPitchRoll.AngleHeading',
                        'CAN_VelForward':'Reference_CAN.NAVconfig_AP_Passat_154_28-02-2017.VelocityLevel.VelForward',
                        'CAN_yawRateEgoCur_rad':'Reference_CAN.NAVconfig_AP_Passat_154_28-02-2017.RateVehicle.AngRateZ',
                        'xPosition_m':'ADC426_AlgoData.TestView.OdoEstimationPort.xPosition_m',
                        'yPosition_m':'ADC426_AlgoData.TestView.OdoEstimationPort.yPosition_m',
                        'yawAngle_rad':'ADC426_AlgoData.TestView.OdoEstimationPort.yawAngle_rad',
                        'vehVelocityCur_mps':'ADC426_AlgoData.TestView.OdoEstimationPort.vehVelocityCur_mps',
                        'steerAngCur_rad':'ADC426_AlgoData.TestView.OdoEstimationPort.steerAngCur_rad',
                        'motionStatus_nu':'ADC426_AlgoData.TestView.OdoEstimationPort.motionStatus_nu',
                        'timestamp_us':'ADC426_AlgoData.TestView.OdoEstimationPort.timestamp_us',
                        'HMI_HeadUnit': 'ADC426_AlgoData.TestView.HMIOutputPort_t.userActionHeadUnit_nu',
                        'yawRateEgoCur_rad':'ADC426_AlgoData.TestView.OdoEstimationPort.yawRateEgoCur_rad'}

#for VEDODO
enable_VEDODO_sliding_windows = 0
x_y_graph = {}
x_y_graph_info = {}
x_y_CAN_graph_info = {}


# ==================================================================================
# calculate virtual signals for VEDODO
# ==================================================================================
def VEDODO_Signals(curQuantDict, pool, time):
    # search for the time when Screen HeadUnit = ManeuverActive and Screen HeadUnit = ManeuverFunished
    maxOdoErrX_m         = 0
    maxOdoErrY_m         = 0
    maxOdoErrYaw_deg     = 0
    t_collision_detected = None
    t_idx_coll_dtct      = None
    t_func_activated     = None
    t_idx_func_activated = None
    t_func_stopped       = None
    t_idx_func_stopped   = None
    IGNORED_SAMPLES      = 3
    virtualSignals       = []  # virtual signals to be copied in mdfs
    index                = 0
    if len(curQuantDict['AP.headUnitVisualizationPort.screen_nu']) > IGNORED_SAMPLES:
        startSample = IGNORED_SAMPLES  # ignore first samples, might not be relevant
    else:
        startSample = None
    for value in list(curQuantDict['AP.headUnitVisualizationPort.screen_nu'][startSample:]):
        if value == PARKSM_SHU_ManeuverActive:
            t_idx_func_activated = index
            break
        index = index + 1
    index = 0
    # ignore the first 3 samples, which if might not be relevant
    for value in list(curQuantDict['AP.headUnitVisualizationPort.screen_nu'][startSample:]):
        if value == PARKSM_SHU_ManeuverFinished:
            t_idx_func_stopped = index
            break
        index = index + 1

    if t_idx_func_stopped != None and t_idx_func_activated != None:
        if t_idx_func_stopped < t_idx_func_activated:
            t_idx_func_stopped = None
    dummyArray = np.array([0.0])

    # translate the signals to start from 0
    translate_signals = {"AP.odoDebugPort.odoCmRefxPosition_m": "AP.odoDebugPort.odoCmRefxPosition_m_VS", \
                         "AP.odoEstimationPort.xPosition_m": "AP.odoEstimationPort.xPosition_m_VS", \
                         "AP.odoDebugPort.odoCmRefyPosition_m": "AP.odoDebugPort.odoCmRefyPosition_m_VS", \
                         "AP.odoEstimationPort.yPosition_m": "AP.odoEstimationPort.yPosition_m_VS", \
                         "AP.odoDebugPort.odoCmRefyawAngEgoRaCur_rad": "AP.odoDebugPort.odoCmRefyawAngEgoRaCur_rad_VS", \
                         "AP.odoEstimationPort.yawAngle_rad": "AP.odoEstimationPort.yawAngle_rad_VS"}

    for signal in translate_signals.keys():
        offset = curQuantDict[signal][IGNORED_SAMPLES]  # ignore the first 3 samples, which might not be relevant
        signal_translated = []
        for value in curQuantDict[signal][IGNORED_SAMPLES:]:
            signal_translated.append(value - offset)
        translated = Signal(samples=signal_translated,
                            timestamps=np.delete(time, [0, 1, 2]),
                            name=translate_signals[signal],
                            unit='m')
        virtualSignals.append(translated)
    YawRate_Diff = []
    for item in range(len(list(curQuantDict["AP.odoEstimationPort.yawRate_radps"]))):
        Odo_yawRateEgoCur_deg = np.around(
            np.multiply(curQuantDict["AP.odoEstimationPort.yawRate_radps"][item], 180.0 / math.pi), 2)
        CM_YawRate_deg = np.around(np.multiply(curQuantDict["Car.YawRate"][item], 180.0 / math.pi), 2)
        YawRate_Diff.append(np.abs(Odo_yawRateEgoCur_deg - CM_YawRate_deg))
    YawRate_signal = Signal(samples=YawRate_Diff,
                            timestamps=time,
                            name="YawRate_Diff_deg_VS",
                            unit="rad")
    virtualSignals.append(YawRate_signal)
    # Calculate Odometry Error
    absErrLong_m, absErrLat_m, relErrLong_perc, relErrLat_perc, absErrYaw_rad, drivenDist_arr_m, relErrYaw = CalcOdoError(
        curQuantDict['AP.odoDebugPort.odoCmRefxPosition_m'],
        curQuantDict['AP.odoDebugPort.odoCmRefyPosition_m'],
        curQuantDict['AP.odoDebugPort.odoCmRefyawAngEgoRaCur_rad'],
        curQuantDict['AP.odoEstimationPort.yawAngle_rad'],
        curQuantDict['AP.odoEstimationPort.xPosition_m'],
        curQuantDict['AP.odoEstimationPort.yPosition_m'])
    # convert the Yaw in degree
    absErrYaw_deg = np.around(np.multiply(absErrYaw_rad,
                                          180.0 / math.pi), 5)
    if enable_VEDODO_sliding_windows:
        # dict with Key - return array of maxError_distWindow function and value - arguments for the function call
        params_maxError_distWindow = {'latErr_15m_VS': (drivenDist_arr_m, absErrLat_m, 15),
                                      'absErrYaw_15m_VS': (drivenDist_arr_m, absErrYaw_deg, 15),
                                      'longErr_15m_VS': (drivenDist_arr_m, absErrLong_m, 15),
                                      'longErr_0_5m_VS': (drivenDist_arr_m, absErrLong_m, 0.5),
                                      'latErr_0_5m_VS': (drivenDist_arr_m, absErrLat_m, 0.5),
                                      'absErrYaw_0_5m_VS': (drivenDist_arr_m, absErrYaw_deg, 0.5)}
    maxOdoErrX_m = np.around(np.amax(np.abs(absErrLong_m[t_idx_func_activated:t_idx_func_stopped])), 3)
    maxOdoErrY_m = np.around(np.amax(np.abs(absErrLat_m[t_idx_func_activated:t_idx_func_stopped])), 3)
    maxOdoErrYaw_deg = np.around(np.amax(absErrYaw_deg[t_idx_func_activated:t_idx_func_stopped]), 3)
    # print ("Len absErrLong_m, absErrLat_m, relErrLong_perc, relErrLat_perc, absErrYaw_rad, drivenDist_arr_m, relErrYaw", absErrLong_m, absErrLat_m, relErrLong_perc, relErrLat_perc, absErrYaw_rad, drivenDist_arr_m, relErrYaw)
    RMSE_Long_m = np.around(math.sqrt(
        np.divide(np.sum(np.square(absErrLong_m[t_idx_func_activated:t_idx_func_stopped])),
                  len(drivenDist_arr_m[t_idx_func_activated:t_idx_func_stopped]))), 4)
    RMSE_Lat_m = np.around(math.sqrt(
        np.divide(np.sum(np.square(absErrLat_m[t_idx_func_activated:t_idx_func_stopped])),
                  len(drivenDist_arr_m[t_idx_func_activated:t_idx_func_stopped]))), 4)
    RMSE_Yaw_deg = np.around(math.sqrt(
        np.divide(np.sum(np.square(absErrYaw_deg[t_idx_func_activated:t_idx_func_stopped])),
                  len(drivenDist_arr_m[t_idx_func_activated:t_idx_func_stopped]))), 5)
    if enable_VEDODO_sliding_windows:
        # delete the first samples, since the values are not relevant
        # delete the last value to have the same lenght as the calculated maximum errors using maxError_distWindow function
        L = len(time)
        timeArray = np.delete(time, [0, 1, 2, L - 1])
        results = {}
        for each_thread in params_maxError_distWindow.keys():  # make the calculation using multiprocessing
            result = pool.apply_async(maxError_distWindow, params_maxError_distWindow[each_thread])
            results[each_thread] = result

        for i in results.keys():
            res_th = results[i].get()
            res_th = np.delete(res_th, [0, 1, 2])  # delete first samples, with big values because are not relevant
            if 'yaw' in str(i):
                unit = 'deg'
            else:
                unit = 'm'
            th_VS = Signal(samples=res_th,
                           timestamps=timeArray,
                           name=str(i),
                           unit=unit)
            virtualSignals.append(th_VS)

    absErrYaw_deg_VS = Signal(samples=absErrYaw_deg,
                              timestamps=time,
                              name='absErrYaw_deg_VS',
                              unit='degree')
    absErrLong_m_VS = Signal(samples=np.delete(np.abs(absErrLong_m), [0, 1, 2, 3]),
                             timestamps=np.delete(time, [0, 1, 2, 3]),
                             name='absErrLong_m_VS',
                             unit='m')
    absErrLat_m_VS = Signal(samples=np.delete(np.abs(absErrLat_m), [0, 1, 2, 3]),
                            timestamps=np.delete(time, [0, 1, 2, 3]),
                            name='absErrLat_m_VS',
                            unit='m')
    # dummy signals, used to compare the results obtained with STET vs Py
    maxOdoErrX_m_Py = Signal(samples=np.array([maxOdoErrX_m] * len(time)),
                             timestamps=time,
                             name='maxOdoErrX_m_Py',
                             unit='m')
    maxOdoErrY_m_py = Signal(samples=np.array([maxOdoErrY_m] * len(time)),
                             timestamps=time,
                             name='maxOdoErrY_m_py',
                             unit='m')
    maxOdoErrYaw_deg_py = Signal(samples=np.array([maxOdoErrYaw_deg] * len(time)),
                                 timestamps=time,
                                 name='maxOdoErrYaw_deg_py',
                                 unit='m')
    RMSE_Long_m_Py = Signal(samples=np.array([RMSE_Long_m] * len(time)),
                            timestamps=time,
                            name='RMSE_Long_m_Py',
                            unit='m')
    RMSE_Lat_m_Py = Signal(samples=np.array([RMSE_Lat_m] * len(time)),
                           timestamps=time,
                           name='RMSE_Lat_m_Py',
                           unit='m')
    RMSE_Yaw_deg_Py = Signal(samples=np.array([RMSE_Yaw_deg] * len(time)),
                             timestamps=time,
                             name='RMSE_Yaw_deg_Py',
                             unit='deg')
    relErrLong_perc_VS = Signal(samples=relErrLong_perc,
                                timestamps=time,
                                name='relErrLong_perc_VS',
                                unit='%')
    drivenDist_arr_m_VS = Signal(samples=drivenDist_arr_m,
                                 timestamps=time,
                                 name='drivenDist_arr_m_VS',
                                 unit='m')
    relErrLat_perc_VS = Signal(samples=relErrLat_perc,
                               timestamps=time,
                               name='relErrLat_perc_VS',
                               unit='%')

    virtualSignals.append(relErrLat_perc_VS)
    virtualSignals.append(relErrLong_perc_VS)
    virtualSignals.append(drivenDist_arr_m_VS)
    virtualSignals.append(absErrYaw_deg_VS)
    virtualSignals.append(absErrLong_m_VS)
    virtualSignals.append(absErrLat_m_VS)
    virtualSignals.append(maxOdoErrX_m_Py)
    virtualSignals.append(maxOdoErrY_m_py)
    virtualSignals.append(maxOdoErrYaw_deg_py)
    virtualSignals.append(RMSE_Long_m_Py)
    virtualSignals.append(RMSE_Lat_m_Py)
    virtualSignals.append(RMSE_Yaw_deg_Py)
    return virtualSignals

# ==========================================================================================
# Create an mdf which will be used for evaluation with STET - will contain:
# - signals from the corresponding erg file
# - virtual signals with calculations which cannot be done with STET, using signals from bsig
# ==========================================================================================
def createMDF(ergFilePath, requiredQuants, virtualSignals, testCaseSet = None):
    mdf_signals = []
    if testCaseSet == None: #VEDODO reports for MTS bsig files
        #path_measurements = os.path.abspath(os.path.join(ergFilePath, '..'))
        measurement = Measurement(ergFilePath)
    else:
        measurement = ERG(ergFilePath)
        #path_measurements = os.path.join(PATH_measurements, testCaseSet)

    path_measurements = os.path.abspath(os.path.join(ergFilePath, '..'))
    #Add the signals in requiredQuants in the mdf measurement
    for name in requiredQuants:
        mdf_signals.append(measurement.get(name))

    measurementList = [os.path.join(path_measurements, x) for x in os.listdir(path_measurements)]
    new_mdf_path = os.path.join(ergFilePath.split('.erg')[0])
    #remove existing .mf4 files with the same name as the one which will be created.
    for measurement in measurementList:
        if new_mdf_path in measurement and measurement.endswith(".mf4"):
            try:
                os.remove(measurement)
            except:
                pass
    new_mdf = MDF4()
    new_mdf.append(mdf_signals)
    new_mdf.append(virtualSignals)
    new_mdf.save(new_mdf_path)

# ====================================================================================================
# identify the plots which needs to have xy graphics
# ====================================================================================================
def identify_x_y_plots(sheet_name, path_excel, function):
    col_graph_info = 0
    row_graph_info = 0
    testcases = {}
    testcases[function] = []
    wb        = xlrd.open_workbook(path_excel)
    sh        = wb.sheet_by_name(sheet_name)
    '''find the column with the relevant information (which contains X/Y Graphics [function name])'''
    for i in range(sh.nrows):
        row = sh.row(i)
        for j in range(sh.ncols):
            if row[j].value.strip() == 'X/Y Graphics ' + function:
                col_graph_info = j
                break
        if col_graph_info:
            row_graph_info = i
            break
    else:
        raise EvaluateError('Invalid excel export: could not parse keywords')
    '''update the list in the dictionary with the graph number read from the excel file'''
    for i in range(row_graph_info+1,sh.nrows):
        row = sh.row(i)
        cell_value = row[col_graph_info].value.strip()
        if cell_value!='end':
            if cell_value != '':
                testcases[function].append(cell_value)
        else:
            break
    return testcases

# ==================================================================================
# calculate virtual signals for VEDODO, to be used for open loop tests
# call createMDF(ergFilePath, requiredQuantsVEDODO, virtualSignals)
# ==================================================================================
def VEDODO_Signals_MTS(ergFilePath, requiredQuantsVEDODO):
    import report
    from report.html.pyHTMLReport.htmlReport import cHTMLReportGraphYData, cHTMLReportGraph, cHTMLReportVerticalLine
    t_idx_func_activated = None
    t_func_stopped = None
    t_idx_func_stopped = None
    measurementList = [os.path.join(ergFilePath, x) for x in os.listdir(ergFilePath)]
    for measurement in measurementList:
        if measurement.endswith('mf4'):
            try:
                os.remove(measurement)
            except:
                pass
    rawTestrunList = os.listdir(ergFilePath)
    testrunList = []
    cpus = multiprocessing.cpu_count()
    pool = Pool (processes = cpus)
    print("Start mdf generation...")
    for f in rawTestrunList:
        if os.path.isfile(os.path.join(ergFilePath,f)):
            print("file", f)
            if (f.endswith('.bsig') is True):
                testrunList.append({"name":f.replace('.bsig', ''), "path": os.path.join(ergFilePath,f)})
    for curRun in testrunList:
        virtualSignals = []  # virtual signals to be copied in mdfs
        curName = curRun["name"]
        curErgFilePath = curRun["path"]
        logging.info('Creating mf4 file for --> {0}'.format(curName))
        # Extract the data from the erg files
        curQuantDict = {}
        ParsingSuccessful = False
        #Export the BSIG files to mdf format
        BSIG(curErgFilePath).export_mdf().save(curErgFilePath + ".mf4")
        try:
            Signals = load_data(curErgFilePath + ".mf4", list(requiredQuantsVEDODO.values()))
            ParsingSuccessful = True
            t_idx_func_activated = None
            t_idx_func_stopped = None
        except:
            logging.warning('Parsing erg-file aborted and failed for {0}'.format(curName))
        if ParsingSuccessful is True:
            #logging.info('...done. Reading quantities...')
            for quant in requiredQuantsVEDODO.values():
                # store all values as numpy array
                curQuantDict[quant] = np.array(Signals[quant].signal)

            (SIM_xEgoRACur_m, SIM_yEgoRACur_m, SIM_yawAngEgoCur_rad) = coordinate_transformation(
                        curQuantDict[requiredQuantsVEDODO['CAN_RangeHunterPosLocalX']],
                        curQuantDict[requiredQuantsVEDODO['xEgoRACur_m']],
                        curQuantDict[requiredQuantsVEDODO['CAN_RangeHunterPosLocalY']],
                        curQuantDict[requiredQuantsVEDODO['yEgoRACur_m']],
                        np.deg2rad(360 - curQuantDict[requiredQuantsVEDODO['CAN_AngleHeading']]),
                        curQuantDict[requiredQuantsVEDODO['yawAngEgoCur_rad']],\
                        curQuantDict[requiredQuantsVEDODO['timestamp_us']],\
                        curQuantDict[requiredQuantsVEDODO['HMI_HeadUnit']])
            # Calculate Odometry Error
            absErrLong_m, absErrLat_m, relErrLong_perc, relErrLat_perc, absErrYaw_rad, drivenDist_arr_m, relErrYaw = CalcOdoError(
                curQuantDict[requiredQuantsVEDODO['xEgoRACur_m']],
                curQuantDict[requiredQuantsVEDODO['yEgoRACur_m']],
                curQuantDict[requiredQuantsVEDODO['yawAngEgoCur_rad']],
                SIM_yawAngEgoCur_rad,
                SIM_xEgoRACur_m,
                SIM_yEgoRACur_m)
            # convert the Yaw in degree
            absErrYaw_deg = np.around(np.multiply(absErrYaw_rad,
                                                  180.0 / math.pi), 5)
            if enable_VEDODO_sliding_windows:
                #dict with Key - return array of maxError_distWindow function and value - arguments for the function call
                params_maxError_distWindow = {'latErr_15m_VS' : (drivenDist_arr_m, absErrLat_m , 15),
                                              'absErrYaw_15m_VS':(drivenDist_arr_m, absErrYaw_deg, 15),
                                              'longErr_15m_VS':(drivenDist_arr_m, absErrLong_m , 15),
                                              'longErr_0_5m_VS': (drivenDist_arr_m, absErrLong_m, 0.5),
                                              'latErr_0_5m_VS': (drivenDist_arr_m, absErrLat_m , 0.5),
                                              'absErrYaw_0_5m_VS':(drivenDist_arr_m,absErrYaw_deg, 0.5)}
            maxOdoErrX_m = np.around(np.amax(np.abs(absErrLong_m[t_idx_func_activated:t_idx_func_stopped])), 3)
            maxOdoErrY_m = np.around(np.amax(np.abs(absErrLat_m[t_idx_func_activated:t_idx_func_stopped])), 3)
            maxOdoErrYaw_deg = np.around(np.amax(absErrYaw_deg[t_idx_func_activated:t_idx_func_stopped]), 3)
            #print ("Len absErrLong_m, absErrLat_m, relErrLong_perc, relErrLat_perc, absErrYaw_rad, drivenDist_arr_m, relErrYaw", absErrLong_m, absErrLat_m, relErrLong_perc, relErrLat_perc, absErrYaw_rad, drivenDist_arr_m, relErrYaw)
            RMSE_Long_m = np.around(math.sqrt(
                np.divide(np.sum(np.square(absErrLong_m[t_idx_func_activated:t_idx_func_stopped])),
                          len(drivenDist_arr_m[t_idx_func_activated:t_idx_func_stopped]))), 4)
            RMSE_Lat_m = np.around(math.sqrt(
                np.divide(np.sum(np.square(absErrLat_m[t_idx_func_activated:t_idx_func_stopped])),
                          len(drivenDist_arr_m[t_idx_func_activated:t_idx_func_stopped]))), 4)
            RMSE_Yaw_deg = np.around(math.sqrt(
                np.divide(np.sum(np.square(absErrYaw_deg[t_idx_func_activated:t_idx_func_stopped])),
                          len(drivenDist_arr_m[t_idx_func_activated:t_idx_func_stopped]))), 5)
            timeArray = Signals[list(requiredQuantsVEDODO.values())[0]].time
            #print("len time", len(timeArray))
            if enable_VEDODO_sliding_windows:
                #delete the first samples, since the values are not relevant
                #delete the last value to have the same lenght as the calculated maximum errors using maxError_distWindow function
                L = len(Signals[list(requiredQuantsVEDODO.values())[0]].time)
                timeArray_reduced = np.delete(timeArray, [0,1, 2, L-1])
                results = {}
                for each_thread in params_maxError_distWindow.keys():
                    result = pool.apply_async(maxError_distWindow, params_maxError_distWindow[each_thread])
                    results[each_thread] = result

                for i in results.keys():
                    res_th = results[i].get()
                    res_th = np.delete(res_th, [0, 1, 2]) #delete first samples, with big values because are not relevant
                    if 'yaw' in str(i):
                        unit = 'deg'
                    else:
                        unit = 'm'
                    th_VS = Signal(samples = res_th,
                                        timestamps = timeArray_reduced,
                                        name = str(i),
                                        unit = unit)
                    virtualSignals.append(th_VS)

            x_y_graph[curName] = timeArray
            x_y_graph_info[curName] = [timeArray, SIM_xEgoRACur_m, SIM_yEgoRACur_m]
            x_y_CAN_graph_info[curName] = [timeArray, curQuantDict[requiredQuantsVEDODO['CAN_RangeHunterPosLocalX']],\
                                       curQuantDict[requiredQuantsVEDODO['CAN_RangeHunterPosLocalY']]]
            x_yEgoRACur_m_VS = Signal(samples = SIM_yEgoRACur_m,
                                       timestamps = timeArray,
                                       name = 'x_yEgoRACur_m_VS',
                                       unit = 'm')
            SIM_xEgoRACur_m_VS = Signal(samples = SIM_xEgoRACur_m,
                                       timestamps = timeArray,
                                       name = 'Translated_Ref_xEgoRACur_m_VS',
                                       unit = 'm')
            SIM_yEgoRACur_m_VS = Signal(samples = SIM_yEgoRACur_m,
                                       timestamps = timeArray,
                                       name = 'Translated_Ref_yEgoRACur_m_VS',
                                       unit = 'm')
            SIM_yawAngEgoCur_rad_VS = Signal(samples=SIM_yawAngEgoCur_rad,
                                       timestamps=timeArray,
                                       name='Translated_Ref_yawAngEgoCur_rad_VS',
                                       unit='rad')
            absErrYaw_deg_VS = Signal (samples = absErrYaw_deg,
                                       timestamps = timeArray,
                                       name = 'absErrYaw_deg_VS',
                                       unit = 'degree')
            absErrLong_m_VS = Signal (samples = np.delete(np.abs(absErrLong_m), [0,1,2,3]),
                                       timestamps = np.delete(timeArray, [0,1,2,3]),
                                       name = 'absErrLong_m_VS',
                                       unit = 'm')
            absErrLat_m_VS = Signal (samples =  np.delete(np.abs(absErrLat_m),[0,1,2,3]),
                                       timestamps = np.delete(timeArray, [0,1,2,3]),
                                       name = 'absErrLat_m_VS',
                                       unit = 'm')
            #dummy signals, used to compare the results obtained with STET vs Py
            maxOdoErrX_m_Py = Signal(samples =  np.array([maxOdoErrX_m]*len(timeArray)),
                                       timestamps = timeArray,
                                       name = 'maxOdoErrX_m_Py',
                                       unit = 'm')
            maxOdoErrY_m_py = Signal(samples =  np.array([maxOdoErrY_m]*len(timeArray)),
                                       timestamps = timeArray,
                                       name = 'maxOdoErrY_m_py',
                                       unit = 'm')
            maxOdoErrYaw_deg_py = Signal(samples =  np.array([maxOdoErrYaw_deg]*len(timeArray)),
                                       timestamps = timeArray,
                                       name = 'maxOdoErrYaw_deg_py',
                                       unit = 'm')
            RMSE_Long_m_Py = Signal(samples =  np.array([RMSE_Long_m]*len(timeArray)),
                                       timestamps = timeArray,
                                       name = 'RMSE_Long_m_Py',
                                       unit = 'm')
            RMSE_Lat_m_Py = Signal(samples =  np.array([RMSE_Lat_m]*len(timeArray)),
                                       timestamps = timeArray,
                                       name = 'RMSE_Lat_m_Py',
                                       unit = 'm')
            RMSE_Yaw_deg_Py = Signal(samples =  np.array([RMSE_Yaw_deg]*len(timeArray)),
                                       timestamps = timeArray,
                                       name = 'RMSE_Yaw_deg_Py',
                                       unit = 'deg')
            relErrLong_perc_VS = Signal(samples = relErrLong_perc,
                                       timestamps = timeArray,
                                       name = 'relErrLong_perc_VS',
                                       unit = '%')
            drivenDist_arr_m_VS = Signal(samples = drivenDist_arr_m,
                                       timestamps = timeArray,
                                       name = 'drivenDist_arr_m_VS',
                                       unit = 'm')
            relErrLat_perc_VS = Signal(samples = relErrLat_perc,
                                       timestamps = timeArray,
                                       name = 'relErrLat_perc_VS',
                                       unit = '%')
            virtualSignals.append(x_yEgoRACur_m_VS)
            virtualSignals.append(SIM_yawAngEgoCur_rad_VS)
            virtualSignals.append(SIM_xEgoRACur_m_VS)
            virtualSignals.append(SIM_yEgoRACur_m_VS)
            virtualSignals.append(relErrLat_perc_VS)
            virtualSignals.append(relErrLong_perc_VS)
            virtualSignals.append(drivenDist_arr_m_VS)
            virtualSignals.append(absErrYaw_deg_VS)
            virtualSignals.append(absErrLong_m_VS)
            virtualSignals.append(absErrLat_m_VS)
            virtualSignals.append(maxOdoErrX_m_Py)
            virtualSignals.append(maxOdoErrY_m_py)
            virtualSignals.append(maxOdoErrYaw_deg_py)
            virtualSignals.append(RMSE_Long_m_Py)
            virtualSignals.append(RMSE_Lat_m_Py)
            virtualSignals.append(RMSE_Yaw_deg_Py)
            createMDF(curErgFilePath+".mf4", list(requiredQuantsVEDODO.values()), virtualSignals)
    pool.close()
    logging.info('Creating mf4 files for VEDODO finished')


# ================================================================================================
# Transformation of the CAN coordinates in coordinates from Simulation for x, y axis and yaw Angle
# ================================================================================================
def coordinate_transformation(CAN_xEgoRACur_m, SIM_xEgoRACur_m, CAN_yEgoRACur_m, SIM_yEgoRACur_m,\
                              CAN_yawAngEgoCur_rad, SIM_yawAngEgoCur_rad, timestamp_us, HMI_HeadUnit):
    #calculate the offset using mean values of the first 50 samples
    start_index = 0
    end_index   = None
    TOGGLE_AP_ACTIVE = 28 #reset in the measurement
    time_window_50ms = 50 * 10000 # 50 ms
    # calculate the offset after the last reset in the rrec
    for i in range(len(HMI_HeadUnit)-1, 0, -1):
        if HMI_HeadUnit[i] == TOGGLE_AP_ACTIVE:
            start_index = i
            break
    for i in range(start_index, len(timestamp_us)):
        #end_index: the index after 50 ms elapsed starting start_index
        if timestamp_us[i] - timestamp_us[start_index] > time_window_50ms:
            end_index = i
            break
    SIM_yawAngEgoCur_rad_VS = np.array([])
    SIM_xEgoRACur_m_VS = np.array([])
    SIM_yEgoRACur_m_VS = np.array([])
    #calculate the offset as the mean values in 50 ms after the last reset
    offsetX_m     = np.mean(CAN_xEgoRACur_m[start_index:end_index]) - np.mean(SIM_xEgoRACur_m[start_index:end_index])
    offsetY_m     = np.mean(CAN_yEgoRACur_m[start_index:end_index]) - np.mean(SIM_yEgoRACur_m[start_index:end_index])
    offsetYaw_rad = -(np.mean(CAN_yawAngEgoCur_rad[start_index:end_index]) - np.mean(SIM_yawAngEgoCur_rad[start_index:end_index]))
    shift360 = 0
    for i in range(0, len(CAN_xEgoRACur_m)):
        yawAngEgoCur_rad = CAN_yawAngEgoCur_rad[i] + offsetYaw_rad
        #Start to substract 360 degree in the calculations, when the DGPS reaches the values >360 deg and it goes to 0.
        #Stop to substract 360 degree in the calculations, when the DGPS reaches the values 0 deg and it goes to 360.
        #Check if there is a jump considering
        if i>0:
            if -np.rad2deg(CAN_yawAngEgoCur_rad[i-1]) + np.rad2deg(CAN_yawAngEgoCur_rad[i]) > 357:
                shift360 = 1
            elif -np.rad2deg(CAN_yawAngEgoCur_rad[i]) + np.rad2deg(CAN_yawAngEgoCur_rad[i-1]) > 357:
                shift360 = 0
            if shift360 == 1:
                yawAngEgoCur_rad = yawAngEgoCur_rad - np.deg2rad(360)
        SIM_yawAngEgoCur_rad_VS = np.append(SIM_yawAngEgoCur_rad_VS, yawAngEgoCur_rad)
        oldOdoX_m = CAN_xEgoRACur_m[i]
        oldOdoY_m = CAN_yEgoRACur_m[i]
        SIM_xEgoRACur_m_VS = np.append(SIM_xEgoRACur_m_VS, oldOdoX_m * cos(offsetYaw_rad) - oldOdoY_m * sin(offsetYaw_rad) - (cos(offsetYaw_rad) * offsetX_m - sin(offsetYaw_rad) * offsetY_m))
        SIM_yEgoRACur_m_VS = np.append(SIM_yEgoRACur_m_VS, oldOdoX_m * sin(offsetYaw_rad) + oldOdoY_m * cos(offsetYaw_rad) - (sin(offsetYaw_rad) * offsetX_m + cos(offsetYaw_rad) * offsetY_m))
    return(SIM_xEgoRACur_m_VS, SIM_yEgoRACur_m_VS, SIM_yawAngEgoCur_rad_VS)


####################################################################################################
#graphic_number - 'G3'
#x_y_graph - empty dictionary with following info: Key - name of the measurement, value -  timeArray
####################################################################################################

def updategraph_x_y_all(x_y_graph, path_VEDODO, graphic_number):
    GRAPH_PATH = os.path.abspath(os.path.join(path_VEDODO,'graphData'))
    graph_name_start = 'graphData_' + graphic_number + '_'
    START_CHANGING = 'var GraphData' + graphic_number
    '''start strings which are used when updating the lines in java script files'''
    L_WINDOW = 'dateWindow:'
    UNIT = 'xlabel: '
    SIGNAL_NAMES = 'labels: [' + r"'" + r"'"
    COLORS = 'colors: ['
    IGNORED_SAMPLES = 5
    '''order of the signals'''
    signals_no = 0
    pos_x1 = 0
    pos_y1 = 1
    pos_x2 = 2
    pos_y2 = 3
    '''lists with the values of the signals'''
    x1 = []
    '''list of all js files created in graphData folder'''
    js_files = [f for f in listdir(GRAPH_PATH) if isfile(join(GRAPH_PATH, f))]
    replace_js = {}

    '''replace_js - dictionary with following info:
                    key - name of the measurement, value - path to the corresponding js file for the graph x,y'''
    '''perform the calculations only for the measurements not added in replace_js dictionary'''
    for measurement in list(x_y_graph.keys()):
         graph_name_end = measurement.split('.')[0]
         graph_name = graph_name_start + graph_name_end
         graph_file = None
         found = None
         for f in js_files:
             '''search for a graph which does not correspond to another measurement=> str(f) not in str(list(replace_js.values()))'''
             if str(graph_name) in str(f) and str(f) not in str(list(replace_js.values())):
                 graph_file = f
                 break
         '''if a graph was found, start the calculations'''
         if graph_file:
             graph_path_file = os.path.abspath(os.path.join(path_VEDODO, 'graphData', graph_file))
             output_path = os.path.abspath(os.path.join(path_VEDODO, 'graphData', str(measurement) + '.js'))
             index = 0
             with open(graph_path_file, 'r+') as fh, open(output_path, 'w') as fout:
                 start_updating = 0
                 second_graph_info = ''
                 substitute = ''
                 '''lists with the values of the signals'''
                 x1 = []
                 l = len(str(round(x_y_graph[measurement][index], 6)))
                 test = 0
                 for line in fh:
                     test = test + 1
                     new_line = line
                     '''x_y_graph - dictionary with follwing info:
                                 key - measurement name
                                 value - timeArray
                     replace the values of the time array with the information of the x coordinates
                     in order to do the replacement, the time info (search_string) must be found inside the line
                     if the values in the time array do not match, copy the line as it is'''
                     if START_CHANGING in str(line):
                         '''start the updates on the line following the one which contains START_CHANGING'''
                         start_updating = 1
                     if start_updating and index < len(x_y_graph[measurement]):
                         search_string = '[' + str(round(x_y_graph[measurement][index], 6))[0:l - 1]
                         '''new line must contain: [SIM_xEgoRACur_m[index],SIM_yEgoRACur_m[index],<null>]'''
                         if search_string in str(line):
                             index = index + 1
                             line_data_raw = line.replace('[', '')
                             line_data_raw = line_data_raw.replace('],', '')
                             line_data_raw = line_data_raw.replace('\n', '')
                             '''line_data - list with info: [time, x1, y1, x2, y2]'''
                             line_data = line_data_raw.split(',')
                             '''calculate the lenght of line_data to decide what info needs to be plotted'''
                             if index == 1:
                                 signals_no = len(line_data)
                             '''use every pair of xy signals and ignore the first signal which represents the time information'''
                             for signal_position in range(1,int((signals_no+1)/2)):
                                 x1.append(round(float(line_data[2*signal_position - 1]), 6))
                             if signals_no ==3 and index > IGNORED_SAMPLES:
                                 '''1 xy graph to be plotted: [time, x1, y1]'''
                                 substitute = '[' + str(round(float(line_data[pos_x1 + 1]), 6)) + ',' + \
                                              str(round(float(line_data[pos_y1 + 1]), 6)) + '],' + "\n"
                                 last_line = ''

                             elif signals_no==5 and index > IGNORED_SAMPLES:
                                 '''2 xy graphs to be plotted: [time, x1, y1, x2, y2]'''
                                 substitute = '[' + str(round(float(line_data[pos_x1 + 1]), 6)) + ',' + \
                                              str(round(float(line_data[pos_y1 + 1]), 6)) + ',null],' + "\n"
                                 last_line = '[' + str(round(float(line_data[pos_x2 + 1]), 6)) + ',null,' + \
                                             str(round(float(line_data[pos_y2 + 1]), 6)) + '],' + "\n"
                             elif signals_no == 3 or signals_no == 5 and index <= IGNORED_SAMPLES:
                                 '''ignore first samples'''
                                 substitute = ''
                                 last_line = ''
                             else:
                                 '''no xy plot to be showen'''
                                 substitute = line
                                 last_line = ''
                             second_graph_info = second_graph_info + last_line
                             if index == len(x_y_graph_info[measurement][0]):
                                 substitute = substitute + second_graph_info
                             new_line = substitute

                     elif L_WINDOW in str(line) and not found:
                         found = 1
                         if signals_no>2:
                            new_line = L_WINDOW + '[' + str(round(min(x1))) + ',' + str(round(max(x1))) + r"],\Z" + "\n"
                            new_line = new_line.replace("Z", "")
                         else:
                             new_line = line
                     elif SIGNAL_NAMES in str(line):
                         line_data_raw = line.replace('[', '')
                         line_data_raw = line_data_raw.replace('],', '')
                         line_data = line_data_raw.split(',')
                         if signals_no ==3:
                             new_line = (SIGNAL_NAMES + ',' + line_data[pos_y1+1] + '],' + r'\Z' + "\n").replace("Z", "")
                         elif signals_no ==5:
                            new_line = (SIGNAL_NAMES + ',' + line_data[pos_y1+1] + ',' + line_data[
                                 pos_y2+1] + '],' + r'\Z' + "\n").replace("Z", "")
                         else:
                             new_line = line
                     elif UNIT in str(line):
                         new_line = line.replace('t [s]', 'm')
                     elif COLORS in str(line):
                         '''line_data = line.split(',')
                         new_line = line.replace(line_data[1], '')
                         new_line = line.replace(line_data[2], '')'''
                         new_line = (r"colors: ['rgb(0,255,0)','rgb(255,0,0)',]," + r"\Z" + "\n").replace("Z", "")
                     fout.write(new_line)

                     '''if all the values of the time array were replaced by the info in SIM_xEgoRACur_m, update the dictionary'''
                     if index == len(x_y_graph_info[measurement][0]):
                         replace_js[os.path.splitext(os.path.basename(output_path))[0]] = graph_path_file
             fh.close()
             fout.close()

    for js_created in replace_js.keys():
        #original_renamed = os.path.join(replace_js[js_created],'original')
        os.rename(replace_js[js_created], replace_js[js_created] +'original')
        os.rename(os.path.join(GRAPH_PATH, js_created + '.js'), replace_js[js_created])
#####imported from AP_SimRunner.py#####
# ==================================================================================
# Calculate Odometry Error by using perpendicular point
# ==================================================================================

def CalcOdoError(x_GT, y_GT, Psi_GT, Psi_Odo, x_Odo, y_Odo):
    # Calculates longitudinal and lateral error

    leng = len(x_GT)
    drivenDist_m = 0.0
    drivenDist_arr_m = np.zeros(leng)
    absErrLong_m = np.zeros(leng)
    relErrLong_perc = np.zeros(leng)
    absErrLat_m = np.zeros(leng)
    relErrLat_perc = np.zeros(leng)
    absErrYaw_rad = np.zeros(leng)
    relErrYaw = np.zeros(leng)

    if len(x_GT) == len(y_GT) == len(Psi_GT) == len(x_Odo) == len(y_Odo) == len(Psi_Odo):
        for idx, val in np.ndenumerate(Psi_GT):
            P_GT = [[x_GT[idx]], [y_GT[idx]]]
            P_ODO = [[x_Odo[idx]], [y_Odo[idx]]]

            # calculate orthogonal point - Lotpunkt
            A = np.matrix([[math.cos(Psi_GT[idx[0]]), -math.sin(Psi_GT[idx[0]])],
                           [math.sin(Psi_GT[idx[0]]), math.cos(Psi_GT[idx[0]])]])

            b = np.subtract(P_ODO, P_GT)

            x = np.matmul(np.linalg.inv(A), b)

            LP = P_GT + np.multiply(x[0], [[math.cos(Psi_GT[idx])], [math.sin(Psi_GT[idx])]])

            # absolute error
            absErrLong_m[idx] = np.linalg.norm(np.subtract(LP, P_GT))
            absErrLat_m[idx] = np.linalg.norm(np.subtract(LP, P_ODO))
            absErrYaw_rad[idx] = np.abs(np.subtract(Psi_Odo[idx], Psi_GT[idx]))

            # driven distance
            if idx[0] > 0:
                delta_Dist_m = np.linalg.norm(np.subtract(P_GT, P_GT_LL))

                drivenDist_m = drivenDist_m + np.linalg.norm(np.subtract(P_GT, P_GT_LL))
                drivenDist_arr_m[idx] = drivenDist_m

            P_GT_LL = P_GT

            # relative error
            if drivenDist_m != 0.0:
                relErrLong_perc[idx] = absErrLong_m[idx] / drivenDist_m * 100.0
                relErrLat_perc[idx] = absErrLat_m[idx] / drivenDist_m * 100.0
                relErrYaw[idx] = absErrYaw_rad[idx] / drivenDist_m
            else:
                relErrLong_perc[idx] = 0.0
                relErrLat_perc[idx] = 0.0
                relErrYaw[idx] = 0.0

    else:
        logging.warning('Array dimensions do not match for CalcOdoError!')
    return absErrLong_m, absErrLat_m, relErrLong_perc, relErrLat_perc, absErrYaw_rad, drivenDist_arr_m, relErrYaw

# ==================================================================================
# Implementation of scipy.signal.savgol_filter
# ==================================================================================
def savitzky_golay(y, window_size, order, deriv=0, rate=1):
    try:
        window_size = np.abs(np.int(window_size))
        order = np.abs(np.int(order))
    except ValueError:
        raise ValueError("window_size and order have to be of type int")
    if window_size % 2 != 1 or window_size < 1:
        raise TypeError("window_size size must be a positive odd number")
    if window_size < order + 2:
        raise TypeError("window_size is too small for the polynomials order")
    order_range = range(order + 1)
    half_window = (window_size - 1) // 2
    # precompute coefficients
    b = np.mat([[k ** i for i in order_range] for k in range(-half_window, half_window + 1)])
    m = np.linalg.pinv(b).A[deriv] * rate ** deriv * math.factorial(deriv)
    # pad the signal at the extremes with
    # values taken from the signal itself
    firstvals = y[0] - np.abs(y[1:half_window + 1][::-1] - y[0])
    lastvals = y[-1] + np.abs(y[-half_window - 1:-1][::-1] - y[-1])
    y = np.concatenate((firstvals, y, lastvals))
    return np.convolve(m[::-1], y, mode='valid')

# ===============================================================================================================================================
# iterating  a dynamic sliding window with a default value of delta_dist along the distance array and get the max error value in every iteration
# ===============================================================================================================================================
def maxError_distWindow(r1, r2, delta_dist): #r1 - distance, r2 - error
    b = len(r1)
    list = []
    MaxValueYEachIt = np.array([])

    ##
    for i in range(b):
        for ind in range(i, b):
            if r1[ind] - r1[i] >= delta_dist:
                break

        if r1[ind] - r1[i] > delta_dist:
            ind_arr = ind - 1

        else:
            ind_arr = ind

        if i != ind_arr:
            list.append((i, ind_arr))
            MaxValueYEachIt = np.append(MaxValueYEachIt, np.amax(r2[i:ind_arr + 1]))

        elif abs(ind - ind_arr) == 1:
            list.append((i, ind))
            MaxValueYEachIt = np.append(MaxValueYEachIt, np.amax(r2[i:ind + 1]))

    return MaxValueYEachIt

##### End of - imported from AP_SimRunner.py#####
# ====================================================================================================
# identify the plots which needs to have xy graphics
# ====================================================================================================
def identify_x_y_plots(sheet_name, path_excel, function):
    col_graph_info = 0
    row_graph_info = 0
    testcases = {}
    testcases[function] = []
    wb        = xlrd.open_workbook(path_excel)
    sh        = wb.sheet_by_name(sheet_name)
    '''find the column with the relevant information (which contains X/Y Graphics [function name])'''
    for i in range(sh.nrows):
        row = sh.row(i)
        for j in range(sh.ncols):
            if row[j].value.strip() == 'X/Y Graphics ' + function:
                col_graph_info = j
                break
        if col_graph_info:
            row_graph_info = i
            break
    else:
        raise EvaluateError('Invalid excel export: could not parse keywords')
    '''update the list in the dictionary with the graph number read from the excel file'''
    for i in range(row_graph_info+1,sh.nrows):
        row = sh.row(i)
        cell_value = row[col_graph_info].value.strip()
        if cell_value!='end':
            if cell_value != '':
                testcases[function].append(cell_value)
        else:
            break
    return testcases