#====================================================================
# SYSTEM IMPORTS
#====================================================================
import os
import sys
import imp
import ctypes
import logging
#====================================================================
# OWN IMPORTS
#====================================================================
from . import stk_bsig

#====================================================================
# PARAMETER
#====================================================================
FCT_SIM_EXT_NO_TASK  = 0
FCT_SIM_EXT_VEH_TASK = 1
FCT_SIM_EXT_SEN_TASK = 2

devicePrefixes               = {"ARS": "ARS4xx Device",            "SMFC": "MFC4xx Device" }
deviceVehDynIdentifier       = {"ARS": "AlgoVehCycle",             "SMFC": "VDY"}
deviceSimVfbEmIdentifier     = {"ARS": "SIM VFB EM.DataProcCycle", "SMFC": "SIM VFB.EM"}
deviceSimVfbVehIdentifier    = {"ARS": "SIM VFB EM.AlgoVehCycle",  "SMFC": "SIM VFB.FCTVehicle"}

deviceIdentifierReplacementsSimVfb = [("pFCTVehOutput", {"ARS": "SIM VFB EM.AlgoVehCycle", "SMFC": "SIM VFB.FCTVehicle"}),\
                                      ("pFCTSenOutput", {"ARS": "SIM VFB EM.AlgoSenCycle", "SMFC": "SIM VFB.FCTSensor"})]

deviceIdentifierReplacementsDevice = [("pFCTVehOutput", {"ARS": "ARS4xx Device.AlgoVehCycle", "SMFC": "MFC4xx Device.FCTVehicle"}),\
                                      ("pFCTSenOutput", {"ARS": "ARS4xx Device.AlgoSenCycle", "SMFC": "MFC4xx Device.FCTSensor"})]

logging.basicConfig(level=logging.INFO, format='%(asctime)s %(message)s', datefmt='%d-%m-%Y %H:%M:%S')

class fct_sim_bsig_exception( Exception ) :
    def __init__(self, description):
        self.__description = str( description )
    def __str__( self ):
        self.errror_description = "=====================================================\n"
        self.errror_description += "ERROR: " + self.__description
        self.errror_description += "\n=====================================================\n"
        return str( self.errror_description )
    def Description(self):
        return self.__description

""" 
Abstract base class cFctBsigRunner
"""
class cFctBsigRunner(object):
    def __init__(self, device="ARS"):
        self._device              = device
        self._RequestedOutputURLs = []

    def AddRequestedOutputURL(self, outputValue):
        self._RequestedOutputURLs.append(outputValue)
        
    def Run(self, bsigFilePath):
        """
        for out_sig in self._RequestedOutputTypes:
            self._output[out_sig] = []
        return self._output
        """
        raise Exception('Do not call the abstract basis class!')

class cFctBsigRunnerDevice(cFctBsigRunner):
    def Run(self, bsigFilePath):
        #Load data from bsig
        bsig = stk_bsig.stkBsig()
        try:
            bsig.open(bsigFilePath)
        except stk_bsig.bsig_200_exception:
            logging.warning('This file is not a valid bsig! {0}'.format(currFile))
        
        # Retrieve output
        output = {}
        output['t_sim'] = bsig.get_signal_by_name('MTS.Package.TimeStamp')
        for currOutputValueURL in self._RequestedOutputURLs:
            # Compose valid Signal URL for Sim VFB Output in bsig
            modifiedValueURL = currOutputValueURL
            for currReplacement in deviceIdentifierReplacementsDevice:
                modifiedValueURL = modifiedValueURL.replace(currReplacement[0], currReplacement[1][self._device])
            modifiedValueURL = modifiedValueURL.replace("->", ".")

            output[currOutputValueURL] = bsig.get_signal_by_name(modifiedValueURL)
        return output

class cFctBsigRunnerSimVFB(cFctBsigRunner):
    def Run(self, bsigFilePath):
        #Load data from bsig
        bsig = stk_bsig.stkBsig()
        try:
            bsig.open(bsigFilePath)
        except stk_bsig.bsig_200_exception:
            logging.warning('This file is not a valid bsig! {0}'.format(currFile))
        
        # Retrieve output
        output = {}
        output['t_sim'] = bsig.get_signal_by_name('MTS.Package.TimeStamp')
        for currOutputValueURL in self._RequestedOutputURLs:
            # Compose valid Signal URL for Sim VFB Output in bsig
            modifiedValueURL = currOutputValueURL
            for currReplacement in deviceIdentifierReplacementsSimVfb:
                modifiedValueURL = modifiedValueURL.replace(currReplacement[0], currReplacement[1][self._device])
            modifiedValueURL = modifiedValueURL.replace("->", ".")

            output[currOutputValueURL] = bsig.get_signal_by_name(modifiedValueURL)
        return output

class cFctBsigRunnerResim(cFctBsigRunner):
    def __init__(self, fctSimExtDllPath, fctDataTypesPath, device="ARS"):
        super(cFctBsigRunnerResim, self).__init__(device)
        self._fctSimExtDllPath     = fctSimExtDllPath
        self._output               = {}
        
        # Import FCT Data Types
        module_folder = os.path.split(os.path.dirname( fctDataTypesPath ))[-1]
        moduleName, moduleExtension = os.path.splitext(os.path.split(fctDataTypesPath)[-1])
        moduleName_unique = moduleName + '_' + module_folder
        self._FctDataTypes = imp.load_source(moduleName_unique, fctDataTypesPath)
        
        # Dim Workaround
        self._dim_uiTimeStamp = 0

    def UnloadDll(self):
        notFound = True
        while notFound:
            try:
                _ctypes.FreeLibrary(self._fctSimExtDll._handle)
            except:
                notFound = False
        del self._fctSimExtDll
        self = None

    def UpdateDataBuffer( self, bsig ):
        for signal in self._subscribed_signals:
            self._data_buffer[signal] = bsig.get_signal_by_name(signal, Offset=self.curr_idx, SampleCount=self._data_buffer_length)
            
        self._data_buffer_curr_read_pos = 0

    def UpdateDIM( self, bsig ):
        """
        Comment: DIM signals exported to bsigs are very confuse. There are a lot more signals as needed for the algo.
                 The TimeStamp is not found in BSIG. Workaround is to build an own one.
        """
        self._dim_uiTimeStamp += 20*1000
        # Input Generic
        self.pFCTVehInput.contents.pDIMInputGeneric.contents.sSigHeader.uiTimeStamp = ctypes.c_uint32( self._dim_uiTimeStamp )
        
        self.pFCTVehInput.contents.pDIMInputGeneric.contents.fAccelPedalPos = ctypes.c_float( self.ReadData('{0}.DIM.DimInputData.Global.GasPedalPosition.uValue.fValue'.format(deviceSimVfbVehIdentifier[self._device])) )
        self.pFCTVehInput.contents.pDIMInputGeneric.contents.eAccelPadelStat = self._FctDataTypes.DIMInputSignalState_OK #self.ReadData('SIM VFB.FCTVehicle.DIM.DimInputData.Global.GasPedalPosition.eSignalQuality') )
        
        self.pFCTVehInput.contents.pDIMInputGeneric.contents.fAccelPedalGrad = ctypes.c_float( self.ReadData('{0}.DIM.DimInputData.Global.GasPedalGradient.uValue.fValue'.format(deviceSimVfbVehIdentifier[self._device])) )
        self.pFCTVehInput.contents.pDIMInputGeneric.contents.eAccelPadelGradStat = self._FctDataTypes.DIMInputSignalState_OK #ctypes.c_uint8( self.ReadData('SIM VFB.FCTVehicle.DIM.DimInputData.Global.GasPedalGradient.eSignalQuality') )
        
        self.pFCTVehInput.contents.pDIMInputGeneric.contents.fSteeringWheelAngle = ctypes.c_float( self.ReadData('{0}.DIM.DimInputData.Global.SteeringWheelAngle.uValue.fValue'.format(deviceSimVfbVehIdentifier[self._device])))
        self.pFCTVehInput.contents.pDIMInputGeneric.contents.eSteeringWheelAngleStat = self._FctDataTypes.DIMInputSignalState_OK #ctypes.c_uint8( self.ReadData('SIM VFB.FCTVehicle.DIM.DimInputData.Global.SteeringWheelAngle.eSignalQuality'))
        
        self.pFCTVehInput.contents.pDIMInputGeneric.contents.fSteeringWheelAngleGrad = ctypes.c_float( self.ReadData('{0}.DIM.DimInputData.Global.SteeringWheelGradient.uValue.fValue'.format(deviceSimVfbVehIdentifier[self._device])))
        self.pFCTVehInput.contents.pDIMInputGeneric.contents.eSteeringWheelAngleGradStat = self._FctDataTypes.DIMInputSignalState_OK #ctypes.c_uint8( self.ReadData('SIM VFB.FCTVehicle.DIM.DimInputData.Global.SteeringWheelGradient.eSignalQuality'))
        
        self.pFCTVehInput.contents.pDIMInputGeneric.contents.eDriverBraking = ctypes.c_uint8( self.ReadData('{0}.DIM.DimInputData.Global.DriverBraking.uValue.uiValue'.format(deviceSimVfbVehIdentifier[self._device])) )
        
        self.pFCTVehInput.contents.pDIMInputGeneric.contents.eTurnIndicator = ctypes.c_uint8( self.ReadData('{0}.DIM.DimInputData.Global.TurnIndicator.uValue.uiValue'.format(deviceSimVfbVehIdentifier[self._device])) )
        # not found in bsig FCT_pDIMGenericDataIn->eDriverSetting
        
        # Input Custom
        self.pFCTVehInput.contents.pDIMInputCustom.contents.sSigHeader.uiTimeStamp = ctypes.c_uint32( self._dim_uiTimeStamp )
        # @todo: this signal is used in algo, but not found in bsig: pFCTVehInput->pDIMCustDataIn->eSpeedLimitActive


    def ReadData( self, SignalName ):
        try:
            res = self._data_buffer[SignalName][self._data_buffer_curr_read_pos]
        except:
            # may not added to the databuffer yet
            if not SignalName in self._subscribed_signals:
                self._subscribed_signals.append(SignalName)
            
            try:
                res = self._bsig.get_signal_by_name(SignalName, Offset=self.curr_idx, SampleCount=1)[0]
            except:
                raise fct_sim_bsig_exception('Signal Name unknown: {0}'.format(SignalName) )
        return res

    def UpdateEgoVehicleData( self, bsig ):
        self.pFCTSenInput.contents.pEgoDynRaw.contents.uiVersionNumber = self._FctDataTypes.AlgoInterfaceVersionNumber_t( self.ReadData('{0}.{1}.VehDyn.uiVersionNumber'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )

        self.pFCTSenInput.contents.pEgoDynRaw.contents.sSigHeader.uiCycleCounter       = self._FctDataTypes.AlgoCycleCounter_t( self.ReadData('{0}.{1}.VehDyn.sSigHeader.uiCycleCounter'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )
        self.pFCTSenInput.contents.pEgoDynRaw.contents.sSigHeader.eSigStatus           = self._FctDataTypes.AlgoSignalState_t( self.ReadData('{0}.{1}.VehDyn.sSigHeader.eSigStatus'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )
        self.pFCTSenInput.contents.pEgoDynRaw.contents.sSigHeader.uiMeasurementCounter = self._FctDataTypes.AlgoCycleCounter_t( self.ReadData('{0}.{1}.VehDyn.sSigHeader.uiMeasurementCounter'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )
        self.pFCTSenInput.contents.pEgoDynRaw.contents.sSigHeader.uiTimeStamp          = self._FctDataTypes.AlgoDataTimeStamp_t( self.ReadData('{0}.{1}.VehDyn.sSigHeader.uiTimeStamp'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )

        self.pFCTSenInput.contents.pEgoDynRaw.contents.Longitudinal.MotVar.Velocity        = ctypes.c_float( self.ReadData('{0}.{1}.VehDyn.Longitudinal.MotVar.Velocity'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )
        self.pFCTSenInput.contents.pEgoDynRaw.contents.Longitudinal.MotVar.Accel           = ctypes.c_float( self.ReadData('{0}.{1}.VehDyn.Longitudinal.MotVar.Accel'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )
        self.pFCTSenInput.contents.pEgoDynRaw.contents.Longitudinal.MotVar.varVelocity     = ctypes.c_float( self.ReadData('{0}.{1}.VehDyn.Longitudinal.MotVar.varVelocity'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )
        self.pFCTSenInput.contents.pEgoDynRaw.contents.Longitudinal.MotVar.varAccel        = ctypes.c_float( self.ReadData('{0}.{1}.VehDyn.Longitudinal.MotVar.varAccel'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )
        self.pFCTSenInput.contents.pEgoDynRaw.contents.Longitudinal.VeloCorr.corrFact      = ctypes.c_float( self.ReadData('{0}.{1}.VehDyn.Longitudinal.VeloCorr.corrFact'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )
        self.pFCTSenInput.contents.pEgoDynRaw.contents.Longitudinal.VeloCorr.corrVar       = ctypes.c_float( self.ReadData('{0}.{1}.VehDyn.Longitudinal.VeloCorr.corrVar'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )
        self.pFCTSenInput.contents.pEgoDynRaw.contents.Longitudinal.VeloCorr.corrVelo      = ctypes.c_float( self.ReadData('{0}.{1}.VehDyn.Longitudinal.VeloCorr.corrVelo'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )
        self.pFCTSenInput.contents.pEgoDynRaw.contents.Longitudinal.VeloCorr.corrVeloVar   = ctypes.c_float( self.ReadData('{0}.{1}.VehDyn.Longitudinal.VeloCorr.corrVeloVar'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )
        self.pFCTSenInput.contents.pEgoDynRaw.contents.Longitudinal.VeloCorr.minVelo       = ctypes.c_float( self.ReadData('{0}.{1}.VehDyn.Longitudinal.VeloCorr.minVelo'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )
        self.pFCTSenInput.contents.pEgoDynRaw.contents.Longitudinal.VeloCorr.maxVelo       = ctypes.c_float( self.ReadData('{0}.{1}.VehDyn.Longitudinal.VeloCorr.maxVelo'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )
        self.pFCTSenInput.contents.pEgoDynRaw.contents.Longitudinal.VeloCorr.corrQual      = self._FctDataTypes.corrQual_t( self.ReadData('{0}.{1}.VehDyn.Longitudinal.VeloCorr.corrQual'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )
        self.pFCTSenInput.contents.pEgoDynRaw.contents.Longitudinal.AccelCorr.corrAccel    = ctypes.c_float( self.ReadData('{0}.{1}.VehDyn.Longitudinal.AccelCorr.corrAccel'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )
        self.pFCTSenInput.contents.pEgoDynRaw.contents.Longitudinal.AccelCorr.corrAccelVar = ctypes.c_float( self.ReadData('{0}.{1}.VehDyn.Longitudinal.AccelCorr.corrAccelVar'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )

        self.pFCTSenInput.contents.pEgoDynRaw.contents.Lateral.YawRate.YawRate         = ctypes.c_float( self.ReadData('{0}.{1}.VehDyn.Lateral.YawRate.YawRate'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )
        self.pFCTSenInput.contents.pEgoDynRaw.contents.Lateral.YawRate.Variance        = ctypes.c_float( self.ReadData('{0}.{1}.VehDyn.Lateral.YawRate.Variance'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )
        self.pFCTSenInput.contents.pEgoDynRaw.contents.Lateral.YawRate.Quality         = ctypes.c_float( self.ReadData('{0}.{1}.VehDyn.Lateral.YawRate.Quality'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )
        self.pFCTSenInput.contents.pEgoDynRaw.contents.Lateral.Curve.Curve             = ctypes.c_float( self.ReadData('{0}.{1}.VehDyn.Lateral.Curve.Curve'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )
        self.pFCTSenInput.contents.pEgoDynRaw.contents.Lateral.Curve.C1                = ctypes.c_float( self.ReadData('{0}.{1}.VehDyn.Lateral.Curve.C1'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )
        self.pFCTSenInput.contents.pEgoDynRaw.contents.Lateral.Curve.Gradient          = ctypes.c_float( self.ReadData('{0}.{1}.VehDyn.Lateral.Curve.Gradient'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )
        self.pFCTSenInput.contents.pEgoDynRaw.contents.Lateral.Curve.varC0             = ctypes.c_float( self.ReadData('{0}.{1}.VehDyn.Lateral.Curve.varC0'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )
        self.pFCTSenInput.contents.pEgoDynRaw.contents.Lateral.Curve.varC1             = ctypes.c_float( self.ReadData('{0}.{1}.VehDyn.Lateral.Curve.varC1'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )
        self.pFCTSenInput.contents.pEgoDynRaw.contents.Lateral.Curve.Quality           = ctypes.c_float( self.ReadData('{0}.{1}.VehDyn.Lateral.Curve.Quality'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )
        # Lateral.Curve. float32 CrvError; uint8 CrvConf;
        self.pFCTSenInput.contents.pEgoDynRaw.contents.Lateral.DrvIntCurve.Curve       = ctypes.c_float( self.ReadData('{0}.{1}.VehDyn.Lateral.DrvIntCurve.Curve'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )
        self.pFCTSenInput.contents.pEgoDynRaw.contents.Lateral.DrvIntCurve.Variance    = ctypes.c_float( self.ReadData('{0}.{1}.VehDyn.Lateral.DrvIntCurve.Variance'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )
        self.pFCTSenInput.contents.pEgoDynRaw.contents.Lateral.DrvIntCurve.Gradient    = ctypes.c_float( self.ReadData('{0}.{1}.VehDyn.Lateral.DrvIntCurve.Gradient'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )
        self.pFCTSenInput.contents.pEgoDynRaw.contents.Lateral.Accel.LatAccel          = ctypes.c_float( self.ReadData('{0}.{1}.VehDyn.Lateral.Accel.LatAccel'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )
        self.pFCTSenInput.contents.pEgoDynRaw.contents.Lateral.Accel.Variance          = ctypes.c_float( self.ReadData('{0}.{1}.VehDyn.Lateral.Accel.Variance'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )
        self.pFCTSenInput.contents.pEgoDynRaw.contents.Lateral.SlipAngle.SideSlipAngle = ctypes.c_float( self.ReadData('{0}.{1}.VehDyn.Lateral.SlipAngle.SideSlipAngle'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )
        self.pFCTSenInput.contents.pEgoDynRaw.contents.Lateral.SlipAngle.Variance      = ctypes.c_float( self.ReadData('{0}.{1}.VehDyn.Lateral.SlipAngle.Variance'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )

        self.pFCTSenInput.contents.pEgoDynRaw.contents.MotionState.Confidence = ctypes.c_float( self.ReadData('{0}.{1}.VehDyn.MotionState.Confidence'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )
        self.pFCTSenInput.contents.pEgoDynRaw.contents.MotionState.MotState   = self._FctDataTypes.MotState_t( self.ReadData('{0}.{1}.VehDyn.MotionState.MotState'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )

        self.pFCTSenInput.contents.pEgoDynRaw.contents.Legacy.YawRateMaxJitter = ctypes.c_float( self.ReadData('{0}.{1}.VehDyn.Legacy.YawRateMaxJitter'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )
        self.pFCTSenInput.contents.pEgoDynRaw.contents.Legacy.bStandStill      = self._FctDataTypes.boolean( self.ReadData('{0}.{1}.VehDyn.Legacy.bStandStill'.format(devicePrefixes[self._device], deviceVehDynIdentifier[self._device])) )

        """ #Todo
        for i in range(0,12):
            self.pFCTSenInput.contents.pEgoDynRaw.contents.State[i] = self._FctDataTypes.VDYIoStateTypes_t( self.ReadData('{0}.{1}.VehDyn.State{1:.0f}'.format(i)) )
        """

        # copy to obj sync
        self.pFCTSenInput.contents.pEgoDynObjSync = self.pFCTSenInput.contents.pEgoDynRaw
        # copy to veh input struct
        self.pFCTVehInput.contents.pEgoDynRaw     = self.pFCTSenInput.contents.pEgoDynRaw

    def UpdateGenObjectList( self, bsig ):
        self.pFCTSenInput.contents.pEmGenObjList.contents.sSigHeader.uiCycleCounter       = self._FctDataTypes.AlgoCycleCounter_t( self.ReadData('{0}.EmGenObjectList.sSigHeader.uiCycleCounter'.format(deviceSimVfbEmIdentifier[self._device])) )
        self.pFCTSenInput.contents.pEmGenObjList.contents.sSigHeader.eSigStatus           = self._FctDataTypes.AlgoSignalState_t( self.ReadData('{0}.EmGenObjectList.sSigHeader.eSigStatus'.format(deviceSimVfbEmIdentifier[self._device])) )
        self.pFCTSenInput.contents.pEmGenObjList.contents.sSigHeader.uiMeasurementCounter = self._FctDataTypes.AlgoCycleCounter_t( self.ReadData('{0}.EmGenObjectList.sSigHeader.uiMeasurementCounter'.format(deviceSimVfbEmIdentifier[self._device])) )
        self.pFCTSenInput.contents.pEmGenObjList.contents.sSigHeader.uiTimeStamp          = self._FctDataTypes.AlgoDataTimeStamp_t( self.ReadData('{0}.EmGenObjectList.sSigHeader.uiTimeStamp'.format(deviceSimVfbEmIdentifier[self._device])) )
        self.pFCTSenInput.contents.pEmGenObjList.contents.uiVersionNumber                 = self._FctDataTypes.AlgoInterfaceVersionNumber_t( self.ReadData('{0}.EmGenObjectList.uiVersionNumber'.format(deviceSimVfbEmIdentifier[self._device])) )

        # range(0,40)
        for iObj in range(0,40):
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].Attributes.eDynamicProperty  = self._FctDataTypes.EM_t_GenObjDynamicProperty( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].Attributes.eDynamicProperty'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )        
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].Attributes.uiDynConfidence   = ctypes.c_uint8( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].Attributes.uiDynConfidence'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )        
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].Attributes.eClassification   = self._FctDataTypes.EM_t_GenObjClassification( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].Attributes.eClassification'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )        
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].Attributes.uiClassConfidence = ctypes.c_uint8( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].Attributes.uiClassConfidence'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )        
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].Attributes.eObjectOcclusion  = self._FctDataTypes.EM_t_GenObjOcclusionState( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].Attributes.eObjectOcclusion'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )        	    

            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].Kinematic.fDistX    = ctypes.c_float( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].Kinematic.fDistX'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].Kinematic.fDistXStd = ctypes.c_float( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].Kinematic.fDistXStd'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].Kinematic.fDistY    = ctypes.c_float( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].Kinematic.fDistY'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].Kinematic.fDistYStd = ctypes.c_float( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].Kinematic.fDistYStd'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].Kinematic.fVrelX    = ctypes.c_float( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].Kinematic.fVrelX'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].Kinematic.fVrelXStd = ctypes.c_float( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].Kinematic.fVrelXStd'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].Kinematic.fVrelY    = ctypes.c_float( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].Kinematic.fVrelY'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].Kinematic.fVrelYStd = ctypes.c_float( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].Kinematic.fVrelYStd'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].Kinematic.fArelX    = ctypes.c_float( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].Kinematic.fArelX'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].Kinematic.fArelXStd = ctypes.c_float( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].Kinematic.fArelXStd'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].Kinematic.fArelY    = ctypes.c_float( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].Kinematic.fArelY'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].Kinematic.fArelYStd = ctypes.c_float( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].Kinematic.fArelYStd'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].Kinematic.fVabsX    = ctypes.c_float( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].Kinematic.fVabsX'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].Kinematic.fVabsXStd = ctypes.c_float( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].Kinematic.fVabsXStd'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].Kinematic.fVabsY    = ctypes.c_float( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].Kinematic.fVabsY'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].Kinematic.fVabsYStd = ctypes.c_float( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].Kinematic.fVabsYStd'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].Kinematic.fAabsX    = ctypes.c_float( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].Kinematic.fAabsX'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].Kinematic.fAabsXStd = ctypes.c_float( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].Kinematic.fAabsXStd'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].Kinematic.fAabsY    = ctypes.c_float( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].Kinematic.fAabsY'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].Kinematic.fAabsYStd = ctypes.c_float( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].Kinematic.fAabsYStd'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )        

            shapePointStates = self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].Geometry.aShapePointStates'.format(deviceSimVfbEmIdentifier[self._device],iObj))
            for i in range(0,4):
                self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].Geometry.aShapePointStates[i]                = self._FctDataTypes.EM_t_GenObjShapePointState(shapePointStates[i])

                self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].Geometry.aShapePointCoordinates[i].fPosX     = ctypes.c_float( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].Geometry.aShapePointCoordinates[{2:.0f}].fPosX'.format(deviceSimVfbEmIdentifier[self._device],iObj,i)) )
                self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].Geometry.aShapePointCoordinates[i].fPosY     = ctypes.c_float( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].Geometry.aShapePointCoordinates[{2:.0f}].fPosY'.format(deviceSimVfbEmIdentifier[self._device],iObj,i)) )
                self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].Geometry.aShapePointCoordinates[i].uiPosXStd = ctypes.c_uint16( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].Geometry.aShapePointCoordinates[{2:.0f}].uiPosXStd'.format(deviceSimVfbEmIdentifier[self._device],iObj,i)) )
                self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].Geometry.aShapePointCoordinates[i].uiPosYStd = ctypes.c_uint16( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].Geometry.aShapePointCoordinates[{2:.0f}].uiPosYStd'.format(deviceSimVfbEmIdentifier[self._device],iObj,i)) )

            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].General.fLifeTime               = ctypes.c_float( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].General.fLifeTime'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )        
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].General.uiLifeCycles            = self._FctDataTypes.AlgoCycleCounter_t( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].General.uiLifeCycles'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].General.uiLastMeasuredTimeStamp = self._FctDataTypes.AlgoDataTimeStamp_t( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].General.uiLastMeasuredTimeStamp'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].General.uiLastMeasuredCycle     = self._FctDataTypes.AlgoCycleCounter_t( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].General.uiLastMeasuredCycle'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].General.eMaintenanceState       = self._FctDataTypes.EM_t_GenObjMaintenanceState( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].General.eMaintenanceState'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].General.uiID                    = ctypes.c_uint8( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].General.uiID'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].General.eSplitMergeState        = self._FctDataTypes.EM_t_GenObjSplitMergeState( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].General.eSplitMergeState'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].General.uiMergeID               = ctypes.c_uint8( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].General.uiMergeID'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].General.uiSplitID               = ctypes.c_uint8( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].General.uiSplitID'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].General.eSensorSource           = self._FctDataTypes.EM_t_GenObjSensorSource( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].General.eSensorSource'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )


            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].Qualifiers.uiProbabilityOfExistence = ctypes.c_uint8( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].Qualifiers.uiProbabilityOfExistence'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].Qualifiers.uiAccObjQuality          = ctypes.c_uint8( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].Qualifiers.uiAccObjQuality'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].Qualifiers.uiEbaObjQuality          = ctypes.c_uint8( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].Qualifiers.uiEbaObjQuality'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].Qualifiers.eEbaHypCat               = self._FctDataTypes.EM_t_GenEbaHypCat( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].Qualifiers.eEbaHypCat'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )
            self.pFCTSenInput.contents.pEmGenObjList.contents.aObject[iObj].Qualifiers.eEbaInhibitionMask       = self._FctDataTypes.EM_t_GenEbaInhibit( self.ReadData('{0}.EmGenObjectList.aObject[{1:.0f}].Qualifiers.eEbaInhibitionMask'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )

    def fillARSObjectList( self, bsig ):
        # range(0,40)
        for iObj in range(0,40):
            self.pFCTSenInput.contents.pEmARSObjList.contents.aObject[iObj].Attributes.uiClassConfidence = ctypes.c_uint8( self.ReadData('{0}.EmARSObjectList.aObject[{1:.0f}].Attributes.uiClassConfidence'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )
            self.pFCTSenInput.contents.pEmARSObjList.contents.aObject[iObj].Geometry.fWidth              = ctypes.c_float( self.ReadData('{0}.EmARSObjectList.aObject[{1:.0f}].Geometry.fWidth'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )
            self.pFCTSenInput.contents.pEmARSObjList.contents.aObject[iObj].Geometry.fLength             = ctypes.c_float( self.ReadData('{0}.EmARSObjectList.aObject[{1:.0f}].Geometry.fLength'.format(deviceSimVfbEmIdentifier[self._device],iObj)) )
            #self.pFCTSenInput.contents.pEmARSObjList.contents.aObject[iObj].SensorSpecific.fRCS          = ctypes.c_float( self.ReadData('{0}.EmARSObjectList.aObject[{1:.0f}].SensorSpecific.fRCS'.format(deviceSIMVFBIdentifier[self._device],iObj)) )

    def RunInit(self, bsigFilePath):
        self._fctSimExtDll = ctypes.CDLL(self._fctSimExtDllPath)

        #zero init of all i/o structs
        self._fctSimExtDll.FCTSimInit()  #  manage init state machine and call FCTSimInitialize */

        FCTGetVehInput          = self._fctSimExtDll.FCTGetVehInput
        FCTGetVehInput.restype  = ctypes.POINTER(self._FctDataTypes.reqFCTVehPrtList_t)
        self.pFCTVehInput       = FCTGetVehInput()

        FCTGetVehOutput         = self._fctSimExtDll.FCTGetVehOutput
        FCTGetVehOutput.restype = ctypes.POINTER(self._FctDataTypes.proFCTVehPrtList_t)
        self.pFCTVehOutput      = FCTGetVehOutput()

        FCTGetSenInput          = self._fctSimExtDll.FCTGetSenInput
        FCTGetSenInput.restype  = ctypes.POINTER(self._FctDataTypes.reqFCTSenPrtList_t)
        self.pFCTSenInput       = FCTGetSenInput()

        FCTGetSenOutput         = self._fctSimExtDll.FCTGetSenOutput
        FCTGetSenOutput.restype = ctypes.POINTER(self._FctDataTypes.proFCTSenPrtList_t)
        self.pFCTSenOutput      = FCTGetSenOutput()

        self.curr_idx            = 0
        self.curr_sim_time       = 0
        self.last_time           = 0
        self.last_sen_time_stamp = 0
        self.last_veh_time_stamp = 0

        self._data_buffer               = {}
        self._data_buffer_length        = 20000
        self._data_buffer_curr_read_pos = self._data_buffer_length-2 # important to be self._data_buffer_length-2!
        self._subscribed_signals        = []

        self._output = {"t_sim": []}
        for currOutputURL in self._RequestedOutputURLs:
            self._output[currOutputURL] = []

        #Load data from bsig
        self._bsig = stk_bsig.stkBsig()
        try:
            self._bsig.open(bsigFilePath)
        except stk_bsig.bsig_200_exception as e:
            logging.warning('This file is not a valid bsig! {0}'.format(bsigFilePath))
            logging.warning('This file will not be resimulated!')
            raise stk_bsig.bsig_200_exception( 'Not a valid BSIG File: {0}'.format(bsigFilePath) )

        self.sample_count = len(self._bsig.get_signal_by_name('MTS.Package.TimeStamp'))
        self.stepSize_us  = (self._bsig.get_signal_by_name('MTS.Package.TimeStamp', Offset=1, SampleCount=1)[0] - self._bsig.get_signal_by_name('MTS.Package.TimeStamp', Offset=0, SampleCount=1)[0])

        self.last_sen_time_stamp = self._bsig.get_signal_by_name('MTS.Package.TimeStamp', Offset=0, SampleCount=1)[0] -max(self.stepSize_us, 60001)
        self.last_veh_time_stamp = self._bsig.get_signal_by_name('MTS.Package.TimeStamp', Offset=0, SampleCount=1)[0] -max(self.stepSize_us, 20001)

    def RunMainLoop(self):
        t_sim = []
        # Main Loop
        while self.curr_idx < (self.sample_count-1):
            # use dynamic data buffer
            self._data_buffer_curr_read_pos +=1
            if (self._data_buffer_curr_read_pos >= self._data_buffer_length):
                self.UpdateDataBuffer(self._bsig)

            # Simulate:
            self.curr_sim_time = self.ReadData('MTS.Package.TimeStamp')

            # logging
            if (self.curr_sim_time - self.last_time) > (1000*1000*60):
                logging.info('Cycle {0}/{1}'.format(self.curr_idx, self.sample_count))
                self.last_time = self.curr_sim_time
            else:
                logging.debug('Cycle {0}/{1} , SimTime {2}ms'.format(self.curr_idx, self.sample_count, self.curr_sim_time/1000))

            # Fill Input and run Simulation
            if ((self.curr_sim_time-self.last_veh_time_stamp) >= 20*1000 ):
                self.UpdateEgoVehicleData(self._bsig)
                self.UpdateDIM(self._bsig)
                self._fctSimExtDll.FCTSimCycle(ctypes.c_uint( FCT_SIM_EXT_VEH_TASK ), ctypes.c_bool(True))

                self.last_veh_time_stamp = self.curr_sim_time

                logging.debug('VEH Task run')

            if ((self.curr_sim_time-self.last_sen_time_stamp) >= 60*1000 ):
                self.UpdateGenObjectList(self._bsig)
                if devicePrefixes[self._device] == "ARS4xx Device":
                    self.fillARSObjectList(self._bsig)
                #######  DEBUG ###########
                #if self.curr_sim_time >= 1422546769635848:
                #    raw_input("Push button to contiune")   
                self._fctSimExtDll.FCTSimCycle(ctypes.c_uint( FCT_SIM_EXT_SEN_TASK ), ctypes.c_bool(True))

                self.last_sen_time_stamp = self.curr_sim_time

                logging.debug('SEN Task run')

            # Store Results
            self._output["t_sim"].append(self.curr_sim_time)
            for currOutputURL in self._RequestedOutputURLs:
                evalString = "self._output['{0}'].append(self.{1})".format(currOutputURL, currOutputURL.replace("->", ".contents."))
                try:
                    eval(evalString)
                except Exception as e:
                    raise Exception('Output Signal Error: {0}\n Pointers have to be dereferenced by -> operator!\n{1}'.format(currOutputURL,e) )

            self.curr_idx += 1

    def Run(self, bsigFilePath):
        self.RunInit(bsigFilePath)
        self.RunMainLoop()

        return self._output
