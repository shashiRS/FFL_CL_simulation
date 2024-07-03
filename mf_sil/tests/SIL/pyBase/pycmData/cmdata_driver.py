from cmdata_common import cDynamicProperty, cSimpleDataType

### ROAD CLASS ###
class cCMDriver(cDynamicProperty):
    def __init__(self, srcTestRunFile=None):
        self._Attributes = []
        ################################
        ## WORKAROUND FOR FASTER USE
#        self._Attributes.append(('Driver', cSimpleDataType("Driver.ParamIdent", str, """ = IPG-DRIVER 2
#Driver.Mode = std
#Driver.Long.DrivMaxSpeed = 0
#Driver.Long.CruisingSpeed = 150
#Driver.CornerCutCoef = 0.5
#Driver.ConsiderTraffic = 1
#Driver.Traffic.TimeGapMin = 1.8
#Driver.Traffic.TimeGapMax = 5.0
#Driver.Traffic.DistMin = 6
#Driver.Traffic.DistMax = 250
#Driver.Traffic.EcoCoef = 0.75
#Driver.Long.dtAccBrake = 0.5
#Driver.Long.axMax = 3.0
#Driver.Long.axMin = -4.0
#Driver.Long.ayMax = 4.0
#Driver.Long.GGExp:
#\t50 1.0 1.0
#Driver.Long.DevMax = 0.0
#Driver.Long.tReact = 0.0
#Driver.Long.TractionControl = 1
#Driver.DecShift.Active = 1
#Driver.DecShift.UseBrakePark = 0
#Driver.DecShift.tSwitchGear = 1.0
#Driver.DecShift.nEngine.Limits:
#\t1500 4000
#Driver.DecShift.nEngine.Shift:
#\t2000 3000
#Driver.Lat.DevMax = 0.0
#Driver.Lat.tReact = 0.0
#Driver.Knowl.Long.tActionMin = 4
#Driver.Knowl.Lat.StWhlAngleMax = 630
#Driver.Knowl.Lat.StWhlAngleVelMax = 500
#Driver.Knowl.Lat.StWhlAngleAccMax = 3000
#Driver.Learn.VehicleLimits.TestRun =
#Driver.Learn.VehicleLimits.Date = 0
#Driver.Learn.ControllerDyn.TestRun =
#Driver.Learn.ControllerDyn.Date = 0
#Driver.Learn.MaxSpeed.TestRun =
#Driver.Learn.MaxSpeed.Date = 0
#Driver.Learn.Remember = 0
#Driver.Knowl.Long.tPreviewBra = 0.6
#Driver.Knowl.Long.tPreviewAcc = 1.5
#Driver.Knowl.Lat.tPreview = 0.8""", splitter="")))
        self._Attributes.append(('ParamIdent',                  cSimpleDataType("Driver.ParamIdent", str, "IPGDriver 5")))
        self._Attributes.append(('Mode',                        cSimpleDataType("Driver.Mode", str, "std")))
        self._Attributes.append(('Long.DrivMaxSpeed',           cSimpleDataType("Driver.Long.DrivMaxSpeed", int, 0)))
        self._Attributes.append(('Long.CruisingSpeed',          cSimpleDataType("Driver.Long.CruisingSpeed", float, 150.0)))
        self._Attributes.append(('CornerCutCoef',               cSimpleDataType("Driver.CornerCutCoef", float, 0.5)))
        self._Attributes.append(('ConsiderTraffic',             cSimpleDataType("Driver.ConsiderTraffic", int, 0)))
        self._Attributes.append(('TimeGapMin',                  cSimpleDataType("Driver.Traffic.TimeGapMin", float, 1.8)))
        self._Attributes.append(('TimeGapMax',                  cSimpleDataType("Driver.Traffic.TimeGapMax", float, 5.0)))
        self._Attributes.append(('DistMin',                     cSimpleDataType("Driver.Traffic.DistMin", float, 6.0)))
        self._Attributes.append(('DistMax',                     cSimpleDataType("Driver.Traffic.DistMax", float, 250.0)))
        self._Attributes.append(('EcoCoef',                     cSimpleDataType("Driver.Traffic.EcoCoef", float, 0.75)))
        self._Attributes.append(('Overtake',                  cSimpleDataType("Driver.Traffic.Overtake", int, 0)))
        self._Attributes.append(('Overtake_Rate',                  cSimpleDataType("Driver.Traffic.Overtake_Rate", int, 1)))
        self._Attributes.append(('Overtake_dSpeedMin',                  cSimpleDataType("Driver.Traffic.Overtake_dSpeedMin", int, 10)))
        self._Attributes.append(('Long.dtAccBrake',             cSimpleDataType("Driver.Long.dtAccBrake", float, 0.5)))
        self._Attributes.append(('Long.axMax',                  cSimpleDataType("Driver.Long.axMax", float, 3.0)))
        self._Attributes.append(('Long.axMin',                  cSimpleDataType("Driver.Long.axMin", float, -4.0)))
        self._Attributes.append(('Long.ayMax',                  cSimpleDataType("Driver.Long.ayMax", float, 4.0)))
        self._Attributes.append(('Long.GGExp',                  cSimpleDataType("Driver.Long.GGExp", str, "\t50 1.0 1.0", splitter=":\n")))
        self._Attributes.append(('Long.DevMax',                 cSimpleDataType("Driver.Long.DevMax", float, 0.0)))
        self._Attributes.append(('Long.tReact',                 cSimpleDataType("Driver.Long.tReact", float, 0.0)))
        self._Attributes.append(('Long.TractionControl',        cSimpleDataType("Driver.Long.TractionControl", int, 1)))
        #self._Attributes.append(('DecShift.Active',             cSimpleDataType("Driver.DecShift.Active", int, 1)))
        self._Attributes.append(('DecShift.UseBrakePark',       cSimpleDataType("Driver.DecShift.UseBrakePark", int, 0)))
        self._Attributes.append(('DecShift.tSwitchGear',        cSimpleDataType("Driver.DecShift.tSwitchGear", float, 1.0)))
        self._Attributes.append(('DecShift.nEngine.Limits',     cSimpleDataType("Driver.DecShift.nEngine.Limits", str, "\t1500 4000", splitter=":\n")))
        self._Attributes.append(('DecShift.nEngine.Shift',      cSimpleDataType("Driver.DecShift.nEngine.Shift", str, "\t2000 3000", splitter=":\n")))
        self._Attributes.append(('Lat.DevMax',                  cSimpleDataType("Driver.Lat.DevMax", float, 0.0)))
        self._Attributes.append(('Lat.tReact',                  cSimpleDataType("Driver.Lat.tReact", float, 0.0)))
        self._Attributes.append(('Knowl.Long.tActionMin',       cSimpleDataType("Driver.Knowl.Long.tActionMin", float, 4.0)))
        self._Attributes.append(('Knowl.Lat.StWhlAngleMax',     cSimpleDataType("Driver.Knowl.Lat.StWhlAngleMax", float, 630.0)))
        self._Attributes.append(('Knowl.Lat.StWhlAngleVelMax',  cSimpleDataType("Driver.Knowl.Lat.StWhlAngleVelMax", float, 500.0)))
        self._Attributes.append(('Knowl.Lat.StWhlAngleAccMax',  cSimpleDataType("Driver.Knowl.Lat.StWhlAngleAccMax", float, 3000.0)))
        self._Attributes.append(('Learn.VehicleLimits.TestRun', cSimpleDataType("Driver.Learn.VehicleLimits.TestRun", str)))
        self._Attributes.append(('Learn.VehicleLimits.Date',    cSimpleDataType("Driver.Learn.VehicleLimits.Date", float, 0.0)))
        self._Attributes.append(('Learn.ControllerDyn.TestRun', cSimpleDataType("Driver.Learn.ControllerDyn.TestRun", str)))
        self._Attributes.append(('Learn.ControllerDyn.Date',    cSimpleDataType("Driver.Learn.ControllerDyn.Date", float, 0.0)))
        self._Attributes.append(('Learn.MaxSpeed.TestRun',      cSimpleDataType("Driver.Learn.MaxSpeed.TestRun", str)))
        self._Attributes.append(('Learn.MaxSpeed.Date',         cSimpleDataType("Driver.Learn.MaxSpeed.Date", float, 0.0)))
        self._Attributes.append(('Learn.Remember',              cSimpleDataType("Driver.Learn.Remember", int, 0)))
        self._Attributes.append(('Knowl.Long.tPreviewBra',      cSimpleDataType("Driver.Knowl.Long.tPreviewBra", float, 0.6)))
        self._Attributes.append(('Knowl.Long.tPreviewAcc',      cSimpleDataType("Driver.Knowl.Long.tPreviewAcc", float, 1.5)))
        self._Attributes.append(('Knowl.Lat.tPreview',          cSimpleDataType("Driver.Knowl.Lat.tPreview", float, 0.8)))
        self._Attributes.append(('Learn.NEng_S',                cSimpleDataType("Driver.Learn.NEng_S", int, 1)))
        ################################
        self._setAttributes()

        if srcTestRunFile is not None:
            for element in self._Attributes:
                if type(element[1]) is cSimpleDataType:
                    cSimpleDataType.readfromFile(element[1], srcTestRunFile, EndStrNxtLn="Driver.Learn.NEng_S") ##  EndStrNxtLn for Workaround for Text parts
                elif type(element[1]) is cSimple3Dfloat:
                    cSimple3Dfloat.readfromFile(element[1], srcTestRunFile)

    def GetTestRunText(self):
        testRunText = ""
        for currAttribute in self._Attributes:
            testRunText += currAttribute[1].getConfigFileText()
        return testRunText