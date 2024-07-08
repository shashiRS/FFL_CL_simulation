import os
import os.path
import datetime

from cmdata_common  import cDynamicProperty, cSimpleDataType, cSimple3Dfloat
from cmdata_vehicle import cCMVehicle, cCMVehicleLoad
from cmdata_trailer import cCMTrailer, cCMTrailerLoad
from cmdata_tire    import cCMTire
from cmdata_drivman import cCMDrivMan
from cmdata_traffic import cCMTraffic
from cmdata_road    import cCMRoad
from cmdata_driver  import cCMDriver

### TESTRUN CLASS ###
class cCMTestRun(cDynamicProperty):
    def __init__(self, srcTestRunFilePath=None, name=None):
        self._Name = "TestRun"
        # Set TestRun Name
        if name is not None:
            self._Name = name
        else:
            if srcTestRunFilePath is not None:
                self._Name = os.path.splitext(os.path.basename(srcTestRunFilePath))[0]

        srcVehicleFilePath = None
        srcTireFilePath    = None
        # Find further files for TestRun
        if srcTestRunFilePath is not None:
            with open(srcTestRunFilePath, 'r') as srcTestRunFile:
                for currRow in srcTestRunFile:
                    if "Vehicle = " in currRow:
                        relVehicleFilePath = currRow[len("Vehicle = "):-1]
                        srcVehicleFilePath = srcTestRunFilePath[:srcTestRunFilePath.find("TestRun")] + "Vehicle\\" + relVehicleFilePath
                    if "Tire.0 = " in currRow:
                        relTireFilePath = currRow[len("Tire.0 = "):-1]
                        srcTireFilePath = srcTestRunFilePath[:srcTestRunFilePath.find("TestRun")] + "Tire\\" + relTireFilePath

        # Call necessary classes
        self.Vehicle     = cCMVehicle(srcVehicleFilePath)
        self.Tire        = cCMTire(srcTireFilePath)
        self.Trailer     = cCMTrailer(srcTestRunFilePath)
        self.Snapshot    = cCMSnapshot(srcTestRunFilePath)
        self.VehicleLoad = cCMVehicleLoad(srcTestRunFilePath)
        self.TrailerLoad = cCMTrailerLoad(srcTestRunFilePath)
        self.Road        = cCMRoad(srcTestRunFilePath)
        self.DrivMan     = cCMDrivMan(srcTestRunFilePath)
        self.Traffic     = cCMTraffic(srcTestRunFilePath)         
        self.ErrorClass  = cCMErrorClass(srcTestRunFilePath)   
        self.EnvironmentClass  = cCMEnvironmentClass(srcTestRunFilePath)   
        self.Driver      = cCMDriver(srcTestRunFilePath) 

        self._Attributes = []
        ################################
        self._Attributes.append(('InfoLine',      cSimpleDataType("#INFOFILE", str, "1.1 - Do not remove this line!", splitter="")))
        self._Attributes.append(('FileIdent',     cSimpleDataType("FileIdent", str, "CarMaker-TestRun 8"))) # this is a patchwork for the transition to CarMaker 8 - the actual testrun templete is CarMaker 5
        self._Attributes.append(('FileCreator',   cSimpleDataType("FileCreator", str, "CarMaker 8.1.1 " + str(datetime.datetime.now().strftime("%Y-%m-%d"))))) # this is a patchwork for the transition to CarMaker 8 - the actual testrun templete is CarMaker 5
        self._Attributes.append(('Description',   cSimpleDataType("Description", str, "\tCourse: One segment\n\tArbitrary segments can be added", splitter=":\n\t")))
        #self._Attributes.append(('Description',   cSimpleDataType("Description:", str, "")))
        #self._Attributes.append(('FileClass.New', cSimpleDataType("FileClass.New", float, 5.0)))
        ################################
        self._setAttributes()

        if srcTestRunFilePath is not None:
            for element in self._Attributes:
                if type(element[1]) is cSimpleDataType:
                    cSimpleDataType.readfromFile(element[1], srcTestRunFilePath)
                elif type(element[1]) is cSimple3Dfloat:
                    cSimple3Dfloat.readfromFile(element[1], srcTestRunFilePath)

    def GetTestRunText(self):
        testRunText = ""
        for currAttribute in self._Attributes:
            testRunText += currAttribute[1].getConfigFileText()
        return testRunText

    def GetName(self):
        return self._Name

    def SetName(self, value):
        self._Name = value

    #def createTestRun(self, vEgo, offset):
    #    self.vEgo = vEgo
    #    self.offset = offset
   
    def save(self, folder):
        fileName = os.path.join(folder, self._Name)
        with open(fileName,"w") as f:
            f.write(self.GetTestRunText())
            f.write(self.Vehicle.GetTestRunText())
            f.write(self.Trailer.GetTestRunText()) 
            f.write(self.Tire.GetTestRunText())
            f.write(self.Snapshot.GetTestRunText())
            f.write(self.VehicleLoad.GetTestRunText())
            f.write(self.TrailerLoad.GetTestRunText())
            f.write(self.DrivMan.GetTestRunText())
            f.write(self.Traffic.GetTestRunText())
            f.write(self.ErrorClass.GetTestRunText())
            f.write(self.Road.GetTestRunText())
            f.write(self.EnvironmentClass.GetTestRunText())
            f.write(self.Driver.GetTestRunText())


### SNAPSHOT CLASS ###
class cCMSnapshot(cDynamicProperty):
    def __init__(self, srcTestRunFile=None):
        self._Attributes = []
        self._Attributes.append(('TimeLimit', cSimpleDataType("Snapshot.TimeLimit", float, valueChecker=lambda x: x>=0.0)))
        self._Attributes.append(('DistLimit', cSimpleDataType("Snapshot.DistLimit", float)))

    def GetTestRunText(self):
        testRunText = ""
        for currAttribute in self._Attributes:
            testRunText += currAttribute[1].getConfigFileText()
        return testRunText

###EnvironmentClass CLASS####
class cCMEnvironmentClass(cDynamicProperty):
    def __init__(self, srcTestRunFile=None):
        self._Attributes = []
        self._Attributes.append(('StartTime.Year', cSimpleDataType("Env.StartTime.Year", int, 2014)))
        self._Attributes.append(('StartTime.Month', cSimpleDataType("Env.StartTime.Month", int, 1)))
        self._Attributes.append(('StartTime.Day', cSimpleDataType("Env.StartTime.Day", int, 1)))
        self._Attributes.append(('StartTime.Hour', cSimpleDataType("Env.StartTime.Hour", int, 12)))
        self._Attributes.append(('StartTime.Min', cSimpleDataType("Env.StartTime.Min", int, 0)))
        self._Attributes.append(('StartTime.Sec', cSimpleDataType("Env.StartTime.Sec", int, 0)))
        self._Attributes.append(('StartTime.DeltaUTC', cSimpleDataType("Env.StartTime.DeltaUTC", float, 0.0)))

        self._Attributes.append(('GNSS.Active', cSimpleDataType("Env.GNSS.Active", int, 1)))
        self._Attributes.append(('Temperature', cSimpleDataType("Env.Temperature", float, 20.0)))
        self._Attributes.append(('AirDensity', cSimpleDataType("Env.AirDensity", float, 1.205)))
        self._Attributes.append(('AirPressure', cSimpleDataType("Env.AirPressure", float, 1.013)))
        self._Attributes.append(('AirHumidity', cSimpleDataType("Env.AirHumidity", float, 60.0)))
        self._Attributes.append(('SolarRadiation', cSimpleDataType("Env.SolarRadiation", float, 400.0)))

        self._Attributes.append(('Wind.Kind', cSimpleDataType("Env.Wind.Kind", str, "none")))
        self._Attributes.append(('Wind.Velocity', cSimpleDataType("Env.Wind.Velocity", float, 0.0)))
        self._Attributes.append(('Wind.Angle', cSimpleDataType("Env.Wind.Angle", float, 0.0)))

        self._Attributes.append(('Kind', cSimpleDataType("Env.Kind", str, "Generic")))
        self._Attributes.append(('Temp.Offset_Elev', cSimpleDataType("Env.Temp.Offset_Elev", float, -0.0065)))
        self._Attributes.append(('Temp.Offset_sRoad.Amplify', cSimpleDataType("Env.Temp.Offset_sRoad.Amplify", float, 1.0)))
        self._Attributes.append(('Temp.Offset_sRoad.On', cSimpleDataType("Env.Temp.Offset_sRoad.On", int, 0)))
        self._Attributes.append(('Temp.Offset_Time.Amplify', cSimpleDataType("Env.Temp.Offset_Time.Amplify", float, 1.0)))
        self._Attributes.append(('Temp.Temp.Offset_Time.On', cSimpleDataType("Env.Temp.Offset_Time.On", int, 1)))
        self._Attributes.append(('Temp.Offset_Time', cSimpleDataType("Env.Temp.Offset_Time:", str, """\t0.0 -2.0
	3.0 -2.5
	6.0 -2.7
	7.5 -2.7
	9.0 -2.5
	10.0 -2.3
	11.0 -1.6
	12.0 0.0
	13.0 1.4
	14.0 2.1
	15.5 2.5
	17.0 2.2
	18.0 1.7
	19.0 1.1
	20.0 0.2
	21.0 -0.6
	22.0 -1.1
	23.0 -1.6
	24.0 -2.0""", splitter="\n")))
        self._setAttributes()
    def GetTestRunText(self):
        testRunText = ""
        for currAttribute in self._Attributes:
            testRunText += currAttribute[1].getConfigFileText()
        return testRunText

### ErrorClass CLASS ###
class cCMErrorClass(cDynamicProperty):
    def __init__(self, srcTestRunFile=None):
        self._Attributes = []
        ################################
        ## WORKAROUND FOR FASTER USE
        self._Attributes.append(('ErrorClass', cSimpleDataType("ErrorClass.0.", str, """Action = abort
ErrorClass.0.Save = 0
ErrorClass.0.WarningLimit = 3 5
ErrorClass.1.Action = abort
ErrorClass.1.Save = 0
ErrorClass.1.WarningLimit = 3 5
ErrorClass.2.Action = abort
ErrorClass.2.Save = 0
ErrorClass.2.WarningLimit = 3 5
ErrorClass.3.Action = abort
ErrorClass.3.Save = 0
ErrorClass.3.WarningLimit = 3 5
ErrorClass.4.Action = abort
ErrorClass.4.Save = 0
ErrorClass.4.WarningLimit = 3 5
ErrorClass.5.Action = abort
ErrorClass.5.Save = 0
ErrorClass.5.WarningLimit = 3 5
ErrorClass.10.Action = abort
ErrorClass.10.Save = 0
ErrorClass.10.WarningLimit = 3 5
ErrorClass.11.Action = abort
ErrorClass.11.Save = 0
ErrorClass.11.WarningLimit = 3 5""", splitter="")))
        self._setAttributes()

        if srcTestRunFile is not None:
            for element in self._Attributes:
                if type(element[1]) is cSimpleDataType:
                    cSimpleDataType.readfromFile(element[1], srcTestRunFile, EndStrNxtLn='Road') ##  EndStrNxtLn for Workaround for Text parts
                elif type(element[1]) is cSimple3Dfloat:
                    cSimple3Dfloat.readfromFile(element[1], srcTestRunFile)

    def GetTestRunText(self):
        testRunText = ""
        for currAttribute in self._Attributes:
            testRunText += currAttribute[1].getConfigFileText()
        return testRunText



