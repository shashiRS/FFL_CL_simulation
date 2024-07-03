import datetime
from cmdata_common import cDynamicProperty, cSimpleDataType, cSimple3Dfloat, cSimple6Dfloat


### SINGLE SIDE SLIP ANGLE SENSOR ###
class cCMSSASensor(cDynamicProperty):
    def __init__(self, name, pos, Mounting, SensorID):
        self._Attributes = []
        self._Attributes.append(('name',     cSimpleDataType("SSASensor." + str(SensorID) + ".name", str, name)))
        self._Attributes.append(('pos',      cSimple3Dfloat("SSASensor." + str(SensorID) + ".pos", pos)))
        self._Attributes.append(('Mounting', cSimpleDataType("SSASensor." + str(SensorID) + ".Mounting", str, Mounting)))
        self._setAttributes()


### SIDE SLIP ANGLE SENSORS ###
class cCMSSASensors(cDynamicProperty):
    def __init__(self, name="", pos=[2.35, 0.0, 0.6], Mounting="Fr1A", srcVehicleFile=None):
        self._SensorList = []  
        if srcVehicleFile is not None:
            with open(srcVehicleFile, 'r') as f:
                for row in f:
                    if row.find("SSASensor.N") is 0:
                        NrSSAS = int(row[len("SSASensor.N = "):row.find("\n")])
                        for SensorID in range(NrSSAS):
                            parameters = self.SearchParameters(f, SensorID)
                            self._SensorList.append(cCMSSASensor(parameters[0], parameters[1], parameters[2], SensorID))
        else:
            if name=="":
                name="A00"
            self.add(name, pos, Mounting)
    

    def SearchParameters(self, f, SensorID):
        nameline = next(f)
        name = (nameline[len("SSASensor." + str(SensorID) + ".name = "):nameline.find("\n")])
        posline = next(f)
        dummylist = posline[len("SSASensor." + str(SensorID) + ".pos = "):].split(' ')
        pos = []
        for item in dummylist:
            pos.append(float(item))
        mountline = next(f)
        Mounting = (mountline[len("SSASensor." + str(SensorID) + ".Mounting = "):mountline.find("\n")]) 
        return (name, pos, Mounting)
     
                  
    def add(self, name="", pos=[2.35, 0.0, 0.6], Mounting="Fr1A"):
        if name=="":
            name = "A%0*d" %(2, len(self._SensorList))
        for element in self._SensorList:
            if name == element.name.value:
                raise Exception("ERROR in cCMSSASensors: Sensor with the name {0} exists already!".format(name))
        self._SensorList.append(cCMSSASensor(name, pos, Mounting, len(self._SensorList)))


    def GetTestRunText(self):
        testRunText = "SSASensor.N = " + str(len(self._SensorList)) + "\n"
        for currElement in self._SensorList:
            for currAttribute in currElement._Attributes:
                testRunText += currAttribute[1].getConfigFileText()
        return testRunText

    def __getitem__(self,index):
        return self._SensorList[index]

    def __setitem__(self, index, newvalue):
        self._SensorList[index] = newvalue

    def __delitem__(self, index):
         del self._SensorList[index]


### SINGLE BODY SENSOR ###
class cCMBdySensor(cDynamicProperty):
    def __init__(self, name, pos, rot, CalcClass, Mounting, SensorID):
        self._Attributes = []
        self._Attributes.append(('name',      cSimpleDataType("BdySensor." + str(SensorID) + ".name", str, name)))
        self._Attributes.append(('pos',       cSimple3Dfloat("BdySensor." + str(SensorID) + ".pos", pos)))
        self._Attributes.append(('rot',       cSimple3Dfloat("BdySensor." + str(SensorID) + ".rot", rot)))               
        self._Attributes.append(('CalcClass', cSimpleDataType("BdySensor." + str(SensorID) + ".CalcClass", str, CalcClass)))
        self._Attributes.append(('Mounting',  cSimpleDataType("BdySensor." + str(SensorID) + ".Mounting", str, Mounting)))
        self._setAttributes()


### BODY SENSORS ###
class cCMBdySensors(cDynamicProperty):
    def __init__(self, srcVehicleFile=None):
        self._SensorList = []  
        if srcVehicleFile is not None:
            with open(srcVehicleFile, 'r') as f:
                for row in f:
                    if row.find("BdySensor.N") is 0:
                        NrBdyS = int(row[len("BdySensor.N = "):row.find("\n")])
                        for SensorID in range(NrBdyS):
                            parameters = self.SearchParameters(f, SensorID)
                            self._SensorList.append(cCMBdySensor(parameters[0], parameters[1], parameters[2], parameters[3], parameters[4], SensorID))

    def SearchParameters(self, f, SensorID):
        nameline = next(f)
        name = (nameline[len("BdySensor." + str(SensorID) + ".name = "):nameline.find("\n")])
        posline = next(f)
        dummylist = posline[len("BdySensor." + str(SensorID) + ".pos = "):].split(' ')
        pos = []
        for item in dummylist[:3]:
            pos.append(float(item))
        rotline = next(f)
        dummylist = rotline[len("BdySensor." + str(SensorID) + ".rot = "):].split(' ')
        rot = []
        for item in dummylist:
            rot.append(float(item))
        calcclassline = next(f)
        CalcClass = (calcclassline[len("BdySensor." + str(SensorID) + ".CalcClass = "):calcclassline.find("\n")])
        mountline = next(f)
        Mounting = (mountline[len("BdySensor." + str(SensorID) + ".Mounting = "):mountline.find("\n")]) 
        return (name, pos, rot, CalcClass, Mounting)
             
    def add(self, name="YRS", pos=[2.9, 0.3, 0.8], rot=[0.0, 0.0, 0.0], CalcClass="Global+Local", Mounting="Fr1A"):
        for element in self._SensorList:
            if name == element.name.value:
                raise Exception("ERROR in cCMBdySensors: Sensor with the name {0} exists already!".format(name))
        self._SensorList.append(cCMBdySensor(name, pos, rot, CalcClass, Mounting, len(self._SensorList)))

    def GetTestRunText(self):
        testRunText = "BdySensor.N = " + str(len(self._SensorList)) + "\n"
        for currElement in self._SensorList:
            for currAttribute in currElement._Attributes:
                testRunText += currAttribute[1].getConfigFileText()
        return testRunText

    def __getitem__(self,index):
        return self._SensorList[index]

    def __setitem__(self, index, newvalue):
        self._SensorList[index] = newvalue

    def __delitem__(self, index):
         del self._SensorList[index]


### SINGLE DRIVER ASSISTANCE SENSOR ###
class cCMDASensor(cDynamicProperty):
    def __init__(self, name, pos, rot, alpha, theta, Range, UpdRate, nCycleOffset, tracking, CalcClass, movietheme, Mounting, ShowDDict, ExtMotion, SensorID):
        self._Attributes = []
        self._Attributes.append(('name',         cSimpleDataType("DASensor." + str(SensorID) + ".name", str, name)))
        self._Attributes.append(('pos',          cSimple3Dfloat("DASensor." + str(SensorID) + ".pos", pos)))
        self._Attributes.append(('rot',          cSimple3Dfloat("DASensor." + str(SensorID) + ".rot", rot)))               
        self._Attributes.append(('alpha',        cSimpleDataType("DASensor." + str(SensorID) + ".alpha", float, alpha)))
        self._Attributes.append(('theta',        cSimpleDataType("DASensor." + str(SensorID) + ".theta", float, theta)))
        self._Attributes.append(('Range',        cSimpleDataType("DASensor." + str(SensorID) + ".range", float, Range)))
        self._Attributes.append(('UpdRate',      cSimpleDataType("DASensor." + str(SensorID) + ".UpdRate", int, UpdRate, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('nCycleOffset', cSimpleDataType("DASensor." + str(SensorID) + ".nCycleOffset", int, nCycleOffset, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('tracking',     cSimpleDataType("DASensor." + str(SensorID) + ".tracking", str, tracking)))
        self._Attributes.append(('CalcClass',    cSimpleDataType("DASensor." + str(SensorID) + ".CalcClass", str, CalcClass)))
        self._Attributes.append(('movietheme',   cSimpleDataType("DASensor." + str(SensorID) + ".movietheme", str, movietheme)))
        self._Attributes.append(('Mounting',     cSimpleDataType("DASensor." + str(SensorID) + ".Mounting", str, Mounting)))
        self._Attributes.append(('ShowDDict',    cSimpleDataType("DASensor." + str(SensorID) + ".ShowDDict", int, ShowDDict, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('ExtMotion',    cSimpleDataType("DASensor." + str(SensorID) + ".ExtMotion", int, ExtMotion, valueChecker=lambda x: x>=0)))  
        self._setAttributes()


### DRIVER ASSISTANCE ###
class cCMDASensors(cDynamicProperty):
    def __init__(self, srcVehicleFile=None):        
        self._SensorList = []  
        if srcVehicleFile is not None:
            with open(srcVehicleFile, 'r') as f:
                for row in f:
                    self._ObservRadius = 500.0
                    if row.find("DASensor.ObservRadius") is 0:
                        self._ObservRadius = float(row[len("DASensor.ObservRadius = "):row.find("\n")])
                        break
            with open(srcVehicleFile, 'r') as f:
                for row in f:
                    if row.find("DASensor.N") is 0:
                        NrDAS = int(row[len("DASensor.N = "):row.find("\n")])
                        next(f)
                        for SensorID in range(NrDAS):
                            parameters = self.SearchParameters(f, SensorID)
                            self._SensorList.append(cCMDASensor(parameters[0], parameters[1], parameters[2], parameters[3], parameters[4], parameters[5],\
                                                                parameters[6], parameters[7], parameters[8], parameters[9], parameters[10], parameters[11],                                 
                                                                parameters[12], parameters[13], SensorID))
                        break

    def SearchParameters(self, f, SensorID):
        nameline = next(f)
        name = (nameline[len("DASensor." + str(SensorID) + ".name = "):nameline.find("\n")])
        posline = next(f)
        dummylist = posline[len("DASensor." + str(SensorID) + ".pos = "):].split(' ')
        pos = []
        for item in dummylist:
            pos.append(float(item))
        rotline = next(f)
        dummylist = rotline[len("DASensor." + str(SensorID) + ".rot = "):].split(' ')
        rot = []
        for item in dummylist:
            rot.append(float(item))
        DAmembers = ["alpha", "theta", "range", "UpdRate", "nCycleOffset", "tracking", "CalcClass", "movietheme", "Mounting", "ShowDDict", "ExtMotion"]
        parameters = []
        for element in DAmembers:
            nextline = next(f)
            parameters.append(nextline[len("DASensor." + str(SensorID) + "." + element + " = "):nextline.find("\n")])
        return (name, pos, rot, float(parameters[0]), float(parameters[1]), float(parameters[2]), int(parameters[3]), int(parameters[4]),\
                                str(parameters[5]), str(parameters[6]), str(parameters[7]), str(parameters[8]), int(parameters[9]), int(parameters[10]))
             
    def add(self, name="", pos=[0.0, 0.0, 0.0], rot=[0.0, 0.0, 0.0], alpha=10.0, theta=10.0, Range=0.0, UpdRate=1000, nCycleOffset=0, tracking="ClosestObject",\
                  CalcClass="NearestPoint", movietheme="NotVisible", Mounting="Fr1A", ShowDDict=0, ExtMotion=0):
        if name=="":
            name = "C%0*d" %(2, len(self._SensorList))
        for element in self._SensorList:
            if name == element.name.value:
                raise Exception("ERROR in cCMDASensors: Sensor with the name {0} exists already!".format(name))
        self._SensorList.append(cCMDASensor(name, pos, rot, alpha, theta, Range, UpdRate, nCycleOffset, tracking, CalcClass,\
                                            movietheme, Mounting, ShowDDict, ExtMotion, len(self._SensorList)))

    def GetTestRunText(self):
        testRunText = "DASensor.N = " + str(len(self._SensorList)) + "\n"
        if len(self._SensorList) > 0:
            testRunText += "DASensor.ObservRadius = " + str(self._ObservRadius) + "\n"
        for currElement in self._SensorList:
            for currAttribute in currElement._Attributes:
                testRunText += currAttribute[1].getConfigFileText()
        return testRunText

    def __getitem__(self,index):
        return self._SensorList[index]

    def __setitem__(self, index, newvalue):
        self._SensorList[index] = newvalue

    def __delitem__(self, index):
         del self._SensorList[index]


### SINGLE ROAD PREVIEW SENSOR ###
class cCMRPSensor(cDynamicProperty):
    def __init__(self, name, PreviewDist, PreviewMode, ConsiderBumps, pos, rot_z, SensorID):
        self._Attributes = []
        self._Attributes.append(('name',          cSimpleDataType("RPSensor." + str(SensorID) + ".name", str, name)))
        self._Attributes.append(('PreviewDist',   cSimpleDataType("RPSensor." + str(SensorID) + ".PreviewDist", float, PreviewDist)))               
        self._Attributes.append(('PreviewMode',   cSimpleDataType("RPSensor." + str(SensorID) + ".PreviewMode", str, PreviewMode)))
        self._Attributes.append(('ConsiderBumps', cSimpleDataType("RPSensor." + str(SensorID) + ".ConsiderBumps", int, ConsiderBumps, valueChecker=lambda x: x>=0)))                
        self._Attributes.append(('pos',           cSimple3Dfloat("RPSensor." + str(SensorID) + ".pos", pos)))
        self._Attributes.append(('rot_z',         cSimpleDataType("RPSensor." + str(SensorID) + ".rot_z", float, rot_z)))                       
        self._setAttributes()


### ROAD PREVIEW SENSORS ###
class cCMRPSensors(cDynamicProperty):
    def __init__(self, srcVehicleFile=None):
        self._SensorList = []  
        if srcVehicleFile is not None:
            with open(srcVehicleFile, 'r') as f:
                for row in f:
                    if row.find("RPSensor.N") is 0:
                        NrRPS = int(row[len("RPSensor.N = "):row.find("\n")])
                        for SensorID in range(NrRPS):
                            parameters = self.SearchParameters(f, SensorID)
                            self._SensorList.append(cCMRPSensor(parameters[0], parameters[1], parameters[2], parameters[3], parameters[4], parameters[5], SensorID))
    

    def SearchParameters(self, f, SensorID):
        RPmembers = ["name", "PreviewDist", "PreviewMode", "ConsiderBumps"]
        parameters = []
        for element in RPmembers:
            nextline = next(f)
            parameters.append(nextline[len("RPSensor." + str(SensorID) + "." + element + " = "):nextline.find("\n")])
        posline = next(f)
        dummylist = posline[len("RPSensor." + str(SensorID) + ".pos = "):].split(' ')      
        pos = []
        for item in dummylist:
            pos.append(float(item))
        if parameters[2] == "AlongVhcl":
            rot_zline = next(f)
            rot_z = float(rot_zline[len("RPSensor." + str(SensorID) + ".rot_z = "):rot_zline.find("\n")]) 
        else:
            rot_z = 0.0
        return (parameters[0], float(parameters[1]), parameters[2], int(parameters[3]), pos, rot_z)
     
                  
    def add(self, name="", PreviewDist=0.0, PreviewMode="AlongVhcl", ConsiderBumps=0, pos=[3.25, 0.0, 0.3], rot_z=0.0):
        if name=="":
            name = "D%0*d" %(2, len(self._SensorList))
        for element in self._SensorList:
            if name == element.name.value:
                raise Exception("ERROR in cCMRPSensors: Sensor with the name {0} exists already!".format(name))
        self._SensorList.append(cCMRPSensor(name, PreviewDist, PreviewMode, ConsiderBumps, pos, rot_z, len(self._SensorList)))


    def GetTestRunText(self):
        testRunText = "RPSensor.N = " + str(len(self._SensorList)) + "\n"
        for currElement in self._SensorList:
            for currAttribute in currElement._Attributes:
                testRunText += currAttribute[1].getConfigFileText()
        return testRunText

    def __getitem__(self,index):
        return self._SensorList[index]

    def __setitem__(self, index, newvalue):
        self._SensorList[index] = newvalue

    def __delitem__(self, index):
         del self._SensorList[index]


### SENSORS CLASS ###
class cCMSensors(cDynamicProperty):
    def __init__(self, srcVehicleFile=None): 
        self.SSASensors  = cCMSSASensors(srcVehicleFile=srcVehicleFile)
        self.BdySensors  = cCMBdySensors(srcVehicleFile=srcVehicleFile)      
        self.DASensors   = cCMDASensors(srcVehicleFile=srcVehicleFile)      
        self.RPSensors   = cCMRPSensors(srcVehicleFile=srcVehicleFile)      
        self._Attributes = []
        #####################################
        self._Attributes.append(('headline', cSimpleDataType("## Sensors ##", str, "#############################################################", splitter="")))   
        self._Attributes.append(('Comment',          cSimpleDataType("Sensor.Comment",   str, "\n", splitter=":\n")))
        #####################################
        self._setAttributes()
                
        if srcVehicleFile is not None:
            for element in self._Attributes:
                if type(element[1]) is cSimpleDataType:
                    cSimpleDataType.readfromFile(element[1], srcVehicleFile)
                elif type(element[1]) is cSimple3Dfloat:
                    cSimple3Dfloat.readfromFile(element[1], srcVehicleFile)

    def GetTestRunText(self):
        testRunText = ""
        for currAttribute in self._Attributes:
            testRunText += currAttribute[1].getConfigFileText()
            if currAttribute[1].getConfigFileText().find("## Sensors ##") >= 0:
                testRunText += self.SSASensors.GetTestRunText()
                testRunText += self.BdySensors.GetTestRunText()
                testRunText += self.DASensors.GetTestRunText()
                testRunText += self.RPSensors.GetTestRunText()
        return testRunText
