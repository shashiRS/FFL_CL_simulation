from cmdata_common import cDynamicProperty, cSimpleDataType, cSimple3Dfloat

### TRAFFIC ELEMENT CLASS ###
class cCMTrafficElement(cDynamicProperty):
    def __init__(self, Type, TrafficID, srcTestRunFile=None):
        if Type is None or Type is "CompactCar":
            self.CompactCar(TrafficID, srcTestRunFile)

    def CompactCar(self, TrafficID, srcTestRunFile):
        self._Attributes = []
        self._Attributes.append(('ObjectKind',          cSimpleDataType("Traffic." + str(TrafficID) + ".ObjectKind", str, "StatWithName")))
        self._Attributes.append(('Name',                   cSimpleDataType("Traffic." + str(TrafficID) + ".Name", str, "T%0*d" % (2,TrafficID))))
        self._Attributes.append(('Info',                   cSimpleDataType("Traffic." + str(TrafficID) + ".Info", str, "Compact Car")))
        self._Attributes.append(('Movie.Geometry',         cSimpleDataType("Traffic." + str(TrafficID) + ".Movie.Geometry", str, "VW_Passat_B8_Variant_noWheels_rotated_black.obj")))
        self._Attributes.append(('Color',                  cSimple3Dfloat("Traffic." + str(TrafficID) + ".Color", [1.0, 1.0, 1.0])))
        self._Attributes.append(('Basics.Dimension',       cSimple3Dfloat("Traffic." + str(TrafficID) + ".Basics.Dimension", [4.767, 1.832, 1.2])))
        self._Attributes.append(('Basics.Offset',          cSimpleDataType("Traffic." + str(TrafficID) + ".Basics.Offset", str, "0.0 0.0")))
        self._Attributes.append(('Basics.Fr12CoM',          cSimpleDataType("Traffic." + str(TrafficID) + ".Basics.Fr12CoM", float, 2.15)))
        self._Attributes.append(('Basics.Contour.Mirror',  cSimpleDataType("Traffic." + str(TrafficID) + ".Basics.Contour.Mirror", int, 0)))
        self._Attributes.append(('Basics.Contour.Contour', cSimpleDataType("Traffic." + str(TrafficID) + ".Basics.Contour", str, "", splitter=":\n")))
        self._Attributes.append(('Init.Orientation',       cSimple3Dfloat("Traffic." + str(TrafficID) + ".Init.Orientation", [0.0, 0.0, 0.0])))
        self._Attributes.append(('Attrib',               cSimpleDataType("Traffic." + str(TrafficID) + ".Attrib", str, "0.0 0.0")))
        self._Attributes.append(('RCSClass',               cSimpleDataType("Traffic." + str(TrafficID) + ".RCSClass", str, "RCS_Car")))
        #self._Attributes.append(('Oncoming',               cSimpleDataType("Traffic." + str(TrafficID) + ".Oncoming", int, 0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('FreeMotion',             cSimpleDataType("Traffic." + str(TrafficID) + ".FreeMotion", int, 0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('DetectMask',               cSimpleDataType("Traffic." + str(TrafficID) + ".DetectMask", str, "1.0 1.0")))
        self._Attributes.append(('Route',               cSimpleDataType("Traffic." + str(TrafficID) + ".Route", str, "0 0")))
        self._Attributes.append(('Init.v',                 cSimpleDataType("Traffic." + str(TrafficID) + ".Init.v", float, 0.0)))
        self._Attributes.append(('Init.Road',              cSimpleDataType("Traffic." + str(TrafficID) + ".Init.Road", str, (str(10.0*TrafficID)+" 0.0"))))
        self._Attributes.append(('Man.N',                  cSimpleDataType("Traffic." + str(TrafficID) + ".Man.N", str, "0")))
        #self._Attributes.append(('Man.StartCondition',     cSimpleDataType("Traffic." + str(TrafficID) + ".Man.StartCondition", str, "")))
        self._Attributes.append(('Man.TreatAtEnd',         cSimpleDataType("Traffic." + str(TrafficID) + ".Man.TreatAtEnd", str, "")))
        
        
        self._setAttributes()

        if srcTestRunFile is not None:
            for element in self._Attributes:
                if element[0] == "Man.N":
                    with open(srcTestRunFile, 'r') as f:
                        existingvarlist = f.readlines()
                        counter = 0
                        for row in existingvarlist:
                            counter += 1
                            if (row).find("Traffic." + str(TrafficID) + ".Man.N") is 0:
                                helpvar = row[len("Traffic." + str(TrafficID) + ".Man.N = "):]
                                while (existingvarlist[counter].find("Traffic." + str(TrafficID+1) + ".Name") is -1 and existingvarlist[counter].find("Traffic") is 0) \
                                or existingvarlist[counter].find("DrivMan.OW.") is 0:
                                    helpvar = helpvar + existingvarlist[counter] 
                                    counter += 1
                                element[1]._value = element[1]._datatype((helpvar[:-1]))
                                element[1]._rightDataType(element[1]._value)                    
                elif type(element[1]) is cSimpleDataType:
                    cSimpleDataType.readfromFile(element[1], srcTestRunFile)
                elif type(element[1]) is cSimple3Dfloat:
                    cSimple3Dfloat.readfromFile(element[1], srcTestRunFile)


### TRAFFIC CLASS ###
class cCMTraffic(cDynamicProperty):
    def __init__(self, srcTestRunFile=None):
        self.SpeedUnit  =  "ms"
        self.Traffic  =  []
        self.ObjectKind  =  "StatWithName"
        self.Dimensions  =  []
        #self.Traffic.append(cCMTrafficElement(Type=None, TrafficID=len(self.Traffic)))

        if srcTestRunFile is not None:
            with open(srcTestRunFile, 'r') as f:
                for row in f:
                    if row.find("Traffic.SpeedUnit = ") is 0:
                        self.SpeedUnit = row[len("Traffic.SpeedUnit = "):-1]
                    if row.find("Traffic.N =") is 0:
                        NoTrafficElements = int(row[len("Traffic.N ="):])
                    if row.find("Traffic.Dimensions=") is 0:
                        self.Dimensions=row[len("Traffic.Dimensions="):-1]
            for actTrafficElement in range(NoTrafficElements):
                self.Traffic.append(cCMTrafficElement(Type=None, TrafficID=actTrafficElement, srcTestRunFile=srcTestRunFile))

    
    def add(self, Type=None):
        self.Traffic.append(cCMTrafficElement(Type=Type, TrafficID=len(self.Traffic)))

    def GetTestRunText(self):
        testRunText = "Traffic.N = " + str(len(self.Traffic)) + "\n"
        testRunText += "Traffic.SpeedUnit = " + self.SpeedUnit + "\n"
    
        for currElement in self.Traffic:
            for currAttribute in currElement._Attributes:
                # add contour specific lines in textfile only if there are contour points given. Otherwise there will be an error in carmaker missing contour points
                if "Contour" in currAttribute[0]:
                    for currContourAttribute in currElement._Attributes:
                        if "Basics.Contour.Contour" in currContourAttribute[0]:
                            if currContourAttribute[1].value != "":
                                testRunText += currAttribute[1].getConfigFileText()
                else:
                    testRunText += currAttribute[1].getConfigFileText()
        return testRunText


