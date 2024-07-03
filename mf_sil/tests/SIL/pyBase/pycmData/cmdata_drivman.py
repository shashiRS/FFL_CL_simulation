from cmdata_common import cDynamicProperty, cSimpleDataType

class cCMnewDrivMan(cDynamicProperty):
    def __init__(self, DrivManID, srcTestRunFile):
        self._Attributes = []
        ################################
        self._Attributes.append(('Info',         cSimpleDataType("DrivMan." + str(DrivManID) + ".Info",         str, "")))
        self._Attributes.append(('Label',        cSimpleDataType("DrivMan." + str(DrivManID) + ".Label",        str, "")))
        self._Attributes.append(('TimeLimit',    cSimpleDataType("DrivMan." + str(DrivManID) + ".TimeLimit",    float)))
        self._Attributes.append(('DistLimit',    cSimpleDataType("DrivMan." + str(DrivManID) + ".DistLimit",    float)))
        self._Attributes.append(('EndCondition', cSimpleDataType("DrivMan." + str(DrivManID) + ".EndCondition", str, "")))
        self._Attributes.append(('Cmds',         cSimpleDataType("DrivMan." + str(DrivManID) + ".Cmds",str, "", splitter=':\n')))
        self._Attributes.append(('Clutch',       cSimpleDataType("DrivMan." + str(DrivManID) + ".Clutch",       str, "0 0 0.2 abs"), "Manual"))
        self._Attributes.append(('Gas',          cSimpleDataType("DrivMan." + str(DrivManID) + ".Gas",          str, "0 0 0.2 abs"), "Manual"))
        self._Attributes.append(('Brake',        cSimpleDataType("DrivMan." + str(DrivManID) + ".Brake",        str, "0 0 0.2 abs"), "Manual"))
        self._Attributes.append(('BrakePark',    cSimpleDataType("DrivMan." + str(DrivManID) + ".BrakePark",    str, "0 0 0.2 abs"), "Manual"))
        self._Attributes.append(('Gear',         cSimpleDataType("DrivMan." + str(DrivManID) + ".Gear",         str, "0 0 0.0 abs"), "Manual"))
        self._Attributes.append(('LongDyn',      cSimpleDataType("DrivMan." + str(DrivManID) + ".LongDyn",      str, "Driver")))
        self._Attributes.append(('LatDyn',       cSimpleDataType("DrivMan." + str(DrivManID) + ".LatDyn",       str, "Driver 0")))
        self._Attributes.append(('SteerMode',    cSimpleDataType("DrivMan." + str(DrivManID) + ".SteerMode",    str,   "")))
        ################################
        self._setAttributes()

        if srcTestRunFile is not None:
            for element in self._Attributes:
                if type(element[1]) is cSimpleDataType:
                    cSimpleDataType.readfromFile(element[1], srcTestRunFile)
                elif type(element[1]) is cSimple3Dfloat:
                    cSimple3Dfloat.readfromFile(element[1], srcTestRunFile)

class cCMDrivManList(cDynamicProperty):
    def __init__(self, srcTestRunFile=None):
        self._DrivManList = []
        if srcTestRunFile is None:
            self.add()
        else:
            with open(srcTestRunFile, 'r') as f:
                for row in f:
                    if row.find("DrivMan.nDMan") is 0:
                        noDrivMan = int(row[len("DrivMan.nDMan = "):row.find("\n")])
                        for DrivManID in range(noDrivMan):
                            self._DrivManList.append(cCMnewDrivMan(DrivManID, srcTestRunFile))

    def add(self):
        self._DrivManList.append(cCMnewDrivMan(len(self._DrivManList), None))

    def GetTestRunText(self):
        testRunText = "DrivMan.nDMan = " + str(len(self._DrivManList)) + "\n"
        for currElement in self._DrivManList:
            for currAttribute in currElement._Attributes:
                if currAttribute[0] ==  'LongDyn':
                    testValue = currAttribute[1].value
            for currAttribute in currElement._Attributes:
                if   (len(currAttribute) > 2          and testValue != currAttribute[2])  \
                  or (currAttribute[0] == 'Info'      and currAttribute[1].value == "" )  \
                  or (currAttribute[0] == 'Label'     and currAttribute[1].value == "" )  \
                  or (currAttribute[0] == 'DistLimit' and currAttribute[1].value == 0  )  \
                  or (currAttribute[0] == 'Cmds'      and currAttribute[1].value == "" ):
                    pass
                else:
                    testRunText += currAttribute[1].getConfigFileText()
        return testRunText

    def __getitem__(self,index):
        return self._DrivManList[index]

    def __setitem__(self, index, newvalue):
        self._DrivManList[index] = newvalue

    def __delitem__(self, index):
         del self._DrivManList[index]

### DRVING MANEUVER CLASS ###
class cCMDrivMan(cDynamicProperty):
    def __init__(self, srcTestRunFile=None):
        self.DrivManList = cCMDrivManList(srcTestRunFile)
        self._Attributes = []
        ################################
        self._Attributes.append(('Cmds',                cSimpleDataType("DrivMan.Cmds", str, "",   splitter=':\n')))
        self._Attributes.append(('Init.Velocity',       cSimpleDataType("DrivMan.Init.Velocity", int, 0)))
        self._Attributes.append(('Init.GearNo',         cSimpleDataType("DrivMan.Init.GearNo", int, 0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('Init.SteerAng',       cSimpleDataType("DrivMan.Init.SteerAng", int, 0)))
        self._Attributes.append(('Init.LaneOffset',     cSimpleDataType("DrivMan.Init.LaneOffset", float, 0.0)))
        self._Attributes.append(('Init.OperatorActive', cSimpleDataType("DrivMan.Init.OperatorActive", int, 1)))
        self._Attributes.append(('Init.OperatorState',  cSimpleDataType("DrivMan.Init.OperatorState", str, "drive")))
        self._Attributes.append(('VhclOperator.Kind',   cSimpleDataType("DrivMan.VhclOperator.Kind", str, "IPGOperator 1")))
        #self._Attributes.append(('nDMan',              cSimpleDataType("DrivMan.nDMan", int, 1, valueChecker=lambda x: x>=0)))
        
        self._Attributes.append(('OW.Quantities',       cSimpleDataType("DrivMan.OW.Active", int, 0)))
        self._Attributes.append(('OW.Quantities',       cSimpleDataType("DrivMan.OW.Quantities", str, "Time")))
        #self._Attributes.append(('OW.FName',            cSimpleDataType("DrivMan.OW.FName", str, "")))
        self._Attributes.append(('OW.StartGearNo',      cSimpleDataType("DrivMan.OW.StartGearNo", int)))
        self._Attributes.append(('OW.StartVelocity',    cSimpleDataType("DrivMan.OW.StartVelocity", float)))
        self._Attributes.append(('OW.GasMax',           cSimpleDataType("DrivMan.OW.GasMax", float, 0.5)))
        self._Attributes.append(('OW.Time.Name',        cSimpleDataType("DrivMan.OW.Time.Name", str, "")))
        self._Attributes.append(('OW.Time.Factor',      cSimpleDataType("DrivMan.OW.Time.Factor", float, 1.0)))
        self._Attributes.append(('OW.Time.Offset',      cSimpleDataType("DrivMan.OW.Time.Offset", float, 0.0)))
        #self._Attributes.append(('OW.Velocity.Name',    cSimpleDataType("DrivMan.OW.Velocity.Name", str, "")))
        #self._Attributes.append(('OW.Velocity.Factor',  cSimpleDataType("DrivMan.OW.Velocity.Factor", float, 0.0)))
        #self._Attributes.append(('OW.Velocity.Offset',  cSimpleDataType("DrivMan.OW.Velocity.Offset", float, 0.0)))
        #self._Attributes.append(('OW.User1.Name',       cSimpleDataType("DrivMan.OW.User1.Name", str, "")))
        #self._Attributes.append(('OW.User1.Factor',     cSimpleDataType("DrivMan.OW.User1.Factor", float, 0.0)))
        #self._Attributes.append(('OW.User1.Offset',     cSimpleDataType("DrivMan.OW.User1.Offset", float, 0.0)))
        #self._Attributes.append(('OW.User2.Name',       cSimpleDataType("DrivMan.OW.User2.Name", str, "")))
        #self._Attributes.append(('OW.User2.Factor',     cSimpleDataType("DrivMan.OW.User2.Factor", float, 0.0)))
        #self._Attributes.append(('OW.User2.Offset',     cSimpleDataType("DrivMan.OW.User2.Offset", float, 0.0)))
        #self._Attributes.append(('OW.User3.Name',       cSimpleDataType("DrivMan.OW.User3.Name", str, "")))
        #self._Attributes.append(('OW.User3.Factor',     cSimpleDataType("DrivMan.OW.User3.Factor", float, 0.0)))
        #self._Attributes.append(('OW.User3.Offset',     cSimpleDataType("DrivMan.OW.User3.Offset", float, 0.0)))
        #self._Attributes.append(('OW.User4.Name',       cSimpleDataType("DrivMan.OW.User4.Name", str, "")))
        #self._Attributes.append(('OW.User4.Factor',     cSimpleDataType("DrivMan.OW.User4.Factor", float, 0.0)))
        #self._Attributes.append(('OW.User4.Offset',     cSimpleDataType("DrivMan.OW.User4.Offset", float, 0.0)))
        #self._Attributes.append(('OW.User5.Name',       cSimpleDataType("DrivMan.OW.User5.Name", str, "")))
        #self._Attributes.append(('OW.User5.Factor',     cSimpleDataType("DrivMan.OW.User5.Factor", float, 0.0)))
        #self._Attributes.append(('OW.User5.Offset',     cSimpleDataType("DrivMan.OW.User5.Offset", float, 0.0)))
        #self._Attributes.append(('OW.User6.Name',       cSimpleDataType("DrivMan.OW.User6.Name", str, "")))
        #self._Attributes.append(('OW.User6.Factor',     cSimpleDataType("DrivMan.OW.User6.Factor", float, 0.0)))
        #self._Attributes.append(('OW.User6.Offset',     cSimpleDataType("DrivMan.OW.User6.Offset", float, 0.0)))
        ################################
        self._setAttributes()
        
        #HACK, TODO ueberarbeiten
        self.fname = cSimpleDataType("DrivMan.OW.FName", str, "")
        
        if srcTestRunFile is not None:
            
            cSimpleDataType.readfromFile(self.fname, srcTestRunFile, EndStrNxtLn='DrivMan.OW.StartGearNo')    
                    
            for element in self._Attributes:
                if type(element[1]) is cSimpleDataType:
                    #cSimpleDataType.readfromFile(element[1], srcTestRunFile, EndStrNxtLn='Traffic.N') ##  EndStrNxtLn for Workaround for Text parts
                    cSimpleDataType.readfromFile(element[1], srcTestRunFile) ##  EndStrNxtLn for Workaround for Text parts
                elif type(element[1]) is cSimple3Dfloat:
                    cSimple3Dfloat.readfromFile(element[1], srcTestRunFile)

    def GetTestRunText(self):
        testRunText = ""
        for currAttribute in self._Attributes:
            testRunText += currAttribute[1].getConfigFileText()
        testRunText += self.DrivManList.GetTestRunText()
        return testRunText
    
    def get_fname(self):
        return self.fname.value

