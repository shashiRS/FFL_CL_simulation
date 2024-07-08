import datetime
from cmdata_common import cDynamicProperty, cSimpleDataType, cSimple3Dfloat, cSimple6Dfloat

### TRAILER CLASS ###
class cCMTrailer(cDynamicProperty):
    def __init__(self, srcTestRunFile=None):
        ################################
        self._srcFile = "Trailer ="
        ################################

        if srcTestRunFile is not None:
            with open(srcTestRunFile, 'r') as f:
                existingvarlist = f.readlines()
                for row in existingvarlist:
                    if (row).find(self._srcFile) is 0:
                        self._srcFile = self._srcFile + row[len(self._srcFile):-1]
                

    def GetTestRunText(self):
        return self._srcFile + "\n"

### TRAILERLOAD CLASS ###
class cCMTrailerLoad(cDynamicProperty):
    def __init__(self, srcTestRunFile=None):
        self._Attributes = []
        ################################
        self._Attributes.append(('no0.mass', cSimpleDataType("TrailerLoad.0.mass", int, 0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('no0.pos',  cSimple3Dfloat("TrailerLoad.0.pos", [0.0, 0.0, 0.0])))
        self._Attributes.append(('no1.mass',  cSimpleDataType("TrailerLoad.1.mass", int, 0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('no1.pos',   cSimple3Dfloat("TrailerLoad.1.pos", [0.0, 0.0, 0.0])))
        self._Attributes.append(('no2.mass',  cSimpleDataType("TrailerLoad.2.mass", int, 0, valueChecker=lambda x: x>=0)))
        self._Attributes.append(('no2.pos',   cSimple3Dfloat("TrailerLoad.2.pos", [0.0, 0.0, 0.0])))
        ################################
        self._setAttributes()

        if srcTestRunFile is not None:
            for element in self._Attributes:
                if type(element[1]) is cSimpleDataType:
                    cSimpleDataType.readfromFile(element[1], srcTestRunFile)
                elif type(element[1]) is cSimple3Dfloat:
                    cSimple3Dfloat.readfromFile(element[1], srcTestRunFile)

    def GetTestRunText(self):
        testRunText = ""
        for currAttribute in self._Attributes:
            testRunText += currAttribute[1].getConfigFileText()
        return testRunText