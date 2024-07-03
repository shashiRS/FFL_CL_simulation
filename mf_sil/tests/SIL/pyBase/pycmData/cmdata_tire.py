from cmdata_common import cDynamicProperty, cSimpleDataType

### TIRE CLASS ###
class cCMTire(cDynamicProperty):
    def __init__(self, srcTireFile=None):

        self._srcFile = ""

        if srcTireFile is not None:
            self._srcFile = srcTireFile[srcTireFile.find("Tire\\")+len("Tire\\"):]
            self._FileContent = ""
            with open(srcTireFile, 'r') as f:
                existingvarlist = f.readlines()
                for row in existingvarlist:
                    self._FileContent = self._FileContent + row


    def GetTestRunText(self):
        testRunText = ""
        testRunText += "Tire.0 = " + self._srcFile + "\n"
        testRunText += "Tire.1 = " + self._srcFile + "\n"
        testRunText += "Tire.2 = " + self._srcFile + "\n"
        testRunText += "Tire.3 = " + self._srcFile + "\n"
        return testRunText
   
    def save(self, fileName):
        with open(fileName,"w") as f:
            f.write(self._FileContent)
