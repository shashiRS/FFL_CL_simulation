#====================================================================
# Simple Text Replacement cmData Substitute
#====================================================================
import os

class cmTestRunSimple(object):
    def __init__(self, name, baseFilePath):
        self._name               = name
        self._baseFilePath       = baseFilePath
        self._replacements       = []
        self._replacementsBefore = []

    def addReplacement(self, lineStart, newValue):
        self._replacements.append({"currReplacement": lineStart, "newValue": newValue})
        
    def addReplacementBefore(self, lineStart, newValue):
        self._replacementsBefore.append({"currReplacement": lineStart, "newValue": newValue})

    def GetName(self):
        return self._name

    def save(self, outputFolder):
        # Load base file
        with open (self._baseFilePath, "r") as baseFile:
            baseFileLines = baseFile.readlines()
        
        # Save newly created file
        outputFilePath = os.path.join(outputFolder, self._name)
        with open(outputFilePath,"w") as outputFile:
            for currLine in baseFileLines:
                # replace lines in base file
                for currReplacement in self._replacements:
                    if currLine.startswith(currReplacement["currReplacement"]):
                        currLine = currReplacement["currReplacement"] + currReplacement["newValue"] + "\n"
                for currReplacement in self._replacementsBefore:
                    if currLine.startswith(currReplacement["currReplacement"]):
                        currLine = currReplacement["newValue"] + "\n" + currReplacement["currReplacement"] + "\n"
                outputFile.write(currLine)

    def getValues(self, lineStart):
        # Load base file
        with open (self._baseFilePath, "r") as baseFile:
            baseFileLines = baseFile.readlines()

        # Find line containing the desired values
        strWithValues = ""
        for currLine in baseFileLines:
            if currLine.startswith(lineStart):
                strWithValues = currLine[len(lineStart):]
                break

        #Extract values
        listOfValues = []
        for x in strWithValues.split():
            try:
                int(x)
                listOfValues.append(x)
            except ValueError:
                try:
                    float(x)
                    listOfValues.append(x)
                except ValueError:
                    pass
        return listOfValues

