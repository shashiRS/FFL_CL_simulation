import os

class cmTestRun(object):
    """
    Class that defines a testrun object for cmEvalRunner.
    The idea is to load a base testrun and change some parameters to have a permutation.
    Therefore it offers methods to replace specific lines in the testrun file and save a new adapted testrun file.

    :param name: Name of the testrun. Does not need to match the name of a testrun file.
    :param baseFilePath: Absolute path to a testrun file to be loaded
    """

    def __init__(self, name, baseFilePath):
        self._name         = name
        self._baseFilePath = baseFilePath
        self._replacements = []
        self._execStatus   = None
        self._errMsg       = None
        self._ergPath      = None
        self._flag         = None

    def addReplacement(self, lineStart, newValue):
        """
        Replace a string in the testrun file by another one.
        Ex: addReplacement("DrivMan.Init.Velocity =", "30")
        old line: DrivMan.Init.Velocity =20
        new line: DrivMan.Init.Velocity =30

        :param lineStart: Start string of a line in the testrun file that shall be replaced
        :param newValue: New value for the selected line. The string "newValue" is appended to the lineStart. \
        """
        self._replacements.append({"currReplacement": lineStart, "newValue": newValue})

    def GetName(self):
        """
        Return the name of the testrun

        :return: Name of the testrun
        """

        return self._name

    def GetStatus(self):
        """
        Return if the execution of the testrun was successful. If True: successful. If False: Not successful.

        :return: Execution status of the testrun
        """

        return self._execStatus

    def GetErgPath(self):
        """
        Return the path to the erg result file of the testrun. Return None, if the testrun was not successfully executed.

        :return: Path to erg file if the execution was successful. Otherwise None.
        """

        return self._ergPath

    def SetFlag(self, flag):
        """
        Set a flag to a testrun. The flag is not evaluated by pyBase. So the user can give anything. This is a possibility to \
        pass some additional information to a testrun object that can be useful for the custom evaluation. \
        A flag does not need to be given and is optional.

        :param flag: Flag for the testrun. Can be any data type. This flag is not evaluated by pyBase.
        """

        self._flag = flag

    def GetFlag(self):
        """
        Return a user-defined flag.

        :return: User-defined flag.
        """

        return self._flag


    def SetExecStatusAndErgPath(self, execStatus, ergPath):
        """
        Set the execution status and erg path for the testrun.

        :param execStatus: Status of the execution. True if the execution was successful. False if not.
        :param ergPath: Path to erg file if the execution was successful. None if not.
        """
        self._execStatus = execStatus
        self._ergPath = ergPath

    def SetErrMsg(self, msg):
        """
        Set the error message for the testrun if any.

        :param msg: Error message.
        """
        self._errMsg = msg

    def GetErrMsg(self):
        """
        Return the error message if any.

        :return: Error message.
        """
        return self._errMsg

    def save(self, outputFolder):
        """
        Saves the testrun. The filename will be the testrun name. It is saved in the given output folder.

        :param outputFolder: Absolute path to output folder where the testrun shall be saved
        """

        # Load base file
        with open (self._baseFilePath, "r") as baseFile:
            baseFileLines=baseFile.readlines()

        # Save newly created file
        outputFilePath = os.path.join(outputFolder, self._name)
        with open(outputFilePath,"w") as outputFile:
            for currLine in baseFileLines:
                # replace lines in base file
                for currReplacement in self._replacements:
                    if currLine.startswith(currReplacement["currReplacement"]):
                        currLine = currReplacement["currReplacement"] + currReplacement["newValue"] + "\n"
                outputFile.write(currLine)