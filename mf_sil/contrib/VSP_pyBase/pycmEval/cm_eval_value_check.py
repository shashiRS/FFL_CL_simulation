#====================================================================
# System Imports
#====================================================================
import os
import sys

#====================================================================
# Own Imports
#====================================================================
sys.path.append( os.path.join(os.path.dirname(__file__), '..', 'pycmEval') )
from pyBase.pycmEval.cm_eval_erg_parser import cErgFile
#====================================================================
# Value Checkers
#====================================================================

class ValueCheckBase(object):
    def __str__(self):
        return "Value Check"
    def evaluate(self, value):
        pass

class ValueCheckSmaller(ValueCheckBase):
    def __init__(self, smallerThanValue):
        self._smallerThanValue = smallerThanValue
    def evaluate(self, actualValue):
        result = {}
        result["asBool"]   = actualValue < self._smallerThanValue
        if result["asBool"] == True:
            result["asString"] = "{0} smaller than {1}".format(actualValue, self._smallerThanValue)
        else:
            result["asString"] = "{0} not smaller than {1}".format(actualValue, self._smallerThanValue)
        return result

class ValueCheckBigger(ValueCheckBase):
    def __init__(self, biggerThanValue):
        self._biggerThanValue = biggerThanValue
    def evaluate(self, actualValue):
        result = {}
        result["asBool"]   = actualValue > self._smallerThanValue
        if result["asBool"] == True:
            result["asString"] = "{0} bigger than {1}".format(actualValue, self._smallerThanValue)
        else:
            result["asString"] = "{0} not bigger than {1}".format(actualValue, self._smallerThanValue)
        return result

class ValueCheckAbsoluteDeviation(ValueCheckBase):
    def __init__(self, centerValue, allowedDeviation):
        self._centerValue      = centerValue
        self._allowedDeviation = allowedDeviation
    def __str__(self):
        return "Value Check absolute Deviation smaller " + str(self._allowedDeviation)
    def evaluate(self, actualValue):
        result = {}
        result["asBool"]   = (self._centerValue - self._allowedDeviation) < actualValue < (self._centerValue + self._allowedDeviation)
        if result["asBool"] == True:
            result["asString"] = "{0} between {1} and {2}".format(actualValue, self._centerValue - self._allowedDeviation, self._centerValue + self._allowedDeviation)
        else:
            result["asString"] = "{0} not between {1} and {2}".format(actualValue, self._centerValue - self._allowedDeviation, self._centerValue + self._allowedDeviation)
        return result

class ValueCheckBetween(ValueCheckBase):
    def __init__(self, minValue, maxValue):
        self._minValue = minValue
        self._maxValue = maxValue
    def __str__(self):
        return "Value Check between " + str(self._minValue) + " and " + str(self._maxValue)
    def evaluate(self, actualValue):
        result = {}
        result["asBool"]   = self._minValue < actualValue < self._maxValue
        if result["asBool"] == True:
            result["asString"] = "{0} between {1} and {2}".format(actualValue, self._minValue, self._maxValue)
        else:
            result["asString"] = "{0} not between {1} and {2}".format(actualValue, self._minValue, self._maxValue)
        return result

class cCMEvalValueChecker(object):
    def __init__(self, checks=None):
        self._checks = checks

    def addCheck(self, valueName, checkRecordID,  valueCheck ):
        self._checks.append({"valueName": valueName, "checkRecordID": checkRecordID, "valueCheck": valueCheck})

    def addChecks(self, checks):
        self._checks.extend(checks)

    def runChecks(self, ergFilePath ):
        result = []
        # Open erg file for reading
        ergFile = cErgFile( ergFilePath )

        for currCheck in self._checks:
            actualValue = ergFile.readSingleSignalValue( currCheck["valueName"], currCheck["checkRecordID"] )
            checkResult = currCheck["valueCheck"].evaluate(actualValue)
            result.append({"valueName": currCheck["valueName"], "checkString": checkResult["asString"], "checkResult": checkResult["asBool"]})

        return result
