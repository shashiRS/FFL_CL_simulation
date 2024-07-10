#====================================================================
# System Imports
#====================================================================
import os
import sys
from itertools import product

#====================================================================
# Own Imports
#====================================================================
sys.path.append( os.path.join(os.path.dirname(__file__), '..', 'pycmEval') )
from cm_testrun import cmTestRun

#====================================================================
# Parameter
#====================================================================
activationEnabledDefault = True
targetName               = 'T00'
activationEnabledSuffix  = "_ae"

#====================================================================
# Local Functions
#====================================================================
def AddActivation(testRun, activationEnabled):
    if activationEnabled:
        testRun.DrivMan.DrivManList[0].Cmds.value += "\tEval FCT.ActivationEnabled = 1 \n"
    else:
        testRun.DrivMan.DrivManList[0].Cmds.value += "\tEval FCT.ActivationEnabled = 0 \n"


#====================================================================
# TestRun Factories
#====================================================================
def FactoryTestRunBase(testRunName, testRunBasePath, activationEnabled):
    if activationEnabled:
        testRunName += activationEnabledSuffix

    testRun = cmTestRun(testRunName, testRunBasePath)
    testRun.addReplacement("Traffic.0.Name =", targetName)


    return testRun

def FactoryTestHostConstantTargetStraight(testRunName, activationEnabled, vTarget, vHost, cm_version):
     TestcaseHostConstantTargetStraight = os.path.join(os.path.dirname(__file__), 'cmData', f'CM_{cm_version}', 'TestRun', 'TurnAssist_Turning_Left_Host_ConstantUseCase2.testrun')
     testRun = FactoryTestRunBase(testRunName, TestcaseHostConstantTargetStraight, activationEnabled)

     #Host Variables
     CurveMinimum= vHost*vHost/127
     CurveLenght= (3.1415 * CurveMinimum)/2 #The distance the car needs to run
     IntersectionPoint = 400.7
     CarLenght= 6
     StartingPosition= IntersectionPoint-CurveMinimum-CarLenght
     CarSpeed= vHost/3.6 #Convering K/h to meters/s)
     CurveDistance = IntersectionPoint - StartingPosition
     DurationTilIntersectionPoint= CurveDistance/CarSpeed
     
     #TargetVariables
     DurationTilIntersectionPoint2= CurveDistance/2/CarSpeed
     CarSpeedTarget= vTarget/3.6 #Convering K/h to meters/s)
     TargetDistance = CarSpeedTarget * DurationTilIntersectionPoint2
     offsetX = 396 - TargetDistance
     offsetY= -1.6

     #Host
     testRun.addReplacement("DrivMan.Init.Velocity =", str(vHost))
     testRun.addReplacement("DrivMan.0.TimeLimit =", str(DurationTilIntersectionPoint))
     testRun.addReplacement("DrivMan.0.DistLimit =", str(CurveDistance))
     testRun.addReplacement("Road.VhclStartPos =", "0")
     testRun.addReplacement("DrivMan.0.LatDyn =", "Driver 0")
     testRun.addReplacement("DrivMan.0.LongDyn =", "{0}" "{1}" "{2}" "{3}".format("Driver "," 1"," 0 ",str(vHost)))
     testRun.addReplacement("DrivMan.1.LongDyn =", "{0}" "{1}" "{2}" "{3}".format("Driver "," 1"," 0 ",str(vHost)))
     if vHost==20 :
         testRun.addReplacement("DrivMan.0.LatDyn =", "Driver 0")
     if vHost==30 :
         testRun.addReplacement("DrivMan.0.LatDyn =", "Driver 0")
     #Target
     testRun.addReplacement("Traffic.0.Init.Road =", "{0} {1}".format(offsetX, offsetY))
     testRun.addReplacement("Traffic.0.Init.v =", str(vTarget))

     if activationEnabled:
         testRun.SetFlag("AE")
     else:
         testRun.SetFlag("BASE")

     return testRun


def GetTestCases(testCaseSet, CMPrjDir, cm_version, activationEnabled=activationEnabledDefault):
    TestCases = []
    if testCaseSet == "smokeTest":
        targetVelocityList = [50,60]
        hostVelocityList = [20,30]
        for currSetup in product(hostVelocityList, targetVelocityList):
            currHostVelocity  = currSetup[0]
            currTargetVelocity = currSetup[1]
            currName = "UseCases-001_TarStraight_vHost_{0:.2f}_vTar_{1:.2f}".format(currHostVelocity,currTargetVelocity)
            TestCases.append({"caseType":"useCase", "cmTestRun":FactoryTestHostConstantTargetStraight(currName, activationEnabled, currTargetVelocity, currHostVelocity, cm_version)})
    else:
        raise SystemError("TestCaseSet {0} unknown".format(testCaseSet))
    return TestCases

def GetTestCasesHeadless(CMPrjDir):
    TestCases = []

    name = 'UseCases-001_TarStraight_vHost_20.00_vTar_50.00'
    path = os.path.join(CMPrjDir, 'Data', 'TestRun', 'smokeTests', 'UseCases-001_TarStraight_vHost_20.00_vTar_50.00')
    testRun = cmTestRun(name, path)
    TestCases.append(testRun)

    name = 'UseCases-001_TarStraight_vHost_20.00_vTar_60.00'
    path = os.path.join(CMPrjDir, 'Data', 'TestRun', 'smokeTests', 'UseCases-001_TarStraight_vHost_20.00_vTar_60.00')
    testRun = cmTestRun(name, path)
    TestCases.append(testRun)

    name = 'UseCases-001_TarStraight_vHost_30.00_vTar_50.00'
    path = os.path.join(CMPrjDir, 'Data', 'TestRun', 'smokeTests', 'UseCases-001_TarStraight_vHost_30.00_vTar_50.00')
    testRun = cmTestRun(name, path)
    TestCases.append(testRun)

    name = 'UseCases-001_TarStraight_vHost_30.00_vTar_60.00'
    path = os.path.join(CMPrjDir, 'Data', 'TestRun', 'smokeTests', 'UseCases-001_TarStraight_vHost_30.00_vTar_60.00')
    testRun = cmTestRun(name, path)
    TestCases.append(testRun)

    return TestCases


def GetCrashingTestcase(cm_version):
    return cmTestRun('CrashingTestrun', os.path.join(os.path.dirname(__file__), 'cmData', f'CM_{cm_version}', 'TestRun', 'FailTestrun-001_vHost_80_vTar_50.testrun'))

def GetCrashingTestcaseHeadless(CMPrjDir):
    return cmTestRun('CrashingTestrun', os.path.join(CMPrjDir, 'Data', 'TestRun', 'smokeTests', 'CrashingTestrun'))

#def GetLongTestcase(cm_version):
#    return cmTestRun('LongTestrun', os.path.join(os.path.dirname(__file__), 'cmData', f'CM_{cm_version}', 'TestRun', 'LongTestrun.testrun'))

def GetLongTestcaseHeadless(CMPrjDir):
    return cmTestRun('LongTestrun', os.path.join(CMPrjDir, 'Data', 'TestRun', 'smokeTests', 'LongTestrun'))

if __name__ == '__main__':
    # Retrieve all TestCases and TestRuns
    testCasesBase = GetTestCases('performanceFull', CMPrjDir, False)
    testCasesAE   = GetTestCases('performanceFull', CMPrjDir, True)