#====================================================================
# System Imports
#====================================================================
import os
import sys
import unittest
import platform
import logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s [%(name)s] %(message)s', datefmt='%d-%m-%Y %H:%M:%S')

#====================================================================
# Own Imports
#====================================================================
import testcases
sys.path.append( os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'pycmEval') )
sys.path.append( os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'pycmData') )
sys.path.append( os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'pyHTMLReport') )
sys.path.append( os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'pyUtils') )
sys.path.append( os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'packImages') )
from cm_eval_runner import cCMEvalTestRunner

#====================================================================
# Config
#====================================================================
CM_VERSION = "8.1.1"
CMPrjDir                   = os.path.join(os.path.dirname(os.path.abspath(__file__)), f'cm_prj_{CM_VERSION}')
testCaseSetName            = 'smokeTest'
maxTimeInSimulationSeconds = 30.0
N_RETRIES                  = 5
TEST_CORE_LICENSE_MODE     = True # only possible if your user account has assigned a core license
export_quantities = [
    "Car.Fr1.ax", \
    "Car.Fr1.ay", \
    "Time", \
]


class TestPyBase(unittest.TestCase):
    """
    Class that contains tests for pyBase. These tests shall be executed before a new version is released to ensure the
    consistency and stability of pyBase.
    """

    #====================================================================
    # Usecases
    #====================================================================

    def getTestruns(self):
        """
        Helper function to define the testruns that shall be executed.
        """
        testCases = testcases.GetTestCases(testCaseSetName, CMPrjDir, CM_VERSION, False)
        global testRunsBase
        testRuns = [currTestCase["cmTestRun"] for currTestCase in testCases]
        return testRuns
    
    def getTestrunsHeadless(self, CMPrjDir):
        """
        Helper function to define the testruns that shall be executed especially for headless mode.
        """
        testCases = testcases.GetTestCasesHeadless(CMPrjDir)
        return testCases
    
    def test_headlessModeMultiInstance_longTestrun(self):
        NUMBER_OF_CARMAKER_INSTANCES = 2
        testRuns = self.getTestrunsHeadless(CMPrjDir)
        testRuns.insert(1, testcases.GetLongTestcaseHeadless(CMPrjDir))
        numTestruns = len(testRuns)
        # Run all TestRuns
        testRunner  = cCMEvalTestRunner(CMPrjDir, export_quantities, NUMBER_OF_CARMAKER_INSTANCES, sampleRate='default')
        testRunList = testRunner.runEvaluations(testCaseSetName, testRuns, maxTimeInSimulationSeconds=maxTimeInSimulationSeconds, 
                                                headlessMode = True, useCoreLicense = False, nRetries = N_RETRIES,
                                                ignoreExistingResFile = True)
        self.assertEqual(len(testRunList), numTestruns)
        for currTestrun in testRunList:
            self.assertEqual(currTestrun.GetStatus(), True)
    
    
    def test_guiMode(self):
        """
        Test the Carmaker simulation GUI mode.
        """
        NUMBER_OF_CARMAKER_INSTANCES = 1
        testRuns = self.getTestruns()
        numTestruns = len(testRuns)
        # Run all TestRuns
        if platform.system() == 'Linux':
            testRunner  = cCMEvalTestRunner(CMPrjDir, export_quantities, NUMBER_OF_CARMAKER_INSTANCES, sampleRate='default', cmGuiPath=r"/opt/ipg/carmaker/linux64-{}/bin/CM".format(CM_VERSION))
        else:
            testRunner  = cCMEvalTestRunner(CMPrjDir, export_quantities, NUMBER_OF_CARMAKER_INSTANCES, sampleRate='default', cmGuiPath=r"C:\LegacyApp\IPG\carmaker\win64-{}\bin\CM.exe".format(CM_VERSION))
        testRunList = testRunner.runEvaluations(testCaseSetName, testRuns, maxTimeInSimulationSeconds=maxTimeInSimulationSeconds, 
                                                headlessMode = False, useCoreLicense = False, nRetries = N_RETRIES,
                                                ignoreExistingResFile = True)
        self.assertEqual(len(testRunList), numTestruns)
        for currTestrun in testRunList:
            self.assertEqual(currTestrun.GetStatus(), True)
    
    
    def test_headlessMode(self):
        """
        Test the Carmaker simulation headless mode.
        """
        NUMBER_OF_CARMAKER_INSTANCES = 1
        testRuns = self.getTestrunsHeadless(CMPrjDir)
        numTestruns = len(testRuns)
        # Run all TestRuns
        testRunner  = cCMEvalTestRunner(CMPrjDir, export_quantities, NUMBER_OF_CARMAKER_INSTANCES, sampleRate='default')
        testRunList = testRunner.runEvaluations(testCaseSetName, testRuns, maxTimeInSimulationSeconds=maxTimeInSimulationSeconds, 
                                                headlessMode = True, useCoreLicense = False, nRetries = N_RETRIES,
                                                ignoreExistingResFile = True)
        self.assertEqual(len(testRunList), numTestruns)
        for currTestrun in testRunList:
            self.assertEqual(currTestrun.GetStatus(), True)
    
    
    def test_hpclightMode(self):
        """
        Test the Carmaker simulation HPC light mode.
        """
        NUM_TESTRUNS_IN_TSFILE = 4
        # Run all TestRuns
        testRunner  = cCMEvalTestRunner(CMPrjDir, export_quantities, cmGuiPath = r"C:\LegacyApp\IPG\carmaker\win64-{}\bin\CM.exe".format(CM_VERSION))
        ergFilePaths = testRunner.runEvaluations(testCaseSetName, 'smokeTest.ts', nRetries = N_RETRIES, executable = os.path.join(CMPrjDir, "src", "CarMaker.win64.exe"))
        self.assertEqual(len(ergFilePaths), NUM_TESTRUNS_IN_TSFILE)
    
    
    def test_corelicenseMode(self):
        """
        Test the Carmaker simulation continuous testing / core license mode.
        """
        if TEST_CORE_LICENSE_MODE:
            NUMBER_OF_CARMAKER_INSTANCES = 1
            testRuns = self.getTestrunsHeadless(CMPrjDir)
            numTestruns = len(testRuns)
            # Run all TestRuns
            testRunner  = cCMEvalTestRunner(CMPrjDir, export_quantities, NUMBER_OF_CARMAKER_INSTANCES, sampleRate='default')
            testRunList = testRunner.runEvaluations(testCaseSetName, testRuns, maxTimeInSimulationSeconds=maxTimeInSimulationSeconds, 
                                                    headlessMode = True, useCoreLicense = True, nRetries = N_RETRIES,
                                                    ignoreExistingResFile = True)
            self.assertEqual(len(testRunList), numTestruns)
            for currTestrun in testRunList:
                self.assertEqual(currTestrun.GetStatus(), True)
    
    
    def test_corelicenseModeMultiInstance(self):
        """
        Test the Carmaker simulation continuous testing / core license mode with multiple instances.
        """
        if TEST_CORE_LICENSE_MODE:
            NUMBER_OF_CARMAKER_INSTANCES = 2
            testRuns = self.getTestrunsHeadless(CMPrjDir)
            numTestruns = len(testRuns)
            # Run all TestRuns
            testRunner  = cCMEvalTestRunner(CMPrjDir, export_quantities, NUMBER_OF_CARMAKER_INSTANCES, sampleRate='default')
            testRunList = testRunner.runEvaluations(testCaseSetName, testRuns, maxTimeInSimulationSeconds=maxTimeInSimulationSeconds, 
                                                    headlessMode = True, useCoreLicense = True, nRetries = N_RETRIES,
                                                    ignoreExistingResFile = True)
            self.assertEqual(len(testRunList), numTestruns)
            for currTestrun in testRunList:
                self.assertEqual(currTestrun.GetStatus(), True)
    
    
    def test_headlessModeMultiInstance(self):
        """
        Test the Carmaker simulation headless mode with multiple instances.
        """
        NUMBER_OF_CARMAKER_INSTANCES = 2
        testRuns = self.getTestrunsHeadless(CMPrjDir)
        numTestruns = len(testRuns)
        # Run all TestRuns
        testRunner  = cCMEvalTestRunner(CMPrjDir, export_quantities, NUMBER_OF_CARMAKER_INSTANCES, sampleRate='default')
        testRunList = testRunner.runEvaluations(testCaseSetName, testRuns, maxTimeInSimulationSeconds=maxTimeInSimulationSeconds, 
                                                headlessMode = True, useCoreLicense = False, nRetries = N_RETRIES,
                                                ignoreExistingResFile = True)
        self.assertEqual(len(testRunList), numTestruns)
        for currTestrun in testRunList:
            self.assertEqual(currTestrun.GetStatus(), True)
    
    
    def test_corelicenseMode_inconsistentCfg(self):
        """
        Test the Carmaker simulation in core license mode with inconsistent headlessMode/useCoreLicense parameter configuration.
        """
        if TEST_CORE_LICENSE_MODE:
            NUMBER_OF_CARMAKER_INSTANCES = 1
            testRuns = self.getTestrunsHeadless(CMPrjDir)
            numTestruns = len(testRuns)
            # Run all TestRuns
            testRunner  = cCMEvalTestRunner(CMPrjDir, export_quantities, NUMBER_OF_CARMAKER_INSTANCES, sampleRate='default')
            testRunList = testRunner.runEvaluations(testCaseSetName, testRuns, maxTimeInSimulationSeconds=maxTimeInSimulationSeconds, 
                                                    headlessMode = False, useCoreLicense = True, nRetries = N_RETRIES,
                                                    ignoreExistingResFile = True)
            self.assertEqual(len(testRunList), numTestruns)
            for currTestrun in testRunList:
                self.assertEqual(currTestrun.GetStatus(), True)
    
    def test_headlessTimeout(self):
        """
        Test the timeout of the Carmaker simulation in headless mode. 
        """
    
        NUMBER_OF_CARMAKER_INSTANCES = 1
        testRuns = self.getTestrunsHeadless(CMPrjDir)
        numTestruns = len(testRuns)
        # Run all TestRuns
        testRunner  = cCMEvalTestRunner(CMPrjDir, export_quantities, NUMBER_OF_CARMAKER_INSTANCES, sampleRate='default')
        testRunList = testRunner.runEvaluations(testCaseSetName, testRuns, maxTimeInSimulationSeconds=maxTimeInSimulationSeconds, 
                                                headlessMode = True, useCoreLicense = False, nRetries = N_RETRIES,
                                                ignoreExistingResFile = True, maxExecTimeSeconds=0.1, raiseSimExecExceptions = False)
        self.assertEqual(len(testRunList), numTestruns)
        for currTestrun in testRunList:
            self.assertEqual(currTestrun.GetStatus(), False)
    
    
    #====================================================================
    # Negative / Robustness tests
    #====================================================================
    #def test_corelicenseMode_failedTestrun(self):
    #    """
    #    Test the Carmaker simulation continuous testing / core license mode and one testrun produces an error while simulation.
    #    """
    #    if TEST_CORE_LICENSE_MODE:
    #        NUMBER_OF_CARMAKER_INSTANCES = 1
    #        NUM_FAILED_TESTRUNS = 1
    #        testRuns = self.getTestrunsHeadless(CMPrjDir)
    #        testRuns.append(testcases.GetCrashingTestcase(CM_VERSION))
    #        numTestruns = len(testRuns)
    #        # Run all TestRuns
    #        testRunner  = cCMEvalTestRunner(CMPrjDir, export_quantities, NUMBER_OF_CARMAKER_INSTANCES, sampleRate='default')
    #        testRunList = testRunner.runEvaluations(testCaseSetName, testRuns, maxTimeInSimulationSeconds=maxTimeInSimulationSeconds, 
    #                                                headlessMode = True, useCoreLicense = True, nRetries = N_RETRIES,
    #                                                ignoreExistingResFile = True, showCMLogOutput = True,
    #                                                raiseSimExecExceptions = False)
    #        numFailedTestruns = 0
    #        for currTestrun in testRunList:
    #            if not currTestrun.GetStatus():
    #                numFailedTestruns += 1
    #        self.assertEqual(len(testRunList), numTestruns)
    #        self.assertTrue(numFailedTestruns==NUM_FAILED_TESTRUNS)
    
    def test_guiMode_wrongGuiPath(self):
        """
        Test the Carmaker simulation in GUI mode with wrong GUI path.
        """
        NUMBER_OF_CARMAKER_INSTANCES = 1
        testRuns = self.getTestruns()
        numTestruns = len(testRuns)
        # Run all TestRuns
        testRunner  = cCMEvalTestRunner(CMPrjDir, export_quantities, NUMBER_OF_CARMAKER_INSTANCES, sampleRate='default', cmGuiPath=r"D:\somewhere\CM.exe")
        with self.assertRaises(Exception):
            testRunner.runEvaluations(testCaseSetName, testRuns, maxTimeInSimulationSeconds=maxTimeInSimulationSeconds, 
                                      headlessMode = False, useCoreLicense = False, nRetries = N_RETRIES,
                                      ignoreExistingResFile = True)
    
    def test_headlessMode_wrongExePath(self):
        """
        Test the Carmaker simulation in headless mode with wrong executable path.
        """
        NUMBER_OF_CARMAKER_INSTANCES = 1
        testRuns = self.getTestrunsHeadless(CMPrjDir)
        numTestruns = len(testRuns)
        # Run all TestRuns
        testRunner  = cCMEvalTestRunner(CMPrjDir, export_quantities, NUMBER_OF_CARMAKER_INSTANCES, sampleRate='default')
        with self.assertRaises(Exception):
            testRunner.runEvaluations(testCaseSetName, testRuns, maxTimeInSimulationSeconds=maxTimeInSimulationSeconds, 
                                      headlessMode = True, useCoreLicense = False, nRetries = N_RETRIES,
                                      ignoreExistingResFile = True, executable="D:/somewhere/CM.exe")
    
    def test_corelicenseMode_wrongExePath(self):
        """
        Test the Carmaker simulation in core license mode with wrong executable path.
        """
        NUMBER_OF_CARMAKER_INSTANCES = 1
        testRuns = self.getTestrunsHeadless(CMPrjDir)
        numTestruns = len(testRuns)
        # Run all TestRuns
        testRunner  = cCMEvalTestRunner(CMPrjDir, export_quantities, NUMBER_OF_CARMAKER_INSTANCES, sampleRate='default')
        with self.assertRaises(Exception):
            testRunner.runEvaluations(testCaseSetName, testRuns, maxTimeInSimulationSeconds=maxTimeInSimulationSeconds, 
                                      headlessMode = True, useCoreLicense = True, nRetries = N_RETRIES,
                                      ignoreExistingResFile = True, executable="D:/somewhere/CM.exe")
    
    def test_headlessMode_wrongCMPrjPath(self):
        """
        Test the Carmaker simulation headless mode.
        """
        NUMBER_OF_CARMAKER_INSTANCES = 1
        testRuns = self.getTestrunsHeadless(CMPrjDir)
        numTestruns = len(testRuns)
        # Run all TestRuns
        with self.assertRaises(Exception):
            testRunner  = cCMEvalTestRunner('some/where', export_quantities, NUMBER_OF_CARMAKER_INSTANCES, sampleRate='default')

    
if __name__ == '__main__':
    unittest.main(verbosity=2)