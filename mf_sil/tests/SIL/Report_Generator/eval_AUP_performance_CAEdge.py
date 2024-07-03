#====================================================================
# System Imports
#====================================================================
import os
import sys
import logging

# workaround: avoid that using STET will disable all existing loggers
@property
def disabled(self):
    return self._disabled

@disabled.setter
def disabled(self, disabled):
    self._disabled = False

logging.Logger.disabled = disabled

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s [%(name)s] %(message)s', datefmt='%d-%m-%Y %H:%M:%S')
logger = logging.getLogger("eval_AUP_performance")

from optparse import OptionParser

#====================================================================
# Simulation Imports
#====================================================================
PYBASE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'contrib', 'VSP_pyBase')
sys.path.append( os.path.join(PYBASE_PATH, 'pycmEval'))
sys.path.append( os.path.join(PYBASE_PATH, 'pyHTMLReport') )
sys.path.append( os.path.join(PYBASE_PATH, 'pyUtils') )

from cm_eval_runner         import cCMEvalTestRunner
from cm_testrun             import cmTestRun
from cm_testrun             import cmTestRun

EVALPATH = os.path.dirname(os.path.abspath(__file__))
PATH_AUP_import_signal_list = os.path.abspath(os.path.join(EVALPATH, "ImportList.txt"))

maxExecutionTime_AUP          = 300
maxExecutionTime_TCE          = 9000
maxSimulationTime_AUP         = 250

CMPrjDir                      = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'CarMaker')


#====================================================================
# Parameters
#====================================================================
'''Function to read all signals (CM Quantities) from a txt file and return the list with the signals'''
def importSignalList(PATH_import_signal_list):
    with open(PATH_import_signal_list) as import_list:
        '''signals inside import_list can be found: <SW_signal_name> [architecture_name]
        <SW_signal_name> - signal as it is found in the SW
        [architecture_name] - name which should be used in the test specification; optional
        key words of the dictionary structures are searched inside [architecture_name]'''
        '''Read all signals from txt file and save the signals in Quantities_Import list'''
        Quantities_Import = []
        '''structures - dictionary with keyword: structure name in the import list. The signals inside needs to be multiplied
                             value: maximum index number '''
        structures = {"envModelPort":8, "TRJPLA":10}
        for line in import_list:
            if len(line) > 0:
                '''if it is not an empty line, add the <SW_signal_name> in the list'''
                Quantities_Import.append(line.rstrip("\n").split()[0])
                ''' if [architecture_name] exists, use the dictionary to search if the signals should be multiplied'''
                if len(line.split())>1:
                    for struct in structures.keys():
                        if struct in line.split()[1]:
                            for i in range(1,structures[struct]):
                                updated_line = line.rstrip("\n").split()[0].replace("_0.","_"+str(i)+".",1)
                                Quantities_Import.append(updated_line)
    return Quantities_Import


def change_CM_config_line_endings():
    '''
    PI-2402: change the line endings of OutputQuantities and SimParameters from windows
    endings to Linux endings
    '''
    WINDOWS_LINE_ENDING = b'\r\n'
    UNIX_LINE_ENDING = b'\n'
    files = [os.path.join(CMPrjDir, "Data", "Config", "OutputQuantities"),
             os.path.join(CMPrjDir, "Data", "Config", "SimParameters")]
    for file_path in files:
        with open(file_path, 'rb') as open_file:
            content = open_file.read()
            content = content.replace(WINDOWS_LINE_ENDING, UNIX_LINE_ENDING)
        with open(file_path, 'wb') as open_file:
            open_file.write(content)

def getAllTestcaseFilesInFolder(folder):
    testCases = []
    for path, dirs, files in os.walk(folder):
        for file in files:
            filePath = os.path.join(path, file)
            testCases.append({"caseType": "useCase", "cmTestRun": cmTestRun(file, filePath)})
    return testCases

def GetTestCasesFromDir(testCasesDir):
    errorCode = 0
    TestCases = []
    TestCases = TestCases + getAllTestcaseFilesInFolder(os.path.join(os.path.dirname(__file__), '..', 'CarMaker', 'Data', 'TestRun', testCasesDir))
    return (TestCases, errorCode)

def CalcPerformance(testrun_dir, out_dir_name, fail_silent=False):
    if testrun_dir:
        os.environ["CARMAKER_GENERATE_JSON"] = "1"
        os.environ["CARMAKER_GENERATE_JSON_ROOT"] = os.path.join(CMPrjDir, "SimOutput")
        Quantities = importSignalList(PATH_AUP_import_signal_list)
        maxExecTime = maxExecutionTime_AUP
        maxTimeInSimulationSeconds = maxSimulationTime_AUP
    
    logger.info("Generating Activation Performance Report")
    
    (testCasesBase, errorCodeScenarios) = GetTestCasesFromDir(testrun_dir)
    testRunsBase       = [currTestCase["cmTestRun"] for currTestCase in testCasesBase]

    testRunner         = cCMEvalTestRunner(CMPrjDir, Quantities, numberOfCarMakerInstances = 4, sampleRate = '10ms')
    ergFilePaths = testRunner.runEvaluations(out_dir_name, testRunsBase, maxTimeInSimulationSeconds=maxTimeInSimulationSeconds,
                                             headlessMode=True, useCoreLicense=True, returnErgPaths=True,
                                             maxExecTimeSeconds=maxExecTime, nRetries=240, retryTimeSeconds=30)
    change_CM_config_line_endings()
    return errorCodeScenarios


if __name__ == "__main__":
    # Parse Command Line Options
    optparser = OptionParser(usage="usage: %prog [options]")
    optparser.add_option("-d", "--directory", dest='testCasesDirectory', default=None, help="TestCase-Directory relative to the CarMaker TestRun directory in which all testruns are located.")
    optparser.add_option("-o", "--outputPath", dest='outputPath', default=None, help="Output path for the result files relative to SimOutput folder.")
    args      = optparser.parse_args()[0]
    ### Calculate TestCase Performance
    errorCode = CalcPerformance(args.testCasesDirectory, args.outputPath)
    #temporary solution to to upload artifacts if the tests are failed.
    #exit(errorCode)
