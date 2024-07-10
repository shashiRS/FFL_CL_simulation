#====================================================================
# System Imports
#====================================================================
import os
import re
import numpy as np
import sys
import logging
logger = logging.getLogger("pyBase")
import threading
import psutil
import shutil
import time
import socket
import queue
try:
    from . import cm_eval_erg_parser
except(ImportError):
    import cm_eval_erg_parser
try:
    from .cm_testrun import cmTestRun
except(ImportError):
    from cm_testrun import cmTestRun

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "pycmCommon"))
try:
    from cmpyglue import CMRC,CMRC_HeadlessInstance, DEFAULT_CM_PORT
    from cmpyglue import CARMAKER_DEFAULT_PATHS
except:
    from pyBase.pycmCommon.cmpyglue import CMRC,CMRC_HeadlessInstance, DEFAULT_CM_PORT, CARMAKER_DEFAULT_PATHS
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "pycmCpyUtilsommon"))
#from pyHelpers import EventHook
import configparser

import datetime
from collections import OrderedDict

import sys
import stat
import platform
import subprocess
import re
event = threading.Event()
try:
    from cmEval.src.FSF_Generic import read_configFile
except:
    pass


try:
    config = configparser.ConfigParser(allow_no_value=True)
    config.optionxform = str
    crashed_scenarios = os.path.join('./input', 'conf', 'Crashed_Scenario.cfg')
    config_file = os.path.join('./input', 'conf', 'FSF_config.ini')
    config.read(config_file)
    time_delay = config.get('Delay', 'time_stamp')
    launch_IPG = config.get('IPGMovie_Settings', 'launch_movie')
    update_excel = OrderedDict()  # For Rec Generation Status
except:
    pass

if platform.system() == 'Linux':
    IS_LINUX_SYSTEM = True
else:
    IS_LINUX_SYSTEM = False

try:
    sys.path.append( os.path.join(os.path.dirname(__file__), '..', 'pycmCommon') )
    sys.path.append( os.path.join(os.path.dirname(__file__), '..', 'pyUtils') )
except:
    pass

PYBASE_SIMPARAMETER_NAME = 'PyBaseEvalActive'
CARAMKER_EXE_NAME = 'carmaker_node.win64.exe'
config = configparser.ConfigParser(allow_no_value=True)
config.optionxform = str
crashed_scenarios = os.path.join('./input', 'conf', 'Crashed_Scenario.cfg')
missing_erg = []
CM_instance_error = 0
DEFAULT_CM_PORT = 2327
CarMakerInstance_Opened = 0
MAX_NUMBER_OF_CARMAKER_INSTANCES = 8 # eval runner will never spawn more than this number of CarMaker instances
update_excel = OrderedDict()         # For Rec Generation Status
event = threading.Event()
flag = False
executed = 0
simulated_testruns = []
time_delay = 5

# Among other things, spawns threads that execute CarMaker test runs.
class BreakOutofLoops(Exception):
    """Raised when error seen in CM (license error)"""
    pass

class SimulationExecutionException(Exception):
    """
    Class for simulation execution exceptions.
    """
    def __init__(self, msg):
        self.message = msg
        super().__init__(self.message)

class cCMEvalTestRunner(object):
    """
    Class that executes a Carmaker simulation.
    The CMEvalTestRunner ensures the proper execution of the simulation and handles errors if there are any.
    This class is intended to be used by a developer. It handles the telnet connection, remote control of the GUI or headless simulation until the result
    files are available.

    :param cmPrjDir: Absolute path Carmaker project directory
    :param cmQuantities: List of strings with quantity names that shall be exported to the result file
    :param numberOfCarMakerInstances: Number of parallel Carmaker instances to simulate given testruns.\
    Note that if you choose a number greater than 1, each instance uses and blocks one license from the world wide Carmaker license pool \
    that cannot be used by somebody else. Usually this mode is desired to be used for nightly tests. If you pass your testruns as ts file in runEvaluations function of this class \
    this parameter has no effect as Carmaker will use the hpc mode by default (four parallel simulations with one office license).
    :param sampleRate: Sample rate for the data which get written to the result file. ("default": 0.1s) Additional available options: "1000ms", "100ms", "10ms", "1ms" \
    Note that the size of the result file increases with higher sample rate.
    :param cmGuiPath: Absolute path to Carmaker GUI. If no is given, the latest Carmaker version in the standard installation path of Carmaker is used. If the headless \
    mode chosen in runEvaluations function of this class, this parameter has no effect.
    """
    def __init__(self, cmPrjDir, cmQuantities, numberOfCarMakerInstances = 1, MDF = 0,sampleRate='default', cmGuiPath = None):
        self._cmPrjDir     = cmPrjDir
        if not os.path.isabs(cmPrjDir):
            raise Exception("The given Carmaker project path is not an absolute path: {}.".format(cmPrjDir))
        self._cmQuantities = cmQuantities
        self.MDF = MDF
        self._cmGuiPath    = cmGuiPath
        self._workDistributionLock = threading.Lock() # lock used to synchronize the spawned threads when distributing work
        self._accumulatorLock = threading.Lock() # lock used to synchronize writing to the accumulator
        self._imageAcumulatorLock = threading.Lock()  # lock used to synchronize writing images to the accumulator
        # set sample rate
        if sampleRate == 'default':
            self._sampleRate = 0.1
        else:
            self._sampleRate = int(re.search(r'(\d+)ms', sampleRate).group(1))/1000


        # Event Hooks
        #self.onTestRunFinished = EventHook()
        self._numberOfCarMakerInstances = min(numberOfCarMakerInstances, MAX_NUMBER_OF_CARMAKER_INSTANCES)
        self._simExecExceptions = []

        try:
            # remove write protection of config files
            for (dirpath, dirnames, filenames) in os.walk(os.path.join(cmPrjDir, "Data", "Config")):
                for currFileName in filenames:
                    if IS_LINUX_SYSTEM:
                        os.chmod(os.path.join(dirpath, currFileName), stat.S_IRWXU)
                    else:
                        os.chmod(os.path.join(dirpath, currFileName), stat.S_IWRITE)
        except:
            pass
    #Get lisence server name from the Lisence file in IPG dir.
    @staticmethod
    def get_License_server(file_name):
        with open(file=file_name, mode='r') as fp:
            file_cont = fp.read()
            return file_cont.split()[-1]

    def _WriteSimParametersFile(self):
        """
        Write the sim parameters file. This is necessary to set the output path for the results and the correct sample rate.
        Raise an exception if SimParameters file does not exist (which is available by default in a Carmaker project).
        """

        simParametersDirectoryPath = os.path.join( self._cmPrjDir, 'Data', 'Config')
        simParametersFilePath = os.path.join( simParametersDirectoryPath, 'SimParameters' )

        # create a directory for the simParameters-file
        if not os.path.exists(simParametersDirectoryPath):
            os.makedirs(simParametersDirectoryPath)

        # Check if there are configurations in SimParameters file existing to not override them.
        # If OutPath is not specified in this file, we will do here so that erg files are stored
        # to desired subfolder in SimOutput
        newSimParamFile = []
        outPathExisting = False
        pyBaseParamExisting = False
        if os.path.exists(simParametersFilePath):
            fileContent = None
            with open(simParametersFilePath, 'r') as file:
                fileContent = file.readlines()
            for idx, currLine in enumerate(fileContent):
                if 'DStore.OutPath' in currLine:
                    outPathExisting = True
                    newSimParamFile.append('DStore.OutPath = %o/%f\n')
                elif PYBASE_SIMPARAMETER_NAME in currLine:
                    pyBaseParamExisting = True
                    newSimParamFile.append('{} = 1\n'.format(PYBASE_SIMPARAMETER_NAME))
                else:
                    newSimParamFile.append(currLine)
            if not outPathExisting:
                newSimParamFile.append('DStore.OutPath = %o/%f\n')
            if not pyBaseParamExisting:
                newSimParamFile.append('{} = 1\n'.format(PYBASE_SIMPARAMETER_NAME))
            if fileContent != newSimParamFile:
                os.remove(simParametersFilePath)
                with open(simParametersFilePath, 'w') as newFile:
                    newFile.writelines(newSimParamFile)
        else:
            raise Exception('SimParameters file does not exist.')

    def _ResetPyBaseSimParameter(self):
        """
        Reset the parameter PyBaseEvalActive back to zero in the SimParameters file.
        Raise an exception if SimParameters file does not exist (which is available by default in a Carmaker project).
        """
        simParametersFilePath = os.path.join( self._cmPrjDir, 'Data', 'Config', 'SimParameters')
        if os.path.exists(simParametersFilePath):
            fileContent = None
            newFileContent = []
            with open(simParametersFilePath, 'r') as file:
                fileContent = file.readlines()
            for currLine in fileContent:
                if PYBASE_SIMPARAMETER_NAME in currLine:
                    newFileContent.append(currLine.replace('= 1', '= 0'))
                else:
                    newFileContent.append(currLine)
            if fileContent != newFileContent:
                os.remove(simParametersFilePath)
                with open(simParametersFilePath, 'w') as newFile:
                    newFile.writelines(newFileContent)
        else:
            raise Exception('SimParameters file does not exist.')

    @staticmethod
    def KillCEMNodes(CarmakerWin64path):
        """
                Kill the CEM nodes after getting the node name from the exe name
        """
        if CarmakerWin64path is not None:
            Base_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(CarmakerWin64path))))
            Exefilepath = os.path.join(Base_path,"cem200","lib","cem200")
            try:
                exe_Names_array = os.listdir(Exefilepath)
                for each_exe_name in exe_Names_array:
                    subprocess.call('taskkill /F /IM ' + each_exe_name, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            except:pass

    def _WriteOutputQuantities(self,FileName):
        """
        Write the output quanitites file. This is necessary to configure which output quantities shall be exported for the simulation run.
        """

        outputQuantitiesDirectoryPath = os.path.join( self._cmPrjDir, 'Data', 'Config')
        quantFilePath = os.path.join( outputQuantitiesDirectoryPath, FileName )

        # create a directory for the quantities-file
        if not os.path.exists(outputQuantitiesDirectoryPath):
            os.makedirs(outputQuantitiesDirectoryPath)

        # delete existing quantities-file
        if ( os.path.exists( quantFilePath ) ):
            os.remove( quantFilePath )

        try:
            config.read(config_file)
            TFQuantities = config.get('OutputQuantities', 'mfcprojectquantities')

            if int(TFQuantities):
                self._sampleRate = 0.020
                # Update FSF_config.ini file
                config['OutputQuantities']['mfcprojectquantities'] = '0'
                with open(config_file, 'w') as file:  # save
                    config.write(file)
        except:
            pass

        f = open(quantFilePath, mode='w')

        f.write(
            """#INFOFILE1.1 - Do not remove this line!
            FileIdent = CarMaker-OutputQuantities 1

            DStore.Format = erg

            DStore.dt.fast = 0.001
            DStore.dt.normal = {}
            DStore.dt.slow = 0.1

            DStore.Quantities.fast:

            DStore.Quantities.normal:\n\t""".format(self._sampleRate))

        f.write('\n \t'.join(self._cmQuantities))
        f.write('\n')
        f.write("DStore.Quantities.slow: \n")
        f.close()


    def _SetSimParameters(self, value):
        simPara_Path = os.path.join(self._cmPrjDir, 'Data', 'Config', 'SimParameters')
        if (os.path.exists(simPara_Path)):
            changeText_0, changeText_1, changeText_2 = '', '', ''
            with open(simPara_Path, 'r') as cf:
                data = cf.readlines()

                cT0 = re.compile('.*DStore.dtFile.*')
                cT1 = re.compile('.*TestRunEnd.DVA_ReleaseAll.*')
                cT2 = re.compile('.*DStore.OutputQuantities.*')
                if not any(cT0.match(line) for line in data):
                    changeText_0 = "DStore.dtFile = 0.01\n"
                if not any(cT1.match(line) for line in data):
                    changeText_1 = "TestRunEnd.DVA_ReleaseAll = 1\n"
                if not any(cT2.match(line) for line in data):
                    changeText_2 = "DStore.OutputQuantities = {0}\n\n".format(value)
                else:
                    for i, line in enumerate(data):
                        if "DStore.OutputQuantities" in line:
                            data[i] = "DStore.OutputQuantities = {0}\n".format(value)

            data.insert(3, changeText_0)
            data.insert(4, changeText_1)
            data.insert(5, changeText_2)

            with open(simPara_Path, 'w') as file:
                data = "".join(data)
                file.writelines(data)

    def config_GUI(self, CMPrjDir, CMprojectexe, sim_output):
        self.CMPrjDir = CMPrjDir
        self.CMprojectexe = CMprojectexe
        config_GUI = os.path.join(CMPrjDir, "Data", "Config", "GUI")
        if os.path.exists(config_GUI):
            os.remove(config_GUI)

        f = open(config_GUI, mode='w')
        if sim_output != '2':
            CM_version = '8.0'
        else:
            CM_version = '10.0'

        write_config = (
"""#INFOFILE1.1 - Do not remove this line!
FileIdent = GUI - Config {1}
CM.Exe = {0}
CM.Args =
CM.Exe.History:
CM.Args.History:
GPUParameters.FName = """).format(self.CMprojectexe, CM_version)
        f.writelines(write_config)
        f.close()

    def delete_CEMNodes_log(self, CMprojectwin64exe):
        CEM_log_dir = os.path.join(CMprojectwin64exe.partition("\\install")[0], "CEM_node_log")
        try:
            for filename in os.listdir(CEM_log_dir):
                try:
                    os.remove(os.path.join(CEM_log_dir, filename))
                except:
                    pass
        except:
            pass

    # Iterates over all running processes and kills all running CarMaker.win32.exe(s)
    @staticmethod
    def _killCarMakerProcesses():
        """
        Kill running Carmaker processes.
        This is necessary at the end of a simulation as sometimes the Carmaker binary executable stays open after the Carmaker GUI was closed.
        If there is an error while simulation or something crashed, this function ensures that every opened process is killed.
        """
        PROCNAMES = ["ars540_node.exe", "carmaker_node.win64.exe", "CarMaker.win32.exe", "CarMaker.win64.exe", "HIL.exe",
                     "apobrokerd.exe"]
        for currProcName in PROCNAMES:
            for proc in psutil.process_iter():
                try:
                    procDict = proc.as_dict(attrs=["name", "exe", "pid"])
                    if procDict["name"] == currProcName:
                        logging.info(f"Killing the process {currProcName} with pid {procDict['pid']}in the backend!!!")
                        psutil.Process(procDict["pid"]).kill()
                except:
                    pass  # some processes may be protected by the OS
    @staticmethod
    def ping_check(host):
        # response = subprocess.check_output("ping -{0} 1 {1}".format('c' if IS_LINUX_SYSTEM else 'n', host), shell=True)
        response = os.system("ping -{0} 5 {1} {2}".format('c' if IS_LINUX_SYSTEM else 'n', host, '> /dev/null 2>&1' if IS_LINUX_SYSTEM else '> NUL'))
        if response == 0:
            return True
        else:
            return False

    def outputAllLogFiles(self):
        """
        Output the content of all log files in the log directory {project_dir}/SimOutput/{hostname}/Log to logger.info.
        """

        logFolderPath = os.path.join(self._cmPrjDir, "SimOutput", socket.gethostname(), "Log")
        logger.info("Output log files in folder: {}".format(logFolderPath))
        if os.path.exists(logFolderPath) and os.path.isdir(logFolderPath):
            files = os.listdir(logFolderPath)
            if len(files) > 0:
                logger.info("List of available log files in {}:\n{}".format(logFolderPath, "".join(files)))
            else:
                logger.info("No log files found. Folder empty.")
            for file in files:
                absFilePath = os.path.join(logFolderPath, file)
                if os.path.isfile(absFilePath):
                    with open(absFilePath, "r") as f:
                        logger.info("\n========== Log file start: {} ==========\n{}\n========== Log file end ==========".format(absFilePath, "".join(f.readlines())))
        else:
            logger.warning("No log folder found in {}. Run a simulation before reading log files.".format(logFolderPath))


    def runEvaluations(self,outputFolderName,testRuns,CMprojectexe = None,extract = None,rec_extract =None,rec_video_extract = None, check_flag = None, rec_path =None, rec_bat_path=None, CMprojectwin64exe=None, signal_mapping=None,rec_folders =None,sim_output=0 ,worker=None,maxTimeInSimulationSeconds = None,init_per = None,total_runs =None,statistics = None,MDF = 0,simulation_count =0,cemvalues=None,nRetries = 0, custom_test_run_folder=None,maxExecTimeSeconds = None,retryTimeSeconds = 2, executable = None, ignoreExistingResFile = False,cmGuiPath = None,headlessMode = False, useCoreLicense = False,dataPoolPath=None,signal_to_evaluate=[], returnErgPaths = False, raiseSimExecExceptions = True, returnExtendedInformation = False, showCMLogOutput=False):

        """
        Start the execution of given testruns with given configuration. After the method finished and there were no errors while the simulation, the result files are
        available in configured location. The Carmaker binary is restarted after each testrun to ensure a proper execution and initialization without side effects to the
        test results from previous testruns.

        :param outputFolderName: Name of the output folder in that the result files are stored (will be a subfolder in SimOutput directory in the Carmaker project)
        :param testRuns: Option 1: List of objects of class cmTestRun (defined in pyBase/pycmEval/cm_testrun.py). Option 2: Name of a Carmaker ts file with defined testruns \
        placed in the TestRun folder. In this case the parameters maxTimeInSimulationSeconds, maxExecTimeSeconds, useCoreLicense and ignoreExistingResFile are not used! \
        Setting them will not have an effect on the simulation!
        :param maxTimeInSimulationSeconds: Maximum time a single testrun is allowed to simulate in simulated time.
        :param headlessMode: If True, Carmaker will run in a headless mode. If False, the GUI will open for simulation. If you set useCoreLicense to True, headless mode is chosen \
        automatically and this parameter has no effect.
        :param useCoreLicense: Use core license of Carmaker for simulation (True/False). Note that your user has to be approved to use the core licenses. If you are not approved \
        or you are not sure, choose False to use normal office license to simulate.
        :param nRetries: Number of retries if something goes wrong while simulation, e.g. temporary loss of network connection and so no license checkout possible or \
        Carmaker does not react in configured timeouts due to high work load on the machine.
        :param executable: Absolute path to Carmaker binary executable. If no is given, the configured path from the GUI file in the Carmaker project will be used.
        :param ignoreExistingResFile: If True, always all testruns will be executed, regardless if a result file is already existing. If False, it will skip testruns with \
        an already existing result file.
        :param returnErgPaths: If True, the function returns a list of paths to erg files as pyBase used to do it all the time. If False pyBase returns the given list of testRuns \
        and adds the paths to the erg file and an execution status. Note: The old functionality (in case the parameter is set to True) will be removed in further pyBase versions. \
        In case of using the HPC light mode, this parameter does not have an effect.
        :param maxExecTimeSeconds: Maximum time a single testrun is allowed to simulate in wall-clock time.
        :param retryTimeSeconds: Time to wait between retries if something goes wrong in the simulation.
        :param raiseSimExecExceptions: Exceptions that occur while the simulation are written to the testrun object. If True exceptions will be raised. If False they will not. \
        Only has an effect for headless and core license mode.
        :param showCMLogOutput: If True, the content from the latest log file created by CarMaker is shown in the logger. This option does not work if multiple \
        instances of the CarMaker are running in parallel, i.e. it effects only if numberOfCarMakerInstances in the class constructor is given as 1. Only works in headless and core \
        license mode.

        :return: List of testrun objects as passed in parameter testRuns with updated attributes. If parameter returnErgPaths is True, list of absolute paths of all result files as \
        in older versions. Exception: If using the HPC light mode, it returns always a list of absolute paths to erg files.
        """


        global initpercent,statistics_counts,missing_erg
        self.CEMValues = cemvalues
        self.dataPoolPath = dataPoolPath

        # eCAL data pool shared path
        if sim_output == '1' or sim_output == '2':
            self.update_DataPoolPath(self._cmPrjDir, self.dataPoolPath)

        counts = simulation_count
        _is_hpclight_mode = False
        cCMEvalTestRunner._killCarMakerProcesses()
        self.count = 0          #
        if init_per == 0:
            missing_erg = []
        self._ergFilePaths = {} # the accumulator written by all worker threads
        self._testRunList = [] # used in future instead of ergFilePaths
        self._showCMLogOutput = showCMLogOutput
        self.worker = worker
        # self._signal_mapping = signal_mapping
        # if Project_name == "ACC":###########################################################
        #     self._WriteOutputQuantities("OutputQuantities")
        # elif Project_name == "EBA":
        self._WriteOutputQuantities("TF_OutputQuantities")

        def find_latest_version(carmakerPath, prefix):
            """Since with Carmaker version 10.x the major version number has two digits for the first time, it is not possible anymore
            to use the latest folder delivered by listdir function as this does not provide the latest version when mixing one and
            two digits in the folder name."""

            ver_list = {}
            for version in os.listdir(carmakerPath):
                # get version numbers of valid CarMaker paths
                # a valid version is build from the prefix followed by two or three numbers, e.g. linux64-10.2.1
                # if the version only has two numbers use None as the third number
                match = re.match(f'{prefix}(\d*)\.(\d*)(\.(\d*))?', version)
                if match:
                    major_ver = int(match.group(1))
                    minor_ver = int(match.group(2))
                    patch_ver = int(match.group(4)) if match.group(4) is not None else 0
                    ver_list.update({version: [major_ver, minor_ver, patch_ver]})
            latest_ver_path = None
            latest_version = [0, 0, 0]
            for ver_path, ver in ver_list.items():
                if ver > latest_version:
                    latest_version = ver
                    latest_ver_path = ver_path
            return latest_ver_path

        if self._cmGuiPath is None:
            if not CMprojectexe:
                for currCarMakerDefaultPath in CARMAKER_DEFAULT_PATHS:
                    if os.path.exists(currCarMakerDefaultPath):
                        if IS_LINUX_SYSTEM:
                            latest_version = find_latest_version(currCarMakerDefaultPath, "linux64-")
                            if latest_version:
                                CMprojectexe = os.path.join(currCarMakerDefaultPath, latest_version, r"bin/CM") # take latest version of CarMaker default install directory
                        else:
                            latest_version = find_latest_version(currCarMakerDefaultPath, "win64-")
                            if latest_version:
                                CMprojectexe = os.path.join(currCarMakerDefaultPath, latest_version, r"bin\CM.exe") # take latest version of CarMaker default install directory
        else:
            CMprojectexe = self._cmGuiPath
        if not os.path.exists(CMprojectexe):
            raise RuntimeError('No CarMaker installation found.')
        if self.MDF == 1:
            self._SetSimParameters("OutputQuantities_FCT_MDF")
        else:
            self._SetSimParameters("TF_OutputQuantities")
        if useCoreLicense or headlessMode:
            self._WriteSimParametersFile()
        # Added ecAL condition
        if sim_output == '1' or sim_output == '2':
            self.config_GUI(self._cmPrjDir, CMprojectwin64exe, sim_output)
            self.delete_CEMNodes_log(CMprojectwin64exe)
            if sim_output == '2':
                msg = "Launching eCAL and Carmaker GUI"
            else:
                msg = "Launching ROS and Carmaker GUI"

            logger.info(msg)
            try:
                worker.signals.logsignal.emit(msg)
            except Exception as e:
                print(e)  # cmd line execution


        initpercent = init_per
        statistics_counts = statistics
        # Make it work for single TestRuns
        if isinstance(testRuns, str):
            # testscript file assumed
            if str(testRuns).endswith(".ts"):
                _is_hpclight_mode = True
                logger.info("======= Running in HPC Light Mode =======")
                workerThread = cCMEvalTestRunner.TestSeriesRunningThread(self, testRuns, self._cmPrjDir, outputFolderName, DEFAULT_CM_PORT, nRetries, retryTimeSeconds, executable, self._cmGuiPath)
                workerThread.start()
                workerThread.join()
        elif not isinstance(testRuns, list):
            testRuns = [testRuns]

        if not _is_hpclight_mode:
            if not isinstance(outputFolderName, list):
                outputFolderName = [outputFolderName]

            self._signal_mapping = signal_mapping
            self._image_extract = True if extract == 1 else False
            self.videoextract = True if rec_video_extract == "1" else False
            self._testRuns = testRuns

            self._nextTestRunToExecuteIndex = 0 # index of the next test run to execute by one of the worker-threads, this variable is read and incremented by each of the worker threads
            self._outputFolderName = outputFolderName
            #if Project_name =="ACC":
            for i in range(len(list(set(outputFolderName)))):
                if custom_test_run_folder:
                    self._evaluationTestRunDirectory = self._outputFolderName[i]
                else:
                    if sim_output == '1':
                        self._evaluationTestRunDirectory = os.path.join(self.dataPoolPath, "Data", "TestRun", outputFolderName[i])
                    else:

                        self._evaluationTestRunDirectory = os.path.join(self._cmPrjDir, "Data", "TestRun", outputFolderName[i])

                if not os.path.exists(self._evaluationTestRunDirectory):
                    os.makedirs(self._evaluationTestRunDirectory)

            self.check_flag = check_flag
            self.rec_path = rec_path

            startTime = time.time()
            if not IS_LINUX_SYSTEM:
                cCMEvalTestRunner.KillCEMNodes(CMprojectwin64exe)

            instanceCount = self._numberOfCarMakerInstances
            if not headlessMode and not useCoreLicense:
                self.Check_Missing_Erg(self._cmPrjDir, CMprojectexe, CMprojectwin64exe, check_flag, rec_bat_path,rec_extract, rec_video_extract, rec_folders, sim_output, maxTimeInSimulationSeconds,
                                   worker, total_runs, init_per, statistics, nRetries, custom_test_run_folder,maxExecTimeSeconds, retryTimeSeconds, executable, ignoreExistingResFile, cmGuiPath,
                                   headlessMode, useCoreLicense, showCMLogOutput=showCMLogOutput, file_type=2)

                instanceCount = self.count_CMinstances()

            # Function to Call threads to Start simulation
            signal_validity = self.thread_calling(self._cmPrjDir,CMprojectexe, CMprojectwin64exe, check_flag, rec_bat_path,
                               rec_extract, rec_video_extract, rec_folders,sim_output, maxTimeInSimulationSeconds, worker, total_runs, init_per, statistics,nRetries, custom_test_run_folder,maxExecTimeSeconds,retryTimeSeconds, executable, ignoreExistingResFile,cmGuiPath,headlessMode, useCoreLicense,signal_to_evaluate=signal_to_evaluate, raiseSimExecExceptions=raiseSimExecExceptions, showCMLogOutput=showCMLogOutput, instanceCount=instanceCount)
            # kill all CarMaker.win32.exe instance, TODO: do this only for those instances started by the script, the user may have started another CarMaker-instance for other purposes
            cCMEvalTestRunner._killCarMakerProcesses()
            if not IS_LINUX_SYSTEM:
                cCMEvalTestRunner.KillCEMNodes(CMprojectwin64exe)

            if (rec_extract == "1" or rec_video_extract == "1") and signal_validity == True: # Call only for Rec Generation
                self.check_MissingCases(self._cmPrjDir,CMprojectexe, CMprojectwin64exe, check_flag, rec_bat_path,
                               rec_extract, rec_video_extract, rec_folders, sim_output, maxTimeInSimulationSeconds, worker, total_runs, init_per, statistics,nRetries, custom_test_run_folder,maxExecTimeSeconds,retryTimeSeconds, executable, ignoreExistingResFile,cmGuiPath,headlessMode, useCoreLicense, showCMLogOutput=showCMLogOutput )
            elif not headlessMode and not useCoreLicense:
                if signal_validity == True:
                    self.Check_Missing_Erg(self._cmPrjDir,CMprojectexe, CMprojectwin64exe, check_flag, rec_bat_path,
                                rec_extract, rec_video_extract, rec_folders, sim_output, maxTimeInSimulationSeconds,
                                worker, total_runs, init_per, statistics,nRetries, custom_test_run_folder,maxExecTimeSeconds,retryTimeSeconds, executable, ignoreExistingResFile,cmGuiPath,headlessMode, useCoreLicense, showCMLogOutput=showCMLogOutput,file_type=1 )

            totalTime = time.time() - startTime
            totalTime = str(time.strftime('%H:%M:%S', time.gmtime(totalTime))) + " (h:mm:ss)"
            totalSimulationTime = {"startTime": startTime, "endTime": time.time(), "totalTime": totalTime, "carmakerInstance": instanceCount}

            if os.path.exists(crashed_scenarios):
                self.carmaker_crashing()
            if not IS_LINUX_SYSTEM:
                cCMEvalTestRunner.KillCEMNodes(CMprojectwin64exe)
            self._SetSimParameters("OutputQuantities")          #Function to set back TF_OutputQuantities to OutputQuantities

        if useCoreLicense or headlessMode:
            self._ResetPyBaseSimParameter()
        if returnErgPaths or _is_hpclight_mode:
            cCMEvalTestRunner._killCarMakerProcesses()
            return self._ergFilePaths
        elif returnExtendedInformation:
            return self._ergFilePaths,initpercent,statistics_counts, totalSimulationTime, update_excel, signal_validity
        else:
            return self._testRunList

    def count_CMinstances(self):
        if len(self.input_file_list) == 0:
            instanceCount = self._numberOfCarMakerInstances
        elif self._numberOfCarMakerInstances > len(self.input_file_list):
            instanceCount = len(self.input_file_list)
        else:
            instanceCount = self._numberOfCarMakerInstances
        return instanceCount

    def update_DataPoolPath(self, cmprjdir, dataPoolPath):
        projectPath = os.path.join(cmprjdir, "Data", "Config", "Project")
        try:
            with open(projectPath, 'r') as cf:
                data = cf.readlines()
                for i, line in enumerate(data):
                    if "DataPool.Shared " in line:
                        data[i] = ('DataPool.Shared = {0}\n').format(dataPoolPath)
            with open(projectPath, 'w') as file:
                file.writelines(data)
        except Exception as e:
            print(e)

    # Function to Start Simulation in Threads
    def thread_calling(self, cmPrjDir, CMprojectexe, CMprojectwin64exe, check_flag, rec_bat_path,
                           rec_extract, rec_video_extract, rec_folders,sim_output, maxTimeInSimulationSeconds, worker, total_runs, init_per, statistics,nRetries, custom_test_run_folder=None,maxExecTimeSeconds = None,retryTimeSeconds = 2, executable = None, ignoreExistingResFile = False,cmGuiPath = None,headlessMode = False, useCoreLicense = False,signal_to_evaluate = [], raiseSimExecExceptions = True, showCMLogOutput = False, instanceCount=None):

        workerThreads = []
        signal_verifier =True
        id = 0
        if useCoreLicense or headlessMode:
            self._outputFolderName = self._outputFolderName[0]
            if useCoreLicense:
                logger.info("======= Running in Continuous Testing / Core License Mode =======")
            else:
                logger.info("======= Running in Headless Mode =======")
            for i in range(0, self._numberOfCarMakerInstances):
                workerThread = cCMEvalTestRunner.SimulationRunningThreadHeadless(self, maxTimeInSimulationSeconds, maxExecTimeSeconds, nRetries, retryTimeSeconds, executable, ignoreExistingResFile, useCoreLicense, self._numberOfCarMakerInstances, showCMLogOutput)
                workerThreads.append(workerThread)
        else: # standard GUI license
            for i in range(0, instanceCount):
                id = id + 1  # Increment the id for multiple instances (ROS Approach)
                workerThread = cCMEvalTestRunner.SimulationRunningThread(self, DEFAULT_CM_PORT + i, id, cmPrjDir, CMprojectexe, CMprojectwin64exe, check_flag, rec_bat_path,
                               rec_extract, rec_video_extract, rec_folders,sim_output, maxTimeInSimulationSeconds, worker, total_runs, init_per, statistics,self.MDF,self.CEMValues,nRetries, custom_test_run_folder,maxExecTimeSeconds,retryTimeSeconds, executable, ignoreExistingResFile,cmGuiPath,signal_to_evaluate,instanceCount)
                workerThreads.append(workerThread)
                signal_verifier = workerThread.signal_verifier
        for workerThread in workerThreads:
            workerThread.start()
            if self._image_extract or self.videoextract:
                time.sleep(10)
        for workerThread in workerThreads:
            workerThread.join()

        # collect exceptions from worker threads
        for workerThread in workerThreads:
            e = workerThread.get_exception()
            for currException in e:
                self._simExecExceptions.append(SimulationExecutionException(currException))

        # raise exceptions from worker threads if desired
        if raiseSimExecExceptions:
            for e in self._simExecExceptions:
                raise e

        return signal_verifier

    # Check the missing rec generation and Simulate the missing cases
    def check_MissingCases(self,cmPrjDir, CMprojectexe, CMprojectwin64exe, check_flag, rec_bat_path,
                           rec_extract, rec_video_extract, rec_folders,sim_output, maxTimeInSimulationSeconds, worker, total_runs, init_per, statistics,nRetries,custom_test_run_folder,maxExecTimeSeconds,retryTimeSeconds, executable , ignoreExistingResFile ,cmGuiPath ,headlessMode , useCoreLicense, raiseSimExecExceptions=True, showCMLogOutput = False):
        file_list, rec_generated_files, rec_lists = [], [], []
        self.rec_list =[]
        if rec_folders == '1':
            for dir_path, dir_names, filenames in os.walk(self.rec_path):
                file_list.extend(filenames)
        else:
            file_list.append(str(self.rec_path).split('/')[-1])
        try:
            usecase_names = set(str(i).split("_")[0] for i in file_list)
            for names in usecase_names:
                rec_path = os.path.join(self._cmPrjDir, "SimOutput", self._outputFolderName, names)
                if os.path.exists(rec_path):
                    rec_generated_files.extend(os.listdir(rec_path))
            for i in rec_generated_files:
                first, middle, last = str(i).partition("VSPData_")
                rec_lists.append(last)
                self.rec_list = [sub.replace('.rec', '') for sub in rec_lists]
            self.resimulate_missingCases(File_Type=0,ignoreExistingResFile = ignoreExistingResFile)  # Check the missing scenarios ****File_Type=0 means rec

        except:
            print("Folders not found in SimOut")

        # If missing Recs found
        if self.input_file_list:
            while(self.count<3): # Repeat missing scenario simulation for 3times max
                self._testRuns = self.input_file_list
                self._nextTestRunToExecuteIndex = 0
                total_runs = len(self.input_file_list)

                # Function to Call threads for resimulation
                logger.info("########### Resimulating Scenarios for missing Rec files ###########")
                try:
                    worker.signals.logsignal.emit("########### Resimulating Scenarios for missing Rec files ###########")
                except:pass # For command line exe
                instanceCount = self.count_CMinstances()
                self.thread_calling(cmPrjDir, CMprojectexe, CMprojectwin64exe, check_flag, rec_bat_path,
                                    rec_extract, rec_video_extract, rec_folders, sim_output, maxTimeInSimulationSeconds,
                                    worker, total_runs, init_per, statistics,nRetries,custom_test_run_folder,maxExecTimeSeconds,retryTimeSeconds, executable , ignoreExistingResFile ,cmGuiPath ,headlessMode , useCoreLicense,[], raiseSimExecExceptions=raiseSimExecExceptions, showCMLogOutput=showCMLogOutput, instanceCount=instanceCount)
                self.count = self.count + 1
                # Call function again to check the remaining rec for further simulations
                self.check_MissingCases(cmPrjDir, CMprojectexe, CMprojectwin64exe, check_flag, rec_bat_path,
                           rec_extract, rec_video_extract, rec_folders,sim_output, maxTimeInSimulationSeconds, worker, total_runs, init_per, statistics,nRetries,custom_test_run_folder,maxExecTimeSeconds,retryTimeSeconds, executable , ignoreExistingResFile ,cmGuiPath ,headlessMode , useCoreLicense, raiseSimExecExceptions=raiseSimExecExceptions, showCMLogOutput=showCMLogOutput)
        else:pass

        # Check the missing erg generation and Simulate the missing cases
    def Check_Missing_Erg(self,cmPrjDir, CMprojectexe, CMprojectwin64exe, check_flag, rec_bat_path,
                           rec_extract, rec_video_extract, rec_folders, sim_output, maxTimeInSimulationSeconds,
                           worker, total_runs, init_per, statistics,nRetries,custom_test_run_folder,maxExecTimeSeconds,retryTimeSeconds, executable , ignoreExistingResFile ,cmGuiPath ,headlessMode , useCoreLicense, raiseSimExecExceptions=True, showCMLogOutput = False, file_type=1):
        self.erg_list = []
        self.erg_path_list = []
        outputFoldersetName = list(set(self._outputFolderName))

        if 'L3_SW_Requirements' in self._outputFolderName:
            for i in self._testRuns:
                if os.path.exists(os.path.join(self._cmPrjDir, "SimOutput", 'L3_SW_Requirements', i._erg_folder_name, i._name+'.erg')):
                    if os.path.getsize(os.path.join(self._cmPrjDir, "SimOutput", 'L3_SW_Requirements', i._erg_folder_name, i._name+'.erg')) > 0:
                        self.erg_path_list.append(i)
        else:
            for i in range(len(list(set(self._outputFolderName)))):
                for root, dirs, files in os.walk(os.path.join(self._cmPrjDir, "SimOutput", outputFoldersetName[i])):
                    # select file name
                    for file in files:
                        # check the extension of files
                        if file.endswith('.erg') or file.endswith('_ae.erg') or file.endswith('.mf4') or file.endswith('_ae.mf4'):
                            # print whole path of files
                            #Erg_File_Paths = os.path.join(self._cmPrjDir, "SimOutput", outputFoldersetName[i],file)
                            if "All_Usecases" in outputFoldersetName[i]:
                                Erg_File_Paths = os.path.join(root, file)
                            else:
                                Erg_File_Paths = os.path.join(self._cmPrjDir, "SimOutput", outputFoldersetName[i], file)
                            if os.path.getsize(Erg_File_Paths) > 0:
                                self.erg_path_list.append(file.split('.')[0])
                            elif os.path.getsize(Erg_File_Paths) <= 15: # Check if ERG size is less than 15kb
                                try:
                                    os.remove(Erg_File_Paths)
                                    os.remove(str(Erg_File_Paths)+".info")
                                except:pass
        for names in self.erg_path_list:
            self.erg_list.append(names)
        try:
            self.resimulate_missingCases(File_Type=file_type,ignoreExistingResFile = ignoreExistingResFile)  # Check the missing scenarios ****File_Type=1 means erg
        except:
            print("Folders not found in tf folder")

        # If missing ERGs found
        if self.input_file_list and file_type==1:
            while (self.count < 1):  # Repeat missing scenario simulation for 2times max
                self._testRuns = self.input_file_list
                self._nextTestRunToExecuteIndex = 0
                total_runs = len(self.input_file_list)
                if total_runs != 0:
                    # Function to Call threads for resimulation
                    logger.info("########### Resimulating Scenarios for missing ERG files ###########")
                    try:
                        worker.signals.logsignal.emit("########### Resimulating Scenarios for missing ERG files ###########")
                    except:pass # For command line exe

                cCMEvalTestRunner.KillCEMNodes(CMprojectwin64exe)
                instanceCount = self.count_CMinstances()
                self.thread_calling(cmPrjDir,CMprojectexe, CMprojectwin64exe, check_flag, rec_bat_path,
                                    rec_extract, rec_video_extract, rec_folders, sim_output,
                                    maxTimeInSimulationSeconds,
                                    worker, total_runs, init_per, statistics,nRetries,custom_test_run_folder,maxExecTimeSeconds,retryTimeSeconds, executable , ignoreExistingResFile ,cmGuiPath ,headlessMode , useCoreLicense,[], raiseSimExecExceptions=raiseSimExecExceptions, showCMLogOutput=showCMLogOutput, instanceCount=instanceCount)
                self.count = self.count + 1

                # Call function again to check the remaining erg for further simulations
                self.Check_Missing_Erg(cmPrjDir,CMprojectexe, CMprojectwin64exe, check_flag, rec_bat_path,
                                       rec_extract, rec_video_extract, rec_folders, sim_output,
                                       maxTimeInSimulationSeconds, worker, total_runs, init_per, statistics,nRetries,custom_test_run_folder,maxExecTimeSeconds,retryTimeSeconds, executable , ignoreExistingResFile ,cmGuiPath ,headlessMode , useCoreLicense, raiseSimExecExceptions=raiseSimExecExceptions, showCMLogOutput=showCMLogOutput, file_type=file_type)
        else:pass

    # Check the missing scenarios to generate rec and erg
    def resimulate_missingCases(self,File_Type,ignoreExistingResFile =False):
        self.input_file_list = []
        file1 = ""
        Crashed_list= []
        input_List = []
        if File_Type == 0:
            input_List = self.rec_list
        elif File_Type == 1 or File_Type == 2:
            input_List = self.erg_list

        for i in self._testRuns:
            if ignoreExistingResFile:
                input_file = i.GetName()
            else:
                input_file = i
            if 'L3_SW_Requirements' in self._outputFolderName:
                if i not in input_List:
                    self.input_file_list.append(i)
                    result_names_types = [".erg", ".erg.info"]
                    for results in result_names_types:
                        try:
                            os.remove(os.path.join(self._cmPrjDir, "SimOutput", 'L3_SW_Requirements', i._erg_folder_name, i._name+results))
                        except:pass
            # elif (i.GetName() not in input_List) and (i.GetName() not in Crashed_list):
            elif i.GetName().split(".")[0] not in input_List:
                self.input_file_list.append(input_file)
                # Deleting erg's if exists for resimulation
                if File_Type == 1:
                    try:
                        os.remove(crashed_scenarios)
                        result_names_types = [".erg", ".erg.info", ".zip", "_ae.erg", "_ae.erg.info"]
                        for results in result_names_types:
                            result_names_1 = input_file + results
                            os.remove(os.path.join(self._cmPrjDir, "SimOutput", self._outputFolderName, result_names_1))
                    except:pass
            else:
                update_excel[input_file] = "Available"

    # Delete Crashed Scenarios and ERGs
    def delete_ergs(self, ergs):  # Function to remove ERGs
        if os.path.exists(ergs):
            os.remove(ergs)
            if os.path.exists(str(ergs) + ".info"):
                os.remove(str(ergs) + ".info")

    def delete_names(self, names):  # Function to remove names
        if names in self._ergFilePaths.keys():
            del self._ergFilePaths[names]

    # Function to call if carmaker crashes
    def carmaker_crashing(self):
        config.read(crashed_scenarios)
        for name, erg in config.items("Crashed_Info"):
            self.delete_ergs(erg)
            self.delete_names(name)
            if self.check_flag != "2":  # If simulation + validation button
                try:
                    ergs = erg.split("_ae")
                    ergs = ergs[0] + ergs[1]
                    self.delete_ergs(ergs)
                    self.delete_names(name.split("_ae")[0])
                except:
                    ergs = erg.split(".")
                    ergs = ergs[0] + "_ae." + ergs[1]
                    self.delete_ergs(ergs)
                    self.delete_names(str(name) + "_ae")
            else:pass


    class SimulationRunningThreadHeadless(threading.Thread):
        """
        Class for a worker thread for Carmaker simulations using the headless mode. Optionally it allows to use a core license for the simulation.
        This class inherits threading.Thread.
        This class is called and it's methods are handled internally by the cCMEvalTestRunner class.

        :param testRunner: Instance of cCMEvalTestRunner class
        :param maxTimeInSimulationSeconds: Maximum time a single testrun is allowed to simulate in simulated time.
        :param maxExecTimeSeconds: Maximum time a single testrun is allowed to simulate in wall-clock time.
        :param nRetries: Number of retries if something goes wrong while simulation, e.g. temporary loss of network connection and so no license checkout possible or \
        Carmaker does not react in configured timeouts due to high work load on the machine.
        :param retryTimeSeconds: Time to wait between retries if something goes wrong in the simulation.
        :param executable: Absolute path to Carmaker binary executable. If no is given, the configured path from the GUI file in the Carmaker project will be used. \
        If no paths are given in the GUI file or the GUI file does not exist, a standard path in the CarMaker project is used: CM_Project/src/CarMaker.win64.exe.
        :param ignoreExistingResFile: If True, always all testruns will be executed, regardless if a result file is already existing. If False, it will skip testruns with \
        an already existing result file.
        :param useCoreLicense: If True, it will use core licenses to simulate. If False, it will use usual office licenses.
        :param numberOfCarMakerInstances: Number of parallel Carmaker instances to simulate given testruns.
        :param showCMLogOutput: If True, the content from the latest log file created by CarMaker is shown in the logger. This option does not work if multiple \
        instances of the CarMaker are running in parallel, i.e. it effects only if numberOfCarMakerInstances equals 1.
        """

        # testRunner - the cCMEvalTestRunner instance this thread belongs to
        def __init__(self, testRunner, maxTimeInSimulationSeconds=None, maxExecTimeSeconds=None, nRetries=1,
                     retryTimeSeconds=2, executable=None, ignoreExistingResFile=False, useCoreLicense=False,
                     numberOfCarMakerInstances=1, showCMLogOutput=False):
            threading.Thread.__init__(self)
            self._testRunner = testRunner
            self._cmCtrl = None
            self._iLockedWorkDistributionLock = False
            self._iLockedAccumulatorLock = False
            self._maxTimeInSimulationSeconds = maxTimeInSimulationSeconds
            self._maxExecTimeSeconds = maxExecTimeSeconds
            self._nRetries = nRetries
            self._retryTimeSeconds = retryTimeSeconds
            self._executable = executable
            self._ignoreExistingResFile = ignoreExistingResFile
            self._useCoreLicense = useCoreLicense
            self._exception = []
            self._numberOfCarMakerInstances = numberOfCarMakerInstances
            self._showCMLogOutput = showCMLogOutput

        def run(self):
            """
            Method to start the worker thread.
            """

            try:
                while True:
                    self._testRunner._workDistributionLock.acquire()  # sync start
                    self._iLockedWorkDistributionLock = True
                    success = None
                    if self._testRunner._nextTestRunToExecuteIndex >= len(self._testRunner._testRuns):
                        break  # no more work
                    else:
                        successfulProcessing = False
                        # make a local copy of the global index, increment the global index and release the lock
                        nextTestRunToExecuteIndex = self._testRunner._nextTestRunToExecuteIndex
                        self._testRunner._nextTestRunToExecuteIndex += 1
                        self._testRunner._workDistributionLock.release()  # sync end
                        self._iLockedWorkDistributionLock = False
                        # execute the available test run
                        logger.info("####### Processing TestRun {0} of {1} #######".format(nextTestRunToExecuteIndex + 1,
                                                                                            len(
                                                                                                self._testRunner._testRuns)))
                        currTestRun = self._testRunner._testRuns[nextTestRunToExecuteIndex]
                        currTestRunName = currTestRun.GetName()
                        logger.info("#### TestRun Name: {0} ####".format(currTestRunName))
                        currErgFilePath = os.path.join(self._testRunner._cmPrjDir, "SimOutput",
                                                       self._testRunner._outputFolderName, currTestRunName + ".erg")
                        # Check whether corresponding *.erg File already exists
                        if not os.path.exists(currErgFilePath) or self._ignoreExistingResFile:
                            if os.path.exists(currErgFilePath):
                                os.remove(currErgFilePath)
                            currErgInfoFilePath = os.path.join(currErgFilePath, ".info")
                            if os.path.exists(currErgInfoFilePath):
                                os.remove(currErgInfoFilePath)
                            # Generate TestRun
                            # logger.info('Setting up TestRun for {0}\\{1}'.format(self._testRunner._outputFolderName, currTestRunName))
                            currTestRun.save(self._testRunner._evaluationTestRunDirectory)

                            # Start Simulation to generate *.erg File
                            # logger.info('Running Simulation for {0}\\{1}'.format(self._testRunner._outputFolderName, currTestRunName))
                            if self._cmCtrl is None:
                                if self._executable is not None:
                                    self._cmCtrl = CMRC_HeadlessInstance(self._testRunner._cmPrjDir, self._executable,
                                                                         self._testRunner._outputFolderName,
                                                                         self._useCoreLicense)
                                else:
                                    # get executable path from GUI file if available
                                    if os.path.exists(os.path.join(self._testRunner._cmPrjDir, "Data", "Config", "GUI")):
                                        with open(os.path.join(self._testRunner._cmPrjDir, "Data", "Config", "GUI"),
                                                  'r') as f:
                                            guiFileContent = f.readlines()

                                        guiPathsFollowing = False
                                        cmExePath = None
                                        for currLine in guiFileContent:
                                            if currLine.startswith('CM.Exe =') and len(currLine) > 10:
                                                if IS_LINUX_SYSTEM:
                                                    cmExePath = currLine.replace("CM.Exe =", "").strip()
                                                else:
                                                    cmExePath = currLine.replace("CM.Exe =", "").replace("/", "\\").strip()
                                                break
                                            if "CM.Exe.History" in currLine:
                                                guiPathsFollowing = True
                                                continue
                                            if guiPathsFollowing:
                                                if IS_LINUX_SYSTEM:
                                                    cmExePath = currLine.replace("\t", "").replace("\n", "")
                                                else:
                                                    cmExePath = currLine.replace("\t", "").replace("/", "\\").replace("\n",
                                                                                                                      "")
                                                break
                                        if cmExePath is None:
                                            if IS_LINUX_SYSTEM:
                                                absExePath = os.path.join(self._testRunner._cmPrjDir, 'src', 'CarMaker.linux64')
                                            else:
                                                absExePath = os.path.join(self._testRunner._cmPrjDir, 'src', 'CarMaker.win64.exe')
                                            logger.info(
                                                "Could not find executable path in GUI file. Try standard directory: {}".format(
                                                    absExePath))
                                        else:
                                            absExePath = cmExePath if os.path.isabs(cmExePath) else os.path.join(
                                                self._testRunner._cmPrjDir, cmExePath)
                                            logger.info(
                                                "Found executable path in GUI file. Using path: {}".format(absExePath))
                                    else:
                                        if IS_LINUX_SYSTEM:
                                            absExePath = os.path.join(self._testRunner._cmPrjDir, 'src', 'CarMaker.linux64')
                                        else:
                                            absExePath = os.path.join(self._testRunner._cmPrjDir, 'src', 'CarMaker.win64.exe')
                                        logger.info(
                                                "No executable path given and GUI file does not exist. Try standard directory: {}".format(
                                                    absExePath))

                                    self._cmCtrl = CMRC_HeadlessInstance(self._testRunner._cmPrjDir, absExePath,
                                                                         self._testRunner._outputFolderName,
                                                                         self._useCoreLicense)
                            for nRetry in range(self._nRetries):
                                if not os.path.exists(currErgFilePath):
                                    success, reason = self._cmCtrl.execTestrun(
                                        os.path.join(self._testRunner._outputFolderName, currTestRunName),
                                        self._maxExecTimeSeconds)  # TODO
                                    if reason == 'TIMEOUT':  # do not try again if the cancel reason was a timeout which was given by the user
                                        break

                                    if self._showCMLogOutput and self._numberOfCarMakerInstances == 1:
                                        # get latest log file to show in stdout
                                        logFolderPath = os.path.join(os.path.dirname(currErgFilePath), socket.gethostname(), "Log")
                                        latest_logfile = os.listdir(logFolderPath)[-1]
                                        latest_logfile_path = os.path.join(logFolderPath, latest_logfile)
                                        if os.path.exists(latest_logfile_path) and os.path.isfile(latest_logfile_path):
                                            logfile_content = []
                                            with open(latest_logfile_path, "r") as file:
                                                logfile_content = file.readlines()
                                            logger.info("\n========== Log output from Carmaker start ==========\n{}\n========== Log output from Carmaker end =========="
                                                         .format("".join(logfile_content)))
                                        else:
                                            logger.error("Could not find latest log file. Expected it here: {}.".format(latest_logfile_path))
                                    elif self._showCMLogOutput and self._numberOfCarMakerInstances != 1:
                                        logger.info("Info: Parameter showCMLogOutput can only work if numberOfCarMakerInstances is 1. Hence ignoring it.")

                                    if nRetry > 0:
                                        logger.info("Something went wrong with the simulation. Restarting in %ds...",
                                                     self._retryTimeSeconds)
                                        time.sleep(self._retryTimeSeconds)
                                else:
                                    break

                            testRunFilePathRel = os.path.join(self._testRunner._outputFolderName, currTestRunName)

                        # else:
                        #    logger.info('Erg file {0} already exists. Skipping Simulation.'.format(currErgFilePath))

                        # Load *.erg File
                        self._testRunner._accumulatorLock.acquire()
                        self._iLockedAccumulatorLock = True
                        if os.path.exists(currErgFilePath) and success:
                            self._testRunner._ergFilePaths[currTestRunName] = currErgFilePath
                            currTestRun.SetExecStatusAndErgPath(True, currErgFilePath)
                            self._testRunner._testRunList.append(currTestRun)
                            self._testRunner._accumulatorLock.release()
                            self._iLockedAccumulatorLock = False
                            # logger.info('Successfully generated {0}'.format(currErgFilePath))
                        else:
                            currTestRun.SetExecStatusAndErgPath(False, None)
                            currTestRun.SetErrMsg("Could not find Erg File or simulation was not successful.")
                            self._testRunner._testRunList.append(currTestRun)
                            self._testRunner._accumulatorLock.release()
                            self._iLockedAccumulatorLock = False
                            logger.error("Could not find Erg File or simulation was not successful.")
                            self._exception.append(SimulationExecutionException("Could not find Erg File or simulation was not successful."))

            except Exception as e:
                self._exception.append(e)
            finally:
                if self._testRunner._workDistributionLock.locked() and self._iLockedWorkDistributionLock:
                    # this block is entered when the thread has no more work or when an exception has occurred while working
                    self._testRunner._workDistributionLock.release()  # ensure the lock is released before terminating
                    self._iLockedWorkDistributionLock = False

                if self._testRunner._accumulatorLock.locked() and self._iLockedAccumulatorLock:
                    # this block is entered when the thread has no more work or when an exception has occurred while working
                    self._testRunner._accumulatorLock.release()  # ensure the lock is released before terminating
                    self._iLockedAccumulatorLock = False

        def get_exception(self):
            return self._exception


        # Each of these threads drives a single CarMaker instance.
    class SimulationRunningThread (threading.Thread):
        """
        Class for a worker thread for Carmaker simulations using the office license. This class inherits threading.Thread.
        This class is called and it's methods are handled internally by the cCMEvalTestRunner class.

        :param testRunner: Instance of cCMEvalTestRunner class
        :param carMakerPort: Port of the Carmaker application.
        :param maxTimeInSimulationSeconds: Maximum time a single testrun is allowed to simulate in simulated time.
        :param maxExecTimeSeconds: Maximum time a single testrun is allowed to simulate in wall-clock time.
        :param nRetries: Number of retries if something goes wrong while simulation, e.g. temporary loss of network connection and so no license checkout possible or \
        Carmaker does not react in configured timeouts due to high work load on the machine.
        :param retryTimeSeconds: Time to wait between retries if something goes wrong in the simulation.
        :param executable: Absolute path to Carmaker binary executable. If no is given, the configured path from the GUI file in the Carmaker project will be used.
        :param ignoreExistingResFile: If True, always all testruns will be executed, regardless if a result file is already existing. If False, it will skip testruns with \
        an already existing result file.
        :param cmGuiPath: Absolute path to Carmaker GUI. If no is given, the latest Carmaker version in the standard installation path of Carmaker is used.
        Note that the size of the result file increases with higher sample rate.
        """
        # testRunner - the cCMEvalTestRunner instance this thread belongs to
        def __init__(self, testRunner, carMakerPort, id,cmPrjDir, CMprojectexe, CMprojectwin64exe, check_flag, rec_bat_path, rec_extract, rec_video_extract, rec_folders, sim_output, maxTimeInSimulationSeconds = None,worker = None,total_runs = None, init_per = None,statistics = None, MDF= 0,CEMValues=None,nRetries = 1, custom_test_run_folder=None,maxExecTimeSeconds = None,retryTimeSeconds = 2, executable = None, ignoreExistingResFile = False,cmGuiPath = None,signal_to_evaluate=[],instanceCount=None):
            threading.Thread.__init__(self)
            global initpercent,statistics_counts,project_name,count,flag
            self._testRunner = testRunner
            self._cmCtrl       = None
            self.CEMValues = CEMValues
            self._iLockedWorkDistributionLock = False
            self._iLockedAccumulatorLock = False
            self._iLockedImageAccumulatorLock = False
            self._carMakerPort = carMakerPort
            self.id = id
            self._maxTimeInSimulationSeconds = maxTimeInSimulationSeconds
            self._maxExecTimeSeconds = maxExecTimeSeconds
            self._retryTimeSeconds = retryTimeSeconds
            self._ignoreExistingResFile = ignoreExistingResFile
            self._executable = executable
            self.cmPrjDir = cmPrjDir
            self.instanceCount = instanceCount
            try:
                self.CMprojectexe = CMprojectexe
            except:
                self.CMprojectexe = cmGuiPath
            self.CMprojectwin64exe = CMprojectwin64exe
            self.worker = worker
            self.total_runs = total_runs
            self.rec_bat_path = rec_bat_path
            self.check_flag = check_flag
            self.rec_extract = rec_extract
            self.rec_video_extract = rec_video_extract
            self.sim_output = str(sim_output)
            self.rec_folders = rec_folders
            self.MDF = MDF
            initpercent =init_per
            statistics_counts = statistics
            self._nRetries = nRetries
            self.custom_testrun_path = custom_test_run_folder
            self.Project_Config_read()
            self.signal_verifier = True
            self.signal_to_evaluate = signal_to_evaluate
            self._exception = []
            self.Previous_unwanted_PID =[]

        def run(self):
            """
            Method to start the worker thread.
            """
            global flag,statistics_counts,initpercent
            try:
                simulated_testruns = []
                CM_instance_error = executed = 0
                CarMakerInstance_Opened = 0
                #if project_name == "ACC":
                License_file = os.path.join(re.search('(?i).*IPG', self.CMprojectexe).group(), 'etc', 'Licenses')
                License_server = cCMEvalTestRunner.get_License_server(License_file)
                self.total_testruns = len(self._testRunner._testRuns)
                First_Time = 1
                enter_once = True
                restart_ecal = False
                while True:
                    is_first_time =0
                    self.thread_exe = False
                    self._testRunner._workDistributionLock.acquire() # sync start
                    self._iLockedWorkDistributionLock = True
                    if self._testRunner._nextTestRunToExecuteIndex >= len(self._testRunner._testRuns):
                        break # no more work
                    elif self.signal_verifier == False:
                        logger.info("Signals required for validation is missing in the VSP environment")
                        try:
                            self.worker.signals.logsignal.emit("Signals required for validation is missing in the VSP environment")
                        except:pass # For command line exe
                        break
                    else:
                        # make a local copy of the global index, increment the global index and release the lock
                        nextTestRunToExecuteIndex = self._testRunner._nextTestRunToExecuteIndex
                        self._testRunner._nextTestRunToExecuteIndex += 1
                        self._testRunner._workDistributionLock.release() # sync end
                        self._iLockedWorkDistributionLock = False
                        currTestRun = self._testRunner._testRuns[nextTestRunToExecuteIndex]

                        try:
                            currTestRunName = currTestRun.GetName()
                        except:
                            currTestRunName = currTestRun           # For Rec ReSimulation

                        if self.custom_testrun_path:
                            currErgFilePath = os.path.join(self._testRunner._outputFolderName[0], currTestRunName)
                            testRunFilePathRel = os.path.join(self.custom_testrun_path, currTestRunName)
                        else:
                            try:
                                if 'L3_SW_Requirements' in self._testRunner._outputFolderName:
                                    currErgFilePath = os.path.join(self._testRunner._cmPrjDir, "SimOutput", 'L3_SW_Requirements', currTestRun.GetFolderName(), currTestRunName+".erg")
                                    testRunFilePathRel = os.path.join('L3_SW_Requirements', currTestRun.GetFolderName(), currTestRunName)
                                else:
                                    if "All_Usecases" in self._testRunner._outputFolderName[nextTestRunToExecuteIndex]:
                                        currErgFilePath = os.path.join(self._testRunner._cmPrjDir, "SimOutput",self._testRunner._outputFolderName[nextTestRunToExecuteIndex],self._testRunner._testRuns[nextTestRunToExecuteIndex].folderpath,currTestRunName + ".erg")
                                    else:
                                        currErgFilePath = os.path.join(self._testRunner._cmPrjDir, "SimOutput",self._testRunner._outputFolderName[nextTestRunToExecuteIndex], currTestRunName + ".erg")

                                    if self.MDF == 1:
                                        currErgFilePath = os.path.join(self._testRunner._cmPrjDir, "SimOutput",self._testRunner._outputFolderName[0],currTestRunName + ".mf4")
                                    testRunFilePathRel = os.path.join(self._testRunner._outputFolderName[nextTestRunToExecuteIndex], currTestRunName).replace('ergs', 'testRunVariations')
                            except:
                                if "All_Usecases" in self._testRunner._outputFolderName[0]:
                                    currErgFilePath = os.path.join(self._testRunner._cmPrjDir, "SimOutput",self._testRunner._outputFolderName[0],self._testRunner._testRuns[nextTestRunToExecuteIndex].folderpath,currTestRunName + ".erg")
                                else:
                                    currErgFilePath = os.path.join(self._testRunner._cmPrjDir, "SimOutput",self._testRunner._outputFolderName[0],currTestRunName + ".erg")

                                if self.MDF == 1:
                                    currErgFilePath = os.path.join(self._testRunner._cmPrjDir, "SimOutput",self._testRunner._outputFolderName[0],currTestRunName + ".mf4")
                                testRunFilePathRel = os.path.join(self._testRunner._outputFolderName[0],currTestRunName)
                        # Check whether corresponding *.erg File already exists
                        if not os.path.exists(currErgFilePath):
                            if self.sim_output == '0':
                                logger.info("####### Processing TestRun {0} of {1} #######".format(nextTestRunToExecuteIndex + 1, len(self._testRunner._testRuns)))
                                try:
                                    self.worker.signals.logsignal.emit("####### Processing TestRun {0} of {1} #######".format(nextTestRunToExecuteIndex + 1, len(self._testRunner._testRuns)))
                                    self.worker.signals.logsignal.emit("Starting simulation of " + currTestRunName)
                                except:pass  # cmd line execution
                            try:
                                currTestRun.save(self._testRunner._evaluationTestRunDirectory)
                            except:pass
                            First_Time = First_Time+1
                            if self._cmCtrl is None:
                                cCMEvalTestRunner._killCarMakerProcesses()
                                logger.info(
                                    "Checking for network availability. Simulation will continue only if connected to the network.")
                                self.ping_status = cCMEvalTestRunner.ping_check(License_server)

                                # 3 retries to reconnect to network, displaying info window to the user
                                for nRetry in range(self._nRetries):
                                    if not self.ping_status:
                                        # cCMEvalTestRunner._killCarMakerProcesses()
                                        try:
                                            self.worker.signals.logsignal.emit("CarMaker License not available. Reconnect to the network.")
                                        except:pass # For command line exe
                                        if not flag:
                                            flag = True
                                            try:
                                                self.worker.signals.popwindow.emit("Connection lost.\nReconnect to the network,\nand then click 'OK'.")
                                            except:pass # For command line exe

                                        # Wait until user clicks OK on the message box.
                                        event.wait()
                                        time.sleep(3)
                                        flag = False
                                        logger.info("Checking for network availability. Simulation will continue only if connected to the network.")
                                        self.ping_status = cCMEvalTestRunner.ping_check(License_server)
                                    else:
                                        break

                                # if not reconnected after max retries break.
                                if not self.ping_status:
                                    executed = 0
                                    try:
                                        self.worker.signals.logsignal.emit(f"CarMaker license not available, hence skipping {currTestRunName.split('_')[0]} usecase.")
                                    except:pass # For command line exe
                                    # Update the progress bar(only by one thread) when license not abailable
                                    self._testRunner._workDistributionLock.acquire()
                                    executed += 1
                                    self._testRunner._workDistributionLock.release()
                                    if executed == 1:
                                        statistics_counts += len(set(self._testRunner._testRuns) - set(simulated_testruns))
                                        n = 95 / float(self.total_runs)
                                        initpercent = float(initpercent) + (
                                                    n * len(set(self._testRunner._testRuns) - set(simulated_testruns)))
                                        try:
                                            self.worker.signals.progress.emit(initpercent,"Statistics: " + str(statistics_counts) + "/" + str(self.total_runs) + " Testruns-Simulation-completed")
                                        except:pass # For command line exe
                                    break

                            if self._cmCtrl is None:
                                for nRetry in range(self._nRetries):
                                    is_first_time = 1
                                    self._cmCtrl = CMRC(startCM=True, cmGuiPath=self.CMprojectexe, safeMode=True, port = self._carMakerPort,
                                                    image_extraction=self._testRunner._image_extract, video_extraction = self._testRunner.videoextract,
                                                    CMprojectDir=self.cmPrjDir, CMprojectExe = self.CMprojectwin64exe, sim_output=self.sim_output, worker=self.worker, id = self.id)

                                    if self.sim_output == '1':
                                        try:
                                            self.killCMexe(CEM_process.pid)
                                        except:
                                            pass
                                    if self.sim_output == '1':
                                        CEM_process = subprocess.Popen(['cmd.exe'], stdout=subprocess.PIPE,stdin=subprocess.PIPE, encoding='utf-8')
                                        time.sleep(5)
                                        self.startCemNodes(currTestRunName, CEM_process)
                                        time.sleep(20)
                                        if self._cmCtrl._error:
                                            logger.info("Something went wrong with the simulation. Restarting in %ds...",
                                                     self._retryTimeSeconds)
                                            cCMEvalTestRunner._killCarMakerProcesses()
                                            time.sleep(self._retryTimeSeconds)
                                        else:
                                            break
                                    if self.sim_output == '2' and self._cmCtrl is not None:
                                        break

                                self._cmCtrl.SetProjectDir(self._testRunner._cmPrjDir)
                                if self._executable is not None:

                                    self._cmCtrl.AppExecute(self._executable.replace("\\", "/"))
                                if self.rec_video_extract == "1" or self._testRunner._image_extract:
                                    time.sleep(self.Before_Launch_Movie)
                                    self._cmCtrl.Launch_IPGMovie(self.launch_IPG)
                                    time.sleep(self.After_Launch_Movie )
                                if self.sim_output == '0':
                                    self._cmCtrl.StartAndConnect()
                                    time.sleep(self.After_Start_Connect)
                                self._cmCtrl.SetSaveMode("all")
                                self._cmCtrl.SetSimSpeed("max")

                            try:
                                if self.sim_output == '1' or self.sim_output == '2':
                                    logger.info("####### Processing TestRun {0} of {1} #######".format(nextTestRunToExecuteIndex + 1, len(self._testRunner._testRuns)))
                                    try:
                                        self.worker.signals.logsignal.emit("####### Processing TestRun {0} of {1} #######".format(nextTestRunToExecuteIndex + 1, len(self._testRunner._testRuns)))
                                        self.worker.signals.logsignal.emit("Starting simulation of " + currTestRunName)
                                    except:pass #cmd line execution

                                    if self.sim_output == '1' and is_first_time == 0 and self.CEMValues == '0':
                                        cem_Not_created = self.startCemNodes_next(currTestRunName, CEM_process)
                                        if cem_Not_created == 1:
                                            try:
                                                self.killCMexe(CEM_process.pid)
                                            except:
                                                pass
                                            CEM_process = subprocess.Popen(['cmd.exe'], stdout=subprocess.PIPE, stdin=subprocess.PIPE, encoding='utf-8')
                                            self.startCemNodes(currTestRunName, CEM_process ,cem_Not_created = cem_Not_created)
                                            time.sleep(20)
                                    is_first_time = 0

                                ###################################
                                if self._executable is not None:
                                    self._cmCtrl.AppExecute(self._executable.replace("\\", "/"))

                                # Get PID of Carmaker_win64_node.exe
                                if self.sim_output != '2':
                                    exe_pid = self.get_CMpid()
                                # killing all the nodes
                                try:
                                    if self.sim_output == '2' and restart_ecal == True:
                                        self._cmCtrl.relaunch_ecal_nodes()

                                    self._cmCtrl.StartAndConnect()
                                    restart_ecal=True
                                except Exception as e:
                                    print(e)
                                # reloads the dll, ensures the function reset before every testrun

                                srcDirTestRun = testRunFilePathRel
                                try:
                                    self._cmCtrl.LoadTestRun(srcDirTestRun)
                                    time.sleep(self.After_Load_TestRun)
                                    self._cmCtrl.SetResultFileName(currErgFilePath)
                                    if self.sim_output == '1' and self.CEMValues == '0':
                                        time.sleep(10)    #if it is ROS and CEM real
                                    else:
                                        time.sleep(self.After_SetResultFileName)

                                    testrun = self._cmCtrl.Simulate(self._maxTimeInSimulationSeconds,currTestRunName=currTestRunName,maxExecTimeSeconds=self._maxExecTimeSeconds)
                                except:
                                    ping_status = cCMEvalTestRunner.ping_check(License_server)
                                    if not ping_status:
                                        testrun = 1
                                        self.handle_connectivity(ping_status)
                                        if self.sim_output == '1':
                                            cCMEvalTestRunner.KillCEMNodes(self.CMprojectwin64exe)
                                            try:
                                                self.killCMexe(CEM_process.pid)
                                            except:
                                                pass
                                            CEM_process = subprocess.Popen(['cmd.exe'], stdout=subprocess.PIPE,
                                                                           stdin=subprocess.PIPE, encoding='utf-8')
                                            self.startCemNodes(currTestRunName, CEM_process)
                                            time.sleep(20)
                                        elif self.sim_output == '2':
                                            self._cmCtrl.relaunch_ecal_nodes()
                                        self._cmCtrl.StartAndConnect()
                                        self._cmCtrl.LoadTestRun(srcDirTestRun)
                                        time.sleep(self.After_Load_TestRun)
                                        self._cmCtrl.SetResultFileName(currErgFilePath)
                                        if self.sim_output == '1' and self.CEMValues == '0':
                                            time.sleep(10)  # if it is ROS and CEM real
                                        else:
                                            time.sleep(self.After_SetResultFileName)

                                time.sleep(self.After_Simulate)
                                self._cmCtrl.CM_DisConnect() # Disconnect CM after each simulation
                                #####Kill Unwanted CARMAKER exe########
                                try:
                                    unwanted_pid = self.GetUnwanted_CMTasks()
                                    common_PID = np.intersect1d(unwanted_pid, self.Previous_unwanted_PID)
                                    for eachpid in common_PID:
                                        self.kill_CMpid(int(eachpid), currTestRunName)
                                    self.Previous_unwanted_PID = unwanted_pid
                                except Exception as e:
                                    print(e)

                                #########################################
                                if testrun == 1: #return 1, if carmaker crashes
                                    logger.error("Resimulating scenario :: {0}".format(currTestRunName)) #Resimulating crashed scenarios
                                    try:
                                        self.worker.signals.logsignal.emit("Resimulating scenario :: {0}".format(currTestRunName))
                                    except:pass #cmd line execution
                                    self._cmCtrl.StartAndConnect()  # reloads the dll, ensures the function reset before every testrun
                                    time.sleep(2)
                                    self.kill_CMpid(exe_pid, currTestRunName)
                                    exe_pid = self.get_CMpid()
                                    resimulate = self._cmCtrl.Simulate(self._maxTimeInSimulationSeconds, currTestRunName)
                                    self._cmCtrl.CM_DisConnect() # Disconnect CM after each simulation

                                    if resimulate == 1: #Resimulating crashed scenarios
                                        self.kill_CMpid(exe_pid, currTestRunName) # Kill CM Bin, if Simulation crashes
                                        self.writeintoconfig(currTestRunName, currErgFilePath)
                                        self.deleteCrashedERGs(currErgFilePath)
                                if self._testRunner._image_extract and testrun != 1: # birdeye extraction
                                    self.extract_birdeye_image(currTestRunName, currErgFilePath, self.check_flag)
                                if self.rec_video_extract == "1" and testrun != 1: # rec+video generation
                                    self._cmCtrl.CreateScenarioVideo(self.rec_bat_path, self.sim_output, self._testRunner._cmPrjDir, currTestRunName, os.path.join("SimOutput", self._testRunner._outputFolderName), read_configFile.framerate, read_configFile.width, read_configFile.height)
                                if self.rec_extract == "1" and testrun != 1: # Only rec generation
                                    time.sleep(int(read_configFile.time_delay))
                                    if "." in currTestRunName:
                                        currTestRunName = currTestRunName.replace(".", "_dot_")
                                    sim_bat = self.rec_bat_path + "\\" + str(currTestRunName) + r".bat"
                                    os.system(sim_bat)
                                try:
                                    statistics_counts+=1
                                    n = float(95 / float(self.total_runs))
                                    initpercent = float(initpercent) +n
                                    try:
                                        self.worker.signals.progress.emit(initpercent, "Statistics: " + str(statistics_counts) + "/" + str(self.total_runs) + " Testruns-Simulation-completed")
                                        #os.system('taskkill /F /IM "cmd.exe" /FI "WINDOWTITLE eq {0}"'.format(currTestRunName))
                                    except:pass # For command line exe
                                except:pass#cmdlineexecution
                                try:
                                    self.CEM_pid_Kill(currTestRunName)
                                except:pass

                            except Exception as e:
                                logger.info(f"Exception occured due to {e} in simulation {currTestRunName}") #4th Feb
                                try:
                                    self.worker.signals.logsignal.emit(f"Exception occured due to some error in simulation {currTestRunName}")
                                    self.CEM_pid_Kill(currTestRunName)
                                    self.killCMexe(exe_pid)
                                    #os.system('taskkill /F /IM "cmd.exe" /FI "WINDOWTITLE eq {0}"'.format(currTestRunName))
                                except:pass #cmd line execution
                        else:
                            if os.path.getsize(currErgFilePath) <= 15:   ##If erg size is 0 bytes then delete the erg and info files and resimulate
                                os.remove(currErgFilePath)
                                if os.path.exists(str(currErgFilePath) + ".info"):
                                    os.remove(str(currErgFilePath) + ".info")
                            else:
                                try:
                                    statistics_counts += 1
                                    n = float(95 / float(self.total_runs))
                                    initpercent = initpercent + n
                                    self.worker.signals.progress.emit(initpercent,  "Statistics: " + str(statistics_counts) + "/" + str(self.total_runs) + " Testruns-Simulation-completed")
                                except:pass#cmdl;ine execution
                                logger.info('Erg file {0} already exists. Skipping Simulation.'.format(currErgFilePath))
                                try:
                                    self.worker.signals.logsignal.emit('Erg file {0} already exists. Skipping Simulation.'.format(currErgFilePath))
                                except:pass  # cmd line execution

                        # Load *.erg File
                        self._testRunner._accumulatorLock.acquire()
                        self._iLockedAccumulatorLock = True
                        if os.path.exists(currErgFilePath):
                            if isinstance(currTestRun, cmTestRun):
                                currTestRun.SetExecStatusAndErgPath(True, currErgFilePath)
                                self._testRunner._testRunList.append(currTestRun)
                            self._testRunner._ergFilePaths[currTestRunName] = currErgFilePath
                            self._testRunner._accumulatorLock.release()
                            self._iLockedAccumulatorLock = False
                        else:
                            if isinstance(currTestRun, cmTestRun):
                                currTestRun.SetExecStatusAndErgPath(False, currErgFilePath)
                                currTestRun.SetErrMsg("Could not find Erg File or simulation was not successful.")
                                self._testRunner._testRunList.append(currTestRun)
                            self._testRunner._accumulatorLock.release()
                            self._iLockedAccumulatorLock = False
                            logger.error("Could not find Erg File or simulation was not successful.")
                            # self._exception.append(SimulationExecutionException("Could not find Erg File or simulation was not successful.")) # Commented due to error with Jenkins

                    if os.path.exists(currErgFilePath) and enter_once == True and len(self.signal_to_evaluate) != 0:
                        enter_once = False
                        logger.info("Checking for signals required for validation")
                        try:
                            self.worker.signals.logsignal.emit("Checking for signals required for validation")
                        except:pass
                        parsed = cm_eval_erg_parser.cErgFile(currErgFilePath, True)
                        parsed_signals = parsed.getArrayOfAvailableValues()

                        for signal in self.signal_to_evaluate:
                            if signal in parsed_signals:
                                self.signal_verifier = True
                            else:
                                self.signal_verifier = False
                                break
                        if self.signal_verifier:
                            logger.info("Signals required for validation are present. Proceeding further")
                            try:
                                self.worker.signals.logsignal.emit("Signals required for validation are present. Proceeding further")
                            except:pass
            except Exception as e:
                logger.error(f"Exception occured due to {e}")
                self._exception.append(e)

            finally:
                if self._testRunner._workDistributionLock.locked() and self._iLockedWorkDistributionLock:
                    # this block is entered when the thread has no more work or when an exception has occurred while working
                    self._testRunner._workDistributionLock.release() # ensure the lock is released before terminating
                    self._iLockedWorkDistributionLock = False

                if self._testRunner._accumulatorLock.locked() and self._iLockedAccumulatorLock:
                    # this block is entered when the thread has no more work or when an exception has occurred while working
                    self._testRunner._accumulatorLock.release() # ensure the lock is released before terminating
                    self._iLockedAccumulatorLock = False

                if self.rec_folders != '0':
                    # Close CarMaker
                    if self._cmCtrl is not None:
                        self._cmCtrl.Close()
                        self._cmCtrl = None

        def get_exception(self):
            return self._exception

        def get_CMpid(self):
            """
            Function to get CM binary process ID
            :return: Process ID of CarMaker_win64_node.exe
            """
            time.sleep(5)
            ExeInfo = self._cmCtrl.GetCMExePID()
            exe_pid = self.extract_pid(ExeInfo=ExeInfo)
            return exe_pid

        def kill_CMpid(self, exe_pid, currTestRunName):
            """
            :param exe_pid: Process ID of CarMaker_win64_node.exe
            :param currTestRunName: TestRunName, which Simulation is done
            :return: None
            """
            try:
                self.killCMexe(exe_pid)
                #os.system('taskkill /F /IM "cmd.exe" /FI "WINDOWTITLE eq {0}"'.format(currTestRunName))
            except:pass
            time.sleep(5)

        def GetUnwanted_CMTasks(self):
            unwanted_pid = []
            for proc in psutil.process_iter():
                pinfo = proc.as_dict(attrs=['pid', 'name'])
                pinfo['vms'] = proc.memory_info().num_page_faults
                if pinfo['name'] == CARAMKER_EXE_NAME :# 'carmaker_node.win64.exe':
                    if int(pinfo['vms']) < 100000:
                        unwanted_pid.append(int(pinfo['pid']))
            return unwanted_pid

        def killCMexe(self,exe_pid):
            """
            Function to kill CM binary after each Simulation
            :param exe_pid:  Process ID of CarMaker_win64_node.exe
            :return: None
            """
            try:
                os.kill(int(exe_pid), 9)
                if psutil.pid_exists(int(exe_pid)):
                    os.kill(int(exe_pid), 9)
                    if psutil.pid_exists(int(exe_pid)):
                        os.kill(int(exe_pid), 9)
                else:pass
            except:
                try:
                    os.kill(int(exe_pid), 9)
                except:pass

        # Function to write crashed scenarios and erg path into config file
        def writeintoconfig(self, currTestRunName, currErgFilePath):
            section_name = 'Crashed_Info'
            if not config.has_section(section_name):
                config.add_section(section_name)
            config.set(section_name, currTestRunName, currErgFilePath)
            with open(crashed_scenarios, 'w+') as configfile:
                config.write(configfile)

        # Function to delete crashed ERGS
        def deleteCrashedERGs(self, currErgFilePath):
            if os.path.exists(currErgFilePath):
                os.remove(currErgFilePath)
                if os.path.exists(str(currErgFilePath) + ".info"):
                    os.remove(str(currErgFilePath) + ".info")

        def CEM_pid_Kill(self,testrunname,Check_Node = 0):
            """
            Function to get and Kill the CEM process ID
            :return: Process ID of all CEM nodes and KILL those PID's
            """
            project_Path = self.cmPrjDir.partition("/conf")[0]
            if not os.path.exists(os.path.join(project_Path,"CEM_node_log")):
                os.makedirs(os.path.join(project_Path,"CEM_node_log"))
            path_to_file = os.path.join(project_Path,"CEM_node_log",testrunname+".txt")
            if os.path.exists(path_to_file):
                if os.path.getsize(path_to_file) > 0:
                    try:
                        file_contents = open(path_to_file,"r")
                    except:return 1
                else:
                    time.sleep(5)
                    if os.path.getsize(path_to_file) > 0:
                        try:
                            file_contents = open(path_to_file, "r")
                        except:return 1
                    else:return 1
                search_txt = 'process started with pid'

                for each_line in file_contents:
                    if search_txt in each_line:
                        Node_Exe = each_line.split()[-1].strip().split("[")[-1].split("]")[0]
                        self.killCMexe(int(Node_Exe))
                file_contents.close()
            else:return 0

        def startCemNodes(self,currTestRunName,CEM_process,cem_Not_created=0):
            """
            Function to Start CEM nodes
            :param currTestRunName: TestRunName to Start the Simulation
            :return: NonecurrTestRunName
            """
            if self.sim_output == '1' and self.CEMValues == '0':
                cm_prjfolder = self.cmPrjDir.partition("/conf")[0]
                config = configparser.ConfigParser(allow_no_value=True,interpolation=configparser.ExtendedInterpolation())
                config_ros_path = os.path.join(cm_prjfolder, "scripts", "ros2.ini")
                config.read(config_ros_path)
                ros_path = config.get("ROS2 INSTALL PATH", "ros2_path")
                ros_path = os.path.join(str(ros_path), "local_setup.bat")
                cm_exe_path = self.CMprojectwin64exe.partition("\\install")[0]
                CM_batch = os.path.join(cm_exe_path, "install", "local_setup.bat")
                if not os.path.exists(os.path.join(cm_exe_path, "CEM_node_log")):
                    os.makedirs(os.path.join(cm_exe_path, "CEM_node_log"))
                path_to_file = os.path.join(cm_exe_path, "CEM_node_log", currTestRunName + ".txt")
                cem_nodes = 'ros2 launch cem200 cem.launch.py > {0}'.format(path_to_file)

                def start_ROS():
                    command = r'title "{3}" & set ROS_DOMAIN_ID={0} & {1} & {2} & {4}'.format(self.id, str(ros_path), str(CM_batch), currTestRunName,cem_nodes)
                    CEM_process.stdin.write(command + '\n')
                    CEM_process.stdin.flush()
                threadHandling = threading.Thread(target=start_ROS)
                threadHandling.start()
                time.sleep(60)

                if cem_Not_created == 0:
                    self.restart_CEMnodes(currTestRunName,CEM_process)

        def startCemNodes_next(self, currTestRunName,Process_obj):
            """
            Function to launch CEM nodes and stores the CEM nodes information in the respective testrun text files to read the procees ID of each CEM nodes
            :param currTestRunName: To read the CEM nodes process ID's
            :param Process_obj: SubProcess Object to launch CEM nodes in the same cmd prompt
            :return: CEM nodes - (1 - Not Created CEM nodes or 0 - Created CEM nodes)
            """
            cem_node = 0
            cm_exe_path = self.CMprojectwin64exe.partition("\\install")[0]
            if not os.path.exists(os.path.join(cm_exe_path, "CEM_node_log")):
                os.makedirs(os.path.join(cm_exe_path, "CEM_node_log"))
            path_to_file = os.path.join(cm_exe_path, "CEM_node_log", currTestRunName + ".txt")
            cem_nodes = 'ros2 launch cem200 cem.launch.py > {0}'.format(path_to_file)
            Process_obj.stdin.write(cem_nodes + '\n')
            Process_obj.stdin.flush()

            time.sleep(15)
            if not os.path.exists(path_to_file):
                cem_node= 1
                return cem_node

        def restart_CEMnodes(self, currTestRunName,CEM_process):
            """
            Function to re-start CEM nodes based on the Number of CM instances
            :param currTestRunName: TestRunName to Start the Simulation
            :return: None
            """
            NumberofInstance = self.instanceCount
            timeout = time.time() + 60 * 1.5  # 90 sec from now
            while True:
                count = 0
                for each_process in psutil.process_iter():
                    if each_process.name() == "tpf2_node.exe" or each_process.name() == "cem_node.exe": # cem_node.exe for HONDA and tpf2_node.exe for all other Prj
                        count = count + 1
                        if count == NumberofInstance or time.time() > timeout:
                            break
                        else:
                            continue
                if count == NumberofInstance or time.time() > timeout:
                    break
            if time.time() > timeout and count < NumberofInstance:
                cCMEvalTestRunner.KillCEMNodes(self.CMprojectwin64exe)
                self.startCemNodes(currTestRunName,CEM_process)

        def extract_birdeye_image(self, currTestRunName, currErgFilePath, Check_flag):
            if '_ae' in currTestRunName and Check_flag != 3:
                # For EBA
                self._testRunner._imageAcumulatorLock.acquire()
                self._iLockedAccumulatorLock = True
                activationDetectedAE = self._cmCtrl.export_images(self._testRunner._cmPrjDir, currTestRunName,
                                                                  currErgFilePath, self._testRunner._signal_mapping,
                                                                  frame_rate=25)  # export images for birds eye view
                if activationDetectedAE:
                    self._cmCtrl.birdeye_image_copy(self._testRunner._cmPrjDir, currTestRunName)
                self._testRunner._imageAcumulatorLock.release()
                self._iLockedAccumulatorLock = False
            elif Check_flag == 3:
                #For ACC
                self._testRunner._imageAcumulatorLock.acquire()
                self._iLockedAccumulatorLock = True

                ACC_ON = self._cmCtrl.export_images(self._testRunner._cmPrjDir, currTestRunName,currErgFilePath,"")  # export images for birds eye view

                if ACC_ON:

                    self._cmCtrl.birdeye_image_copy(self._testRunner._cmPrjDir, currTestRunName)
                self._testRunner._imageAcumulatorLock.release()
                self._iLockedAccumulatorLock = False

        def Project_Config_read(self):
            if os.path.exists(os.path.join('./input', 'conf', 'Project_config.ini')):
                Project_config = configparser.ConfigParser(allow_no_value=True)
                Project_config_file = os.path.join('./input', 'conf', 'Project_config.ini')
                Project_config.read(Project_config_file)
                self.launch_IPG = str(Project_config.get('IPGMovie_Settings', 'launch_movie'))
                self.Before_Launch_Movie = int(Project_config.get('Sleep', 'Before_Launch_Movie'))
                self.After_Launch_Movie = int(Project_config.get('Sleep', 'After_Launch_Movie'))
                self.After_Start_Connect = int(Project_config.get('Sleep', 'After_Start_Connect'))
                self.After_Load_TestRun = int(Project_config.get('Sleep', 'After_Load_TestRun'))
                self.After_SetResultFileName = int(Project_config.get('Sleep', 'After_SetResultFileName'))
                self.After_Simulate = int(Project_config.get('Sleep', 'After_Simulate'))
            else:
                self.launch_IPG = "DEFAULT"
                self.Before_Launch_Movie = 0
                self.After_Launch_Movie = 0
                self.After_Start_Connect = 0
                self.After_Load_TestRun = 0
                self.After_SetResultFileName = 0
                self.After_Simulate = 0

        def extract_pid(self,ExeInfo):
            lineNum = 0
            pid = ""
            Data =ExeInfo.split()
            for line in Data:
                if lineNum ==1:
                    pid = line
                    break
                if "AppPid" == line.strip():
                    lineNum = lineNum + 1
            return pid

        def handle_connectivity(self, ping_status):
            License_file = os.path.join(re.search('(?i).*IPG', self.CMprojectexe).group(), 'etc', 'Licenses')
            License_server = cCMEvalTestRunner.get_License_server(License_file)
            flag = False
            for nRetry in range(self._nRetries):
                if not ping_status:
                    # cCMEvalTestRunner._killCarMakerProcesses()
                    try:
                        self.worker.signals.logsignal.emit("CarMaker License not available. Reconnect to the network.")
                    except:
                        pass  # For command line exe
                    if not flag:
                        flag = True
                        try:
                            self.worker.signals.popwindow.emit(
                                "Connection lost.\nReconnect to the network,\nClick on 'Retry' on the CarMaker Lisence error window\nand then click 'OK'.")
                            # added sleep for user to select Retry on CarMaker lisence error window.
                            time.sleep(10)
                        except:
                            pass  # For command line exe

                    # Wait until user clicks OK on the message box.
                    event.wait()
                    time.sleep(3)
                    flag = False
                    logger.info(
                        "Checking for network availability. Simulation will continue only if connected to the network.")
                    ping_status = cCMEvalTestRunner.ping_check(License_server)
                else:
                    return
            raise Exception

    class TestSeriesRunningThread (threading.Thread):
        """
        Class for a worker thread for Carmaker simulations using the office license and HPC mode. This class inherits threading.Thread.
        This class is called and it's methods are handled internally by the cCMEvalTestRunner class.

        :param testRunner: Instance of cCMEvalTestRunner class
        :param testRuns: Option 1: List of objects of class cmTestRun (defined in pyBase/pycmEval/cm_testrun.py). Option 2: Name of a Carmaker ts file with defined testruns \
        placed in the TestRun folder.
        :param cmPrjDir: Absolute path Carmaker project directory
        :param outputFolderName: Name of the output folder in that the result files are stored (will be a subfolder in SimOutput directory in the Carmaker project)
        :param carMakerPort: Port of the Carmaker application.
        :param nRetries: Number of retries if something goes wrong while simulation, e.g. temporary loss of network connection and so no license checkout possible or \
        Carmaker does not react in configured timeouts due to high work load on the machine.
        :param retryTimeSeconds: Time to wait between retries if something goes wrong in the simulation.
        :param executable: Absolute path to Carmaker binary executable. If no is given, the configured path from the GUI file in the Carmaker project will be used.
        :param cmGuiPath: Absolute path to Carmaker GUI. If no is given, the latest Carmaker version in the standard installation path of Carmaker is used.
        """

        def __init__(self, testRunner, testRuns, cmPrjDir, outputFolderName, carMakerPort, nRetries = 1, retryTimeSeconds = 2, executable = None, cmGuiPath = None):
            threading.Thread.__init__(self)
            self._testRunner = testRunner
            self._testRuns = testRuns
            self._cmPrjDir = cmPrjDir
            self._outputFolderName = time.strftime("%Y%m%d_%H%M%S") + "_" + outputFolderName
            self._cmCtrl       = None
            self._carMakerPort = carMakerPort
            self._nRetries = nRetries
            self._retryTimeSeconds = retryTimeSeconds
            self._executable = executable
            self._cmGuiPath = cmGuiPath
            self._exception = None

        def run(self):
            """
            Method to start the worker thread.
            """

            try:
                if self._cmCtrl is None:
                    for nRetry in range(self._nRetries):
                        self._cmCtrl = CMRC(startCM=True, safeMode=True, port = self._carMakerPort, cmGuiPath = self._cmGuiPath)
                        if self._cmCtrl._error:
                                logger.info("Something went wrong with the simulation. Restarting in %ds...", self._retryTimeSeconds)
                                cCMEvalTestRunner._killCarMakerProcesses()
                                time.sleep(self._retryTimeSeconds)
                        else:
                            break
                    self._cmCtrl.SetProjectDir(self._cmPrjDir)
                    self._cmCtrl.SetSaveMode("all")
                    self._cmCtrl.SetSimSpeed("max")
                    self._cmCtrl.SetResultFileName("SimOutput/{}/%f".format(self._outputFolderName))

                    if self._executable is not None:
                        self._cmCtrl.AppExecute(self._executable.replace("\\", "/"))
                if self._executable is not None:
                    self._cmCtrl.AppExecute(self._executable.replace("\\", "/"))
                logger.info("####### Loading Testseries {} #######".format(self._testRuns))
                self._cmCtrl.Exec("TestMgr load {}".format(self._testRuns))
                logger.info("####### Clearing TestMgr results #######".format(self._testRuns))
                self._cmCtrl.Exec("TestMgr clearresults")
                logger.info("####### Setting Execution Mode HPC_Light #######")
                self._cmCtrl.Exec("::TestMgr::SetExecMode HPC_Light")
                logger.info("####### Starting Testseries #######")
                self._cmCtrl.Exec("TestMgr start")
                logger.info("####### Wait for CM to write ts file #######")
                time.sleep(3) # Time to write results in seconds
                logger.info("####### Saving Testseries #######")
                self._cmCtrl.Exec("TestMgr save {}".format(self._testRuns))

                self._testRunner._ergFilePaths = []
                with open(os.path.join(self._cmPrjDir, "Data", "TestRun", self._testRuns), "r") as f:
                    lines = f.readlines()
                for currLine in lines:
                    if currLine.startswith("Step.") and "ResFiles" in currLine:
                        self._testRunner._ergFilePaths.append(os.path.join(self._cmPrjDir, currLine.split("=")[1].replace(" ", "").replace("\n", "")))

                # Close CarMaker
                if self._cmCtrl is not None:
                    self._cmCtrl.Close()
                    self._cmCtrl = None
            except Exception as e:
                self._exception = e

        def get_exception(self):
            return self._exception
