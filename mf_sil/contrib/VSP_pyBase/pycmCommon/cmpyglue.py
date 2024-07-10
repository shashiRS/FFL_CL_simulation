# ====================================================================
# System Imports
# ====================================================================
import sys
import platform
import os
import re
import getpass
import telnetlib
import logging

logger = logging.getLogger("pyBase")
import time
import subprocess
import psutil
import numpy as np

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "pycmEval"))
try:
    from cm_eval_erg_calculator import cErgCalculator
except:
    from pyBase.pycmEval.cm_eval_erg_calculator import cErgCalculator
import shutil
from datetime import datetime

import configparser
import time
import math
from collections import Counter
from distutils.version import LooseVersion

# ====================================================================
# Platform Variables
# ====================================================================
try:
    from utils.erg_parser import image_timestamp
    from utils.erg_parser import SignalCollector
except:
    try:
        from cmEval.src.FSF_Generic.common import image_timestamp
    except:
        pass

if platform.system() == 'Linux':
    IS_LINUX_SYSTEM = True
    import signal
else:
    IS_LINUX_SYSTEM = False

# ====================================================================
# ====================================================================
# Default Parameters
# ====================================================================
if IS_LINUX_SYSTEM:
    CARMAKER_GUI_PROCESS_NAME = "HIL.exe"  # name of process HIL.exe on linux as well
    CARMAKER_DEFAULT_PATHS = [r"/opt/ipg/carmaker"]
else:
    CARMAKER_GUI_PROCESS_NAME = "HIL.exe"
    CARMAKER_DEFAULT_PATHS = [r"C:\IPG\hil", \
                              r"C:\IPG\carmaker", \
                              r"C:\LegacyApp\IPG\hil", \
                              r"C:\LegacyApp\IPG\carmaker", \
                              r"C:\cip_tools\carmaker\IPG\carmaker"]
CARMAKER_SIMSTATUS_MSGS = {"Preprocessing": "O-1", "Idle": "O-2", "Postprocessing": "O-3", "Paused": "O-8"}
CARMAKER_SAVE_MODES = {"all": "save_all", "collectonly": "collect_only"}
CARMAKER_SIM_SPEED_MODES = {"real": "1", "double": "2", "max": "999999", "paused": "-1"}

DEFAULT_CM_PORT = 2327
RT_LICENSE_NUM = 661150

TELNET_CONNECTION_TIMEOUT = 90.0
TELNET_WAIT_FOR_RESPONSE_TIME_S = 1.0
TELNET_SHORT_WAIT_FOR_RESPONSE_TIME_S = 0.5
TELNET_LONG_WAIT_FOR_RESPONSE_TIME_S = 5.0

SIMTIMEMAX_S = 540.0
WAIT_FOR_STATUS_MAX_TIME_DEF = 30.0

config = configparser.ConfigParser(allow_no_value=True, interpolation=configparser.ExtendedInterpolation())


# ====================================================================
# Helper Classes
# ====================================================================
class QuantityResult:
    Valid = False
    Value = 0.0


# ====================================================================
# Helper Functions
# ====================================================================
def extractStringBetween(s, first, last):
    try:
        start = s.index(first) + len(first)
        end = s.index(last, start)
        return s[start:end]
    except ValueError:
        return ""


# ====================================================================
# Main Class
# ====================================================================
class CMRC_HeadlessInstance:
    """
    Class to remote-control Carmaker in headless mode. Optionally it allows to use a core license for the simulation.

    :param prjPath: Absolute path to the Carmaker project
    :param cmExePath: Absolute path to the compiled binary application of Carmaker (carmaker.win64.exe)
    :param outputDir: Path to output directory relative to SimOutput folder in Carmaker project
    :param useCoreLicense: If True, it will use core licenses to simulate. If False, it will use usual office licenses.
    """

    def __init__(self, prjPath, cmExePath, outputDir, useCoreLicense=False):
        self._prjPath = prjPath
        self._cmExePath = cmExePath
        self._outputDir = outputDir
        self._useCoreLicense = useCoreLicense

    def execTestrun(self, testrunName, maxExecTimeSeconds=None):
        """
        Execute a testrun

        :param testrunName: Name of the testrun. The testrun must be located in the TestRun/{OutputDir} folder. OutputDir is the path given to the outputDir parameter.
        :param maxExecTimeSeconds: Maximum (wall clock) time for a testrun to finish until the testrun is aborted and skipped (seconds)

        :return: Two values. First: True if the simulation was successful. False if not. Second: Reason for unsuccessful simulation, e.g. 'TIMEOUT', else None.
        """
        success = True
        reason = None
        if self._useCoreLicense:
            cmd = '{} {} -appdir "{}" -simoutput "{}" -dstore -license {}'.format(
                self._cmExePath,
                testrunName,
                self._prjPath,
                os.path.normpath(os.path.join('SimOutput', self._outputDir)),
                RT_LICENSE_NUM)
        else:
            cmd = '{} {} -appdir "{}" -simoutput "{}" -dstore'.format(
                self._cmExePath,
                testrunName,
                self._prjPath,
                os.path.normpath(os.path.join('SimOutput', self._outputDir)))
        # The shell=True parameter is only required in Linux systems but it prevents killing the process in Windows.
        # In Linux, killing a shell=True process is possible by using a process group defined by the
        # preexec_fn=os.setsid parameter, see https://stackoverflow.com/a/4791612
        process = subprocess.Popen(cmd,
                                   stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE,
                                   shell=IS_LINUX_SYSTEM,
                                   preexec_fn=(os.setsid if IS_LINUX_SYSTEM else None),
                                   cwd=self._prjPath)

        try:
            timeout_s = (SIMTIMEMAX_S if maxExecTimeSeconds is None else maxExecTimeSeconds)
            now = datetime.now()
            current_time = now.strftime("%H:%M:%S")
            logger.info("Current Time = {}".format(current_time))
            outs, errs = process.communicate(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            if IS_LINUX_SYSTEM:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)  # Terminate the process group
            else:
                process.kill()
            outs, errs = process.communicate()
            logger.error("CMRC_HeadlessInstance: Simulation Time longer than expected! See SIMTIMEMAX_S @ cmpyglue.py.")
            success = False
            reason = 'TIMEOUT'
        outs = outs.decode()
        errs = errs.decode()
        if outs != '':
            logger.info('execTestrun: {}'.format(outs))
        if errs != '':
            logger.error('execTestrun: {}'.format(errs))
        return success, reason


class CMRC:
    """"
    Class to remote-control Carmaker via telnet connection.

    :param host: IP address of machine where Carmaker shall be started.
    :param port: Port for CM instance.
    :param startCM: If true it starts a new instance of Carmaker. If false, it connects to an already running instance. If the connection fails it will raise an exception.
    :param cmGuiPath: Absolute path to Carmaker GUI executable (CM.exe) in the installation directory
    :param safeMode: If true it will output more information about an error occured in Carmaker if there is any
    """

    def __init__(self, host='localhost', port=DEFAULT_CM_PORT, startCM=False, cmGuiPath=None, safeMode=False,
                 image_extraction=False, video_extraction=False, CMprojectDir=None, CMprojectExe=None, sim_output=None,
                 worker=None, id=0, MDF=0, currTestRunName=""):

        self._tn = None
        self._safeMode = safeMode
        self._subscribedQuantities = []
        self._error = False
        self.CMprojectDir = CMprojectDir
        self.sim_output = sim_output
        self.worker = worker
        self.id = id
        self.CMprojectExe = CMprojectExe
        self.start_cm = startCM
        self.cm_path = cmGuiPath

        if image_extraction or video_extraction:
            # Check whether CarMaker GUI is running
            if self._CMGUIRunning():
                self.CM_exist(host, port)
            else:
                self._CMGUIStart(startCM, cmGuiPath, port, host)
        else:
            # Check whether CarMaker GUI is running
            if self._CMGUIRunning():
                self.CM_exist(host, port)
            else:
                if startCM:
                    self._StartCMGui(cmGuiPath, port, currTestRunName)
                    # wait for carmaker connection
                    connectionOk = False
                    time_start = time.time()
                    while not connectionOk:
                        connectionOk = self._TryConnectToCMGui(host, port)
                        running_time_s = time.time() - time_start
                        if (running_time_s > TELNET_CONNECTION_TIMEOUT):
                            logger.info("CarMaker Telnet Connection Timeout.")
                            self._error = True
                            break

                        time.sleep(0.1)
                else:
                    raise Exception('CMRC: No CarMaker instance runnning')

    def CM_exist(self, host, port):
        # Check whether CarMaker GUI is running
        logger.warn("CMRC: Running CarMaker instance detected")
        connectionOk = self._TryConnectToCMGui(host, port)
        if not connectionOk:
            raise Exception('CMRC: Could not connect to running CarMaker instance at ' + str(host) + ":" + str(port))
            # CarMaker has to be started with parameter "cmdport" to enable communication over TCP

    def _CMGUIStart(self, startCM, cmGuiPath, port, host):
        if startCM:
            self._StartCMGui(cmGuiPath, port)
            # wait for carmaker connection
            connectionOk = False
            time_start = time.time()
            while not connectionOk:
                connectionOk = self._TryConnectToCMGui(host, port)
                running_time_s = time_start - time.time()
                if (running_time_s > TELNET_CONNECTION_TIMEOUT):
                    raise Exception(
                        'CMRC: Could not connect to running CarMaker instance at ' + str(host) + ":" + str(port))
                time.sleep(0.1)
        else:
            raise Exception('CMRC: No CarMaker instance runnning')

    def _CMGUIRunning(self):
        """
        Detect if a CM GUI process is running

        :return: True if the GUI process is running, false if not.
        """
        if IS_LINUX_SYSTEM is True:
            if CARMAKER_GUI_PROCESS_NAME.encode() in \
                    subprocess.Popen(['ps', 'aux'], stdout=subprocess.PIPE).communicate()[0]:
                return True
            else:
                return False
        else:
            currUser = getpass.getuser()  # username needed for tasklist filter
            if CARMAKER_GUI_PROCESS_NAME in subprocess.Popen('tasklist /fi "USERNAME eq {0}"'.format(currUser),
                                                             stdout=subprocess.PIPE).communicate()[0].decode('ascii'):
                return True
            else:
                return False

    def _TryConnectToCMGui(self, host, port):
        """
        Connect to CM GUI.

        :param host: Host address to run the Carmaker application. Default: localhost
        :param port: Port to run the Carmaker application. Default: 2327

        :return: True if telnet connection was successfully established, false if not.
        """
        try:
            self._tn = telnetlib.Telnet(host, port, TELNET_CONNECTION_TIMEOUT)
            return True
        except:
            return False

    def relaunch_ecal_nodes(self):
        """
            This method to kill all the ecal nodes and relaunch ecal and carmaker nodes
        """
        process_list = ["HIL.exe", "ecal_sys.exe"]
        for proc in psutil.process_iter():
            procDict = proc.as_dict(attrs=["name", "exe", "pid"])
            if procDict["name"] in process_list:
                psutil.Process(procDict["pid"]).kill()
        time.sleep(2)
        self._CMGUIStart(self.start_cm, self.cm_path, DEFAULT_CM_PORT, 'localhost')

    def start_ecal_nodes(self):
        cwdir = os.getcwd()
        project_dir = str(os.path.split(self.CMprojectDir)[0])
        os.chdir(project_dir)
        sys.path.insert(0, f"{project_dir}\ecal")
        p = subprocess.Popen(fr"{project_dir}\start_sf_sim.bat")
        time.sleep(5)
        p.terminate()
        os.chdir(cwdir)

    def _StartCMGui(self, cmGuiPath, port, currTestRunName=None):
        """
        Start the Carmaker GUI. Raise an exception if the GUI could not be found.

        :param cmGuiPath: Absolute path to the Carmaker GUI
        :param port: Port to run the Carmaker application. Default: 2327
        """

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

        selectedCMGuiPath = None
        if cmGuiPath is not None:
            if os.path.exists(cmGuiPath):
                selectedCMGuiPath = cmGuiPath
        else:
            # Search for CM GUI exe in default locations
            for currCarMakerDefaultPath in CARMAKER_DEFAULT_PATHS:
                if os.path.exists(currCarMakerDefaultPath):
                    if IS_LINUX_SYSTEM:
                        latest_version = find_latest_version(currCarMakerDefaultPath, "linux64-")
                        if latest_version:
                            currCMGuiPath = os.path.join(currCarMakerDefaultPath, latest_version,
                                                         r"bin/CM")  # take latest version of CarMaker default install directory
                    else:
                        latest_version = find_latest_version(currCarMakerDefaultPath, "win64-")
                        if latest_version:
                            currCMGuiPath = os.path.join(currCarMakerDefaultPath, latest_version,
                                                         r"bin\CM.exe")  # take latest version of CarMaker default install directory
                    if os.path.exists(currCMGuiPath):
                        selectedCMGuiPath = currCMGuiPath

        if selectedCMGuiPath is not None:
            if self.sim_output == '1':
                cm_prjfolder = self.CMprojectDir.partition("/conf")[0]
                config_ros_path = os.path.join(cm_prjfolder, "scripts", "ros2.ini")
                config.read(config_ros_path)
                ros_path = config.get("ROS2 INSTALL PATH", "ros2_path")
                ros_path = os.path.join(str(ros_path), "local_setup.bat")
                cm_exe_path = self.CMprojectExe.partition("\\install")[0]
                CM_batch = os.path.join(cm_exe_path, "install", "local_setup.bat")
                command = r'title "{5}" & set ROS_DOMAIN_ID={4} & {0} & {1} & {2} -cmdport {3}'.format(str(ros_path),
                                                                                                       str(CM_batch),
                                                                                                       str(cmGuiPath),
                                                                                                       port, self.id,
                                                                                                       currTestRunName)
                os.system(command)
            # Todo eCAL setup

            elif self.sim_output == '2':
                logger.info("***Launching eCAL sys*****")
                self.start_ecal_nodes()
            else:

                self.ipg_movie_path = os.path.join(os.path.abspath(os.path.join(selectedCMGuiPath, "../..")), "GUI",
                                                   "Movie.exe")
                if IS_LINUX_SYSTEM:
                    subprocess.Popen('{0} -cmdport {1}'.format(selectedCMGuiPath, port), shell=True)
                # logger.info("CMRC: Starting CarMaker GUI: " + selectedCMGuiPath)
                else:

                    subprocess.Popen('{0} -cmdport {1}'.format(selectedCMGuiPath, port))
                    # subprocess.Popen('{0} -csensor -cmdport {1}'.format(selectedCMGuiPath, port))
        else:
            raise Exception('CMRC: Could not find any CarMaker GUI Executable')

    def WaitForStatus(self, status, maxWaitTime=WAIT_FOR_STATUS_MAX_TIME_DEF):
        """
        Causes the script to wait until a given status is reveived. Raise an exception if status is not received within maximum wait time.

        :param status: Status on which shall be waited
        :param maxWaitTime: Maximum time to wait for the status until an execption is thrown
        """
        # logger.info("CMRC: Waiting for Status " + status)
        time_start = time.time()
        statusMsgToWaitFor = CARMAKER_SIMSTATUS_MSGS[status]
        while True:

            statusMsg = self.Exec('SimStatus')
            if statusMsg == statusMsgToWaitFor:
                # logger.info("CMRC: Status " + status + " detected")
                return

            time_waiting = time.time() - time_start
            if time_waiting > maxWaitTime:
                logger.error("CMRC: Exception!!!")
                raise StandardError('CMRC: CarMaker Status was not received within maximum wait time')
            # logger.info("CMRC: SimStatus requested, received: " + statusMsg)
            time.sleep(0.1)

    def Connect(self):
        """
        Connect function of Carmaker
        """
        logger.info('CMRC: CarMaker Connect')
        self.Exec('Appl::Connect')
        return True

    def CM_DisConnect(self):
        # logger.info('CMRC: CarMaker DisConnect')
        self.Exec('Appl::Disconnect')
        # return True

    def Connected(self):
        # TODO: Check whether CarMaker GUI is connected to CarMaker Core
        pass

    def Disconnect(self):
        """
        Disconnect function of Carmaker
        """
        # logger.info('CMRC: CarMaker disconnecting Telnet Connection')
        self._tn.close()

    def StartAndConnect(self):
        """
        StartAndConnect function of Carmaker
        """
        # logger.info('CMRC: CarMaker Start and Connect')
        self.Exec('Appl::Start')  # Equates to CarMakers "Start & Connect"

    def GetCMExePID(self):
        """
        Get all the info abou the Carmaker exe
        """
        # logger.info('CMRC: CarMaker info')
        CM_exeInfo = self.Exec('array get HIL')  # Equates to CarMakers "array get HIL"
        return CM_exeInfo

    def AppExecute(self, app_path):
        """
        start application with a give pre-compiled binary path

        :param app_path: Full path to compiled binary application of carmaker
        """

        logger.info('CMRC: Starting application in path {}'.format(str(app_path)))
        self.Exec('Application exe {}'.format(str(app_path)))

    def GetSimStatus(self):
        """
        Start application with a give pre-compiled binary path

        :return: Current simulation status (str)
        """
        simStatusResponse = self.Exec('SimStatus')
        for simStatusName, simStatusMsg in CARMAKER_SIMSTATUS_MSGS.items():
            if simStatusMsg == simStatusResponse:
                return simStatusName
        if len(simStatusResponse) > 0:
            if simStatusResponse[0] == 'O':
                return "running"
            else:
                return "error"
        return "error"

    def SetProjectDir(self, projectDir):
        """
        Set the Carmaker project directory

        :param projectDir: Absolute path to Carmaker project directory to be set
        """
        resolvedPath = os.path.realpath(projectDir).replace(os.sep, '/')
        resolvedPath = resolvedPath.replace(' ', r'\ ')
        self.projectDir = projectDir
        # logger.info('CMRC: Selecting Project ' + resolvedPath)
        self.Exec('Project::Select {0}'.format(resolvedPath))

    def SetResultFileName(self, relResultFilePath):
        """
        Set the result file name for a testrun

        :param relResultFilePath: File name for the erg file of the testrun
        """
        # adjustedPath = "%o/" + relResultFilePath.replace('\\','/')
        adjustedPath = relResultFilePath.replace('\\', '/')
        adjustedPath = adjustedPath.replace(' ', '\ ')
        # logger.info('CMRC: Setting Result Filename ' + adjustedPath)
        self.Exec('SetResultFName {0}'.format(adjustedPath))

    def SetOutputSamplePeriod(self, samplePeriod):
        """
        Set the samplePeriod for data in the result (erg) file

        :param samplePeriod: Sample period (in seconds)
        """
        # logger.info('CMRC: Setting Sample Period to {0}'.format(samplePeriod))
        self.Exec('IFileModify SimParameters DStore.dtFile {0}'.format(samplePeriod))

    def SetDVARelease(self, value):
        """
        Set DVA release mode

        :param value: Value for DVA release mode
        """
        # logger.info('CMRC: Setting DVA Release Mode to {0}'.format(value))
        self.Exec('IFileModify SimParameters TestRunEnd.DVA_ReleaseAll {0}'.format(value))

    def SetSaveMode(self, safeMode):
        """
        Set Carmaker save mode.
        Available modes: "all", "collectonly"

        :param safeMode: Safe mode
        """
        safeMode = safeMode.lower()
        if safeMode in CARMAKER_SAVE_MODES:
            # logger.info('CMRC: Setting Save Mode to ' + safeMode)
            self.Exec('SaveMode {0}'.format(CARMAKER_SAVE_MODES[safeMode]))
        else:
            raise Exception('CMRC: CarMaker SafeMode ' + safeMode + " not supported")

    def SetSimSpeed(self, simSpeed):
        """
        Set simulation speed.
        Available modes: "real", "double", "max", "paused"

        :param simSpeed: Simulation speed
        """
        simSpeed = simSpeed.lower()
        if simSpeed in CARMAKER_SIM_SPEED_MODES:
            # logger.info('CMRC: Setting Sim Speed to ' + simSpeed)
            self.Exec('SetSimTimeAcc {0}'.format(CARMAKER_SIM_SPEED_MODES[simSpeed]))
        else:
            raise Exception('CMRC: CarMaker SimSpeed ' + simSpeed + " not supported")

    def SetOutputQuantitiesFormatMDF(self):
        """
        Change the output file format to MDF.
        """
        logger.info('CMRC: Setting Output Quantities to MDF4.1')
        self.Exec('IFileModify SimParameters DStore.OutputQuantities {}'.format("OutputQuantities_FCT_MDF"))

    def SetOutputQuantitiesFormatERG(self, value):
        """
        Change the output file format to ERG.
        """
        logger.info('CMRC: Setting Output Quantities to ERG')
        self.Exec('IFileModify SimParameters DStore.OutputQuantities {}'.format(value))

    def LoadTestRun(self, pathToConfig):
        """
        Load a testrun.

        :param pathToConfig: Path to testrun file relative to TestRun folder in the Carmaker project directory.
        """
        adjustedPath = pathToConfig.replace(os.sep, '/')
        # logger.info("CMRC: Loading TestRun " + adjustedPath)
        self.Exec('LoadTestRun {0} 1\n'.format(adjustedPath), 1.0)
        self.WaitForStatus("Idle")

    def Exec(self, cmd, fireAndForget=False, waitForResponse=0.0):
        """
        Execute a command in Carmaker.

        :param cmd: Command string
        :param fireAndForget: If true, the method does not wait for a response from Carmaker. If false, it waits for a response.
        :param waitForResponse: Waiting time after the command is sent, until the response is checked
        """
        try:
            # Clear Receive Buffer
            self._tn.read_very_eager()
        except:
            if cmd.find("Project::Select") != -1:
                logger.info('########### Carmaker License Issue #################')
            sys.exit()

        # Run given Command
        response = ''
        if True == fireAndForget:
            self._tn.write((cmd + '\n').encode('ascii'))
        else:
            self._tn.write((cmd + '\nexpr {"*END*"}\n').encode('ascii'))
            if waitForResponse > 0.0:
                time.sleep(waitForResponse)
            # response = self._tn.read_until("O*END*".encode('ascii'), timeout=TELNET_LONG_WAIT_FOR_RESPONSE_TIME_S)
            response = \
            (self._tn.read_until("O*END*".encode('ascii'), timeout=TELNET_LONG_WAIT_FOR_RESPONSE_TIME_S)).decode(
                'ascii').split("\r\n\r\n")[0].strip()
            if self._safeMode and len(response) > 0:
                if response[0] == 'E':
                    errorMsg = response[1:]
                    raise Exception('CMRC: CarMaker Error: ' + errorMsg)

        return response

    def Simulate(self, maxTimeInSimulationSeconds=None, currTestRunName="", maxExecTimeSeconds=None):
        """
        Starts the execution of the simulation.
        If a testrun is paused while its execution, all timeouts are deactivated an have no effect. They will be considered again for
        the next testrun.

        :param maxTimeInSimulationSeconds: Maximum simulation time for a testrun to finish until the testrun is aborted and skipped (seconds)
        :param maxExecTimeSeconds: Maximum (wall clock) time for a testrun to finish until the testrun is aborted and skipped (seconds)
        """

        time_start = time.time()

        # logger.info("CMRC: Starting Simulation")
        self.SubscribeToQuantities(['Time'])
        self.Exec('StartSim', False, 1.0)

        carTime_list = []
        testrun_was_paused = False

        while True:
            # Deactivate bring-to-front property of log window
            self.Exec('wm overrideredirect .sessionlog 1')

            simStatus = self.Exec('SimInfo endstatus')
            if simStatus == "Ocompleted":
                # logger.info("CMRC: Simulation Completion detected")
                return
            elif simStatus == "Oaborted":
                # logger.info("CMRC: Simulation Abortion detected")
                self.Exec("Application stop")
                return
            elif simStatus == 'Ofailed':
                self.Exec("Application stop")
                raise Exception("CMRC: Simulation Failed!")

            time_running = time.time() - time_start
            if time_running > (
            SIMTIMEMAX_S if maxExecTimeSeconds == None else maxExecTimeSeconds) and not testrun_was_paused:
                logger.error("CMRC: Simulation Time longer than expected! See SIMTIMEMAX_S @ cmpyglue.py.")
                raise Exception('Simulation Time longer than expected! See SIMTIMEMAX_S!')
            carTime = self.RetrieveQuantity('Time')  # Retrieve time
            carTime_list.append(carTime)

            max_time = Counter(carTime_list)
            if self.GetSimStatus() != "Paused" and not testrun_was_paused:  # if status is or was paused timeouts shall not be considered as manual control is assumed
                if self.GetSimStatus() == "Preprocessing":
                    if max(max_time.values()) > 500:
                        logger.error("CMRC: Simulation Error :: {0}".format(currTestRunName))
                        self.Exec("Application stop")
                        return 1  # if carmaker in preparation state

                if self.GetSimStatus() == "running":  # If Carmaker is running and crashes in between
                    if max(max_time.values()) > 150:
                        logger.error("CMRC: Simulation Error :: {0}".format(currTestRunName))
                        self.Exec("Application stop")
                        return 1  # if carmaker crashes
                        # raise StandardError('Simulation Time longer than expected! See SIMTIMEMAX_S!')

                if (maxTimeInSimulationSeconds is not None) and (maxTimeInSimulationSeconds > 0):
                    try:
                        timeInSimulation = self.RetrieveQuantity('Time')
                    except Exception:
                        timeInSimulation = None

                    if (timeInSimulation is not None) and (timeInSimulation > maxTimeInSimulationSeconds):
                        logger.warning("CMRC: Time in simulation longer than " + str(
                            maxTimeInSimulationSeconds) + " seconds. Stopping simulation.")
                        self.Exec('StopSim', False, 1.0)
                        self.Exec("Application stop")
                        break
            else:  # simulation is paused
                testrun_was_paused = True

            time.sleep(5)

    def StopSim(self):
        """
        Stop the simulation and exit the application.

        """
        self.Exec('StopSim', False, 1.0)
        self.Exec("Application stop")

    def KIllCM(self):
        self.Exec("Application stop")
        time.sleep(0.5)

    def SubscribeToQuantities(self, quantityList, ignoreErrors=False):
        """
        Subscribe to quanitites to export them to the result file

        :param quantityList: List of strings of quantities
        """

        # Subscribe to quantities
        self._subscribedQuantities = []
        self.Exec("QuantSubscribe {}")
        subscriptionCommand = "QuantSubscribe {" + ''.join(i + " " for i in quantityList) + "}"
        self.Exec(subscriptionCommand)
        # Check each of the requested quantities for availability
        for currQuantity in quantityList:
            try:
                self.RetrieveQuantity(currQuantity)
                self._subscribedQuantities.append(currQuantity)
            except:
                raise Exception("Could not subscribe to Quantity: {0}".format(currQuantity))

    def RetrieveQuantity(self, quantityName):
        """
        Retrieve a single quanitity from Carmaker

        :param quantityName: Name of the quantity to retrieve
        """

        result = None
        quantityRequestCommand = 'expr {"%BEGIN%' + quantityName + ':$Qu(' + quantityName + ')%END%"}\n'
        self._tn.write(quantityRequestCommand.encode('ascii'))
        response = self._tn.read_until("%END%".encode('ascii'), 1).decode('ascii')
        if ("%BEGIN%" in response and "%END%" in response):
            payload = extractStringBetween(response, "%BEGIN%", "%END%")
            if len(payload) > 0:
                quantityStringSplitted = payload.split(":")
                result = float(quantityStringSplitted[1])
            else:
                raise Exception("Error while receiving quantity")
        else:
            raise Exception("Error while receiving quantity")
        return result

    def RetrieveQuantities(self, ignoreErrors=False):
        """
        Retrieve all quanitites from Carmaker

        :param ignoreErrors: If true, errors while retrieving are ignored. If true, in case of an error an exeption is thrown
        """

        resultDict = None
        quantityRequestCommand = 'expr {"%BEGIN%' + ';'.join(
            q + ":$Qu(" + q + ")" for q in self._subscribedQuantities) + '%END%"}'
        self._tn.write((quantityRequestCommand + '\n').encode('ascii'))
        response = self._tn.read_until("%END%".encode('ascii'), 1).decode('ascii')
        if not ("%BEGIN%" in response and "%END%" in response):
            if ignoreErrors:
                return resultDict
            else:
                raise Exception("Error while receiving quantities: " + response)
        payload = extractStringBetween(response, "%BEGIN%", "%END%")
        if len(payload) > 0:
            resultDict = {}
            quantityResponseList = payload.split(";")
            for currQuantityString in quantityResponseList:
                currQuantityStringSplitted = currQuantityString.split(":")
                currKey = currQuantityStringSplitted[0]
                currValue = currQuantityStringSplitted[1]
                resultDict[currKey] = float(currValue)

        return resultDict

    def Close(self):
        """
        Close the connected Carmaker.
        """

        self.SetSaveMode('collectonly')  # Set back CM status to collect only
        self.Close_IPGMovie()
        self.Close_IPGControl()
        self.Exec('exit', fireAndForget=True)
        self.Disconnect()

    def Close_IPGMovie(self):
        """
        Close IPG Movie
        """
        try:
            self.Exec('Movie quit', fireAndForget=True)
        except:
            if IS_LINUX_SYSTEM:
                r = os.popen('ps aux').read().strip().split('\n')
                name = "Movie.exe"
                for i in range(len(r)):
                    if name in r[i]:
                        os.system("pkill %s" % (name))
            else:
                r = os.popen('tasklist /v').read().strip().split('\n')
                # r, _ = subprocess.Popen('tasklist', stdout=subprocess.PIPE, shell=IS_LINUX_SYSTEM).communicate().strip().split('\n')
                name = "Movie.exe"
                for i in range(len(r)):
                    if name in r[i]:
                        os.system("taskkill /f /im %s" % (name))

    def Close_IPGControl(self):
        """
        Close IPG control
        """
        if IS_LINUX_SYSTEM:
            r = os.popen('ps aux').read().strip().split('\n')
            name = "ipg-control.exe"
            for i in range(len(r)):
                if name in r[i]:
                    os.system("pkill %s" % (name))
        else:
            # r, _ = subprocess.Popen('tasklist', stdout=subprocess.PIPE, shell=IS_LINUX_SYSTEM).communicate().strip().split('\n')
            r = os.popen('tasklist').read().strip().split('\n')
            name = "ipg-control.exe"
            for i in range(len(r)):
                if name in r[i]:
                    os.system("taskkill /im %s" % (name))

    def Launch_IPGMovie(self, launch_IPG):
        """
        Launch IPG Movie
        """
        self.Exec("Movie start")
        self.Exec("Movie attach")
        self.Exec("Movie camera select \"{0}\" -window 0 -view 0".format(str(launch_IPG)))

    def export_images(self, cmPrjDir, testrun, currErgFilePath, signal_mapping, frame_rate=25):
        activationDetectedAE = False
        try:
            destination_path = os.path.join(cmPrjDir, "SimOutput", "Images", testrun)
            if not os.path.exists(destination_path):
                os.makedirs(destination_path)
            else:
                self.remove_image_files(destination_path)
            source_path = destination_path
            activation, start_time, end_time = image_timestamp(currErgFilePath=currErgFilePath,
                                                               signal_mapping=signal_mapping, test_run=testrun)
            if activation:
                image_ts_name = []
                image_count = (end_time - start_time) * frame_rate
                image_frequency = float(end_time - start_time) / image_count
                for i in range(0, int(round(image_count)) + 1):
                    image_ts_name.append(testrun.split("_")[0] + "_" + testrun.split("_")[-2] + "_" + str(
                        (round((start_time + i * image_frequency), 2))))

                extraction_status = self.export_image_from_IPG(testrun, source_path, start_time, end_time, frame_rate)
                if extraction_status:
                    self.move_images_to_folder(source_path, destination_path, image_ts_name)
                    self.remove_image_files(cmPrjDir)

        except Exception as e:
            print("Exception occurred during image extraction through IPG Movie:", e)
        return activation

    def export_image_from_IPG(self, testrun, path, start_time, end_time, frame_rate):
        self.Exec(
            'Movie export window ./SimOutput/Images/{}/pic.jpg 0 -start {} -end {} -framerate {}\n'.format(testrun,
                                                                                                           start_time,
                                                                                                           end_time,
                                                                                                           frame_rate))
        while len(os.listdir(path)) < (math.ceil((end_time - start_time) * frame_rate)):
            time.sleep(1)
        return True

    def birdeye_image_copy(self, cmPrjDir, testrun):
        time.sleep(2)
        dest_dir = os.path.join(cmPrjDir, "SimOutput", "Images", testrun)
        if not os.path.exists(dest_dir):
            os.makedirs(dest_dir)
        source_dir = os.path.join(cmPrjDir, "src_ext", "Tools", "VisualDebugger", "04_Build", "SnapShots", testrun)
        if not os.path.exists(source_dir):
            os.makedirs(source_dir)
        self.move_images_to_folder(source_dir, dest_dir)

    def remove_image_files(self, cmPrjDir):
        files = os.listdir(cmPrjDir)
        for file in files:
            if file.endswith('.jpg') or file.endswith('.jpeg'):
                os.remove(os.path.join(cmPrjDir, file))

    def move_images_to_folder(self, cmPrjDir, destination_path, image_ts_name=None):
        try:
            files = os.listdir(cmPrjDir)
            count = 0
            source_path = ""
            destn_path = ""
            for i in range(0, len(files)):
                if (files[i].endswith('.jpg') or files[i].endswith('.jpeg')):
                    source_path = (os.path.join(cmPrjDir, files[i]))
                    if image_ts_name != None:
                        destn_path = os.path.join(destination_path, str(image_ts_name[count]) + ".jpg")
                        shutil.move(source_path, destn_path)
                    else:
                        destn_path = os.path.join(destination_path, files[i])
                        shutil.move(source_path, destn_path)
                    count = count + 1
        except Exception as e:
            print("Exception occurred during image extraction through visual debugger: ", e)

    def rec_bat(self, rec_batch, sim_output, projectDir, videos_path, testrun):
        if "." in testrun:
            testrun = testrun.replace(".", "_dot_")
        if sim_output != '1':
            sim_bat = projectDir + r"\src_ext\Tools\VSPRecGen\rec64" + "\\" + str(testrun) + r".bat"
        else:
            sim_bat = str(rec_batch) + "\\" + str(testrun) + r".bat"
        read_1config = open(sim_bat, "rt")
        data = read_1config.read()
        video_files = os.path.join(videos_path + "\\" + testrun + ".mp4")
        try:
            if '.rec' in data:
                data = data.replace('.rec', '.rec {0}'.format(video_files))
        except:
            pass
        read_1config.close()
        write_config = open(sim_bat, "wt")
        write_config.write(data)
        write_config.close()
        os.system(sim_bat)

    def CreateScenarioVideo(self, rec_batch, sim_output, projectDir, testrun, output_dir, framerate, width, height):
        """
        Generate a video from the simulation in avi format

        :param testrun: cmTestRun instance (see pycmEval.cm_testrun)
        :param output_dir: Absolute path of the output directory
        """
        logger.info("CMRC: Creating video for the scenario: " + testrun)
        # Create video folder to store the extracted video
        videos_path = os.path.join(projectDir + "\\" + output_dir + "\Video" + "\\" + testrun)
        if os.path.exists(videos_path):
            if os.path.exists(os.path.join(videos_path, testrun + ".mp4")):
                os.remove(os.path.join(videos_path, testrun + ".mp4"))
        else:
            os.makedirs(videos_path)

        # Export video for the current testrun
        self.Exec(
            'Movie export window {0}.mp4 0 -start 0 -end 200 -framerate {1} -width {2} -height {3}'.format(testrun,
                                                                                                           framerate,
                                                                                                           width,
                                                                                                           height))
        time.sleep(10)

        all_files = os.listdir(projectDir)
        for file in all_files:
            if file.endswith('.mp4'):
                files = file.split(".")[0]
                if os.path.exists(videos_path) and str(files) == str(testrun):
                    shutil.move((os.path.join(projectDir, file)), os.path.join(videos_path, file))

        # Copying mp4 path to batch files
        self.rec_bat(rec_batch, sim_output, projectDir, videos_path, testrun)
