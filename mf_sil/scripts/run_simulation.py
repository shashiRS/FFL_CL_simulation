"""example CLS SIL simulation container"""
import os
import time
from cls_container_pkg import *
import subprocess as sp
import psutil


# container root path
CONTAINER_ROOT_PATH = os.path.join(os.path.dirname(__file__), '..')

# number of parallel CarMaker simulations
CM_NUM_INSTANCES = 4

maxExecutionTime_AUP          = 300
maxExecutionTime_TCE          = 9000
maxSimulationTime_AUP         = 250

# get expected eCAL processes for supervision
CHECKED_PROCESSES = []
PROCESS_CHECK_REPORT = {}


def importSignalList():
    """import signal list"""
    PATH_AUP_import_signal_list = os.path.abspath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "tests", "SIL", "Report_Generator", "ImportList.txt"))
    with open(PATH_AUP_import_signal_list, 'r') as import_list:
        # signals inside import_list can be found: <SW_signal_name> [architecture_name]
        # <SW_signal_name> - signal as it is found in the SW
        # [architecture_name] - name which should be used in the test specification; optional
        # key words of the dictionary structures are searched inside [architecture_name]

        # Read all signals from txt file and save the signals in Quantities_Import list
        Quantities_Import = []

        # structures - dictionary with keyword: structure name in the import list. The signals inside needs to be multiplied
        # value: maximum index number
        structures = {"envModelPort": 8, "TRJPLA": 10}
        for line in import_list:
            if len(line) > 0:
                # if it is not an empty line, add the <SW_signal_name> in the list
                Quantities_Import.append(line.rstrip("\n").split()[0])
                # if [architecture_name] exists, use the dictionary to search if the signals should be multiplied
                if len(line.split()) > 1:
                    for struct_name, struct_val in structures.items():
                        if struct_name in line.split()[1]:
                            for i in range(1, struct_val):
                                updated_line = line.rstrip("\n").split()[0].replace("_0.", "_" + str(i) + ".", 1)
                                Quantities_Import.append(updated_line)
    return Quantities_Import


# user defined measurement quantities to be stored in the ERG measurement file
MEAS_QUANTS = importSignalList()
# all quantities to be stored in the measurement files, including signals for
# optimization and video generation
ALL_QUANTS = list(set(MEAS_QUANTS + VIDEO_QUANTITIES + OPTIMIZE_QUANTITIES))


def change_CM_config_line_endings(carmaker_project: str):
    '''
    PI-2402: change the line endings of OutputQuantities and SimParameters from windows
    endings to Linux endings
    '''
    WINDOWS_LINE_ENDING = b'\r\n'
    UNIX_LINE_ENDING = b'\n'
    files = [os.path.join(carmaker_project, "Data", "Config", "OutputQuantities"),
             os.path.join(carmaker_project, "Data", "Config", "SimParameters")]
    for file_path in files:
        with open(file_path, 'rb') as open_file:
            content = open_file.read()
            content = content.replace(WINDOWS_LINE_ENDING, UNIX_LINE_ENDING)
        with open(file_path, 'wb') as open_file:
            open_file.write(content)

def get_checked_processes(carmaker_project):
    """get processes that shall be checked during the container execution"""
    global CHECKED_PROCESSES
    with open(os.path.join(CONTAINER_ROOT_PATH, "bin", 'cem_automization.ecalsys'), 'r') as ecal_cfg_file:
        for file_line in ecal_cfg_file.readlines():
            match = re.match(r'\s*<algo>.*\\(.*\.exe)<\/algo>', file_line)
            if match:
                CHECKED_PROCESSES.append(match.group(1))


def process_check(expect_running, testrun_name, check_position_text):
    """check status of processes"""
    global PROCESS_CHECK_REPORT
    all_procs = set()
    proc_names = []
    # get names of all running processes
    for parent in [proc for proc in psutil.process_iter()]:
        try:
            all_procs.add(parent.name())
            proc_names += list(all_procs.union(set(c.name() for c in parent.children(True))))
        except:
            pass  # process is gone, ignore it
    # check which of the expected processes are not running
    procs_not_running = set(CHECKED_PROCESSES) - set(proc_names)
    result = list(procs_not_running) if expect_running else list(set(CHECKED_PROCESSES) - procs_not_running)
    # update report dict
    if testrun_name not in PROCESS_CHECK_REPORT:
        PROCESS_CHECK_REPORT.update({testrun_name: {check_position_text: result}})
    else:
        PROCESS_CHECK_REPORT[testrun_name].update({check_position_text: result})
    # log result if process state is unexpected
    if result:
        log_text = check_position_text
        log_text += f': Processes are {"NOT" if expect_running else ""} running '
        log_text += f'but are {"NOT" if not expect_running else ""} expected: '
        log_text += str(result)
        print(log_text)

def start_simulation(
        carmaker_runner,
        testruns: list) -> None:
    """all functions to execute the simulation

    Attributes:
        carmaker_runner: CarMaker evaluation runner instance to control the
        CarMaker simulation execution.

        testruns: list with CarMaker testrun objects
    """
    process_check(testrun_name='',
                  expect_running=False,
                  check_position_text='before eCal start')
    
    ecal_cem_pid = sp.Popen(
        os.path.join(CONTAINER_ROOT_PATH, "bin", "start_cem_nodes.bat"),
        cwd = os.path.join(CONTAINER_ROOT_PATH, "bin"),
        creationflags=sp.CREATE_NEW_CONSOLE | sp.CREATE_NEW_PROCESS_GROUP).pid

    time.sleep(4)

    process_check(testrun_name='',
                  expect_running=True,
                  check_position_text='before simulation start')

    # execute simulation
    cls_run_simulation(
        carmaker_runner=carmaker_runner,
        testruns=testruns,
        carmaker_exec=None,
        max_sim_time=maxSimulationTime_AUP,
        max_exec_time=maxExecutionTime_AUP)
    
    process_check(testrun_name='',
                  expect_running=True,
                  check_position_text='after simulation end')

    # kill eCal nodes
    sp.Popen(f"taskkill /pid {ecal_cem_pid} /f /t")


    # in case the CarMaker executable remains running for some reason
    os.system("taskkill /im HIL.exe /f >nul 2>&1")
    os.system("taskkill /im Movie.exe /f >nul 2>&1")
    os.system("taskkill /im CarMaker.win64.exe /f >nul 2>&1")
    os.system("taskkill /im roadutil.exe /f >nul 2>&1")

    process_check(testrun_name='',
                expect_running=False,
                check_position_text='after eCal end')


def mode_simulation(
        in_path: str,
        out_path: str,
        timeout_value: int,
        carmaker_project: str,
        carmmaker_install_path: str) -> None:
    """all functions that need to be executed for a simulation

    This functions prepares and executes the simulation.
    Testruns are retrieved from the given input folder and moved into
    the CarMaker project folder. The testruns are processed not all at once but
    in batches, this allows to dynamically add further testruns.
    After each testrun batch the measurement files are move to the given
    output folder.
    If no testruns can be found in the current processing cycle the function
    will try to get added testruns until the timeout given by the timeout value
    is exceeded.

    Arguments:
        in_path: Absolute path to input folder.

        out_path: Absolute path to output folder.

        timeout_value: Timeout for retrieval of measurement files.

        carmaker_project: Path to the CarMaker project and CarMaker version of
        the project.

        carmmaker_install_path: Absolute path to the CarMaker installation.
    """
    # change config line endings
    change_CM_config_line_endings(carmaker_project)

    # generate JSON
    os.environ["CARMAKER_GENERATE_JSON"] = "1"
    os.environ["CARMAKER_GENERATE_JSON_ROOT"] = os.path.join(carmaker_project, "SimOutput")

    # get processes for supervision
    get_checked_processes(carmaker_project)

    # get timeout to retrieve testruns
    timeout_cntr = timeout_value
    # prepare simulation execution
    cm_runner = cls_prepare_simulation(
        in_dir_path=in_path,
        carmaker_project=carmaker_project,
        meas_signals=ALL_QUANTS,
        meas_sample_rate='10ms',
        num_carmaker_instances=CM_NUM_INSTANCES)

    # get initial set of testruns
    testruns = cls_get_testrun_input(
        in_dir_path=in_path,
        carmaker_project=carmaker_project,
        num_files=CM_NUM_INSTANCES)

    # loop until timeout is exceeded, timeout is reset when
    # new testruns are found
    while timeout_cntr > 0:
        # if any testruns are available
        if len(testruns) > 0:
            # reset timeout
            timeout_cntr = timeout_value
            # copy all road files to ensure that relevant roads are available
            cls_get_road_input(
                in_dir_path=in_path,
                carmaker_project=carmaker_project)
            # execute the simulation with the available testruns
            start_simulation(
                carmaker_runner=cm_runner,
                testruns=testruns)
            # provide measurement files to output folder
            cls_provide_meas_output(
                out_dir_path=out_path,
                carmaker_project=carmaker_project)
            # get next set of testruns
            testruns = cls_get_testrun_input(
                in_dir_path=in_path,
                carmaker_project=carmaker_project,
                num_files=CM_NUM_INSTANCES)

        else:
            # if not testruns were found in the previous cycle
            # check if new testruns are available now
            testruns = cls_get_testrun_input(
                in_dir_path=in_path,
                carmaker_project=carmaker_project,
                num_files=CM_NUM_INSTANCES)
            # if new testruns were found reset timeout
            if len(testruns) > 0:
                timeout_cntr = timeout_value
            else:
                # wait for new testruns
                timeout_cntr -= 1
                time.sleep(1)


def mode_video_erg(
        in_path: str,
        out_path: str,
        timeout_value: int,
        carmaker_project: str,
        carmmaker_install_path: str,
        cam_select: str = 'CLS_preview') -> None:
    """all functions that need to be executed for video generation from
    measurement files (ERG)

    This functions prepares and executes the video generation.
    Measurements are retrieved from the given input folder and moved into the
    CarMaker project folder. The measurements are processed not all at once but
    in batches, this allows to dynamically add further measurements.
    After each measurement batch the video files are move to the given output
    folder.
    If no measurements can be found in the current processing cycle the
    function will try to get added measurements until the timeout given by the
    timeout value is exceeded.
    The video generation generates the videos according to the selected
    camera view.

    Arguments:
        in_path: Absolute path to input folder.

        out_path: Absolute path to output folder.

        timeout_value: Timeout for retrieval of measurement files.

        carmaker_project: Path to the CarMaker project and CarMaker version
        of the project.

        carmmaker_install_path: Absolute path to the CarMaker installation.

        cam_select: Selected camera view.
    """
    timeout_cntr = timeout_value
    # prepare video generation
    cm_runner = cls_prepare_video_generation(
        carmaker_project=carmaker_project,
        cm_install_path=carmmaker_install_path,
        cam_select=cam_select)
    # get initial set of measurements
    measurements = cls_get_meas_input(
        in_dir_path=in_path,
        carmaker_project=carmaker_project,
        num_files=CM_NUM_INSTANCES)
    while timeout_cntr > 0:
        if len(measurements) > 0:
            timeout_cntr = timeout_value
            # get all testrun files to ensure their availability
            cls_get_testrun_input(
                in_dir_path=in_path,
                carmaker_project=carmaker_project)
            # get all road files to ensure their availability
            cls_get_road_input(
                in_dir_path=in_path,
                carmaker_project=carmaker_project)
            # run video creation
            cls_run_video_creation(
                cm_runner=cm_runner,
                carmaker_project=carmaker_project,
                cm_install_path=carmmaker_install_path,
                erg_files=measurements)
            # provide video files
            cls_provide_video_output(
                out_dir_path=out_path,
                carmaker_project=carmaker_project)
            # get next set of measurements
            measurements = cls_get_meas_input(
                in_dir_path=in_path,
                carmaker_project=carmaker_project,
                num_files=CM_NUM_INSTANCES)
        else:
            # if not measurements were found in the previous cycle
            # check if new measurements are available now
            measurements = cls_get_meas_input(
                in_dir_path=in_path,
                carmaker_project=carmaker_project,
                num_files=CM_NUM_INSTANCES)
            # if new measurements were found reset timeout
            if len(measurements) > 0:
                timeout_cntr = timeout_value
            else:
                # wait for new measurements
                timeout_cntr -= 1
                time.sleep(1)

    # complete video creation
    cls_finish_video_creation(cm_runner=cm_runner)


if __name__ == "__main__":
    # get command line arguments
    args = cls_get_args()
    mode = cls_get_mode(args)

    # get CarMaker project and version
    carmaker_project, carmaker_version = cls_get_carmaker_project()
    # get CarMaker installation path for the given version
    carmmaker_install_path = cls_get_carmaker_install_path(
        carmaker_version=carmaker_version)

    if mode == 'clean':
        # clean CarMaker project
        cls_cleanup_carmaker_project(
            carmaker_project=carmaker_project)

    elif mode == 'simulate':
        # get command line arguments
        in_path, out_path = cls_get_io_paths(args)
        timeout_value = cls_get_timeout(args)

        # run simulation mode
        mode_simulation(
            in_path=in_path,
            out_path=out_path,
            timeout_value=timeout_value,
            carmaker_project=carmaker_project,
            carmmaker_install_path=carmmaker_install_path)

    elif mode == 'video-erg':
        # get command line arguments
        in_path, out_path = cls_get_io_paths(args)
        timeout_value = cls_get_timeout(args)
        cam_select = cls_get_camera_view(args)

        # run simulation mode
        mode_video_erg(
            in_path=in_path,
            out_path=out_path,
            timeout_value=timeout_value,
            carmaker_project=carmaker_project,
            carmmaker_install_path=carmmaker_install_path,
            cam_select=cam_select)

    elif mode == 'video-testrun':
        # get command line arguments
        in_path, out_path = cls_get_io_paths(args)
        timeout_value = cls_get_timeout(args)
        cam_select = cls_get_camera_view(args)

        # run simulation mode
        mode_simulation(
            in_path=in_path,
            out_path=out_path,
            timeout_value=timeout_value,
            carmaker_project=carmaker_project,
            carmmaker_install_path=carmmaker_install_path)
        # run simulation mode
        mode_video_erg(
            in_path=in_path,
            out_path=out_path,
            timeout_value=timeout_value,
            carmaker_project=carmaker_project,
            carmmaker_install_path=carmmaker_install_path,
            cam_select=cam_select)

    # finalize container processing
    cls_finalize(args=args)
