"""Simulation control utilities.

This file is created by the CLS cloud services team and shall not be modified.
"""
import os
import re
import sys
import time
import shutil
import argparse
import subprocess
import logging
import logging.config
from typing import Optional


# check availability of IPGHOME environment variable
if not os.environ.get('IPGHOME'):
    raise AttributeError('IPGHOME variable is not defined!')

# root path based on this file
_ROOT_PATH = os.path.dirname(os.path.abspath(__file__))

# add folder to import pyBase
PYBASE_PATH = os.path.join(_ROOT_PATH, '..', 'contrib')
if PYBASE_PATH not in sys.path:
    sys.path.append(PYBASE_PATH)

# camera configuration settings
_CM_CAMERA_SETTINGS = """#INFOFILE1.1 - Do not remove this line!
FileIdent = IPGMovie-Camera 7
FileCreator = IPGMovie 10.2.1 2021-12-14

Camera.0.Name = CLS_preview
Camera.0.Mode = MoveWithCar
Camera.0.AttachedTo = Egocar
Camera.0.Pos = 6.000 0 0
Camera.0.dist = 40
Camera.0.rot = 0 70 0
Camera.0.FieldOfView = 70

"""

# measurement signals required for video generation
VIDEO_QUANTITIES = [
    'Traffic.*.DetectLevel',
    'Traffic.*.LongVel',
    'Traffic.*.Pitch_X',
    'Traffic.*.Roll_X',
    'Traffic.*.State',
    'Traffic.*.SteerAng',
    'Traffic.*.rx',
    'Traffic.*.ry',
    'Traffic.*.rz',
    'Traffic.*.tx',
    'Traffic.*.ty',
    'Traffic.*.tz',
    'Vhcl.Distance',
    'Vhcl.FL.Fx',
    'Vhcl.FL.Fy',
    'Vhcl.FL.Fz',
    'Vhcl.FL.rot',
    'Vhcl.FL.rx',
    'Vhcl.FL.ry',
    'Vhcl.FL.rz',
    'Vhcl.FL.tx',
    'Vhcl.FL.ty',
    'Vhcl.FL.tz',
    'Vhcl.FR.Fx',
    'Vhcl.FR.Fy',
    'Vhcl.FR.Fz',
    'Vhcl.FR.rot',
    'Vhcl.FR.rx',
    'Vhcl.FR.ry',
    'Vhcl.FR.rz',
    'Vhcl.FR.tx',
    'Vhcl.FR.ty',
    'Vhcl.FR.tz',
    'Vhcl.Fr1.x',
    'Vhcl.Fr1.y',
    'Vhcl.Fr1.z',
    'Vhcl.Pitch',
    'Vhcl.RL.Fx',
    'Vhcl.RL.Fy',
    'Vhcl.RL.Fz',
    'Vhcl.RL.rot',
    'Vhcl.RL.rx',
    'Vhcl.RL.ry',
    'Vhcl.RL.rz',
    'Vhcl.RL.tx',
    'Vhcl.RL.ty',
    'Vhcl.RL.tz',
    'Vhcl.RR.Fx',
    'Vhcl.RR.Fy',
    'Vhcl.RR.Fz',
    'Vhcl.RR.rot',
    'Vhcl.RR.rx',
    'Vhcl.RR.ry',
    'Vhcl.RR.rz',
    'Vhcl.RR.tx',
    'Vhcl.RR.ty',
    'Vhcl.RR.tz',
    'Vhcl.Roll',
    'Vhcl.Steer.Ang',
    'Vhcl.Steer.Trq',
    'Vhcl.Yaw',
    'Vhcl.sRoad',
    'Vhcl.v',
    'Time',
    'LineDetect.*',
    'Sensor.Collision.Vhcl.Fr1.ObjId',
    'Sensor.Collision.Vhcl.WRL.ObjId',
    'Sensor.Collision.Vhcl.WFL.ObjId',
    'Sensor.Collision.Vhcl.WRR.ObjId',
    'Sensor.Collision.Vhcl.WFR.ObjId',
    'Sensor.Road.*'
]


# measurement signals required for scenario optimization
OPTIMIZE_QUANTITIES = [
    'Time',

    'Car.Fr1.tx',
    'Car.Fr1.ty',
    'Car.Fr1.rz',
    'Car.v',
    'Car.Distance',

    'Traffic.*.tx',
    'Traffic.*.ty',
    'Traffic.*.rz',
    'Traffic.*.LongVel',
    'Traffic.*.Distance'
]


# measurement signals required for simple visualization with Matchbox
MATCHBOX_QUANTITIES = [
    'Time',

    'Car.Fr1.tx',
    'Car.Fr1.ty',
    'Car.Fr1.rz',
    'Car.v',
    'Car.Length_m',
    'Car.Width_m',

    'Traffic.*.tx',
    'Traffic.*.ty',
    'Traffic.*.rz',
    'Traffic.*.LongVel',
    'Traffic.*.Length_m',
    'Traffic.*.Width_m',
]


# relevant simulation output file types
_SIM_OUTPUT_TYPES = [
    '.erg',
    '.erg.info',
    '.log',
    '.bsig',
    '.json'
]


# relevant video generation output file types
_VIDEO_OUTPUT_TYPES = [
    '.mp4'
]


# logging configuration
_LOG_CONF = {
    'version': 1,
    'formatters': {
        'formater': {
            'style': '{',
            'format': '{asctime}.{msecs:03.0f} [root] {levelname} {message}',
            'datefmt': '%d-%m-%Y %H:%M:%S'
        }},
    'handlers': {
        'console-handler': {
            'level': 'DEBUG',
            'formatter': 'formater',
            'class': 'logging.StreamHandler',
            'stream': 'ext://sys.stdout'
        }},
    'root': {
        'level': 'NOTSET',
        'handlers': ['console-handler']},
    'loggers': {'pyBase': {'propagate': True}}
}

# setup default logging
logging.config.dictConfig(_LOG_CONF)


# ############################################################
# ################### UTILITY FUNCTIONS ######################
# ############################################################

def cls_get_args() -> dict:
    """get command line arguments

    This function defines the accepted command line parameters.
    It returns the dictionary with the parameters and their values.

    Returns:
        Command line parameters and value.
    """
    # get supported camera views
    supported_cam_views = re.findall(
        r"Camera\.\d+\.Name\s+=\s+(\w+)",
        _CM_CAMERA_SETTINGS)

    # Parse Command Line Options
    optparser = argparse.ArgumentParser(usage='run_simulation.py [OPTIONS]')
    sub_parsers = optparser.add_subparsers(
        title='operation modes',
        metavar='clean | simulate | video-erg | video-testrun',
        dest='mode',
        help='use <operation mode> --help for mode details.')

    # cleanup
    sub_parsers.add_parser(
        'clean',
        description='Cleanup container.')

    # simulation
    sim_parser = sub_parsers.add_parser(
        'simulate',
        description='Run CarMaker simulation.')
    sim_parser.add_argument(
        '-i', '--input',
        dest='input_loc',
        default=None,
        help='Input directory with all required input files. Testruns must be '
             'provided in the subfolder "./Data/TestRun", optional road files '
             'must be provided in the subfolder "./Data/Road".')
    sim_parser.add_argument(
        '-o', '--output',
        dest='output_loc',
        default=None,
        help='Output directory for CarMaker measurement files, the files will '
             'be provided in the subfolder "./SimOutput".')
    sim_parser.add_argument(
        '-l', '--label',
        dest='label',
        default=None,
        help='Label for logging.')
    sim_parser.add_argument(
        '-t', '--timeout',
        dest='timeout',
        default=None,
        help='Timeout in seconds for dynamic loading of testrun files.')

    # video generation, ERG files
    vid_erg_parser = sub_parsers.add_parser(
        'video-erg',
        description='Generate videos from measurement files. '
                    'Supported measurement files: [.erg]')
    vid_erg_parser.add_argument(
        '-i', '--input',
        dest='input_loc',
        default=None,
        help='Input directory with all required input files. Testruns must '
             'be provided in the subfolder "./Data/TestRun", optional road '
             'files must be provided in the subfolder "./Data/Road", erg '
             'measurement files must be provided in the subfolder '
             '"./SimOutput".')
    vid_erg_parser.add_argument(
        '-o', '--output',
        dest='output_loc',
        default=None,
        help='Output directory for video files, the files will be provided in '
             'subfolder "./Video".')
    vid_erg_parser.add_argument(
        '-l', '--label',
        dest='label',
        default=None,
        help='Label for logging.')
    vid_erg_parser.add_argument(
        '-c', '--camera-view',
        dest='camera_view',
        default='CLS_preview',
        help='Selection of camera view for video generation, '
             f'available options: {str(supported_cam_views)}.')
    vid_erg_parser.add_argument(
        '-t', '--timeout',
        dest='timeout',
        default=None,
        help='Timeout in seconds for dynamic loading of measurement files.')

    # video generation, TESTRUN files
    vid_testrun_parser = sub_parsers.add_parser(
        'video-testrun',
        description='Generate videos from testrun files.')
    vid_testrun_parser.add_argument(
        '-i', '--input',
        dest='input_loc',
        default=None,
        help='Input directory with all required input files. Testruns must '
             'be provided in the subfolder "./Data/TestRun", optional road '
             'files must be provided in the subfolder "./Data/Road"')
    vid_testrun_parser.add_argument(
        '-o', '--output',
        dest='output_loc',
        default=None,
        help='Output directory for video files, the files will be provided in '
             'subfolder "./Video".')
    vid_testrun_parser.add_argument(
        '-l', '--label',
        dest='label',
        default=None,
        help='Label for logging.')
    vid_testrun_parser.add_argument(
        '-c', '--camera-view',
        dest='camera_view',
        default='CLS_preview',
        help='Selection of camera view for video generation, '
             f'available options: {str(supported_cam_views)}.')
    vid_testrun_parser.add_argument(
        '-t', '--timeout',
        dest='timeout',
        default=None,
        help='Timeout in seconds for dynamic loading of measurement files.')

    # parse command lines arguments
    args = optparser.parse_args()
    # update logging label
    cls_update_label(args.__dict__)

    return args.__dict__


def cls_get_mode(args: dict) -> str:
    """provide operation mode

    The operation modes defines the behaviour of the automation script.

    Arguments:
        args: Command line arguments dictionary.

    Returns:
        Operation mode.
    """
    return args['mode']


def cls_get_timeout(args: dict) -> int:
    """provide timeout to get testruns and measurement files

    The automation allows the dynamic adding of testrun files for the
    simulation or measurement files for the video generation.
    The timeout specifies the time the automation scripts awaits additional
    files.

    Arguments:
        args: Command line arguments dictionary.

    Returns:
        Timeout in s.
    """
    timeout = args.get('timeout', 1)
    try:
        timeout = int(timeout)
    except ValueError:
        timeout = 1
    except TypeError:
        timeout = 1
    timeout = max(int(timeout), 1)
    return timeout


def cls_get_io_paths(args: dict) -> tuple:
    """provide input and output paths

    The absolute input and output paths are the exchange location between the
    automation and the overlying orchestration.

    Arguments:
        args: Command line arguments dictionary.

    Returns:
        Absolute input and output paths as tuple.
    """
    return args['input_loc'], args['output_loc']


def cls_get_camera_view(args: dict) -> str:
    """provide camera view

    The camera view sets the viewing angle, distance and the field of view
    for the generated video.

    Arguments:
        args: Command line arguments dictionary.

    Returns:
        Camera view.
    """
    return args['camera_view']


def cls_update_label(args: dict) -> None:
    """update label for logging

    Set the label that shall be used for logging.

    Arguments:
        args: Command line arguments dictionary.
    """
    label = args.get('label')
    if label is not None:
        _LOG_CONF['formatters']['formater']['format'] =\
            _LOG_CONF['formatters']['formater']['format'].replace('root',
                                                                  label)
        logging.config.dictConfig(_LOG_CONF)


def cls_get_carmaker_version(carmaker_project: str) -> Optional[str]:
    """determine CarMaker version

    Automatically determine CarMaker version from CM project
    configuration file.

    Arguments:
        carmaker_project: path to the CarMaker project folder

    Returns:
        CarMaker version as string or None if version could not be determined.
    """
    cm_ver = None
    conf_file = os.path.join(carmaker_project, 'Data', 'Config', 'Project')
    if os.path.isfile(conf_file):
        with open(conf_file, 'r') as ifile:
            for file_line in ifile.readlines():
                match = re.match(
                    r'FileCreator\s+=\s+CarMaker\s+([\d\.]*)',
                    file_line)
                if match:
                    cm_ver = match.group(1)
                    break
    return cm_ver


def cls_get_carmaker_project() -> tuple:
    """get CarMaker project path and CarMaker version

    Automatically find CarMaker project and version by CM project
    configuration file.

    Returns:
        Path to the CarMaker project and CarMaker version of the project.
    """
    cm_project = None
    prj_root = os.path.join(_ROOT_PATH, '..')
    for dirpath, _, filenames in os.walk(prj_root):
        for filename in filenames:
            if os.path.join('Data', 'Config', 'Project') in\
                    os.path.join(dirpath, filename):
                cm_project = os.path.abspath(os.path.join(dirpath, '..', '..'))
    carmaker_version = cls_get_carmaker_version(carmaker_project=cm_project)
    return cm_project, carmaker_version


def cls_get_carmaker_install_path(carmaker_version: str) -> str:
    """CarMaker installation path

    The absolute path to the CarMaker installation, based on the "IPGHOME"
    environment variable and the CarMaker version.

    Arguments:
        carmaker_version: CarMaker version of the project

    Returns:
        Absolute path to the CarMaker installation.
    """
    if sys.platform == 'linux':
        return os.path.abspath(
            os.path.join(os.environ['IPGHOME'],
                         'carmaker', f'linux64-{carmaker_version}'))
    else:
        return os.path.abspath(
            os.path.join(os.environ['IPGHOME'],
                         'carmaker', f'win64-{carmaker_version}'))


def cls_finalize(args: dict) -> None:
    """finalize container processing

    Move the CarMaker measurements files to the output directory, the subfolder
    "./SimOutput" will be created.

    Arguments:
        args: Command line arguments dictionary.
    """
    cm_prj, _ = cls_get_carmaker_project()
    # check for MISSING_QUANTITIES.txt from video creation
    miss_quants = os.path.join(cm_prj, 'MISSING_QUANTITIES.txt')
    if os.path.isfile(miss_quants):
        with open(miss_quants, 'r') as mq_file:
            logging.warning('Missing quantities found for video generation')
            logging.info(mq_file.read())
    logging.info('Container processing completed')


# ############################################################
# ################## SIMULATION FUNCTIONS ####################
# ############################################################


def cls_get_testrun_input(
        in_dir_path: str,
        carmaker_project: str,
        num_files: Optional[int] = None) -> list:
    """get testrun files from input path

    The testrun files will be moved from the input folder into the CarMaker
    project folder.
    The found testrun files are then collected into a list and provided for
    the further processing.
    The number of files that are retrieved can be set with the "num_files"
    parameter to allow detection of the already processed testruns by the
    "CLS Campaign Control" Service.

    Arguments:
        in_dir_path: Absolute path to the input folder, file path must contain
            the directory structure "./Data/TestRun".

        carmaker_project: Absolute path to the CarMaker project.

        num_files: Number of files that are considered.

    Returns:
        List of pyBase testrun objects.
    """
    try:
        from VSP_PYBASE.pycmEval.cm_testrun import cmTestRun
    except Exception:
        from VSP_pyBase.pycmEval.cm_testrun import cmTestRun

    src = os.path.abspath(os.path.join(in_dir_path, 'Data', 'TestRun'))
    dst = os.path.abspath(os.path.join(carmaker_project, 'Data', 'TestRun'))
    # check if testruns are provided at the expected location
    if not os.path.exists(src):
        raise AttributeError(f'Invalid TestRun path [{src}]')

    testruns = []
    # get testruns from input location
    for root, _, files in os.walk(src):
        for name in files:
            if not name.endswith(".ts"):
                if (num_files is None) or (num_files > 0):
                    os.makedirs(dst, exist_ok=True)
                    if src != dst:
                        shutil.move(
                            os.path.join(root, name),
                            os.path.join(dst, name))
                    # create testrun object
                    tr_name = os.path.splitext(os.path.basename(name))[0]
                    testruns.append(
                        cmTestRun(tr_name, os.path.join(dst, name)))
                    logging.info('Found testrun [%s]', name)
                    if num_files is not None:
                        num_files -= 1
                else:
                    break
        if num_files == 0:
            break

    return testruns


def cls_get_road_input(
        in_dir_path: str,
        carmaker_project: str) -> None:
    """get input road files for simulation

    Move the CarMaker road files from the input directory to the
    CarMaker project.

    Arguments:
        in_dir_path: Absolute path to the input folder, file path must contain
            the directory structure "./Data/Road".

        carmaker_project: Absolute path to the CarMaker project.
    """
    src = os.path.join(in_dir_path, 'Data', 'Road')
    dst = os.path.join(carmaker_project, 'Data', 'Road')
    os.makedirs(dst, exist_ok=True)
    if os.path.exists(src):
        if src != dst:
            for root, _, files in os.walk(src):
                for name in files:
                    if not os.path.isfile(os.path.join(dst, name)):
                        shutil.move(
                            os.path.join(root, name),
                            os.path.join(dst, name))


def cls_provide_meas_output(
        out_dir_path: str,
        carmaker_project: str) -> None:
    """put output erg files

    Move the CarMaker measurements files to the output directory, the subfolder
    "./SimOutput" will be created.

    Arguments:
        out_dir_path: Absolute path to the output folder.

        carmaker_project: Absolute path to the CarMaker project.
    """
    dst = os.path.join(out_dir_path, 'SimOutput')
    src = os.path.join(carmaker_project, 'SimOutput')
    os.makedirs(dst, exist_ok=True)
    logging.info('Providing output ERG files files at %s', dst)
    if src != dst:
        for root, _, files in os.walk(src):
            for name in files:
                if sum(map(lambda ft: name.endswith(ft), _SIM_OUTPUT_TYPES)):
                    shutil.move(
                        os.path.join(root, name),
                        os.path.join(dst, name))


def cls_prepare_simulation(
        in_dir_path: str,
        carmaker_project: str,
        meas_signals: list,
        meas_sample_rate: str = 'default',
        num_carmaker_instances: int = 1):
    """prepare CarMaker simulation

    Setup the pyBase CarMaker simulation runner.

    Arguments:
        in_dir_path: Absolute path to the input folder, file path must contain
            the directory structure "./Data/Road".

        carmaker_project: Absolute path to the CarMaker project.

        meas_signals: List of measurement signals that will be stored in the
            measurement files.

        meas_sample_rate: Sample rate for the measurement signals. The sample
            rate does not affect the simulation itself.

        num_carmaker_instances: Number of parallel running CarMaker instances.
            This value can only be set to values greater 1 if the execution of
            the container allows this.
            For ROS2 or eCAL based containers this value shall usually
            be set to 1.

    Returns:
        The pyBase CarMaker simulation runner.
    """
    # import pyBase classes
    try:
        from VSP_PYBASE.pycmEval.cm_eval_runner import cCMEvalTestRunner
    except Exception:
        from VSP_pyBase.pycmEval.cm_eval_runner import cCMEvalTestRunner

    # check input path
    if (in_dir_path is None) or (not os.path.exists(in_dir_path)):
        raise AttributeError(f'Invalid input path [{in_dir_path}]')
    # check carmaker project path
    if (carmaker_project is None) or (not os.path.exists(carmaker_project)):
        raise AttributeError(
            f'Invalid CarMaker project path [{carmaker_project}]')
    # remove output folder
    if os.path.exists(os.path.join(carmaker_project, 'SimOutput')):
        shutil.rmtree(os.path.join(carmaker_project, 'SimOutput'))
    os.makedirs(os.path.join(carmaker_project, 'SimOutput'))

    # create CarMaker test runner
    cm_runner = cCMEvalTestRunner(
        cmPrjDir=carmaker_project,
        cmQuantities=meas_signals,
        numberOfCarMakerInstances=num_carmaker_instances,
        sampleRate=meas_sample_rate)

    return cm_runner


def cls_run_simulation(
        carmaker_runner,
        testruns: list,
        carmaker_exec: str,
        max_sim_time: Optional[float] = None,
        max_exec_time: Optional[float] = None) -> None:
    """run CarMaker simulation with pyBase

    Execute the CarMaker simulation with pyBase using the container
    CarMaker node executable.
    The simulation will be run for all provided testruns.

    Arguments:
        carmaker_runner: The pyBase CarMaker simulation runner.

        testruns: List of pyBase testrun objects.

        carmaker_exec: Path to the CarMaker node executable.

        max_sim_time: Maximum simulation time.

        max_exec_time: Maximum execution time.
    """
    carmaker_runner.runEvaluations(
        outputFolderName='.',
        testRuns=testruns,
        headlessMode=True,
        useCoreLicense=True,
        nRetries=3,
        maxTimeInSimulationSeconds=max_sim_time,
        maxExecTimeSeconds=max_exec_time,
        raiseSimExecExceptions=False,
        ignoreExistingResFile=True,
        executable=carmaker_exec,
        CMprojectexe=carmaker_exec)


# ############################################################
# #################### VIDEO FUNCTIONS #######################
# ############################################################


def cls_get_meas_input(
        in_dir_path: str,
        carmaker_project: str,
        num_files: Optional[int] = None) -> list:
    """get measurement files from input path

    The measurement files will be moved from the input folder into the CarMaker
    project folder.
    The found measurement files are then collected into a list and provided for
    the further processing.
    The number of files that are retrieved can be set with the "num_files"
    parameter to allow detection of the already processed measurements by the
    "CLS Campaign Control" Service.

    Arguments:
        in_dir_path: Absolute path to the input folder, file path must contain
            the directory structure "./SimOutput".

        carmaker_project: Absolute path to the CarMaker project.

        num_files: Number of files that are considered.

    Returns:
        CarMaker measurement files (.erg).
    """
    src = os.path.join(in_dir_path, 'SimOutput')
    dst = os.path.join(carmaker_project, 'SimOutput')

    meas_files = []
    for root, _, files in os.walk(src):
        for name in files:
            if (num_files > 0) or (num_files is None):
                os.makedirs(dst, exist_ok=True)
                if name.endswith(".erg"):
                    if src != dst:
                        shutil.move(
                            os.path.join(root, name),
                            os.path.join(dst, name))
                        shutil.move(
                            os.path.join(root, name + '.info'),
                            os.path.join(dst, name + '.info'))
                    # create erg object
                    tr_name = os.path.splitext(os.path.basename(name))[0]
                    meas_files.append((tr_name, os.path.join(dst, name)))
                    logging.info('Found measurement [%s]', name)
                    if num_files is not None:
                        num_files -= 1
            else:
                break
        if num_files == 0:
            break

    return meas_files


def cls_provide_video_output(
        out_dir_path: str,
        carmaker_project: str) -> None:
    """put output video files

    Move the generated video files to the output directory, the subfolder
    "./Video" will be created.

    Arguments:
        out_dir_path: Absolute path to the output folder.

        carmaker_project: Absolute path to the CarMaker project.
    """
    dst = os.path.join(out_dir_path, 'Video')
    src = os.path.join(carmaker_project, 'Video')
    os.makedirs(dst, exist_ok=True)
    logging.info('Providing output MP4 files at %s', dst)
    if src != dst:
        for root, _, files in os.walk(src):
            for name in files:
                if sum(map(lambda ft: name.endswith(ft), _VIDEO_OUTPUT_TYPES)):
                    shutil.move(
                        os.path.join(root, name),
                        os.path.join(dst, name))


def _cls_convert_video(video_file_path: str) -> None:
    """convert video to MP4 H.264

    Convert video files (exported with IPGMovie) to MP4 H.264.
    This shall ensure a feasible video file size and compatibility to the
    CLS Dashboard video player.
    The converted video will replace the original video file.
    Audio streams will be removed from the video.
    The conversion is done using the tool "ffmpeg".

    Arguments:
        video_file_path: Absolute path to the video file that
        shall be converted.
    """
    logging.info('Converting video files from folder %s', video_file_path)
    video_loc = os.path.dirname(video_file_path)
    video_file = os.path.basename(video_file_path)
    video_name, video_ext = os.path.splitext(video_file)
    out_name = video_name + '_conv' + video_ext
    if video_ext == '.mp4':
        try:
            # run conversion: overwrite existing file,
            # use H.264 codec for video, disable audio
            subprocess.Popen(
                ['ffmpeg', '-y', '-i', os.path.join(video_loc, video_file),
                 '-vcodec', 'h264', '-an', os.path.join(video_loc, out_name)]
            ).wait()
            # if conversion was successful replace original file
            if os.path.isfile(os.path.join(video_loc, out_name)):
                os.remove(os.path.join(video_loc, video_file))
                os.rename(
                    os.path.join(video_loc, out_name),
                    os.path.join(video_loc, video_file))
        except Exception:
            logging.exception('Problem with video conversion')


def cls_prepare_video_generation(
        carmaker_project: str,
        cm_install_path: str,
        cam_select: str = 'CLS_preview'):
    """prepare video generation

    Start CarMaker and IPGMovie, connect IPGMovie to running CarMaker instance
    and setup the camera view configuration.

    Arguments:
        carmaker_project: Absolute path to the CarMaker project.

        cm_install_path: Absolute path to the CarMaker installation.

        cam_select: Selected camera view configuration.

    Returns:
        pyBase CarMaker control object.
    """
    # import pyBase class
    try:
        from VSP_PYBASE.pycmCommon.cmpyglue import CMRC
    except Exception:
        from VSP_pyBase.pycmCommon.cmpyglue import CMRC
    
    
    # check carmaker project path
    if (carmaker_project is None) or (not os.path.exists(carmaker_project)):
        raise AttributeError(
            f'Invalid CarMaker project path [{carmaker_project}]')
    # prepare video configuration
    cam_cfg_file = os.path.join(carmaker_project, 'Movie', 'Camera.cfg')
    # remove existing camera configuration
    if os.path.isfile(cam_cfg_file):
        os.remove(cam_cfg_file)
    # remove backup file
    if os.path.isfile(cam_cfg_file + '.autobackup'):
        os.remove(cam_cfg_file + '.autobackup')
    # write configuration file
    with open(cam_cfg_file, 'w') as cam_file:
        cam_file.write(_CM_CAMERA_SETTINGS)

    # remove output folder
    if os.path.exists(os.path.join(carmaker_project, 'Video')):
        shutil.rmtree(os.path.join(carmaker_project, 'Video'))
    os.makedirs(os.path.join(carmaker_project, 'Video'))

    try:
        # connect to CarMaker GUI
        if sys.platform == 'linux':
            cm_exe = 'CM'
        else:
            cm_exe = 'CM.exe'
        cm_runner = CMRC(
            startCM=True,
            video_extraction=True,
            cmGuiPath=os.path.join(cm_install_path, 'bin', cm_exe))
        # set project directory
        cm_runner.SetProjectDir(carmaker_project)
        # start IPG Movie
        cm_runner.Launch_IPGMovie(launch_IPG=cam_select)
        time.sleep(1)
    except Exception:
        logging.exception('Exception during movie creation')

    return cm_runner


def cls_run_video_creation(
        cm_runner,
        carmaker_project: str,
        erg_files: list,
        cm_install_path: str) -> None:
    """execute video creation

    This function does everything which is necessary to run the video creation
    and have the video files by hand.

    Arguments:
        cm_runner: pyBase CarMaker control object.

        carmaker_project: Absolute path to the CarMaker project.

        erg_files: CarMaker measurement files (.erg).

        cm_install_path: Absolute path to the CarMaker installation.
    """

    for erg_name, erg_loc in erg_files:
        # set video file parg
        video_file_path = os.path.normpath(os.path.abspath(
            os.path.join(carmaker_project, 'Video', erg_name + '.mp4')))
        os.makedirs(os.path.dirname(video_file_path), exist_ok=True)
        # load simulation data from erg file
        resp = cm_runner.Exec(
            f'Movie loadsimdata {erg_loc.replace(os.sep,"/")}'
            f' -projectDir {carmaker_project.replace(os.sep,"/")}'
            f' -installDir {cm_install_path.replace(os.sep,"/")}',
            waitForResponse=2.0)
        if resp.startswith('E'):
            logging.error(resp)
        else:
            logging.info(resp)
        # start video creation
        resp = cm_runner.Exec(
            f'Movie export window {video_file_path.replace(os.sep,"/")} 0'
            ' -start start -end end '
            '-framerate 25 -width 720 -height 512 '
            '-quality 5 -overwrite -async')
        if resp.startswith('E'):
            logging.error(resp)
        else:
            logging.info(resp)
        # check video creation completed
        video_complete = False
        while not video_complete:
            resp = cm_runner.Exec('Movie export status -detailed')
            if resp.startswith('E'):
                logging.error(resp)
                # abort video creation
                video_complete = True
            else:
                logging.info(resp)
                # check if creation is completed
                if 'inactive' in resp:
                    video_complete = True
        # run video conversion
        _cls_convert_video(video_file_path=video_file_path)


def cls_finish_video_creation(cm_runner) -> None:
    """finish video creation

    Close CarMaker and fininsh video creation.

    Argmunents:
        cm_runner: pyBase CarMaker control object.
    """
    # close IPG Movie
    cm_runner.Close_IPGMovie()

    # close CarMaker
    if sys.platform == 'linux':
        os.system("pkill HIL.exe")
        os.system("pkill Movie.exe")
    else:
        os.system("taskkill /im HIL.exe /f >nul 2>&1")
        os.system("taskkill /im Movie.exe /f >nul 2>&1")


# ############################################################
# ################### CLEANUP FUNCTIONS ######################
# ############################################################


def cls_cleanup_carmaker_project(carmaker_project: str) -> None:
    """clean up container for next execution

    Remove all intermediate files and folder to allow a clean usage
    of the container for simulation or video creation.
    Container specific intermediate files shall be removed in the
    run_simulation.py.

    Arguments:
        carmaker_project: Path to the CarMaker project folder.
    """
    logging.info('Cleaning container')

    cam_cfg_file = os.path.join(carmaker_project, 'Movie', 'Camera.cfg')
    # remove existing camera configuration
    if os.path.isfile(cam_cfg_file):
        logging.info('Removing camera configuration %s', cam_cfg_file)
        os.remove(cam_cfg_file)
    # remove backup file
    if os.path.isfile(cam_cfg_file + '.autobackup'):
        os.remove(cam_cfg_file + '.autobackup')
    # remove road cache folder
    folder_loc = os.path.join(carmaker_project, 'Movie', '.road_cache')
    if os.path.exists(folder_loc):
        logging.info('Removing road cache folder %s', folder_loc)
        shutil.rmtree(folder_loc,
                      ignore_errors=False,
                      onerror=None)
    # remove TESTRUN folder
    folder_loc = os.path.join(carmaker_project, 'Data', 'TestRun')
    if os.path.exists(folder_loc):
        logging.info('Removing TESTRUN folder %s', folder_loc)
        shutil.rmtree(folder_loc,
                      ignore_errors=False,
                      onerror=None)
    # remove ROAD folder
    folder_loc = os.path.join(carmaker_project, 'Data', 'Road')
    if os.path.exists(folder_loc):
        logging.info('Removing ROAD folder %s', folder_loc)
        shutil.rmtree(folder_loc,
                      ignore_errors=False,
                      onerror=None)
    # remove SIMOUTPUT folder
    folder_loc = os.path.join(carmaker_project, 'SimOutput')
    if os.path.exists(folder_loc):
        logging.info('Removing SIMOUTPUT folder %s', folder_loc)
        shutil.rmtree(folder_loc,
                      ignore_errors=False,
                      onerror=None)
    # remove VIDEO folder
    folder_loc = os.path.join(carmaker_project, 'Video')
    if os.path.exists(folder_loc):
        logging.info('Removing VIDEO folder %s', folder_loc)
        shutil.rmtree(folder_loc,
                      ignore_errors=False,
                      onerror=None)
    # remove missing quantities file
    cm_prj, _ = cls_get_carmaker_project()
    # check for MISSING_QUANTITIES.txt from video creation
    miss_quants = os.path.join(cm_prj, 'MISSING_QUANTITIES.txt')
    if os.path.isfile(miss_quants):
        logging.info('Removing MISSING_QUANTITIES.txt file')
        os.remove(miss_quants)
