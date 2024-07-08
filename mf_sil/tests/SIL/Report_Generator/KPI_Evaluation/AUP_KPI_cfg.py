
import os
import xlrd
import sys
import datetime
import shutil

# Generic Imports
import logging
import time
import math
import numpy as np
import subprocess
from pathlib import Path
from collections import OrderedDict, defaultdict, namedtuple
from STET import run_evaluation, read_testcases, read_keywords
from time import gmtime, strftime
from lxml import etree as ET
from math import sin, cos
from distutils.dir_util import copy_tree

#TAPOSD
import shapely
from shapely.geometry import box, Polygon, LineString, Point
#VEDODO
from mfile import BSIG, MDF, ERG, Measurement
from evaluate.core import Signal, Testcase, Verdict, EvaluateError
from STET.stet import load_data
from asammdf import MDF, Signal, MDF4
from mfile import ERG
#import for multiple threading implementation
import multiprocessing
from multiprocessing import Pool

# ParkSM Screen HeadUnit (SHU)
PARKSM_SHU_SystemNotActive      = 0
PARKSM_SHU_ScanningActive       = 1
PARKSM_SHU_ParkSpaceSelection   = 2
PARKSM_SHU_RemoteAppActive      = 3
PARKSM_SHU_ManeuverActive       = 4
PARKSM_SHU_ManeuverFinished     = 5
PARKSM_SHU_ManeuverInterrupted  = 6
PARKSM_SHU_UndoManeuverActive   = 7
PARKSM_SHU_StartRemoteApp       = 8
PARKSM_SHU_ParkOutInit          = 9
PARKSM_SHU_ParkOutSide          = 10
PARKSM_SHU_MenuRem              = 11
PARKSM_SHU_ManeuverActiveLong   = 12
PARKSM_SHU_RemSV                = 13
PARKSM_SHU_RemMan               = 14
PARKSM_SHU_RemManActive         = 15

#====================================================================
# Exit Error Codes        /  bit position 
#====================================================================
NO_ERROR                   = 0
CM_CONNECTION_ERROR        = 0 #'CMRC: Could not connect to running CarMaker instance at '
CM_NO_INSTANCE_RUNNING     = 1 #'CMRC: No CarMaker instance runnning'
CM_GUI_EXE_ERROR           = 2 #'CMRC: Could not find any CarMaker GUI Executable'
CM_NO_STATUS_RECEIVED      = 3 #'CMRC: CarMaker Status was not received within maximum wait time'
CM_SAFE_MODE_NOT_SUPPORTED = 4 #'CMRC: CarMaker SafeMode not supported'
CM_SIMSPEED_NOT_SUPPORTED  = 5 #'CMRC: CarMaker SimSpeed not supported'
CM_ERROR                   = 6 #'CMRC: CarMaker Error: '
SIMULATION_TIME_EXCEEDED   = 7 #'Simulation Time longer than expected! See SIMTIMEMAX_S!'
EXECUTION_TIME_EXCEEDED    = 8 #'Execution Time longer than expected!'
QUANTITY_ERROR             = 9 #'Error while receiving quantity/ Could not subscribe to Quantity'
TEST_SCENARIO_ERROR        = 10
#Error Code for test evaluation dropped
EVALUATION_DROPPED         = 11

EVALPATH = os.path.dirname(__file__)

PATH_selectedVariant       = os.path.abspath(os.path.join(EVALPATH,'..','..', '..','..', 'scripts', 'selected_variant.txt'))

# Check, if selected_variant.txt file exists. In case file is not available, use generic.
if os.path.isfile(PATH_selectedVariant):
    variant = open(PATH_selectedVariant,'r')
    selectedVariant = variant.readline().strip()
    variant.close()
else:
    selectedVariant = "base"

if selectedVariant == 'entry': 
    PATH_NoOfStrokes_TRJPLA    = os.path.join(EVALPATH,"..", 'NoOfStrokes_TRJPLA_CUS.txt')
elif selectedVariant == 'performance': 
    PATH_NoOfStrokes_TRJPLA    = os.path.join(EVALPATH,"..", 'NoOfStrokes_TRJPLA_Performance.txt')
else:
    PATH_NoOfStrokes_TRJPLA    = os.path.join(EVALPATH,"..", 'NoOfStrokes_TRJPLA.txt')
  

PATH_import_signal_list    = os.path.abspath(os.path.join(EVALPATH, "..", "ImportList.txt"))
PATH_import_param_list    = os.path.abspath(os.path.join(EVALPATH,  "..", "ImportParametersList_AUP.txt"))

'''Read all parameters from txt file and save the values inside a dictionary
    keyword - parameter name
    value - parameter value
'''
AUPParameters_Import = {}
'''Read all signals from txt file and save the signals in AUPQuantities_Import list'''
AUPQuantities_Import = []
'''Save all signals inside a dictionary with keyword - architecture name, value - SW name'''
AUPQuantities_Import_SW_Arch = {}
'''structures - dictionary with keyword: structure name in the import list. The signals inside needs to be multiplied
                             value: maximum index number '''
TRJPLA_PLANNED_TRAJ                  = 10
TAPOSD_STATIC_STRUCT                 = 8
structures = {"envModelPort":TAPOSD_STATIC_STRUCT, "TRJPLA":TRJPLA_PLANNED_TRAJ}

with open(PATH_import_signal_list) as import_list:
    '''signals inside import_list can be found: <SW_signal_name> [architecture_name]
    <SW_signal_name> - signal as it is found in the SW
    [architecture_name] - name which should be used in the test specification; optional
    key words of the dictionary structures are searched inside [architecture_name]'''
    for line in import_list:
        if len(line) > 0:
            '''if it is not an empty line, add the <SW_signal_name> in the list'''
            AUPQuantities_Import.append(line.rstrip("\n").split()[0])
            ''' if [architecture_name] exists, use the dictionary to search if the signals should be multiplied'''
            if len(line.split())>1:
                AUPQuantities_Import_SW_Arch[line.split()[1].rstrip("\n")] = line.split()[0]
                for struct in structures.keys():
                    if struct in line.split()[1]:
                        for i in range(1,structures[struct]):
                            SW_signal = line.rstrip("\n").split()[0].replace("_0.","_"+str(i)+".",1)
                            Arch_signal = line.rstrip("\n").split()[1].replace("_0_","_"+str(i)+"_",1)
                            AUPQuantities_Import.append(SW_signal)
                            AUPQuantities_Import_SW_Arch[Arch_signal] = SW_signal

'''create the dictionary with the parameters name/value'''
with open(PATH_import_param_list) as import_param:
    for line in import_param:
        if len(line) > 0:
            #check if the parameters value is not an array
            if '[' not in line.rstrip("\n").split()[1]:
                AUPParameters_Import[line.split()[0]] = float(line.rstrip("\n").split()[1])