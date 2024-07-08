#********************************************************************
# System Imports
#********************************************************************
import sys
import imp
imp.reload(sys)
# sys.setdefaultencoding('utf8')
import WHPExcelReader
import math
from collections import namedtuple
import scipy.interpolate
#import numpy as np
import re
import copy
from pyBase_path_cfg import *

#********************************************************************
# Constants
#********************************************************************
MPS_TO_KPH              = 3.6 # conversion factor from m/s to km/h
KPH_TO_MPS              = 1 / 3.6 # conversion factor from km/h to m/s
S_TO_MS                 = 1000 # conversion factor from s to ms

# Colors
CM_COLOR_GREY           = [0.5, 0.5, 0.5]
CM_COLOR_WHITE          = [1.0, 1.0, 1.0]
CM_COLOR_RED            = [1.0, 0.0, 0.0]
CM_COLOR_BROWN          = [0.65, 0.165, 0.165]
CM_COLOR_YELLOW         = [1.0, 1.0, 0.0]

# 
CM_ACC_DIST_TH          = 0.5  # percentage of traveled distance with Car.ax >= in Forwards and Backwards maneuvers
CM_MAX_BRK_DIST         = 5    # m TODO: verify maximum distance required to stop vehicle from ~27km/h
CM_LANE_WIDTH           = 3.5  # m TODO: lane width to be read from test catalogue in case it becomes variable parameter
CM_CIRC_OBST_NUM_SEG    = 20   # number of segments considered for a circular obstacle contour (set as divisor of 180°)
CM_HMI_BTN_DURATION     = 0.1  # s duration for maintaining HMI button pressed

# Interpolation tables for determining steering wheel angle based on driving wheel angle
# driving wheel angle in °
# steering wheel angle in rad
CM_INNER_WHEEL_INTERP_TABLE = [[0, 1,     2,     3,     4,     5,     10,    15,    20,    25,    30,    35,    38.73], \
                               [0, 0.272, 0.544, 0.806, 1.057, 1.303, 2.576, 3.821, 4.988, 6.091, 7.043, 7.905, 8.527]]
CM_OUTER_WHEEL_INTERP_TABLE = [[0, 1,     2,     3,     4,     5,     10,    15,    20,    25,    30,    31.37], \
                               [0, 0.292, 0.572, 0.845, 1.109, 1.369, 2.700, 4.085, 5.429, 6.759, 8.143, 8.527]]
yInterpInnerWheel   = scipy.interpolate.interp1d(CM_INNER_WHEEL_INTERP_TABLE[0], CM_INNER_WHEEL_INTERP_TABLE[1])
yInterpOuterWheel   = scipy.interpolate.interp1d(CM_OUTER_WHEEL_INTERP_TABLE[0], CM_OUTER_WHEEL_INTERP_TABLE[1])

# Interpolation tables for determining stopping distance based on ego vehicle velocity
CM_EGO_VEHICLE_VELOCITY     = [1,     2,     3,     4,     5,     6,     7,     8,     9,     9.4,   9.5,   10,    11,    12,    13,    14,     15,     16,     17,     18,     19,     20] # km/h
'''
CM_STOP_DIST_FWD_1MPS2      = [0.107, 0.285, 0.548, 0.880, 1.287, 1.770, 2.318, 2.945, 3.647, 3.946, 5.521, 6.006, 7.137, 8.225, 9.276, 10.512, 11.813, 13.482, 14.647, 16.249, 17.980, 19.546] # m, distance required for stopping the forwards driving ego vehicle with a deceleration of 1 m/s²
CM_STOP_DIST_FWD_2MPS2      = [0.084, 0.207, 0.364, 0.551, 0.783, 1.053, 1.347, 1.687, 2.063, 2.222, 3.687, 3.979, 4.587, 5.230, 5.913, 6.638,  7.391,  8.292,  8.998,  9.912,  10.811, 11.758] # m, distance required for stopping the forwards driving ego vehicle with a deceleration of 2 m/s²
CM_STOP_DIST_FWD_3MPS2      = [0.075, 0.181, 0.307, 0.451, 0.619, 0.813, 1.030, 1.282, 1.553, 1.665, 3.084, 3.312, 3.787, 4.284, 4.807, 5.353,  5.920,  6.576,  7.129,  7.803,  8.476,  9.166] # m, distance required for stopping the forwards driving ego vehicle with a deceleration of 3 m/s²
CM_STOP_DIST_FWD_4MPS2      = [0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 2.776, 2.972, 3.378, 3.802, 4.242, 4.703,  5.184,  5.693,  6.158,  6.726,  7.286,  7.858] # m, distance required for stopping the forwards driving ego vehicle with a deceleration of 4 m/s²
CM_STOP_DIST_BWD_1MPS2      = [0.000, 0.177, 0.388, 0.666, 1.023, 1.456, 1.967, 2.555, 3.219, 3.509, 3.583, 3.967, 6.638, 7.696, 8.955, 10.057, 11.352, 12.843, 14.193, 15.719, 17.435, 18.791] # m, distance required for stopping the backwards driving ego vehicle with a deceleration of 1 m/s²
CM_STOP_DIST_BWD_2MPS2      = [0.000, 0.130, 0.243, 0.394, 0.593, 0.821, 1.088, 1.397, 1.744, 1.894, 1.932, 2.129, 4.312, 4.933, 5.600, 6.304,  7.046,  7.827,  8.655,  9.514,  10.445, 11.215] # m, distance required for stopping the backwards driving ego vehicle with a deceleration of 2 m/s²
CM_STOP_DIST_BWD_3MPS2      = [0.000, 0.115, 0.204, 0.310, 0.442, 0.606, 0.800, 1.016, 1.252, 1.356, 1.382, 1.517, 3.585, 4.066, 4.573, 5.110,  5.674,  6.262,  6.875,  7.515,  8.184,  8.757] # m, distance required for stopping the backwards driving ego vehicle with a deceleration of 3 m/s²
CM_STOP_DIST_BWD_4MPS2      = [0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 1.114, 1.220, 3.222, 3.630, 4.063, 4.513,  4.984,  5.477,  5.999,  6.531,  7.058,  7.553] # m, distance required for stopping the backwards driving ego vehicle with a deceleration of 4 m/s²
'''
CM_DECELERATION             = [1,     1,     1,     2,     2,     2,     2,     3,     3,     3,     4,     4,     4,     4,     4,     4,      4,      4,      4,      4,      4,      4] # m/s^2, decelaration for the vehicle stopping maneuvers
CM_STOP_DIST_FWD            = [0.107, 0.285, 0.548, 0.551, 0.783, 1.053, 1.347, 1.282, 1.553, 1.665, 2.776, 2.972, 3.378, 3.802, 4.242, 4.703,  5.184,  5.693,  6.158,  6.726,  7.286,  7.858] # m, distance required for stopping the forwards driving ego vehicle
CM_STOP_DIST_BWD            = [0.000, 0.177, 0.388, 0.394, 0.593, 0.821, 1.088, 1.016, 1.252, 1.356, 1.114, 1.220, 3.222, 3.630, 4.063, 4.513,  4.984,  5.477,  5.999,  6.531,  7.058,  7.553] # m, distance required for stopping the backwards driving ego vehicle
yInterpDeceleration         = scipy.interpolate.interp1d(CM_EGO_VEHICLE_VELOCITY, CM_DECELERATION, fill_value='extrapolate')
yInterpStopDistFwd          = scipy.interpolate.interp1d(CM_EGO_VEHICLE_VELOCITY, CM_STOP_DIST_FWD, fill_value='extrapolate')
yInterpStopDistBwd          = scipy.interpolate.interp1d(CM_EGO_VEHICLE_VELOCITY, CM_STOP_DIST_BWD, fill_value='extrapolate')

# Parking slots
CM_NO_PARK_SLOTS        = 5     # number of available parking slots on each side
CM_PARK_AREA_START_X    = 50    # m longitudinal offset for the start of parking area
CM_PARK_AREA_START_Y    = -3.5  # m lateral offset for the start of parking area
CM_PARK_PERP_H          = 2.75  # m horizontal side of perpendicular parking slot
CM_PARK_PERP_V          = 6     # m vertical side of perpendicular parking slot

# CarMaker maneuver commands
ManCmdEnableVEDODO    = "\t# Enable VEDODO" + \
                      "\n\tEval first() ? AP.vedodoActive_nu = ($OdometryActive=1)"
ManCmdConfigureLSCA   = "\t# Configure LSCA" + \
                      "\n\tEval first() ? AP.lscaDisabled_nu = ($LscaDisabled=0)" + \
                      "\n\tEval first() ? AP.lscaBrakeDisabled_nu = ($LscaBrakeDisabled=1)" + \
                      "\n\tEval first() ? AP.lscaSteeringProposalDisabled_nu = ($LscaSteeringDisabled=1)" + \
                      "\n\tEval first() ? AP.lscaVirtualWallDisabled_nu = ($LscaVirtualWallDisabled=1)"
ManCmdEnableSI        = "\t# Enable Scene Interpretation" + \
                      "\n\tEval first() ? AP.sceneInterpretationActive_nu=($SceneInterpretationActive=1)"
ManCmdEnableLimitFOV  = "\t# Enable Field of view limit" + \
                      "\n\tEval first() ? AP.enableLimitFieldOfView_nu = ($LimitFieldOfViewEnabled=1)"
ManCmdEnableLatency   = "\t# Enable Latency effect" + \
                      "\n\tEval first() ? AP.enableLatencyEffect_nu = ($LatencyEffectEnabled=1)" + \
                      "\n\tEval first() ? AP.latencyEffectTime_s = ($LatencyEffectTime_s=0.1)"
ManCmdDisableLatency  = "\t# Disable Latency effect" + \
                      "\n\tEval first() ? AP.enableLatencyEffect_nu = ($LatencyEffectEnabled=0)"
ManCmdEnableWHP       = "\t# Enable Wheel Protection" + \
                      "\n\tEval first() ? AP.whpDisabled_nu=($WhpEnabled=0)"
ManCmdDisableWHP      = "\t# Disable Wheel Protection" + \
                      "\n\tEval first() ? AP.whpDisabled_nu=($WhpDisabled=1)"  
ManCmdStopTRTime      = "\t# Stop testrun after xx seconds (default: 200)" + \
                      "\n\tEval (Time > 200) ? ManJump(\"END\")"
ManCmdShiftInR        = "\t# Shift in Reverse" + \
                      "\n\tEval DM.ManTime > 0.001 ? DM.SelectorCtrl = -1"
ManCmdEnableAP        = "\t# HMI User input - Toggle AP active" + \
                      "\n\tEval first() ? AP.hmiOutputPort.userActionHeadUnit_nu = 0" + \
                      "\n\tEval DM.ManTime > 0.2 ? AP.hmiOutputPort.userActionHeadUnit_nu = 28" + \
                      "\n\tEval DM.ManTime > 0.3 ? AP.hmiOutputPort.userActionHeadUnit_nu = 0"
ManCmdStartSelection  = "\t# HMI User input - Start selection" + \
                      "\n\tEval DM.ManTime > 0.1 ? AP.hmiOutputPort.userActionHeadUnit_nu = 17" + \
                      "\n\tEval DM.ManTime > 0.3 ? AP.hmiOutputPort.userActionHeadUnit_nu = 0"
ManCmdStartParking    = "\t# HMI User input - Start parking" + \
                      "\n\tEval DM.ManTime > 0.9 ? AP.hmiOutputPort.userActionHeadUnit_nu = 18" + \
                      "\n\tEval DM.ManTime > 1.1 ? AP.hmiOutputPort.userActionHeadUnit_nu = 0"
ManCmdStopTRNoSteer   = "\t# Stop testrun if no steering angle was requested in first xx seconds (default: 10)" + \
                      "\n\tEval first() ? Qu::maneuveringFlag_nu = 0" + \
                      "\n\tEval ((abs(AP.steerCtrlRequestPort.steerAngReq_rad) > 0.1) || (abs(Vhcl.v) > 0.01)) ? Qu::maneuveringFlag_nu = 1" + \
                      "\n\tEval (DM.ManTime > 10) & maneuveringFlag_nu==0 ? ManJump(\"END\")"
ManCmdResetSteerWhl   = "\t# Reset steering wheel angle position" + \
                      "\n\tDVAwr DM.Steer.Ang Abs -1 0"
ManCmdDisableIPGLat   = "\t# Disable lateral control of the IPG Driver" + \
                      "\n\tDVAwr Driver.Lat.passive Abs -1 1"


#********************************************************************
# Configuration tables
#********************************************************************
# maneuverParamConfigType is a named tuple containing for each keyword:
# manFunc = name of maneuver fill function
# manNoArgs = expected number of arguments for maneuver
# maneuverFillFunc is a dictionary mapping maneuver keywords to maneuver fill functions
maneuverParamConfigType = namedtuple("maneuverParamConfigType", "manFunc manNoArgs")

maneuverFillFunc = {                           # manFunc                  manNoArgs
    "Forwards"     : maneuverParamConfigType("self.fillManeuverForwards",     2),
    "Backwards"    : maneuverParamConfigType("self.fillManeuverBackwards",    2),
    "Standstill"   : maneuverParamConfigType("self.fillManeuverStandstill",   1),
    "StartAP"      : maneuverParamConfigType("self.fillManeuverStartAP",      0),
    "MM_Steer_FL"  : maneuverParamConfigType("self.fillManeuverCmdSteerFL",   3),
    "MM_Steer_FR"  : maneuverParamConfigType("self.fillManeuverCmdSteerFR",   3),
    "MM_HMI_t"     : maneuverParamConfigType("self.fillManeuverCmdHMITime",   2),
    "MM_HMI_d"     : maneuverParamConfigType("self.fillManeuverCmdHMIDist",   2),
    "MM_Gear"      : maneuverParamConfigType("self.fillManeuverCmdGear",      2),
    "MM_EPB"       : maneuverParamConfigType("self.fillManeuverCmdEPB",       2),
    "MM_Speed_t"   : maneuverParamConfigType("self.fillManeuverCmdSpeedTime", 2),
    "MM_Speed_d"   : maneuverParamConfigType("self.fillManeuverCmdSpeedDist", 2),
    "MM_Failure"   : maneuverParamConfigType("self.fillManeuverCmdFailure",   2)
    }

# obstacleParamConfigType is a named tuple containing for each keyword:
# obstFunc = name of obstacle fill function
# obstNoArgs = expected number of arguments for obstacle
# obstacleFillFunc is a dictionary mapping obstacle keywords to obstacle fill functions
obstacleParamConfigType = namedtuple("obstacleParamConfigType", "obstFunc obstNoArgs")

obstacleFillFunc = {
    "Box"       : obstacleParamConfigType("self.fillObstacleBox",           7),
    "Cylinder"  : obstacleParamConfigType("self.fillObstacleCylinder",      5),
    "Vehicle"   : obstacleParamConfigType("self.fillObstacleVehicle",       5),
    "Furniture" : obstacleParamConfigType("self.fillObstacleFurniture",     5)
    }

# roadParamConfigType is a named tuple containing for each keyword:
# roadFile = name of the road file
# latH = size for horizontal side of parking slot (CarMaker longitudinal)
# latV = size for vertical side of parking slot (CarMaker lateral)    
roadParamConfigType = namedtuple("roadParamConfigType", "roadFile latH latV")
    
roadParam = {                                   # roadFile              latH             latV
    "Perpendicular" : roadParamConfigType("PDW_Perpendicular.rd5", CM_PARK_PERP_H,  CM_PARK_PERP_V)
}

refPointsCoordsType = namedtuple("refPointsCoordsType", "x y")

class WHPTestCase():
    def __init__(self, data, xlsReader):
        self.data = data
        self.HMIButtons = xlsReader.HMIButtons
        self.Vehicles = xlsReader.Vehicles
        self.StrFurniture = xlsReader.StrFurniture
        self.Parameters = copy.deepcopy(xlsReader.Parameters) # because we need to alter the parameters dict only for this WHPTestCase, but keep a clean copy in xlsReader

        self.refPointsList = ["road"]
        for i in range (1, 2 * CM_NO_PARK_SLOTS + 1):
            self.refPointsList.append("P" + str(i))
            
        self.refPointsCoords = dict.fromkeys(self.refPointsList, refPointsCoordsType(0, 0))
        

    def WHPTestRunFactory(self, myTestRun):
        # TestRun description
        myTestRun.Description.value = self.data["Description"] #+ self.getEvaluationCriteria(myTestRun)
        
        # Overwrite changed parameters
        # self.processChangedParams() #ClPr: disabled for now; will add it later if needed
        
        # Ego vehicle data
        self.fillEgoData(myTestRun)
        
        # Road data
        self.fillRoadData(myTestRun)
        
        # # Parking box
        # # select parking box slot (1 of 5 - "radio button") or select parking box slots (1 to 5 - "checkbox")
        # self.fillParkingBoxData(myTestRun) #TODO: update excel and update script for fake EM generation
        
        # Obstacles
        self.fillObstacleData(myTestRun)
            
        # Maneuvers
        self.fillManeuverData(myTestRun)

    def processChangedParams(self):
        for line in self.data["Changed parameters"].splitlines():
            if line in ["None", ""]:
                continue
            line = line.replace(" ","") # remove blanks from cell
            lineSplitted = line.split("=")
            if len(lineSplitted) != 2:
                print("Test scenario {}: Unexpected format for changed parameter {}.".format(self.data["Situation ID"], line))
                sys.exit(0)            
            try:
                name = lineSplitted[0]
                val = float(lineSplitted[1])
            except ValueError:
                print("Test scenario {}: Unexpected value for changed parameter {}.".format(self.data["Situation ID"], line))
                sys.exit(0)
            if name in self.Parameters.keys():
                factor = self.Parameters[name].factor
            else:
                factor = 1
            self.Parameters[name] = WHPExcelReader.whpParamConfigType(val, factor)
            self.setParamValueInSWBuild(name, lineSplitted[1])

    def setParamValueInSWBuild(self, paramName, newValue):
        reParamLine = r"\"" + paramName + "\":\s*(?P<paramVal>[\d.-]*)" # json format for one parameter is "PARAM_NAME": value (create named group for value)
        replaceLine = "\"" + paramName + "\": " + newValue
        newJsonText = ""
        regexp = re.compile(reParamLine)
        for path in MOD_PARAM_FILES_LIST:
            with open(path, 'r+') as jsonFile:
                jsonText = jsonFile.read()
                newJsonText = regexp.sub(replaceLine, jsonText)
                if newJsonText:
                    break
        if newJsonText:
            with open(path, 'w') as jsonFile:
                jsonFile.write(newJsonText)
            #print("Parameter {} was modified in {}".format(paramName, path))
        else:
            print("Parameter {} from Parameters sheet not found in SW build. Please check if name is correct or if list of json files from pyBase_path_cfg.py must be extended.".format(paramName))
            sys.exit(0)

    def fillEgoData(self, myTestRun):
        egoType = "VW_PassatVariant_20TSI_DSG_LLVW_AP_C_GenSteerAng.car"
        myTestRun.Vehicle._srcFile = egoType
        
        # Tires
        myTestRun.Tire._srcFile = "Continental_ContiPremiumContact5_215_55R17_94W_MF-Tyre_62"

    def fillRoadData(self, myTestRun):
        cat = self.data["Category"]
        
        if cat in list(roadParam.keys()):
            myTestRun.Road.Definition.DigRoad = roadParam[cat].roadFile
            if self.data["Start Long Offset"] == "None":
                self.data["Start Long Offset"] = "0"
            if self.data["Start Lat Offset"] == "None":
                self.data["Start Lat Offset"] = "0"
            if self.data["Start Angle"] == "None":
                self.data["Start Angle"] = "0"
            myTestRun.Road.VhclStartPos.value = self.data["Start Long Offset"] + " " + self.data["Start Lat Offset"] + " " + self.data["Start Angle"]
            
            if cat in ["Perpendicular"]:
                # P1 is set in the middle of the horizontal far side of parking slot
                x_temp = CM_PARK_AREA_START_X + CM_PARK_PERP_H / 2
                y_temp = CM_PARK_AREA_START_Y - CM_PARK_PERP_V
            
            self.refPointsCoords["P1"] = refPointsCoordsType(x_temp, y_temp)
            
            for i in range(1, 6):
                x_temp = self.refPointsCoords["P1"].x + (i - 1) * roadParam[cat].latH
                y_temp = self.refPointsCoords["P1"].y
                self.refPointsCoords["P" + str(i)] = refPointsCoordsType(x_temp, y_temp)
                x_temp = self.refPointsCoords["P" + str(i)].x
                y_temp = -self.refPointsCoords["P" + str(i)].y
                self.refPointsCoords["P" + str(i + 5)] = refPointsCoordsType(x_temp, y_temp)

        else:
            print("Test scenario {}: Road file not available for category {}.".format(self.data["Situation ID"], cat))
            sys.exit(0)

        
    def fillObstacleData(self, myTestRun):
        if self.data["Obstacles"] != "None":
            index = -1
            reDelimiters = '|'.join(map(re.escape, ["(", ")", ","]))
            reArithmeticOps = '|'.join(map(re.escape, ["+", "-", "*", "/"]))
            
            for line in self.data["Obstacles"].splitlines():
                line = line.replace(" ","") # remove blanks from cell
                if line.count('(') != 1 or line.count(')') != 1 or not line.endswith(')'): # verify if a line is in the correct format Keyword(args)\n
                    print("Test scenario {}: Obstacle syntax error in line {}\nCorrect syntax is ObstacleKeyword(Args)'\\n'".format(self.data["Situation ID"], line))
                    sys.exit(0)
                obstArgs = re.split(reDelimiters,line) # split line with separators: '(', ')', ','
                del obstArgs[-1] # remove last element from list - always '' because last character in line is ')'
                
                if not obstArgs[0] in obstacleFillFunc.keys():
                    print("Test scenario {}: Keyword {} not handled.".format(self.data["Situation ID"], obstArgs[0]))
                    sys.exit(0)
                
                if len(obstArgs[1:]) != obstacleFillFunc[obstArgs[0]].obstNoArgs:
                    print("Test scenario {}: Incorrect number of arguments for obstacle {}".format(self.data["Situation ID"], line))
                    sys.exit(0)
                
                if not obstArgs[1].upper() in [x.upper() for x in self.refPointsList]:
                    print("Test scenario {}: Incorrect reference point for obstacle {}".format(self.data["Situation ID"], line))
                    sys.exit(0)
                
                for idx, obstArg in enumerate(obstArgs[2:], start=2): # check if any of the obstacle arguments contains a parameter
                    found = False
                    formulaArgs = re.split(reArithmeticOps, obstArg) # argument could be a formula containing a parameter, need to split by arithmetic operators
                    for arg in formulaArgs:
                        if arg in self.Parameters.keys():
                            val = self.Parameters[arg].value * self.Parameters[arg].factor
                            obstArg = obstArg.replace(arg, str(val)) # replace parameter name with its value
                            found = True
                    if found:
                        obstArgs[idx] = str(eval(obstArg)) # evaluate formula if a parameter was found in the obstacle arguments

                index = eval(obstacleFillFunc[obstArgs[0]].obstFunc)(myTestRun, obstArgs[1:], index) # call corresponding obstacle function indicated by the keyword (obstArgs[0]) with arguments (myTestRun, obstArgs[1:], index)


    def getObstStartPos(self, refPoint, longOffset, latOffset):
        if refPoint.lower() == "road":
            return (longOffset, latOffset)
        
        index = int(refPoint[1:])
        sign = 1
        if index > CM_NO_PARK_SLOTS: # lateral offset must be subtracted if parking on the left side of the road
            sign = -1
            
        return(self.refPointsCoords[refPoint].x + longOffset, self.refPointsCoords[refPoint].y + sign * latOffset)


    def fillObstacleBox(self, myTestRun, obstArgs, index):
        index += 1
        myTestRun.Traffic.add()
        myTestRun.Traffic.Traffic[index].Name.value = "Box" + '{:02d}'.format(index)
        myTestRun.Traffic.Traffic[index].Info.value = "Cuboid"
        myTestRun.Traffic.Traffic[index].Movie.Geometry.value = " "
        myTestRun.Traffic.Traffic[index].Color.value = CM_COLOR_GREY
        myTestRun.Traffic.Traffic[index].Basics.Dimension.value = [float(obstArgs[3]), float(obstArgs[4]), float(obstArgs[5])]
        myTestRun.Traffic.Traffic[index].Init.Orientation.value = [0.0, 0.0, float(obstArgs[6])]
        myTestRun.Traffic.Traffic[index].DetectMask.value = "1 1"
        myTestRun.Traffic.Traffic[index].Route.value = "0 1"

        (startX, startY) = self.getObstStartPos(obstArgs[0], float(obstArgs[1]), float(obstArgs[2]))
        myTestRun.Traffic.Traffic[index].Init.Road.value = str(startX) + " " + str(startY + CM_LANE_WIDTH / 2)
        
        return index

    def fillObstacleCylinder(self, myTestRun, obstArgs, index):
        index += 1
        myTestRun.Traffic.add()
        myTestRun.Traffic.Traffic[index].Name.value = "Cyl" + '{:02d}'.format(index)
        myTestRun.Traffic.Traffic[index].Info.value = "Cylinder"
        myTestRun.Traffic.Traffic[index].Movie.Geometry.value = " "
        myTestRun.Traffic.Traffic[index].Color.value = CM_COLOR_GREY
        
        contour = ""
        radius = float(obstArgs[3])
        for i in range(0, 181, int(180 / CM_CIRC_OBST_NUM_SEG)): # for (i = 0°; i <= 180°; i+= segmentSize)
            circlePx = round(radius + radius * math.cos(math.radians(i)), 3)
            if i == 180:
                circlePy = 0 # this is added because sin(180°) != 0 in python (e-16 value)
            else:
                circlePy = round(radius * math.sin(math.radians(i)), 3)
            contour = "\t" + str(circlePx) + " " + str(circlePy) + "\n" + contour
        
        myTestRun.Traffic.Traffic[index].Basics.Dimension.value = [0.01, 0.01, float(obstArgs[4])] # length and width are disregarded because the size is set via countour values; non-zero values must be used to avoid CarMaker error
        myTestRun.Traffic.Traffic[index].Basics.Contour.Contour.value = contour
        myTestRun.Traffic.Traffic[index].Basics.Contour.Mirror.value = 1
        myTestRun.Traffic.Traffic[index].DetectMask.value = "1 1"
        myTestRun.Traffic.Traffic[index].Route.value = "0 1"

        (startX, startY) = self.getObstStartPos(obstArgs[0], float(obstArgs[1]) - radius, float(obstArgs[2]))
        myTestRun.Traffic.Traffic[index].Init.Road.value = str(startX) + " " + str(startY + CM_LANE_WIDTH / 2)
        
        return index


    def fillObstacleVehicle(self, myTestRun, obstArgs, index):
        index += 1
        myTestRun.Traffic.add()
        myTestRun.Traffic.Traffic[index].Name.value = "Vhcl" + '{:02d}'.format(index)
        myTestRun.Traffic.Traffic[index].Info.value = obstArgs[1]
        myTestRun.Traffic.Traffic[index].Movie.Geometry.value = self.Vehicles[obstArgs[1]].movieGeometry
        myTestRun.Traffic.Traffic[index].Basics.Dimension.value = [self.Vehicles[obstArgs[1]].length, self.Vehicles[obstArgs[1]].width, self.Vehicles[obstArgs[1]].height]
        myTestRun.Traffic.Traffic[index].Init.Orientation.value = [0.0, 0.0, float(obstArgs[4])]
        myTestRun.Traffic.Traffic[index].DetectMask.value = "1 1"
        myTestRun.Traffic.Traffic[index].Route.value = "0 1"

        (startX, startY) = self.getObstStartPos(obstArgs[0], float(obstArgs[2]), float(obstArgs[3]))
        myTestRun.Traffic.Traffic[index].Init.Road.value = str(startX) + " " + str(startY + CM_LANE_WIDTH / 2)
        
        return index
        
    def fillObstacleFurniture(self, myTestRun, obstArgs, index):
        index += 1
        myTestRun.Traffic.add()
        myTestRun.Traffic.Traffic[index].Name.value = "Obst" + '{:02d}'.format(index)
        myTestRun.Traffic.Traffic[index].Info.value = obstArgs[1]
        myTestRun.Traffic.Traffic[index].Movie.Geometry.value = self.StrFurniture[obstArgs[1]].movieGeometry
        myTestRun.Traffic.Traffic[index].Basics.Dimension.value = [self.StrFurniture[obstArgs[1]].length, self.StrFurniture[obstArgs[1]].width, self.StrFurniture[obstArgs[1]].height]
        myTestRun.Traffic.Traffic[index].Init.Orientation.value = [0.0, 0.0, float(obstArgs[4])]
        myTestRun.Traffic.Traffic[index].DetectMask.value = "1 1"
        myTestRun.Traffic.Traffic[index].Route.value = "0 1"

        (startX, startY) = self.getObstStartPos(obstArgs[0], float(obstArgs[2]), float(obstArgs[3]))
        myTestRun.Traffic.Traffic[index].Init.Road.value = str(startX) + " " + str(startY + CM_LANE_WIDTH / 2)
        
        return index


    def fillManeuverData(self, myTestRun):
        # Global settings / Preparation
        self.fillManeuverGlobalSettings(myTestRun)
    
        # Testrun Init
        self.fillManeuverInit(myTestRun)

        index = 0
        reDelimiters = '|'.join(map(re.escape, ["(", ")", ","]))
        reArithmeticOps = '|'.join(map(re.escape, ["+", "-", "*", "/"]))
        
        for line in self.data["Maneuvers"].splitlines():
            line = line.replace(" ","") # remove blanks from cell
            if line.count('(') != 1 or line.count(')') != 1 or not line.endswith(')'): # verify if a line is in the correct format Keyword(args)\n
                print("Test scenario {}: Maneuver syntax error in line {}\nCorrect syntax is ManeuverKeyword(Args)'\\n'".format(self.data["Situation ID"], line))
                sys.exit(0)
            manArgs = re.split(reDelimiters,line) # split line with separators: '(', ')', ','
            manArgs = list(filter(None, manArgs)) # remove ALL empty strings, in some cases there are even 2 and 'del manArgs[-1]' is not enough

            if not manArgs[0] in maneuverFillFunc.keys():
                print("Test scenario {}: Keyword {} not handled.".format(self.data["Situation ID"], manArgs[0]))
                sys.exit(0)
            
            if len(manArgs[1:]) != maneuverFillFunc[manArgs[0]].manNoArgs:
                print("Test scenario {}: Incorrect number of arguments for maneuver {}".format(self.data["Situation ID"], line))
                sys.exit(0)

            for idx, manArg in enumerate(manArgs[1:], start=1): # check if any of the manuever arguments contains a parameter
                found = False
                formulaArgs = re.split(reArithmeticOps, manArg) # argument could be a formula containing a parameter, need to split by arithmetic operators
                for arg in formulaArgs:
                    if arg in self.Parameters.keys():
                        val = self.Parameters[arg].value * self.Parameters[arg].factor
                        manArg = manArg.replace(arg, str(val)) # replace parameter name with its value
                        found = True
                if found:
                    manArgs[idx] = str(eval(manArg)) # evaluate formula if a parameter was found in the manuever arguments

            index = eval(maneuverFillFunc[manArgs[0]].manFunc)(myTestRun, manArgs[1:], index) # call corresponding maneuver function indicated by the keyword (manArgs[0]) with arguments (myTestRun, manArgs[1:], index)
            
        self.fillManeuverEnd(myTestRun, index)

    def fillManeuverGlobalSettings(self, myTestRun):
        # this line breaks the test cases, have to comment it
        # myTestRun.DrivMan.Cmds.value = ManCmdStopTRTime
        pass

    def fillManeuverInit(self, myTestRun):
        myTestRun.DrivMan.DrivManList.add()
        myTestRun.DrivMan.DrivManList[0].Info.value = "Init"
        myTestRun.DrivMan.DrivManList[0].Label.value = ""
        myTestRun.DrivMan.DrivManList[0].TimeLimit.value = 0.5
        myTestRun.DrivMan.DrivManList[0].Cmds.value = ManCmdResetSteerWhl + "\n" + \
                                                      ManCmdDisableIPGLat + "\n" + \
                                                      "\t\"\"\n" + \
                                                      ManCmdEnableVEDODO + "\n" + \
                                                      ManCmdConfigureLSCA + "\n" + \
                                                      ManCmdEnableLimitFOV + "\n" + \
                                                      ManCmdEnableLatency + "\n" + \
                                                      ManCmdEnableSI + "\n" + \
                                                      ManCmdEnableWHP + "\n" + \
                                                      "\t\"\"\n" + \
                                                      ManCmdEnableAP
        myTestRun.DrivMan.DrivManList[0].LongDyn.value = "Manual"
        myTestRun.DrivMan.DrivManList[0].Clutch.value = "0 0 0.2 abs"
        myTestRun.DrivMan.DrivManList[0].Gas.value = "0 0 0.2 abs"
        myTestRun.DrivMan.DrivManList[0].Brake.value = "1 0 0.2 abs"
        myTestRun.DrivMan.DrivManList[0].BrakePark.value = "0 0 0.2 abs"

    def fillManeuverForwards(self, myTestRun, manArgs, index):
        # Speed Control = VelControl 'Speed' 'Max. Deviation' 'Sensitivity' 'Premature end when final speed is reached' 'Manual Gear Shifting' 'Manumatic'
        # Stop Vehicle = Stop 'Deceleration' 'Stop Distance'
        
        index += 1
        myTestRun.DrivMan.DrivManList.add()
        myTestRun.DrivMan.DrivManList[index].Info.value = "Forwards (Car.ax >= 0)"
        myTestRun.DrivMan.DrivManList[index].LongDyn.value = "VelControl " + manArgs[1] + " 0.0 1.0 0 1 0"
        
        dist = float(manArgs[0]) - yInterpStopDistFwd(float(manArgs[1]))
        dist_sRoad = round(dist * math.cos(math.radians(float(self.data["Start Angle"]))), 3)
        
        myTestRun.DrivMan.DrivManList[index].Cmds.value = "\t# Driving forwards with constant velocity complete" + \
                                                          "\n\tEval first() ? Qu::StartPos = Vhcl.sRoad" + \
                                                          "\n\tEval first() ? Qu::ManDuration = DM.ManTime" + \
                                                          "\n\tEval first() ? Qu::ManDistance = 0" + \
                                                          "\n\tEval Vhcl.sRoad >= StartPos + " + str(dist_sRoad) + " ? (ManDuration = DM.ManTime; ManDistance = Vhcl.sRoad - StartPos; ManJump(\"+1\"))" 
        
        index += 1
        myTestRun.DrivMan.DrivManList.add()
        myTestRun.DrivMan.DrivManList[index].Info.value = "Forwards (Car.ax <= 0)"
        myTestRun.DrivMan.DrivManList[index].LongDyn.value = "Stop " + str(yInterpDeceleration(float(manArgs[1]))) + " 0"
        myTestRun.DrivMan.DrivManList[index].Cmds.value = "\t# Driving forwards phase complete" + \
                                                          "\n\tEval Car.v < 0.01 ? ManJump(\"+1\")"
        
        return index

    def fillManeuverBackwards(self, myTestRun, manArgs, index):
        # Drive backwards = Backward 'Acceleration' 'Speed'
        # Stop Vehicle = Stop 'Deceleration' 'Stop Distance'
        
        index += 1
        myTestRun.DrivMan.DrivManList.add()
        myTestRun.DrivMan.DrivManList[index].Info.value = "Backwards (Car.ax >= 0)"
        myTestRun.DrivMan.DrivManList[index].LongDyn.value = "Backward 2 " + manArgs[1]
        
        dist = float(manArgs[0]) - yInterpStopDistBwd(float(manArgs[1]))    
        dist_sRoad = round(dist * math.cos(math.radians(float(self.data["Start Angle"]))), 3)
        
        myTestRun.DrivMan.DrivManList[index].Cmds.value = "\t# Driving backwards with constant velocity complete" + \
                                                          "\n\tEval first() ? Qu::StartPos = Vhcl.sRoad" + \
                                                          "\n\tEval first() ? Qu::ManDuration = DM.ManTime" + \
                                                          "\n\tEval first() ? Qu::ManDistance = 0" + \
                                                          "\n\tEval Vhcl.sRoad <= StartPos - " + str(dist_sRoad) + " ? (ManDuration = DM.ManTime; ManDistance = StartPos - Vhcl.sRoad; ManJump(\"+1\"))"

        index += 1
        myTestRun.DrivMan.DrivManList.add()
        myTestRun.DrivMan.DrivManList[index].Info.value = "Backwards (Car.ax <= 0)"
        myTestRun.DrivMan.DrivManList[index].LongDyn.value = "Stop " + str(yInterpDeceleration(float(manArgs[1]))) + " 0"
        myTestRun.DrivMan.DrivManList[index].Cmds.value = "\t# Driving backwards phase complete" + \
                                                          "\n\tEval Car.v < 0.01 ? ManJump(\"+1\")"
        
        return index

    def fillManeuverStandstill(self, myTestRun, manArgs, index):
        index += 1
        myTestRun.DrivMan.DrivManList.add()
        myTestRun.DrivMan.DrivManList[index].Info.value = "Standstill"
        myTestRun.DrivMan.DrivManList[index].TimeLimit.value = float(manArgs[0])
        myTestRun.DrivMan.DrivManList[index].LongDyn.value = "Manual"
        myTestRun.DrivMan.DrivManList[index].Clutch.value = "0 0 0.2 abs"
        myTestRun.DrivMan.DrivManList[index].Gas.value = "0 0 0.2 abs"
        myTestRun.DrivMan.DrivManList[index].Brake.value = "1 0 0.2 abs"
        myTestRun.DrivMan.DrivManList[index].BrakePark.value = "0 0 0.2 abs"
        myTestRun.DrivMan.DrivManList[index].Cmds.value = "\t# Hold brake pedal"
        return index
    
    def fillManeuverStartAP(self, myTestRun, manArgs, index):
        index += 1
        myTestRun.DrivMan.DrivManList.add()
        myTestRun.DrivMan.DrivManList[index].Info.value = "Start parking"
        myTestRun.DrivMan.DrivManList[index].TimeLimit.value = 1.4
        myTestRun.DrivMan.DrivManList[index].LongDyn.value = "Manual"
        myTestRun.DrivMan.DrivManList[index].Clutch.value = "0 0 0.2 abs"
        myTestRun.DrivMan.DrivManList[index].Gas.value = "0 0 0.2 abs"
        myTestRun.DrivMan.DrivManList[index].Brake.value = "1 0 0.2 abs"
        myTestRun.DrivMan.DrivManList[index].BrakePark.value = "0 0 0.2 abs"
        myTestRun.DrivMan.DrivManList[index].Cmds.value = ManCmdStopTRTime + "\n" + \
                                                          "\t\"\"" + \
                                                          "\n\tEval first() ? Qu::HMIBtnStart = -1\n" + \
                                                          ManCmdStartSelection + \
                                                          "\t\n" + \
                                                          ManCmdStartParking
        index += 1
        myTestRun.DrivMan.DrivManList.add()
        myTestRun.DrivMan.DrivManList[index].Info.value = "AP control"
        myTestRun.DrivMan.DrivManList[index].EndCondition.value = "AP.headUnitVisualizationPort.screen_nu == 5 || AP.headUnitVisualizationPort.screen_nu == 6"
        myTestRun.DrivMan.DrivManList[index].LongDyn.value = "Manual"
        myTestRun.DrivMan.DrivManList[index].Cmds.value = ManCmdStopTRNoSteer + "\n" + \
                                                          "\t\"\"\n" + \
                                                          ManCmdStopTRTime

        return index
    
    def fillManeuverCmdSteerFL(self, myTestRun, manArgs, index):
        if myTestRun.DrivMan.DrivManList[index].Info.value.find("Standstill") != -1:
            if float(manArgs[1]) >= 0:
                # Turning left => FL wheel is inner wheel
                strWhlAng = yInterpInnerWheel(float(manArgs[1]))
            else:
                # Turning right => FL wheel is outer wheel
                strWhlAng = (-1) * yInterpOuterWheel(abs(float(manArgs[1])))
            myTestRun.DrivMan.DrivManList[index].Cmds.value += "\n\t# Rotate steering wheel" + \
                                                               "\n\t[DM.ManTime >= " + manArgs[0] + "] DVAwr DM.Steer.Ang AbsRamp -1 " + str(strWhlAng) + " " + str(float(manArgs[2]) * S_TO_MS)
        else:
            print("Test scenario {}: MM_Steer_FL is only relevant in Standstill maneuver.".format(self.data["Situation ID"]))
            sys.exit(0)
        return index

    def fillManeuverCmdSteerFR(self, myTestRun, manArgs, index):
        if myTestRun.DrivMan.DrivManList[index].Info.value.find("Standstill") != -1:
            if float(manArgs[1]) >= 0:
                # Turning left => FR wheel is outer wheel
                strWhlAng = yInterpOuterWheel(float(manArgs[1]))
            else:
                # Turning right => FR wheel is inner wheel
                strWhlAng = (-1) * yInterpInnerWheel(abs(float(manArgs[1])))
            myTestRun.DrivMan.DrivManList[index].Cmds.value += "\n\t# Rotate steering wheel" + \
                                                               "\n\t[DM.ManTime >= " + manArgs[0] + "] DVAwr DM.Steer.Ang AbsRamp -1 " + str(strWhlAng) + " " + str(float(manArgs[2]) * S_TO_MS)
        else:
            print("Test scenario {}: MM_Steer_FR is only relevant in Standstill maneuver.".format(self.data["Situation ID"]))
            sys.exit(0)
        return index
    
    def fillManeuverCmdHMITime(self, myTestRun, manArgs, index):
        if myTestRun.DrivMan.DrivManList[index].Info.value in ["Standstill", "AP control"]:
            myTestRun.DrivMan.DrivManList[index].Cmds.value += "\n\tEval first(DM.ManTime >= " + manArgs[0] + ") ? HMIBtnStart = DM.ManTime" + \
                                                               "\n\tEval ((DM.ManTime >= " + manArgs[0] + ") && (DM.ManTime <= HMIBtnStart + " + str(CM_HMI_BTN_DURATION) + ")) ? AP.hmiOutputPort.userActionHeadUnit_nu = ($" + self.HMIButtons[manArgs[1]] + ")" + \
                                                               "\n\tEval first((HMIBtnStart >= 0) && (DM.ManTime > HMIBtnStart + " + str(CM_HMI_BTN_DURATION) + ")) ? HMIBtnStart =  -1" + \
                                                               "\n\tEval (HMIBtnStart == -1) ? AP.hmiOutputPort.userActionHeadUnit_nu = 0"
        else:
            myTestRun.DrivMan.DrivManList[index - 1].Cmds.value += "\n\tEval first(DM.ManTime >= " + manArgs[0] + ") ? HMIBtnStart = DM.ManTime" + \
                                                                   "\n\tEval ((DM.ManTime >= " + manArgs[0] + ") && (DM.ManTime <= HMIBtnStart + " + str(CM_HMI_BTN_DURATION) + ")) ? AP.hmiOutputPort.userActionHeadUnit_nu = ($" + self.HMIButtons[manArgs[1]] + ")" + \
                                                                   "\n\tEval first((HMIBtnStart >= 0) && (DM.ManTime > HMIBtnStart + " + str(CM_HMI_BTN_DURATION) + ")) ? HMIBtnStart =  -1" + \
                                                                   "\n\tEval (HMIBtnStart == -1) ? AP.hmiOutputPort.userActionHeadUnit_nu = 0"
            myTestRun.DrivMan.DrivManList[index].Cmds.value += "\n\tEval first((ManDuration <= " + manArgs[0] + ") && (DM.ManTime + ManDuration >= " + manArgs[0] + ")) ? HMIBtnStart = DM.ManTime + ManDuration" + \
                                                               "\n\tEval ((DM.ManTime + ManDuration >= " + manArgs[0] + ") && (DM.ManTime + ManDuration <= HMIBtnStart + " + str(CM_HMI_BTN_DURATION) + ")) ? AP.hmiOutputPort.userActionHeadUnit_nu = ($" + self.HMIButtons[manArgs[1]] + ")" + \
                                                               "\n\tEval first((HMIBtnStart >= 0) && (DM.ManTime + ManDuration > HMIBtnStart + " + str(CM_HMI_BTN_DURATION) + ")) ? HMIBtnStart =  -1" + \
                                                               "\n\tEval (HMIBtnStart == -1) ? AP.hmiOutputPort.userActionHeadUnit_nu = 0"

        return index
        
    def fillManeuverCmdHMIDist(self, myTestRun, manArgs, index):
        if myTestRun.DrivMan.DrivManList[index].Info.value.find("Forwards") != -1:
            myTestRun.DrivMan.DrivManList[index - 1].Cmds.value += "\n\tEval first(Vhcl.sRoad >= StartPos + " + manArgs[0] + ") ? HMIBtnStart = DM.ManTime" + \
                                                                   "\n\tEval ((Vhcl.sRoad >= StartPos + " + manArgs[0] + ") && (DM.ManTime <= HMIBtnStart + " + str(CM_HMI_BTN_DURATION) + ")) ? AP.hmiOutputPort.userActionHeadUnit_nu = ($" + self.HMIButtons[manArgs[1]] + ")" + \
                                                                   "\n\tEval first((HMIBtnStart >= 0) && (DM.ManTime > HMIBtnStart + " + str(CM_HMI_BTN_DURATION) + ")) ? HMIBtnStart =  -1" + \
                                                                   "\n\tEval (HMIBtnStart == -1) ? AP.hmiOutputPort.userActionHeadUnit_nu = 0"
            myTestRun.DrivMan.DrivManList[index].Cmds.value += "\n\tEval first((ManDistance <= " + manArgs[0] + ") && (Vhcl.sRoad >= StartPos + " + manArgs[0] + ")) ? HMIBtnStart = DM.ManTime + ManDuration" + \
                                                               "\n\tEval ((Vhcl.sRoad >= StartPos + " + manArgs[0] + ") && (DM.ManTime + ManDuration <= HMIBtnStart + " + str(CM_HMI_BTN_DURATION) + ")) ? AP.hmiOutputPort.userActionHeadUnit_nu = ($" + self.HMIButtons[manArgs[1]] + ")" + \
                                                               "\n\tEval first((HMIBtnStart >= 0) && (DM.ManTime + ManDuration > HMIBtnStart + " + str(CM_HMI_BTN_DURATION) + ")) ? HMIBtnStart =  -1" + \
                                                               "\n\tEval (HMIBtnStart == -1) ? AP.hmiOutputPort.userActionHeadUnit_nu = 0"
        elif myTestRun.DrivMan.DrivManList[index].Info.value.find("Backwards") != -1:
            myTestRun.DrivMan.DrivManList[index - 1].Cmds.value += "\n\tEval first(Vhcl.sRoad <= StartPos - " + manArgs[0] + ") ? HMIBtnStart = DM.ManTime" + \
                                                                   "\n\tEval ((Vhcl.sRoad <= StartPos - " + manArgs[0] + ") && (DM.ManTime <= HMIBtnStart + " + str(CM_HMI_BTN_DURATION) + ")) ? AP.hmiOutputPort.userActionHeadUnit_nu = ($" + self.HMIButtons[manArgs[1]] + ")" + \
                                                                   "\n\tEval first((HMIBtnStart >= 0) && (DM.ManTime > HMIBtnStart + " + str(CM_HMI_BTN_DURATION) + ")) ? HMIBtnStart =  -1" + \
                                                                   "\n\tEval (HMIBtnStart == -1) ? AP.hmiOutputPort.userActionHeadUnit_nu = 0"
            myTestRun.DrivMan.DrivManList[index].Cmds.value += "\n\tEval first((ManDistance <= " + manArgs[0] + ") && (Vhcl.sRoad <= StartPos - " + manArgs[0] + ")) ? HMIBtnStart = DM.ManTime + ManDuration" + \
                                                               "\n\tEval ((Vhcl.sRoad <= StartPos - " + manArgs[0] + ") && (DM.ManTime + ManDuration <= HMIBtnStart + " + str(CM_HMI_BTN_DURATION) + ")) ? AP.hmiOutputPort.userActionHeadUnit_nu = ($" + self.HMIButtons[manArgs[1]] + ")" + \
                                                               "\n\tEval first((HMIBtnStart >= 0) && (DM.ManTime + ManDuration > HMIBtnStart + " + str(CM_HMI_BTN_DURATION) + ")) ? HMIBtnStart =  -1" + \
                                                               "\n\tEval (HMIBtnStart == -1) ? AP.hmiOutputPort.userActionHeadUnit_nu = 0"
        else:
            print("Test scenario {}: MM_HMI_d not supported in {} maneuver.".format(self.data["Situation ID"], myTestRun.DrivMan.DrivManList[index].Info.value))
            sys.exit(0)

        return index
    
    def fillManeuverCmdGear(self, myTestRun, manArgs, index):
        if myTestRun.DrivMan.DrivManList[index].Info.value not in ["Standstill"]:
            print("Test scenario {}: MM_Gear not supported in {} maneuver.".format(self.data["Situation ID"], myTestRun.DrivMan.DrivManList[index].Info.value))
            sys.exit(0)
        if manArgs[1].upper() == "P":
            myTestRun.DrivMan.DrivManList[index].Cmds.value += "\n\t# Shift gear in Park" + \
                                                               "\n\tEval DM.ManTime >= " + manArgs[0] + " ? DM.SelectorCtrl = -9"
        elif manArgs[1].upper() == "R":
            myTestRun.DrivMan.DrivManList[index].Cmds.value += "\n\t# Shift gear in Reverse" + \
                                                               "\n\tEval DM.ManTime >= " + manArgs[0] + " ? DM.SelectorCtrl = -1"
        elif manArgs[1].upper() == "N":
            myTestRun.DrivMan.DrivManList[index].Cmds.value += "\n\t# Shift gear in Neutral" + \
                                                               "\n\tEval DM.ManTime >= " + manArgs[0] + " ? DM.SelectorCtrl = 0"
        elif manArgs[1].upper() == "D":
            myTestRun.DrivMan.DrivManList[index].Cmds.value += "\n\t# Shift gear in Drive" + \
                                                               "\n\tEval DM.ManTime >= " + manArgs[0] + " ? DM.SelectorCtrl = 1"
        else:
            print("Test scenario {}: Gear {} not handled.".format(self.data["Situation ID"], manArgs[1]))
            sys.exit(0)
        
        return index
    
    def fillManeuverCmdEPB(self, myTestRun, manArgs, index):
        if myTestRun.DrivMan.DrivManList[index].Info.value.find("Standstill") == -1:
            print("Test scenario {}: MM_EPB not supported in {} maneuver.".format(self.data["Situation ID"], myTestRun.DrivMan.DrivManList[index].Info.value))
            sys.exit(0)

        if manArgs[1].upper() == "ON":
            myTestRun.DrivMan.DrivManList[index].Cmds.value += "\n\t# EPB ON" + \
                                                               "\n\tEval DM.ManTime >= " + manArgs[0] + " ? DM.BrakePark = 1"
        elif manArgs[1].upper() == "OFF":
            myTestRun.DrivMan.DrivManList[index].Cmds.value += "\n\t# EPB OFF" + \
                                                               "\n\tEval DM.ManTime >= " + manArgs[0] + " ? DM.BrakePark = 0"
        else:
            print("Test scenario {}: EPB state {} is invalid.".format(self.data["Situation ID"], manArgs[1]))
            sys.exit(0)
        
        return index
        
    def fillManeuverCmdSpeedTime(self, myTestRun, manArgs, index):
        if (myTestRun.DrivMan.DrivManList[index].Info.value.find("Forwards") == -1) and (myTestRun.DrivMan.DrivManList[index].Info.value.find("Backwards") == -1):
            print("Test scenario {}: MM_Speed_t not relevant in {} maneuver.".format(self.data["Situation ID"], myTestRun.DrivMan.DrivManList[index].Info.value))
            sys.exit(0)

        myTestRun.DrivMan.DrivManList[index - 1].Cmds.value += "\n\t\"\"" + \
                                                               "\n\t# Change constant velocity threshold" + \
                                                               "\n\tEval DM.ManTime >= " + manArgs[0] + " ? DM.v.Trgt = " + str(round(float(manArgs[1]) * KPH_TO_MPS, 3))
        
        return index
        
    def fillManeuverCmdSpeedDist(self, myTestRun, manArgs, index):
        if (myTestRun.DrivMan.DrivManList[index].Info.value.find("Forwards") == -1) and (myTestRun.DrivMan.DrivManList[index].Info.value.find("Backwards") == -1):
            print("Test scenario {}: MM_Speed_d not relevant in {} maneuver.".format(self.data["Situation ID"], myTestRun.DrivMan.DrivManList[index].Info.value))
            sys.exit(0)
            
        if myTestRun.DrivMan.DrivManList[index].Info.value.find("Forwards") != -1:
            myTestRun.DrivMan.DrivManList[index - 1].Cmds.value += "\n\t\"\"" + \
                                                                   "\n\t# Change constant velocity threshold" + \
                                                                   "\n\tEval Vhcl.sRoad >= StartPos + " + manArgs[0] + " ? DM.v.Trgt = " + str(round(float(manArgs[1]) * KPH_TO_MPS, 3))
        else:
            myTestRun.DrivMan.DrivManList[index - 1].Cmds.value += "\n\t\"\"" + \
                                                                   "\n\t# Change constant velocity threshold" + \
                                                                   "\n\tEval Vhcl.sRoad <= StartPos - " + manArgs[0] + " ? DM.v.Trgt = " + str(round(float(manArgs[1]) * KPH_TO_MPS, 3))    
        
        return index
    
    def fillManeuverEnd(self, myTestRun, index):
        index += 1
        myTestRun.DrivMan.DrivManList[index].Info.value = "End testrun"
        myTestRun.DrivMan.DrivManList[index].Label.value = "END"
        myTestRun.DrivMan.DrivManList[index].TimeLimit.value = 0.1
