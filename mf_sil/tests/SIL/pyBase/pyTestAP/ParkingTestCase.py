#********************************************************************
# System Imports
#********************************************************************
import sys
import imp
imp.reload(sys)
# sys.setdefaultencoding('utf8')
import AP_ExcelReader
import math
from collections import namedtuple
import numpy as np

#********************************************************************
# Constants
#********************************************************************
AP_ROAD_LENGTH          = 80 # m
AP_PARK_ADD_WIDTH       = [0, 0.25, 0.25, 0.5] # m, parking area border
AP_PARK_ADD_DEPTH       = 0.25 #m, parking area border

# Minimum parking box tolerances (added to vehicle dimensions) # TODO: check values
AP_G_PAR_SLOT_MIN_OFFSET_L_M         = 0.8
AP_G_PAR_SLOT_MIN_OFFSET_W_M         = 0.4
AP_G_PERP_SLOT_MIN_OFFSET_L_M        = 0.5
AP_G_PERP_SLOT_MIN_OFFSET_W_M        = 0.6
AP_G_PAR_SLOT_OFFSET_EM_L_M          = 0.1
AP_G_PERP_SLOT_OFFSET_EM_W_M         = 0.1
AP_G_DIST_CMF_FRONT_HIGH_OBST_M      = 1
AP_G_DIST_CMF_REAR_HIGH_OBST_M       = 1.5
AP_G_DIST_MIN_LSIDE_HIGH_OBST_PAR_M  = 0.31
AP_G_DIST_CMF_SSIDE_HIGH_OBST_PAR_M  = 1.0
AP_G_DIST_CMF_LSIDE_TRAV_PAR_M       = 0.31
AP_G_DIST_CMF_LSIDE_DOOR_OPEN_M      = 0.31
AP_G_DIST_CMF_LSIDE_HIGH_OBST_M      = 1.0
AP_G_DIST_CMF_PARKSLOT_MARKER_M      = 0.7
AP_G_DIST_CMF_LANE_M                 = 0.3
AP_G_DIST_MIN_LSIDE_TRAV_PAR_M       = 0.2
AP_G_DIST_MIN_LSIDE_DOOR_OPEN_PAR_M  = 0.31 # Value taken from code; missing from Parameters document
AP_G_MAX_WIDTH_SMALL_OBSTACLE_M      = 0.4
AP_G_DIST_MIN_NO_DELIMITER_M         = 0.05
AP_G_PAR_SLOT_MAX_OFFSET_W_M         = 0.8
AP_G_DIST_CMF_LSIDE_DOOR_OPEN_PAR_M  = 0.31 # Value taken from code; missing from Parameters document
AP_G_DIST_CMF_LSIDE_HIGH_OBST_PAR_M  = 1.0 # Value taken from code; missing from Parameters document
AP_G_WHEEL_DIST_ROAD_LVL_PAR_M       = 0.15
AP_G_PAR_SLOT_MAX_OFFSET_L_M         = 3.0
AP_G_MIN_SAF_DIST_TRAV_OBST_M        = 0.1
AP_G_PERP_SLOT_MAX_OFFSET_L_M        = 1.8
AP_G_DIST_MIN_PARKSLOT_MARKER_M      = -0.2
AP_G_MAX_DIST_WHEEL_STOPPER_M        = 0.075
AP_G_ANG_MAX_ORI_ANGLE_RAD           = 0.96
AP_G_PERP_MAX_ORI_ANGLE_RAD          = 0.15
AP_G_DIST_MIN_LSIDE_HIGH_OBST_PERP_M = 0.05
AP_G_DIST_LSIDE_MIN_DOOR_OPEN_M      = 0.47

AP_MAX_ROADSIDE_EXT_PAR_M           = 0.4 # corresponding to SI_maxRoadsideExtensionParallel_m
AP_MAX_ROADSIDE_EXT_PERP_M          = 0.4 # corresponding to SI_maxRoadsideExtensionPerp_m

# CarMaker lane types
AP_LANE_DRIVING         = 0
AP_LANE_ROADSIDE        = 5
AP_LANE_PEDESTRIAN      = 11
AP_LANE_PARKING         = 13
# CarMaker lane widths
AP_LW_ROADSIDE          = 2.5
AP_LW_PEDESTRIAN        = 2
# CarMaker road marking types
AP_RM_CONTINUOUS        = 1
AP_RM_BROKEN            = 2
AP_RM_DOTTED            = 3
# CarMaker road marking positions
AP_RM_POS_CENTER        = 0
AP_RM_POS_LEFT          = 1
AP_RM_POS_RIGHT         = -1

AP_PS_WIDTH_TOLERANCE   = 0.3 # width tolerance used for parking space when parked vehicle is missing
AP_PB_HEIGHT            = 0.01 # height of the parking box (used by the environment model)
AP_ANG_OFFSET_PERP_FWD  = 180 # rotation considered for a forwards perpendicular parked vehicle (vehicle rear towards the road); add (for right) or subtract (for left) 90°
AP_ANG_OFFSET_PERP_REV  = 0 # rotation considered for a backwards perpendicular parked vehicle (vehicle front towards the road); add (for right) or subtract (for left) 90°
AP_ANG_OFFSET_PAR_FWD   = 0 # rotation considered for a forwards parallel parked vehicle (vehicle front towards movement direction)
AP_ANG_OFFSET_PAR_REV   = 180 # rotation considered for a backwards parallel parked vehicle (vehicle rear towards movement direction)
AP_LIM_WIDTH            = 0.15 # default limiter width
AP_LINE_LIM_OFFSET_Z    = 0.03 # z offset considered for line limiters (to avoid CarMaker visual bugs)
AP_CIRC_OBST_NUM_SEG    = 20 # number of segments considered for a circular obstacle contour (set as divisor of 180°)
AP_NUM_OBST             = 3 # maximum number of obstacles
AP_CENTER_OF_MASS       = 0.0 # default value for traffic objects center of mass
AP_WHS_DEFAULT_TYPE     = "single_beam" # default type for wheelstopper
AP_SCAN_OFF             = 0 # scanning phase not possible
AP_SCAN_ON_STRAIGHT     = 1 # scanning phase possible (end position 0°)
AP_SCAN_ON_ANGLED       = 2 # scanning phase possible (absolute end position between 0° and AP_MAX_SCAN_ANGLE)
AP_MAX_SCAN_ANGLE       = 7 # maximum angle (in °) for which the scanning phase is possible
AP_STR_SCAN_DRV_DIST    = 24.5 # distance traveled for scanning phase with constant velocity
AP_STR_SCAN_BRK_DIST    = 3.5 # distance traveled for scanning phase while braking (the total scanning distance should be 28m) TODO: use interpolation tables from WHP for distance control based on velocity
AP_ANG_SCAN_BRK_DIST    = 3.312 # distance traveled for scanning phase while braking with AP_SCAN_DECEL m/s²
AP_STR_SCAN_BRK_TOL     = 0.1 # distance subtracted from the scanning phase to compensate for IPG driver overshoot
AP_SCAN_VEL             = 10 # km/h - default scanning velocity
AP_SCAN_DECEL           = 3 # m/s - default deceleration rate (used for angled scanning)
AP_VW_PASSAT_TYRE_RAD   = 0.334 # m - tyre radius for 215/55/R17
AP_VW_PASSAT_TYRE_WIDTH = 0.215 # m - tyre width for 215/55/R17


# Colors
AP_COLOR_GREY           = [0.74, 0.74, 0.74]
AP_COLOR_WHITE          = [1.0, 1.0, 1.0]
AP_COLOR_RED            = [1.0, 0.0, 0.0]
AP_COLOR_BROWN          = [0.65, 0.165, 0.165]
AP_COLOR_YELLOW         = [1.0, 1.0, 0.0]

# Maneuver commands
ManCmdEnableVEDODO    = "\t# Enable VEDODO" + \
                      "\n\tEval first() ? AP.vedodoActive_nu = ($OdometryActive=1)"
ManCmdConfigureLSCA   = "\t# Configure LSCA" + \
                      "\n\tEval first() ? AP.lscaDisabled_nu = ($LscaDisabled=0)" + \
                      "\n\tEval first() ? AP.lscaBrakeDisabled_nu = ($LscaBrakeDisabled=0)" + \
                      "\n\tEval first() ? AP.lscaSteeringProposalDisabled_nu = ($LscaSteeringDisabled=1)" + \
                      "\n\tEval first() ? AP.lscaVirtualWallDisabled_nu = ($LscaVirtualWallDisabled=1)"
ManCmdEnableLimitFOV  = "\t# Enable Field of view limit" + \
                      "\n\tEval first() ? AP.enableLimitFieldOfView_nu = ($LimitFieldOfViewEnabled=1)"
ManCmdDisableLimitFOV = "\t# Disable Field of view limit" + \
                      "\n\tEval first() ? AP.enableLimitFieldOfView_nu = ($LimitFieldOfViewEnabled=0)" # fill testrun map info (coordinates of traffic objects) without scanning phase
ManCmdEnableLatency   = "\t# Enable Latency effect" + \
                      "\n\tEval first() ? AP.enableLatencyEffect_nu = ($LatencyEffectEnabled=1)"
ManCmdDisableLatency  = "\t# Disable Latency effect" + \
                      "\n\tEval first() ? AP.enableLatencyEffect_nu = ($LatencyEffectEnabled=0)"
ManCmdEnableSI        = "\t# Enable Scene Interpretation" + \
                      "\n\tEval first() ? AP.sceneInterpretationActive_nu=($SceneInterpretationActive=1)"
ManCmdDisableSI       = "\t# Disable Scene Interpretation - use fake Environment Model" + \
                      "\n\tEval first() ? AP.sceneInterpretationActive_nu=($SceneInterpretationActive=0)"
ManCmdResetSteerWhl   = "\t# Reset steering wheel angle position" + \
                      "\n\tDVAwr DM.Steer.Ang Abs -1 0"
ManCmdDisableIPGLat   = "\t# Disable lateral control of the IPG Driver" + \
                      "\n\tDVAwr Driver.Lat.passive Abs -1 1"
ManCmdEnableAP        = "\t# HMI User input - Toggle AP active" + \
                      "\n\tEval first() ? AP.hmiOutputPort.userActionHeadUnit_nu = 0" + \
                      "\n\tEval DM.ManTime > 0.1 ? AP.hmiOutputPort.userActionHeadUnit_nu = 28" + \
                      "\n\tEval DM.ManTime > 0.2 ? AP.hmiOutputPort.userActionHeadUnit_nu = 0"
ManCmdStartSelection  = "\t# HMI User input - Start selection" + \
                      "\n\tEval DM.ManTime > 0.1 ? AP.hmiOutputPort.userActionHeadUnit_nu = 17" + \
                      "\n\tEval DM.ManTime > 0.3 ? AP.hmiOutputPort.userActionHeadUnit_nu = 0"
ManCmdSwitchDirection = "\t# HMI User input - Switch parking direction (to Forwards)" + \
                      "\n\tEval DM.ManTime > 0.5 ? AP.hmiOutputPort.userActionHeadUnit_nu = 25" + \
                      "\n\tEval DM.ManTime > 0.7 ? AP.hmiOutputPort.userActionHeadUnit_nu = 0"
ManCmdStartParking    = "\t# HMI User input - Start parking" + \
                      "\n\tEval DM.ManTime > 0.9 ? AP.hmiOutputPort.userActionHeadUnit_nu = 18" + \
                      "\n\tEval DM.ManTime > 1.1 ? AP.hmiOutputPort.userActionHeadUnit_nu = 0"
ManCmdShiftInR        = "\t# Shift in Reverse" + \
                      "\n\tEval DM.ManTime > 0.001 ? DM.SelectorCtrl = -1"
ManCmdMoveOdoArrow    = "\t# Move odometry arrow to visualize odometry error" + \
                      "\n\tAP.vedodoActive_nu==1 ? Traffic.Odo.tx = AP.odoEstimationPort.xPosition_m" + \
                      "\n\tAP.vedodoActive_nu==1 ? Traffic.Odo.ty = AP.odoEstimationPort.yPosition_m" + \
                      "\n\tAP.vedodoActive_nu==1 ? Traffic.Odo.rz = AP.odoEstimationPort.yawAngle_rad"
ManCmdDisableIPGLat   = "\t# Disable IPG lateral control" + \
                      "\n\tDVAwr Driver.Lat.passive Abs -1 1"
ManCmdStopTRNoSteer   = "\t# Stop testrun if no steering angle was requested in first xx seconds (default: 10)" + \
                      "\n\tEval first() ? Qu::maneuveringFlag_nu = 0" + \
                      "\n\tEval ((abs(AP.steerCtrlRequestPort.steerAngReq_rad) > 0.1) || (abs(Vhcl.v) > 0.01)) ? Qu::maneuveringFlag_nu = 1" + \
                      "\n\tEval (DM.ManTime > 10) & maneuveringFlag_nu==0 ? ManJump(\"END\")"
ManCmdStopTRTime      = "\t# Stop testrun after xx seconds (default: 200)" + \
                      "\n\tEval (DM.ManTime > 200) ? ManJump(\"END\")"
ManCmdVelOverwrite    = "\t# Overwrite velocity inputs of Parking State Machine to enable scanning" + \
                      "\n\tEval DM.ManTime > 0.1 ? AP.odoEstimationPort.overwriteVehVelocityCur_nu = 1" + \
                      "\n\tEval DM.ManTime > 0.3 ? AP.odoEstimationPort.overwriteVehVelocityCur_nu = 0"
ManCmdPreventRoll     ="\t# Hold brake to prevent vehicle from unwanted movement" + \
                      "\n\tEval DM.ManTime <= 1.0 ? DM.Brake = 0.3"
ManCmdScanDrvDone     = "\t# Scanning with constant velocity complete" + \
                      "\n\tEval first() ? Qu::StartPos = Vhcl.sRoad" + \
                      "\n\tEval Vhcl.sRoad >= StartPos + " + str(AP_STR_SCAN_DRV_DIST) + " ? ManJump(\"+1\")"
ManCmdScanBrkDone     = "\t# Scanning phase complete" + \
                      "\n\tEval Car.v < 0.01 ? ManJump(\"+1\")"
                      

#********************************************************************
# Configuration tables
#******************************************************************** 
# limTypeParamConfigType is named tuple containing for each limiter type:
# lenFact = multiplier for the limiter length
# orientation = angle of limiter
# factX = multiplier for horizontal (along the road) edge
# factY = multiplier for vertical (perpendicular to the road) edge
# offsetX = X offset for limiter (to compensate for limiter origin in the middle)
# offsetY = Y offset for limiter (to compensate for limiter origin in the middle)
# edgeType = edge type (horizontal / vertical) considered for limiter length
limTypeParamConfigType = namedtuple("limTypeParamConfigType", "lenFact orientation factX factY offsetX offsetY edgeType")

limTypeParamConfig = {                      #lenFact orientation factX  factY   offsetX                 offsetY         edgeType
    "lim_Rd_P"      : limTypeParamConfigType(   3,      0,          2,      0,      0,                  -AP_LIM_WIDTH/2,    "h"), 
    "lim_P1_P"      : limTypeParamConfigType(   1,      180,        0,      0,      AP_LIM_WIDTH/2,     0,                  "v"), # add (for right) or subtract (for left) 90°
    "lim_P2_P"      : limTypeParamConfigType(   1,      180,        1,      0,      -AP_LIM_WIDTH/2,    0,                  "v"), # add (for right) or subtract (for left) 90°
    "lim_P_End"     : limTypeParamConfigType(   3,      0,          2,      1,      0,                  AP_LIM_WIDTH/2,     "h"),
    "lim_ins_P"     : limTypeParamConfigType(   3,      0,          2,      0.5,    0,                  0,                  "h") # TODO: check info for inside limiters
    }


class AP_ParkingTestCase():
    def __init__(self, data, xlsReader):
        self.data = data
        self.limHeight          = xlsReader.limHeight
        self.bumpHeight         = xlsReader.bumpHeight
        self.bumpRampUp         = xlsReader.bumpRampUp
        self.bumpPlateau        = xlsReader.bumpPlateau
        self.bumpRampDown       = xlsReader.bumpRampDown
        self.wheelstopperHeight = xlsReader.wheelstopperHeight
        self.wheelstopperLength = xlsReader.wheelstopperLength
        self.wheelstopperWidth  = xlsReader.wheelstopperWidth
        self.trafficObjID = 0
        self.latOffset = 0 # this is used during the scanning phase to drive in a straight line, taking lateral position (CarMaker y coordinate) into account
        self.scanDist = AP_STR_SCAN_DRV_DIST + AP_STR_SCAN_BRK_DIST # scanning phase distance 
        
        if self.data["Category"].find("Left") != -1:
            self.CMSign = 1
        else:
            self.CMSign = -1
            
        (self.latHPark, self.latVPark, self.latHEgo, self.latVEgo, self.l_Ang, self.w_Ang, self.xOffPStart_Ang, self.xOffPEnd_Ang) = self.getParkingLayout()
        (self.u_Rd, self.v_Rd) = self.getRoadRefCoords()
        (self.u_P, self.v_P) = self.getParkingRefCoords()
        self.obstacles = []       
        
        self.minHSidePB = self.latHEgo
        self.minVSidePB = self.latVEgo

        # Add an extension on the road for high limiters at the end of the parking box (this will be removed in case there are no such limiters)
        if self.data["Category"].find("Perpendicular") != -1:
            self.minVSidePB += AP_MAX_ROADSIDE_EXT_PERP_M
        elif self.data["Category"].find("Parallel") != -1:
            self.minVSidePB += AP_MAX_ROADSIDE_EXT_PAR_M
        elif self.data["Category"].find("Angled") != -1:
            # TODO: Check requirements for road extension in case of high limiters
            self.minVSidePB += AP_MAX_ROADSIDE_EXT_PAR_M
        else:
            print("Scenario catogory not handled - constructor AP_ParkingTestCase")
            sys.exit(0)
                
        self.roadOffsetPB = 0
        if self.data["Category"].find("Perpendicular") != -1:
            self.minHSidePB += AP_G_PERP_SLOT_MIN_OFFSET_W_M - AP_G_PERP_SLOT_OFFSET_EM_W_M
            self.roadOffsetPB = AP_G_PERP_SLOT_MIN_OFFSET_L_M
        elif self.data["Category"].find("Parallel") != -1:
            self.minHSidePB += AP_G_PAR_SLOT_MIN_OFFSET_L_M - AP_G_PAR_SLOT_OFFSET_EM_L_M
            self.roadOffsetPB = AP_G_PAR_SLOT_MIN_OFFSET_W_M
        elif self.data["Category"].find("Angled") != -1:
            # TODO: Check requirements for minimum parking box dimensions
            self.minHSidePB += AP_G_PAR_SLOT_MIN_OFFSET_L_M
            self.roadOffsetPB = AP_G_PAR_SLOT_MIN_OFFSET_W_M
        else:
            print("Scenario catogory not handled - constructor AP_ParkingTestCase")
            sys.exit(0)
            
        if self.data["psi_Ego_0"] == 0: # if the vehicle is parallel to the road axis
            self.scanOn = AP_SCAN_ON_STRAIGHT # straight scanning phase possible
        elif abs(self.data["psi_Ego_0"]) <= AP_MAX_SCAN_ANGLE:
            self.scanOn = AP_SCAN_ON_ANGLED # angled scanning phase possible
        else:
            self.scanOn = AP_SCAN_OFF # angled scanning phase not possible
            
        self.corners_P1 = {}
        self.corners_P2 = {}
        (self.corners_P1, self.corners_P2) = self.getCornersPx()
        self.roadSideEdge = 0 # lateral offset value for the road side edge


    def AP_TestRunFactory(self, myTestRun):
        # #TestRun description
        # myTestRun.Description.value = self.data["Description"] + self.getEvaluationCriteria(myTestRun)

        #Selects CarMaker Vehicle
        self.fillEgoType(myTestRun)
        
        #Road data
        self.fillRoadData(myTestRun)

        #start position for EgoVehicle
        self.fillEgoStartPosition(myTestRun)

        #Calculate obstacle data - required for Parking box (obstacle traffic objects are not created here)
        for i in range(1, AP_NUM_OBST + 1):
            if self.data["shape_Obs" + str(i)] != "none":
                self.calcObstacleData(myTestRun, str(i))
        
        # # TODO: remove this when getCornersPx() will return the corners for 1 vehicle only
        # corners_P1 = {}
        # corners_P2 = {}
        # (corners_P1, corners_P2) = self.getCornersPx()
        
        #Parking Box
        self.createParkingBox(myTestRun)
        
        #Parking Vehicle 1
        if  self.data["exists_P1"] == "true":
            self.fillPxData(myTestRun, "1", self.corners_P1)
        
        #Parking Vehicle 2
        if  self.data["exists_P2"] == "true":
            self.fillPxData(myTestRun, "2", self.corners_P2)
            
        # Parking Space Limiters
        for limiterType in list(limTypeParamConfig.keys()):
            if self.data[limiterType] != "none":
                self.fillLimiterData(myTestRun, limiterType)

        # Obstacles
        for i in range(1, AP_NUM_OBST + 1):
            if self.data["shape_Obs" + str(i)] != "none":
                self.fillObstacleData(myTestRun, str(i))
                
        # Wheelstopper
        if self.is_number_tryexcept(self.data["d_ws"]) == True:
            self.fillWheelstopperData(myTestRun)
       
        # Odometry
        self.fillOdometryData(myTestRun)

        # Manipulate Driver Maneuvers
        self.fillManeuverData(myTestRun)
        
        #TestRun description
        myTestRun.Description.value = self.data["Description"] + self.getEvaluationCriteria(myTestRun)
        

    def getParkingLayout(self):
        latHPark       = 0 # parking box horizontal side (parallel to the road axis)
        latVPark       = 0 # parking box vertical side (perpendicular to the road axis)
        latHEgo        = 0 # ego vehicle horizontal side (parallel to the road axis, once the ego vehicle is parked)
        latVEgo        = 0 # ego vehicle vertical side (perpendicular to the road axis, once the ego vehicle is parked)
        l_Ang          = 0 # length of angled parking slot
        w_Ang          = 0 # width of angled parking slot
        xOffPStart_Ang = 0 # x offset at the start of angled parking slot
        xOffPEnd_Ang   = 0 # x offset at the end of angled parking slot
        if self.data["Category"].find("Perpendicular") != -1:
            latHEgo  = self.data["w_Ego"]
            latVEgo  = self.data["l_Ego"]
            if self.data["w_P_off"] != "-":
                latHPark = latHEgo + self.data["w_P_off"]
            else:
                latHPark = self.data["w_P"]
            if self.data["l_P_off"] != "-":
                latVPark = latVEgo + self.data["l_P_off"]
            else:
                latVPark = self.data["l_P"]
        elif self.data["Category"].find("Parallel") != -1:
            latHEgo  = self.data["l_Ego"]
            latVEgo  = self.data["w_Ego"]
            if self.data["l_P_off"] != "-":
                latHPark = latHEgo + self.data["l_P_off"]
            else:
                latHPark = self.data["l_P"]
            if self.data["w_P_off"] != "-":
                latVPark = latVEgo + self.data["w_P_off"]
            else:
                latVPark = self.data["w_P"]
        elif self.data["Category"].find("Angled") != -1:
            latHEgo = self.data["w_Ego"] * abs(math.sin(math.radians(self.data["alpha"])))
            latVEgo = self.data["l_Ego"] * abs(math.sin(math.radians(self.data["alpha"])))
            if self.data["w_P_off"] != "-":
                w_Ang = self.data["w_Ego"] + self.data["w_P_off"]
            else:
                w_Ang = self.data["w_P"]
            if self.data["l_P_off"] != "-":
                l_Ang = self.data["l_Ego"] + self.data["l_P_off"]
            else:
                l_Ang = self.data["l_P"]
            latHPark = w_Ang / abs(math.sin(math.radians(self.data["alpha"])))
            latVPark = l_Ang * abs(math.sin(math.radians(self.data["alpha"])))
            if self.data["alpha"] < 90:
                xOffPEnd_Ang = l_Ang * abs(math.cos(math.radians(self.data["alpha"])))
            else:
                xOffPStart_Ang = l_Ang * abs(math.cos(math.radians(self.data["alpha"])))
        else:
            print("Scenario catogory not handled - getParkingLayout()")
            sys.exit(0)
        
        return (latHPark, latVPark, latHEgo, latVEgo, l_Ang, w_Ang, xOffPStart_Ang, xOffPEnd_Ang)
        
        
    def getRoadRefCoords(self):
        # origin of the coordinate system used for road objects (blue coordinate system)
        u_Rd = AP_ROAD_LENGTH / 2 + 0.5 * self.latHPark # middle of the parking area is halfway the road length
        v_Rd = self.CMSign * self.data["w_ul"]
            
        return (u_Rd, v_Rd)
        
        
    def getParkingRefCoords(self):
        # origin of the coordinate system used for parking spot objects (red coordinate system)
        if self.data["Category"].find("Angled") != -1:
            u_P = self.u_Rd - self.xOffPStart_Ang + self.xOffPEnd_Ang - self.latHPark / 2
        else:
            u_P = self.u_Rd - self.latHPark / 2
        v_P = self.v_Rd + self.CMSign * self.latVPark

        return (u_P, v_P)
    
    
    def fillEgoType(self, myTestRun):
        # Ego vehicle type
        if self.data["type_Ego"] == "VW Passat B8":
            egoType = "VW_PassatVariant_20TSI_DSG_LLVW_AP_C_GenSteerAng.car"
        elif self.data["type_Ego"] == "MB W222-2014":
            egoType = "MB_S350_W222_2014.car"
        else:
            print("Ego vehicle type not handled by getEgoType().")
            sys.exit(0)
    
        myTestRun.Vehicle._srcFile = egoType
        
        # Tires
        myTestRun.Tire._srcFile = "Continental_ContiPremiumContact5_215_55R17_94W_MF-Tyre_62"
        

    def getAngledScanStartPos(self, myTestRun, endX, endY):
        # psiRad = math.radians(self.data["psi_Ego_0"])
        psiRad = math.radians(-self.CMSign * self.data["psi_Ego_0"])
        limRoad = self.v_Rd / 2
        if self.CMSign == 1:
            limRoad += self.v_Rd
        
        yDelta = self.CMSign * np.sign(self.data["psi_Ego_0"]) * math.sin(abs(psiRad)) * self.scanDist
        startY = endY + yDelta
        
        if np.sign(self.data["psi_Ego_0"]) >= 0 and abs(startY) > abs(limRoad): # the middle of the ego vehicle should not go beyond v_Rd at the start of the scanning phase
            startY = limRoad
            yDelta = abs(startY - endY)
            self.scanDist = yDelta / math.sin(abs(psiRad))
            
        xDelta = self.scanDist * math.cos(abs(psiRad))
        
        startPosEgoX = round(endX - xDelta, 3)
        startPosEgoY = round(startY, 3)
        self.scanDist = round(self.scanDist, 3)
        
        return (startPosEgoX, startPosEgoY)
        
        
    def fillEgoStartPosition(self, myTestRun):
        # (X1, Y1) - middle of the rear axle
        # (X2, Y2) - middle of the rear bumper (accounting for overhang and yaw angle)
        # trigonometric equations inside right triangle
        # r = hypotenuse
        # offset on X = adjacent cathetus
        # offset of Y = opposing cathetus
        # X2 = X1 - r*cos(θ)
        # Y2 = Y1 - r*sin(θ)
        # where:
        # θ = psi_Ego
        # r = overhang
        
        theta = self.data["psi_Ego_0"]
        r = self.data["o_Ego"]
        
        if self.data["Category"].find("Angled") != -1:
            xTemp = self.u_Rd  + self.data["d_C1_Ego"] - self.latHPark
            if self.data["exists_P2"] == "true":
                ipTemp1 = self.data["delta_Pass_Rd"] / math.sin(math.radians(self.data["alpha"])) # projection of delta_Pass_Rd on the longitudinal side of the parking slot
                ipTemp2 = self.data["d_P2_C1"] / math.sin(math.radians(self.data["alpha"])) # horizontal distance from P2 corner to the longitudinal side of the parking slot
                xTempDelta = ipTemp1 * math.cos(math.radians(self.data["alpha"]))
                xTemp = xTemp - (ipTemp2 - xTempDelta)
        else:
            xTemp = self.u_Rd + self.data["d_C1_Ego"]
            if self.data["exists_P1"] == "true":
                xTemp += self.data["d_P1_C1"]
        yTemp = abs(self.v_Rd) / 2 # set starting position on the middle of the driving lane (step valid only for right driving side)
        yTemp += self.v_Rd - self.CMSign * (self.data["d_Pass"] - self.data["delta_Pass_Rd"] + self.data["w_Ego"] / 2)

        startPosEgoX = round(xTemp - r * math.cos(math.radians(theta)),3)
        startPosEgoY = round(yTemp + self.CMSign * r * math.sin(math.radians(abs(theta))) * np.sign(theta),3)
        self.latOffset = startPosEgoY
        
        # Check if scanning phase is possible
        if self.scanOn == AP_SCAN_ON_STRAIGHT:
            startPosEgoX -= (AP_STR_SCAN_DRV_DIST + AP_STR_SCAN_BRK_DIST) # subtract the total distance traveled during scanning phase
            startPosEgoX = round(startPosEgoX, 3)
        elif self.scanOn == AP_SCAN_ON_ANGLED:
            (startPosEgoX, startPosEgoY) = self.getAngledScanStartPos(myTestRun, startPosEgoX, startPosEgoY)
        else:
            pass

        VhclStartPos = str(startPosEgoX) + " " + str(startPosEgoY)+ " " + str(-self.CMSign * theta)
        
        # Ego Vehicle start position
        myTestRun.Road.VhclStartPos.value = VhclStartPos
        
        
    def createParkingBox(self, myTestRun):
        # borders of the parking box in the following order: road, parking vehicle 1, end of parking area, parking vehicle 2
        # N - none
        # Ml - lane marking (on the road)
        # Mp - parking area marking
        # V - vehicle
        # Cx - curbstone, x = t/tb/ntd/nt
        # L - guardrail / wall
        # O - obstacle
        borders = ["N", "N", "N", "N"]
        borders_pos = ["lim_Rd_P", "lim_P1_P", "lim_P_End", "lim_P2_P"]
        # check for limiters
        i = 0
        for idx in borders_pos:
            if self.data[idx] == "line":
                borders[i] = "Mp"
            elif self.data[idx] == "curb_t":
                borders[i] = "Ct"
            elif self.data[idx] == "curb_tb":
                borders[i] = "Ctb"
            elif self.data[idx] == "curb_ntd":
                borders[i] = "Cntd"
            elif self.data[idx] == "curb_nt":
                borders[i] = "Cnt"
            elif self.data[idx] != "none":
                borders[i] = "L"
            i += 1
                
        if borders[0] == "Mp": # if the parking marker is to the road side
            borders[0] = "Ml" # change it to lane marking
        
        # corners_P1 = {}
        # corners_P2 = {}
        # (corners_P1, corners_P2) = self.getCornersPx()
        
        y1_PS = self.v_Rd - self.CMSign * self.roadOffsetPB
        y2_PS = self.v_Rd - self.CMSign * self.roadOffsetPB
        y3_PS = self.v_P
        y4_PS = self.v_P
        
        if self.data["Category"].find("Angled") != -1:
            xOffRd = self.roadOffsetPB / math.tan(math.radians(self.data["alpha"]))
        else:
            xOffRd = 0
        
        x1_PS = self.u_Rd - self.latHPark - xOffRd
        x2_PS = self.u_Rd - xOffRd
        x3_PS = self.u_Rd - self.xOffPStart_Ang + self.xOffPEnd_Ang
        x4_PS = self.u_Rd - self.latHPark - self.xOffPStart_Ang + self.xOffPEnd_Ang

        (x1_PS_temp, y1_PS_temp, x2_PS_temp, y2_PS_temp, x3_PS_temp, y3_PS_temp, x4_PS_temp, y4_PS_temp) = (x1_PS, y1_PS, x2_PS, y2_PS, x3_PS, y3_PS, x4_PS, y4_PS)
        
        if self.data["exists_P1"] == "true":
            if borders[1] == "N":
                borders[1] = "V"
                (x2_PS_temp, y2_PS_temp, x3_PS_temp, y3_PS_temp) = (self.corners_P1["x1"], self.corners_P1["y1"], self.corners_P1["x4"], self.corners_P1["y4"])
                x2_PS =  self.get_interp2d_x(x2_PS_temp, y2_PS_temp, x3_PS_temp, y3_PS_temp, y2_PS)
                x3_PS =  self.get_interp2d_x(x2_PS_temp, y2_PS_temp, x3_PS_temp, y3_PS_temp, y3_PS)
            else:
                overlapPt1 = False
                overlapPt4 = False
                if self.data["Category"].find("Angled") != -1:
                    overlapPt1 = (self.get_point_side(self.corners_P1["x1"], self.corners_P1["y1"], x2_PS, y2_PS, x3_PS, y3_PS) <= 0)
                    overlapPt4 = (self.get_point_side(self.corners_P1["x4"], self.corners_P1["y4"], x2_PS, y2_PS, x3_PS, y3_PS) <= 0)
                else: # TODO: check if solution for angle is generally aplicable
                    overlapPt1 = (self.corners_P1["x1"] < x2_PS)
                    overlapPt4 = (self.corners_P1["x4"] < x3_PS)
                    
                if overlapPt1 or overlapPt4:
                    borders[1] = "V"

                    if overlapPt1: # use P1_1 for interpolation
                        x2_PS_temp = self.corners_P1["x1"]
                        y2_PS_temp = self.corners_P1["y1"]
                    if overlapPt4: # use P1_4 for interpolation
                        x3_PS_temp = self.corners_P1["x4"]
                        y3_PS_temp = self.corners_P1["y4"]
                        
                    x2_PS = self.get_interp2d_x(x2_PS_temp, y2_PS_temp, x3_PS_temp, y3_PS_temp, y2_PS)
                    x3_PS = self.get_interp2d_x(x2_PS_temp, y2_PS_temp, x3_PS_temp, y3_PS_temp, y3_PS)
                    
                    # the following checks are required in case parked vehicle 1 has both corners in the parking area, but the parking box border still crosses the limiter
                    if x2_PS > self.u_Rd - xOffRd:
                        x2_PS = self.u_Rd - xOffRd
                        x3_PS = self.get_interp2d_x(x2_PS, y2_PS, x3_PS_temp, y3_PS_temp, y3_PS)
                    if x3_PS > self.u_Rd - self.xOffPStart_Ang + self.xOffPEnd_Ang:
                        x3_PS = self.u_Rd - self.xOffPStart_Ang + self.xOffPEnd_Ang
                        x2_PS = self.get_interp2d_x(x2_PS_temp, y2_PS_temp, x3_PS, y3_PS, y2_PS)
            
        if self.data["exists_P2"] == "true":
            if borders[3] == "N":
                borders[3] = "V"
                (x1_PS_temp, y1_PS_temp, x4_PS_temp, y4_PS_temp) = (self.corners_P2["x2"], self.corners_P2["y2"], self.corners_P2["x3"], self.corners_P2["y3"])
                x1_PS =  self.get_interp2d_x(x1_PS_temp, y1_PS_temp, x4_PS_temp, y4_PS_temp, y1_PS)
                x4_PS =  self.get_interp2d_x(x1_PS_temp, y1_PS_temp, x4_PS_temp, y4_PS_temp, y4_PS)
            else:
                overlapPt2 = False
                overlapPt3 = False
                if self.data["Category"].find("Angled") != -1:
                    overlapPt2 = (self.get_point_side(self.corners_P2["x2"], self.corners_P2["y2"], x1_PS, y1_PS, x4_PS, y4_PS) >= 0)
                    overlapPt3 = (self.get_point_side(self.corners_P2["x3"], self.corners_P2["y3"], x1_PS, y1_PS, x4_PS, y4_PS) >= 0)
                else: # TODO: check if solution for angle is generally aplicable
                    overlapPt2 = (self.corners_P2["x2"] > x1_PS)
                    overlapPt3 = (self.corners_P2["x3"] > x4_PS)
                    
                if overlapPt2 or overlapPt3:
                    borders[3] = "V"
                    
                    if overlapPt2: # use P2_2 for interpolation
                        x1_PS_temp = self.corners_P2["x2"]
                        y1_PS_temp = self.corners_P2["y2"]
                    if overlapPt3: # use P2_3 for interpolation
                        x4_PS_temp = self.corners_P2["x3"]
                        y4_PS_temp = self.corners_P2["y3"]
                        
                    x1_PS = self.get_interp2d_x(x1_PS_temp, y1_PS_temp, x4_PS_temp, y4_PS_temp, y1_PS)
                    x4_PS = self.get_interp2d_x(x1_PS_temp, y1_PS_temp, x4_PS_temp, y4_PS_temp, y4_PS)
                    
                    # the following checks are required in case parked vehicle 2 has both corners in the parking area, but the parking box border still crosses the limiter
                    if x1_PS < self.u_Rd - self.latHPark - xOffRd:
                        x1_PS = self.u_Rd - self.latHPark - xOffRd
                        x4_PS = self.get_interp2d_x(x1_PS, y1_PS, x4_PS_temp, y4_PS_temp, y4_PS)
                    if x4_PS < self.u_Rd - self.latHPark - self.xOffPStart_Ang + self.xOffPEnd_Ang:
                        x4_PS = self.u_Rd - self.latHPark - self.xOffPStart_Ang + self.xOffPEnd_Ang
                        x1_PS = self.get_interp2d_x(x1_PS_temp, y1_PS_temp, x4_PS, y4_PS, y1_PS)
        
        invalidPB = False # TODO: this flag may be influenced by parked vehicles positions
        # account for obstacles
        numObs = len(self.obstacles)
        if numObs > 0 and self.data["Category"].find("Angled") == -1: # there is at least one obstacle && category is not angled
            obsSortLeft = sorted(self.obstacles, key=lambda k: k['x_L']) # first element is closest to parking vehicle 2 parking space
            obsSortRight = sorted(self.obstacles, key=lambda k: k['x_R'], reverse = True) # first element is closest to parking vehicle 1 parking space
            obsSortRoad = sorted(self.obstacles, key=lambda k: abs(k['y_Rd'])) # first element is closest to v_Rd
            areas = []
            
            # PB hack - save current parking box coordinates to be used if obstacles in parking area do not provide sufficient space (PB will contain obstacles within it)
            (x1_PS_temp, y1_PS_temp, x2_PS_temp, y2_PS_temp, x3_PS_temp, y3_PS_temp, x4_PS_temp, y4_PS_temp) = (x1_PS, y1_PS, x2_PS, y2_PS, x3_PS, y3_PS, x4_PS, y4_PS)
            
            # area 1 - from lim_P2_P to first of obsSortLeft
            area = {}
            area["l"] = self.latVPark
            area["w"] = obsSortLeft[0]["x_L"] - x1_PS
            if area["w"] >= self.minHSidePB:
                area["x1"] = x1_PS
                area["y1"] = y1_PS
                area["x2"] = obsSortLeft[0]["x_L"]
                area["y2"] = y1_PS
                area["x3"] = area["x2"]
                area["y3"] = y3_PS
                area["x4"] = x4_PS
                area["y4"] = y4_PS
                area["b_P1_P"]  = "O"
                area["b_P_End"] = borders[2]
                area["b_P2_P"]  = borders[3]
                areas.append(area)
            
            # area 2 - between lim_Rd_P, lim_P_End, obstacle[i] and obstacle[i + 1]
            for i in range(0, numObs - 1):
                # obstacle i ends before obstacle i + 1 begins (from the start of the road)
                if obsSortLeft[i]["x_R"] < obsSortLeft[i + 1]["x_L"]:
                    area = {}
                    area["l"] = self.latVPark
                    area["w"] = obsSortLeft[i + 1]["x_L"] - obsSortLeft[i]["x_R"]
                    if area["w"] >= self.minHSidePB:
                        area["x1"] = obsSortLeft[i]["x_R"]
                        area["y1"] = y1_PS
                        area["x2"] = obsSortLeft[i + 1]["x_L"]
                        area["y2"] = y2_PS
                        area["x3"] = area["x2"]
                        area["y3"] = y3_PS
                        area["x4"] = area["x1"]
                        area["y4"] = area["y3"]
                        area["b_P1_P"]  = "O"
                        area["b_P_End"] = borders[2]
                        area["b_P2_P"]  = "O"
                        areas.append(area)
                
            # area 3 - from lim_P1_P to first of obsSortRight
            area = {}
            area["l"] = self.latVPark
            area["w"] = x2_PS - obsSortRight[0]["x_R"]
            if area["w"] >= self.minHSidePB:
                area["x1"] = obsSortRight[0]["x_R"]
                area["y1"] = y2_PS
                area["x2"] = x2_PS
                area["y2"] = y2_PS
                area["x3"] = x3_PS
                area["y3"] = y3_PS
                area["x4"] = area["x1"]
                area["y4"] = y3_PS
                area["b_P1_P"]  = borders[1]
                area["b_P_End"] = borders[2]
                area["b_P2_P"]  = "O"
                areas.append(area)
                
            # area 4 - from lim_Rd_P to first of obsSortRoad
            area = {}
            area["l"] = abs(y1_PS - obsSortRoad[0]["y_Rd"])
            area["w"] = self.latHPark
            if area["l"] >= self.minVSidePB:
                area["x1"] = x1_PS
                area["y1"] = y1_PS
                area["x2"] = x2_PS
                area["y2"] = y2_PS
                area["x3"] = x2_PS
                area["y3"] = obsSortRoad[0]["y_Rd"]
                area["x4"] = x1_PS
                area["y4"] = area["y3"]
                area["b_P1_P"]  = borders[1]
                area["b_P_End"] = "O"
                area["b_P2_P"]  = borders[3]
                areas.append(area)
                
            if numObs == 3 and abs(obsSortLeft[0]["y_Rd"]) < abs(obsSortLeft[1]["y_Rd"]) and abs(obsSortLeft[2]["y_Rd"]) < abs(obsSortLeft[1]["y_Rd"]):
                # area 5 - between lim_Rd_P and the three obstacles; middle obstacle (from the start of the road is farthest from the road axis)
                area = {}
                area["l"] = abs(y1_PS - obsSortLeft[1]["y_Rd"])
                area["w"] = obsSortLeft[2]["x_L"] - obsSortLeft[0]["x_R"]
                if area["l"] >= self.minVSidePB and area["w"] >= self.minHSidePB:
                    area["x1"] = obsSortLeft[0]["x_R"]
                    area["y1"] = y1_PS
                    area["x2"] = obsSortLeft[2]["x_L"]
                    area["y2"] = y2_PS
                    area["x3"] = area["x2"]
                    area["y3"] = obsSortLeft[1]["y_Rd"]
                    area["x4"] = area["x1"]
                    area["y4"] = area["y3"]
                    area["b_P1_P"]  = "O"
                    area["b_P_End"] = "O"
                    area["b_P2_P"]  = "O"
                    areas.append(area)
                
                # area 6 - between lim_Rd_P, lim_P2_P, obstacle 1 and obstacle 3
                if abs(obsSortLeft[0]["y_Rd"]) > abs(obsSortLeft[2]["y_Rd"]):
                    area = {}
                    area["l"] = abs(y1_PS - obsSortLeft[0]["y_Rd"])
                    area["w"] = obsSortLeft[2]["x_L"] - x1_PS
                    if area["l"] >= self.minVSidePB and area["w"] >= self.minHSidePB:
                        area["x1"] = x1_PS
                        area["y1"] = y1_PS
                        area["x2"] = obsSortLeft[2]["x_L"]
                        area["y2"] = y2_PS
                        area["x3"] = area["x2"]
                        area["y3"] = obsSortLeft[0]["y_Rd"]
                        area["x4"] = area["x1"]
                        area["y4"] = area["y3"]
                        area["b_P1_P"]  = "O"
                        area["b_P_End"] = "O"
                        area["b_P2_P"]  = borders[3]
                        areas.append(area)
                        
                # area 7 - between lim_Rd_P, lim_P1_P, obstacle 1 and obstacle 3
                elif abs(obsSortLeft[0]["y_Rd"]) < abs(obsSortLeft[2]["y_Rd"]):
                    area = {}
                    area["l"] = abs(y1_PS - obsSortLeft[2]["y_Rd"])
                    area["w"] = x2_PS - obsSortLeft[0]["x_R"]
                    if area["l"] >= self.minVSidePB and area["w"] >= self.minHSidePB:
                        area["x1"] = obsSortLeft[0]["x_R"]
                        area["y1"] = y1_PS
                        area["x2"] = x2_PS
                        area["y2"] = y2_PS
                        area["x3"] = area["x2"]
                        area["y3"] = obsSortLeft[2]["y_Rd"]
                        area["x4"] = area["x1"]
                        area["y4"] = area["y3"]
                        area["b_P1_P"]  = borders[1]
                        area["b_P_End"] = "O"
                        area["b_P2_P"]  = "O"
                        areas.append(area)
                
                else:
                    pass
                    
            elif numObs >= 2:
            
                # area(s) 8 - between lim_P2_P, lim_Rd_P, obsSortLeft[i] and obsSortLeft[i+1]
                for i in range(1, numObs):
                    # obstacle i-1 is closer to the start of the road and farther from the road axis compared to obstacle i
                    if obsSortLeft[i - 1]["x_L"] < obsSortLeft[i]["x_L"] and abs(obsSortLeft[i - 1]["y_Rd"]) > abs(obsSortLeft[i]["y_Rd"]):
                        area = {}
                        area["l"] = abs(y1_PS - obsSortLeft[i - 1]["y_Rd"])
                        area["w"] = obsSortLeft[i]["x_L"] - x1_PS
                        if area["l"] >= self.minVSidePB and area["w"] >= self.minHSidePB:
                            area["x1"] = x1_PS
                            area["y1"] = y1_PS
                            area["x2"] = obsSortLeft[i]["x_L"]
                            area["y2"] = y2_PS
                            area["x3"] = area["x2"]
                            area["y3"] = obsSortLeft[i - 1]["y_Rd"]
                            area["x4"] = area["x1"]
                            area["y4"] = area["y3"]
                            area["b_P1_P"]  = "O"
                            area["b_P_End"] = "O"
                            area["b_P2_P"]  = borders[3]
                            areas.append(area)
                            
                # area(s) 9 - between lim_P1_P, lim_Rd_P, obsSortRight[i] and obsSortRight[i+1]
                for i in range(1, numObs):
                    # obstacle i-1 is closer to the end of the road and farther from the road axis compared to obstacle i
                    if obsSortRight[i - 1]["x_R"] > obsSortRight[i]["x_R"] and abs(obsSortRight[i - 1]["y_Rd"]) > abs(obsSortRight[i]["y_Rd"]):
                        area = {}
                        area["l"] = abs(y1_PS - obsSortRight[i - 1]["y_Rd"])
                        area["w"] = x2_PS - obsSortRight[i]["x_R"]
                        if area["l"] >= self.minVSidePB and area["w"] >= self.minHSidePB:
                            area["x1"] = obsSortRight[i]["x_R"]
                            area["y1"] = y1_PS
                            area["x2"] = x2_PS
                            area["y2"] = y2_PS
                            area["x3"] = area["x2"]
                            area["y3"] = obsSortRight[i - 1]["y_Rd"]
                            area["x4"] = area["x1"]
                            area["y4"] = area["y3"]
                            area["b_P1_P"]  = borders[1]
                            area["b_P_End"] = "O"
                            area["b_P2_P"]  = "O"
                            areas.append(area)
            else:
                pass
            
            if len(areas) > 0: # there is at least one valid parking area
                maxArea = max(areas, key=lambda x:x['w']) # select area with maximum horizontal clearance
                x1_PS = maxArea["x1"]
                x2_PS = maxArea["x2"]
                x3_PS = maxArea["x3"]
                x4_PS = maxArea["x4"]
                y1_PS = maxArea["y1"]
                y2_PS = maxArea["y2"]
                y3_PS = maxArea["y3"]
                y4_PS = maxArea["y4"]
                borders[1] = maxArea["b_P1_P"]
                borders[2] = maxArea["b_P_End"]
                borders[3] = maxArea["b_P2_P"]
            else:
                # invalidPB = True # PB hack - create an invalid parking box
                # PB hack - use the coordinates of the last valid parking area before checking for obstacles (PB will contain obstacles within it)
                (x1_PS, y1_PS, x2_PS, y2_PS, x3_PS, y3_PS, x4_PS, y4_PS) = (x1_PS_temp, y1_PS_temp, x2_PS_temp, y2_PS_temp, x3_PS_temp, y3_PS_temp, x4_PS_temp, y4_PS_temp)
        
        if borders[2] in ["N", "Mp"]: # if there is no high limiter at the end of the parking box, remove extension
            if self.data["Category"].find("Perpendicular") != -1:
                self.minVSidePB -= AP_MAX_ROADSIDE_EXT_PERP_M
            elif self.data["Category"].find("Parallel") != -1:
                self.minVSidePB -= AP_MAX_ROADSIDE_EXT_PAR_M
            elif self.data["Category"].find("Angled") != -1:
                self.minVSidePB -= 0 # TODO: Check this value in requirements
            else:
                print("Scenario catogory not handled - createParkingBox AP_ParkingTestCase")
                sys.exit(0)
        
        y1_PS_prev = y1_PS
        if abs(self.v_Rd - y4_PS) > self.minVSidePB: # if the parking slot is big enough without overlapping with the driving lane
            y1_PS = self.v_Rd
        else:
            y1_PS = self.CMSign * max(abs(y1_PS), abs(y4_PS - self.CMSign * self.minVSidePB)) # overlap the driving lane with only the required distance
        y2_PS = y1_PS
        x1_PS = self.get_interp2d_x(x1_PS, y1_PS_prev, x4_PS, y4_PS, y1_PS)
        x2_PS = self.get_interp2d_x(x2_PS, y1_PS_prev, x3_PS, y3_PS, y2_PS)
        
        if invalidPB == False:
            myTestRun.Traffic.add()
            myTestRun.Traffic.Traffic[self.trafficObjID].Name.value = "PB00"
            myTestRun.Traffic.Traffic[self.trafficObjID].Info.value = "ParkingBox00" + " - " + ''.join(borders)
            myTestRun.Traffic.Traffic[self.trafficObjID].DetectMask.value = "1 1"
            myTestRun.Traffic.Traffic[self.trafficObjID].Color.value = AP_COLOR_RED
            myTestRun.Traffic.Traffic[self.trafficObjID].Movie.Geometry.value = " "
            myTestRun.Traffic.Traffic[self.trafficObjID].Basics.Offset.value = "0.01 0.0" # 1cm above ground
            myTestRun.Traffic.Traffic[self.trafficObjID].Basics.Fr12CoM.value = AP_CENTER_OF_MASS
            
            myTestRun.Traffic.Traffic[self.trafficObjID].Basics.Dimension.value = [0.01, 0.01, AP_PB_HEIGHT] # length and width are filled via contour values; non-zero values must be used to avoid CarMaker error
            
            self.roadSideEdge = y1_PS

            if self.data["Category"].find("Left") != -1: # flip 1<->4 and 2<->3 in order for the contour to have only positive values (clockwise drawing order)
                x1_PS, y1_PS, x4_PS, y4_PS = x4_PS, y4_PS, x1_PS, y1_PS
                x2_PS, y2_PS, x3_PS, y3_PS = x3_PS, y3_PS, x2_PS, y2_PS
            elif self.data["Category"].find("Right") != -1:
                pass
            elif self.data["Category"].find("Angled") != -1: # TODO: check for angled
                pass
            else:
                print("Scenario catogory not handled - createParkingBox AP_ParkingTestCase")
                sys.exit(0)
            
            x4_PS_Contour = 0
            y4_PS_Contour = 0
            x1_PS_Contour = round(x1_PS - x4_PS, 3)
            y1_PS_Contour = round(y1_PS - y4_PS, 3)
            x2_PS_Contour = round(x2_PS - x1_PS + x1_PS_Contour, 3)
            y2_PS_Contour = round(y1_PS_Contour, 3)
            x3_PS_Contour = round(x3_PS - x4_PS, 3)
            y3_PS_Contour = round(y4_PS_Contour, 3)
            myTestRun.Traffic.Traffic[self.trafficObjID].Basics.Contour.Contour.value = "\t"   + str(x4_PS_Contour) + " " + str(y4_PS_Contour) + \
                                                                                        "\n\t" + str(x1_PS_Contour) + " " + str(y1_PS_Contour) + \
                                                                                        "\n\t" + str(x2_PS_Contour) + " " + str(y2_PS_Contour) + \
                                                                                        "\n\t" + str(x3_PS_Contour) + " " + str(y3_PS_Contour) + \
                                                                                        "\n\t" + str(x4_PS_Contour) + " " + str(y4_PS_Contour)
            myTestRun.Traffic.Traffic[self.trafficObjID].Init.Orientation.value = [0.0, 0.0, 0.0] # TODO: sync with road slope and gradient
            
            x_PS = round(x4_PS, 3)
            y_PS = round(y4_PS, 3)
            myTestRun.Traffic.Traffic[self.trafficObjID].Init.Road.value = str(x_PS) + " " + str(y_PS)
            self.trafficObjID = self.trafficObjID + 1
     

    def getCornersPx(self):
        # The corners for right side are:
        #       top left    : x1, y1
        #       top right   : x2, y2
        #       bottom right: x3, y3
        #       bottom left : x4, y4
        # Top and bottom are mirrored for left side (i.e. top is always nearest to road center)
        
        corners_P1 = {}
        corners_P2 = {}
        
        if self.data["exists_P1"] == "true":
            if self.data["Category"].find("Perpendicular") != -1 or self.data["Category"].find("Angled") != -1:
                lat_x1y1_x4y4 = self.data["l_P1"]
                lat_x3y3_x4y4 = self.data["w_P1"]
                if self.data["dir_P1"] == "forward": # determine which segment represents the back of the vehicle
                    idxPct1 = 1
                    idxPct2 = 2
                else:
                    idxPct1 = 3
                    idxPct2 = 4
            elif self.data["Category"].find("Parallel") != -1:
                lat_x1y1_x4y4 = self.data["w_P1"]
                lat_x3y3_x4y4 = self.data["l_P1"]
                if self.data["dir_P1"] == "forward": # determine which segment represents the back of the vehicle
                    idxPct1 = 1
                    idxPct2 = 4
                else:
                    idxPct1 = 2
                    idxPct2 = 3
            else:
                lat_x1y1_x4y4 = 0
                lat_x3y3_x4y4 = 0
            
            if self.data["Category"].find("Angled") != -1:
                corners_P1["y2"] = self.v_Rd - self.CMSign * (self.data["delta_Pass"] - self.data["delta_Pass_Rd"])
                
                d_P1_diff = self.data["d_P1_C1"] - self.data["d_P1_C2"]
                beta = math.asin(abs(d_P1_diff) / self.data["l_Ego"])
                gamma = math.radians(abs(90 - self.data["alpha"])) - np.sign(d_P1_diff) * beta
                theta = math.radians(180 - self.data["alpha"])
                corners_P1["y1"] = corners_P1["y2"] + self.CMSign * (math.sin(gamma) * self.data["w_Ego"]) * np.sign(90 - self.data["alpha"])
                if self.data["alpha"] < 90:
                    corners_P1["x1"] = self.u_Rd + self.data["d_P1_C1"] / math.sin(math.radians(self.data["alpha"])) + abs(corners_P1["y1"] - self.v_Rd) / math.tan(math.radians(self.data["alpha"]))
                    corners_P1["x2"] = corners_P1["x1"] + math.cos(gamma) * self.data["w_Ego"]
                    
                    corners_P1["x4"] = corners_P1["x1"] + math.sin(gamma) * self.data["l_Ego"]
                    corners_P1["y4"] = corners_P1["y1"] + self.CMSign * (math.cos(gamma) * self.data["l_Ego"])
                    
                    corners_P1["x3"] = corners_P1["x4"] + (corners_P1["x2"] - corners_P1["x1"])
                    corners_P1["y3"] = corners_P1["y4"] - self.CMSign * (abs(corners_P1["y2"] - corners_P1["y1"]))
                else:
                    aux = self.data["d_P1_C1"] / math.sin(theta)
                    corners_P1["x1"] = self.get_interp2d_x(self.u_Rd, self.v_Rd, (self.u_P + self.latHPark / 2), self.v_P, corners_P1["y1"]) + aux
                    corners_P1["x2"] = corners_P1["x1"] + math.cos(gamma) * self.data["w_Ego"]
                    
                    corners_P1["x4"] = corners_P1["x1"] - np.sign(self.data["alpha"] - math.degrees(beta) - 90) * math.cos(theta - np.sign(d_P1_diff) * beta) * self.data["l_Ego"] # x4 will be offset to the right once the vehicle exceeds 270°, to the left otherwise
                    corners_P1["y4"] = corners_P1["y1"] + self.CMSign * (math.sin(theta - np.sign(d_P1_diff) * beta) * self.data["l_Ego"])
                    
                    corners_P1["x3"] = corners_P1["x4"] + (corners_P1["x2"] - corners_P1["x1"])
                    corners_P1["y3"] = corners_P1["y4"] + self.CMSign * (abs(corners_P1["y2"] - corners_P1["y1"])) * np.sign(self.data["alpha"] - math.degrees(beta) - 90) # y3 will be closer to the road once the vehicle exceeds 270°, farther otherwise
            
            else:
                corners_P1["x1"] = self.u_Rd + self.data["d_P1_C1"]
                corners_P1["y1"] = self.v_Rd + self.CMSign * self.data["delta_Pass_Rd"]
                
                corners_P1["x4"] = self.u_Rd + self.data["d_P1_C2"]
                corners_P1["y4"] = corners_P1["y1"] + self.CMSign * math.sqrt(lat_x1y1_x4y4**2 - (self.data["d_P1_C1"] - self.data["d_P1_C2"])**2) # Pythagoras in right triangle
        
                corners_P1["x3"] = corners_P1["x4"] + abs(corners_P1["y1"] - corners_P1["y4"]) / lat_x1y1_x4y4 * lat_x3y3_x4y4 # (corners_P1["y1"] - corners_P1["y4"]) / lat_x1y1_x4y4 = (corners_P1["x3"] - corners_P1["x4"]) / lat_x3y3_x4y4 <-> cos in similar right triangles
                corners_P1["y3"] = corners_P1["y4"] + self.CMSign * (self.data["d_P1_C1"] - self.data["d_P1_C2"]) / lat_x1y1_x4y4 * lat_x3y3_x4y4 # (d_P1_C1 - d_P1_C2) / lat_x1y1_x4y4 = (corners_P1["y4"] - corners_P1["y3"]) / lat_x3y3_x4y4 <-> sin in similar right triangles
        
                corners_P1["x2"] = corners_P1["x1"] + (corners_P1["x3"] - corners_P1["x4"])
                corners_P1["y2"] = corners_P1["y1"] + (corners_P1["y3"] - corners_P1["y4"])
            
            (corners_P1["xRef"], corners_P1["yRef"]) = self.getSegmentMiddle(corners_P1["x" + str(idxPct1)], corners_P1["y" + str(idxPct1)], corners_P1["x" + str(idxPct2)], corners_P1["y" + str(idxPct2)])
    
        #vehicle 2
        if self.data["exists_P2"] == "true":
            if self.data["Category"].find("Perpendicular") != -1 or self.data["Category"].find("Angled") != -1:
                lat_x2y2_x3y3 = self.data["l_P2"]
                lat_x3y3_x4y4 = self.data["w_P2"]
                if self.data["dir_P2"] == "forward": # determine which segment represents the back of the vehicle
                    idxPct1 = 1
                    idxPct2 = 2
                else:
                    idxPct1 = 3
                    idxPct2 = 4
            elif self.data["Category"].find("Parallel") != -1:
                lat_x2y2_x3y3 = self.data["w_P2"]
                lat_x3y3_x4y4 = self.data["l_P2"]
                if self.data["dir_P2"] == "forward": # determine which segment represents the back of the vehicle
                    idxPct1 = 1
                    idxPct2 = 4
                else:
                    idxPct1 = 2
                    idxPct2 = 3
            else:
                lat_x2y2_x3y3 = 0
                lat_x3y3_x4y4 = 0
            
            if self.data["Category"].find("Angled") != -1:
                corners_P2["y2"] = self.v_Rd + self.CMSign * self.data["delta_Pass_Rd"]
                
                d_P2_diff = self.data["d_P2_C1"] - self.data["d_P2_C2"]
                beta = math.asin(abs(d_P2_diff) / self.data["l_Ego"])
                gamma = math.radians(90 - self.data["alpha"]) + np.sign(d_P2_diff) * beta
                corners_P2["y1"] = corners_P2["y2"] + self.CMSign * (math.sin(gamma) * self.data["w_Ego"])
                offX2 = self.data["d_P2_C1"] / math.sin(math.radians(self.data["alpha"]))
                # P_Ego-P2 line
                pEgoP2_x1 = self.u_Rd - self.latHPark
                pEgoP2_y1 = self.v_Rd
                pEgoP2_x2 = self.u_P - self.latHPark / 2
                pEgoP2_y2 = self.v_P
                
                corners_P2["x2"] = self.get_interp2d_x(pEgoP2_x1, pEgoP2_y1, pEgoP2_x2, pEgoP2_y2, corners_P2["y2"]) - offX2
                
                corners_P2["x1"] = corners_P2["x2"] - math.cos(gamma) * self.data["w_Ego"]
                
                corners_P2["x4"] = corners_P2["x1"] + math.sin(gamma) * self.data["l_Ego"]
                corners_P2["y4"] = corners_P2["y1"] + self.CMSign * (math.cos(gamma) * self.data["l_Ego"])
                
                corners_P2["x3"] = corners_P2["x4"] + (corners_P2["x2"] - corners_P2["x1"])
                
                if self.data["alpha"] < 90:
                    corners_P2["y3"] = corners_P2["y4"] - self.CMSign * (abs(corners_P2["y2"] - corners_P2["y1"]))
                else:
                    corners_P2["y3"] = corners_P2["y4"] + self.CMSign * (abs(corners_P2["y2"] - corners_P2["y1"])) * np.sign(self.data["alpha"] - math.degrees(beta) - 90) # y3 will be closer to the road once the vehicle exceeds 270°, farther otherwise
            else:            
                corners_P2["x2"] = self.u_Rd - self.latHPark - self.data["d_P2_C1"]
                corners_P2["y2"] = self.v_Rd + self.CMSign * (self.data["delta_Pass_Rd"] - self.data["delta_Pass"])
        
                corners_P2["x3"] = self.u_Rd - self.latHPark - self.data["d_P2_C2"]
                corners_P2["y3"] = corners_P2["y2"] + self.CMSign * math.sqrt(lat_x2y2_x3y3**2 - (self.data["d_P2_C2"] - self.data["d_P2_C1"])**2) # Pythagoras in right triangle
        
                corners_P2["x4"] = corners_P2["x3"] - abs(corners_P2["y2"] - corners_P2["y3"]) / lat_x2y2_x3y3 * lat_x3y3_x4y4 # (corners_P2["y2"] - corners_P2["y3"]) / lat_x2y2_x3y3 = (corners_P2["x3"] - corners_P2["x4"]) / lat_x3y3_x4y4 <-> cos in similar right triangles
                corners_P2["y4"] = corners_P2["y3"] - self.CMSign * (self.data["d_P2_C2"] - self.data["d_P2_C1"]) / lat_x2y2_x3y3 * lat_x3y3_x4y4 # (d_P2_C2 - d_P2_C1) / lat_x2y2_x3y3 = (corners_P2["y4"] - corners_P2["y3"]) / lat_x3y3_x4y4 <-> sin in similar right triangles
        
                corners_P2["x1"] = corners_P2["x2"] + (corners_P2["x4"] - corners_P2["x3"])
                corners_P2["y1"] = corners_P2["y2"] + (corners_P2["y4"] - corners_P2["y3"])
            
            (corners_P2["xRef"], corners_P2["yRef"]) = self.getSegmentMiddle(corners_P2["x" + str(idxPct1)], corners_P2["y" + str(idxPct1)], corners_P2["x" + str(idxPct2)], corners_P2["y" + str(idxPct2)])
    
        return (corners_P1, corners_P2)
    
    
    def fillPxData(self, myTestRun, idxP, corners):
        myTestRun.Traffic.add()
        myTestRun.Traffic.Traffic[self.trafficObjID].Info.value = "Parking Vehicle " + idxP
        myTestRun.Traffic.Traffic[self.trafficObjID].Movie.Geometry.value = self.getMovieGeometry(self.data["type_P" + str(idxP)])
        myTestRun.Traffic.Traffic[self.trafficObjID].Color.value = AP_COLOR_YELLOW
        myTestRun.Traffic.Traffic[self.trafficObjID].Basics.Dimension.value = [self.data["l_P" + idxP], self.data["w_P" + idxP], self.data["h_P" + idxP]]
        myTestRun.Traffic.Traffic[self.trafficObjID].Basics.Fr12CoM.value = AP_CENTER_OF_MASS
        myTestRun.Traffic.Traffic[self.trafficObjID].Basics.Contour.Contour.value = self.getVehicleContour(self.data["type_P" + str(idxP)], idxP)
        myTestRun.Traffic.Traffic[self.trafficObjID].Basics.Contour.Mirror.value = 1
        myTestRun.Traffic.Traffic[self.trafficObjID].Init.Orientation.value = [0.0, 0.0, self.getAnglePx(idxP)]
        myTestRun.Traffic.Traffic[self.trafficObjID].Init.Road.value = str(corners["xRef"]) + " " + str(corners["yRef"])
        myTestRun.Traffic.Traffic[self.trafficObjID].DetectMask.value = "1 1"
        myTestRun.Traffic.Traffic[self.trafficObjID].Basics.Offset.value = "0.2 0.0" # 0.2 is ground clearance TODO: check if car height should be adjusted based on this
        myTestRun.Traffic.Traffic[self.trafficObjID].Route.value = "0 0"
        self.trafficObjID = self.trafficObjID + 1
        
        
    def getMovieGeometry(self, vehType):
        if vehType == "VW Passat B8":
            movieGeometry = "VW_Passat_B8_Variant_noWheels_rotated_black.obj"
        elif vehType == "MB W222-2014":
            movieGeometry = "MB_SClass_W222_2014.mobj"
        elif vehType == "VW Golf 7":
            movieGeometry = "VW_Golf_7_blue.mobj"
        elif vehType == "Jeep Grand Cherokee":
            movieGeometry = "Chrysler_Jeep_GC_Overland_2014.mobj"
        elif vehType == "Ford F-150 Raptor":
            movieGeometry = "3D/Vehicles/Ford_F150_Raptor_2017.mobj"
        elif vehType == "Chevrolet Silverado":
            movieGeometry = "3D/Vehicles/Chevrolet_Silverado1500_2013.mobj"
        elif vehType == "Smart":
            movieGeometry = "VW_Golf_7_blue.mobj"
            print("Movie geometry for Smart car replaced with VW Golf 7")
        else:
            movieGeometry = "VW_Golf_7_blue.mobj"
            print(("Movie geometry for {} replaced with VW Golf 7".format(vehType)))
            
        return movieGeometry
        
        
    def getVehicleContour(self, vehType, idxP):
        if vehType == "VW Passat B8":
            vehContour = "\t0.000 0.000\n\t0.050 0.500\n\t0.120 0.685\n\t0.250 0.770\n\t0.740 0.916\n\t4.350 0.916\n\t4.500 0.850\n\t4.767 0.500\n\t4.767 0.000"
        # elif vehType == "MB W222-2014":
        elif vehType == "VW Golf 7":
            vehContour = "\t0.000 0.000\n\t0.060 0.450\n\t0.200 0.685\n\t0.320 0.820\n\t0.560 0.900\n\t0.760 0.900\n\t3.800 0.900\n\t4.000 0.800\n\t4.120 0.700\n\t4.230 0.520\n\t4.250 0.500\n\t4.300 0.000"
        elif vehType == "Jeep Grand Cherokee":
            vehContour = "\t0.0 0.0\n\t0.02 0.5\n\t0.1 0.8\n\t0.18 0.85\n\t0.3 0.9\n\t0.7 1\n\t3.6 1\n\t4.25 0.98\n\t4.3 0.96\n\t4.5 0.83\n\t4.62 0.75\n\t4.65 0.7\n\t4.75 0.5\n\t4.82 0"
        elif vehType == "Ford F-150 Raptor":
            vehContour = "\t0.00 0.00\n\t0.06 0.85\n\t0.1 0.95\n\t0.6 1.05\n\t1.0 1.13\n\t5.05 1.13\n\t5.26 1.03\n\t5.61 0.55\n\t5.61 0.00"
        elif vehType == "Chevrolet Silverado":
            vehContour = "\t0.00 0.00\n\t0.04 0.9\n\t0.35 1\n\t0.95 1.08\n\t4.9 1.08\n\t5.31 1.02\n\t5.45 0.98\n\t5.63 0.5\n\t5.72 0.00"
        # elif vehType == "Smart":
        else:
            vehContour = "\t0.0 0.0\n\t0.0 " + str(self.data["w_P" + idxP] / 2) + "\n\t" + str(self.data["l_P" + idxP]) + " " + str(self.data["w_P" + idxP] / 2) + "\n\t" + str(self.data["l_P" + idxP]) + " 0.0"
            print(("Vehicle contour for {} replaced with box (length X width)".format(vehType)))
            
        return vehContour
        
        
    def getSegmentMiddle(self, x1, y1, x2, y2):
        xM = x1 + (x2 - x1) / 2
        yM = y1 + (y2 - y1) / 2
        # use only 3 decimals to avoid CarMaker errors (CarMaker will not handle the orientation correctly if there are too many decimals for starting position)
        (xM, yM) = tuple(map(lambda x: round(x, 3), (xM, yM)))
        return (xM, yM)
        
        
    def getAnglePx(self, idxP):
        if self.data["Category"].find("Perpendicular") != -1:
            if self.data["dir_P" + idxP] == "forward":
                offset = AP_ANG_OFFSET_PERP_FWD
            else:
                offset = AP_ANG_OFFSET_PERP_REV
            offset -= self.CMSign * 90
        elif self.data["Category"].find("Parallel") != -1:
            if self.data["dir_P" + idxP] == "forward":
                offset = AP_ANG_OFFSET_PAR_FWD
            else:
                offset = AP_ANG_OFFSET_PAR_REV
        elif self.data["Category"].find("Angled") != -1:
            # TODO: Check if there is a possibility for reverse parking for angled situations
            if self.data["dir_P" + idxP] == "forward":
                offset = AP_ANG_OFFSET_PAR_FWD
            else:
                offset = AP_ANG_OFFSET_PAR_REV
            offset += self.CMSign * self.data["alpha"]
        else:
            offset = 0
        
        return round(offset + self.data["psi_P" + idxP], 3)

        
    def fillLimiterData(self, myTestRun, limiterType): # limiterType references a column in the excel scenario catalogue (not a limiter height)
        myTestRun.Traffic.add()
        myTestRun.Traffic.Traffic[self.trafficObjID].Name.value = "Lim" + '{:02d}'.format(self.trafficObjID)
        myTestRun.Traffic.Traffic[self.trafficObjID].Info.value = "Parking limiter " + limiterType + " - " + self.data[limiterType]
        myTestRun.Traffic.Traffic[self.trafficObjID].Movie.Geometry.value = " " # TODO: get textures for limiters
        
        edge = {}
            
        edge["h"] = self.latHPark
        edge["v"] = self.latVPark
        # TODO: add handling for angled parking
            
        lengthLim = edge[limTypeParamConfig[limiterType].edgeType] * limTypeParamConfig[limiterType].lenFact
        if self.data["Category"].find("Angled") != -1:
            if limTypeParamConfig[limiterType].edgeType == "h":
                lengthLim += self.xOffPStart_Ang + self.xOffPEnd_Ang # unused surface in the final lane sections of the parking area (triangle)
            else:
                lengthLim = self.l_Ang
        lengthLim = round(lengthLim, 3)
        widthLim = AP_LIM_WIDTH # TODO: check if limiter width is important
        
        lim = self.data[limiterType]
        
        if lim in list(self.limHeight.keys()):
            myTestRun.Traffic.Traffic[self.trafficObjID].Basics.Dimension.value = [lengthLim, widthLim, float(self.limHeight[lim])]
            color = AP_COLOR_GREY
            offsetZ = 0.0
            lineLimFact = 1
        else:
            myTestRun.Traffic.Traffic[self.trafficObjID].Basics.Dimension.value = [lengthLim, widthLim, 0.001]
            color = AP_COLOR_WHITE
            offsetZ = AP_LINE_LIM_OFFSET_Z
            lineLimFact = 0
        
        myTestRun.Traffic.Traffic[self.trafficObjID].Basics.Fr12CoM.value = AP_CENTER_OF_MASS
        myTestRun.Traffic.Traffic[self.trafficObjID].Basics.Contour.Contour.value = "\t0.0 0.0\n\t0.0 " + str(widthLim) + "\n\t" + str(lengthLim) + " " + str(widthLim) + "\n\t" + str(lengthLim) + " 0.0\n\t0.0 0.0"
        myTestRun.Traffic.Traffic[self.trafficObjID].Color.value = color       
        myTestRun.Traffic.Traffic[self.trafficObjID].Basics.Offset.value = str(offsetZ) + " 0.0"
        
        startX = self.u_Rd - limTypeParamConfig[limiterType].factX * edge["h"] + limTypeParamConfig[limiterType].offsetX * lineLimFact
        startY = self.v_Rd + self.CMSign * (limTypeParamConfig[limiterType].factY * edge["v"] + limTypeParamConfig[limiterType].offsetY * lineLimFact)
        
        orientationZ = float(limTypeParamConfig[limiterType].orientation)
        # latOrientation = max(math.abs(self.data["slope"]), math.abs(self.data["camber"]) * camberFactor) # TODO: determine camber factor / camber calculation for deduction of angle
        # latOrientation = self.data["slope"]
        if limTypeParamConfig[limiterType].edgeType == "v":
            # orientationX = math.degrees(math.atan(self.data["grad"])) # longSlope
            # orientationY = math.degrees(math.atan(latOrientation))
            if self.data["Category"].find("Angled") != -1:
                orientationZ += self.CMSign * self.data["alpha"] - 180 # Subtracted 180° to start from 0 (angled parking)
                startX = startX + self.CMSign * AP_LIM_WIDTH / 2 # TODO: modify value based on alpha (might be best to redraw limiters with mirroring contour)
                # startY += # TODO: modify value based on alpha
            else:
                orientationZ -= self.CMSign * 90 # if the limiter is perpendicular to the road add (for right) or subtract (for left) 90°
                startX = startX + self.CMSign * AP_LIM_WIDTH / 2
        else:
            startY = startY - AP_LIM_WIDTH / 2
            startX -= self.xOffPStart_Ang
            
        startX = round(startX, 3)
        startY = round(startY, 3)
        
        myTestRun.Traffic.Traffic[self.trafficObjID].Init.Orientation.value = [0.0, 0.0, orientationZ]
        myTestRun.Traffic.Traffic[self.trafficObjID].DetectMask.value = "1 1"
        myTestRun.Traffic.Traffic[self.trafficObjID].Init.Road.value = str(startX) + " " + str(startY)
        self.trafficObjID = self.trafficObjID + 1 
    
    
    def calcObstacleData(self, myTestRun, idxObst):
        obstacle = {}
        contour = ""
        orientation = 0 # TODO: check orientation for left vs right (180° may be necessary)
        if self.data["shape_Obs" + idxObst] == "circle":
            for i in range(0, 181, int(180 / AP_CIRC_OBST_NUM_SEG)): # for (i = 0°; i <= 180°; i+= segmentSize)
                circlePx = round(self.data["r_Obs" + idxObst] + self.data["r_Obs" + idxObst] * math.cos(math.radians(i)), 3)
                if i == 180:
                    circlePy = 0 # this is added because sin(180°) != 0 in python (e-16 value)
                else:
                    circlePy = round(self.data["r_Obs" + idxObst] * math.sin(math.radians(i)), 3)
                contour = "\t" + str(circlePx) + " " + str(circlePy) + "\n" + contour
                halfObst = self.data["r_Obs" + idxObst] / 2
        elif self.data["shape_Obs" + idxObst] == "rect":
            w = str(self.data["w_Obs" + idxObst] / 2)
            l = str(self.data["l_Obs" + idxObst])
            contour = "\t0.0 0.0\n\t0.0 " + w + "\n\t" + l + " " + w + "\n\t" + l + " 0.0"
            halfObst = self.data["w_Obs" + idxObst] / 2
        else:
            print(("Obstacle shape {} not supported.".format(self.data["shape_Obs" + idxObst])))
            sys.exit(0)
            
        self.data["con_Obs" + idxObst] = contour
        if self.data["Category"].find("Angled") != -1:
            theta = 360 - (self.data["alpha"] - 90)
            if self.data["shape_Obs" + idxObst] == "rect":
                orientation = (-1) * self.CMSign * theta                
        self.data["or_Obs" + idxObst] = orientation
        
        if self.data["area_Obs" + idxObst] == "parking":
            startX = self.u_P
            startY = self.v_P
        elif self.data["area_Obs" + idxObst] == "road":
            startX = self.u_Rd
            startY = self.v_Rd
        else:
            print(("Obstacle are {} not supported.".format(self.data["area_Obs" + idxObst])))
            sys.exit(0)
            
        if self.data["Category"].find("Angled") != -1:
            theta = math.radians(theta)
            offX = self.data["u_Obs" + idxObst] * math.cos(theta) - self.data["v_Obs" + idxObst] * math.sin(theta)
            offY = (-1) * self.CMSign * (self.data["u_Obs" + idxObst] * math.sin(theta) + self.data["v_Obs" + idxObst] * math.cos(theta))
            startX += offX
            startY += offY - self.CMSign * halfObst
        else:
            startX = startX + self.data["u_Obs" + idxObst]
            startY = startY - self.CMSign * (self.data["v_Obs" + idxObst] + halfObst) # halfObst is required because the contour is mirrored (0,0 point is the middle of a segment)
            
        if self.data["shape_Obs" + idxObst] == "circle":
            startX = startX - self.data["r_Obs" + idxObst]
        
        self.data["stX_Obs" + idxObst] = round(startX, 3)
        self.data["stY_Obs" + idxObst] = round(startY, 3)
        
        if self.data["area_Obs" + idxObst] == "parking":
            obstacle["x_L"] = startX
            if self.data["shape_Obs" + idxObst] == "rect":
                obstacle["x_R"] = startX + self.data["l_Obs" + idxObst]
                obstacle["y_Rd"] = startY - self.CMSign * self.data["w_Obs" + idxObst] / 2
            else:
                obstacle["x_R"] = startX + 2 * self.data["r_Obs" + idxObst]
                obstacle["y_Rd"] = startY - self.CMSign * self.data["r_Obs" + idxObst]
            
            if self.data["h_Obs" + idxObst] > self.limHeight["curb_t"]: # TODO: account for h_grd_Obs
                self.obstacles.append(obstacle)
    

    def fillObstacleData(self, myTestRun, idxObst):
        myTestRun.Traffic.add()         #add new traffic object
        myTestRun.Traffic.Traffic[self.trafficObjID].Name.value = "Obs" + '{:02d}'.format(self.trafficObjID)
        myTestRun.Traffic.Traffic[self.trafficObjID].Info.value = "Obstacle " +  idxObst + " - " + self.data["shape_Obs" + idxObst]
        myTestRun.Traffic.Traffic[self.trafficObjID].Movie.Geometry.value = " " # TODO: get textures for obstacles
        myTestRun.Traffic.Traffic[self.trafficObjID].Color.value = AP_COLOR_BROWN
        myTestRun.Traffic.Traffic[self.trafficObjID].Basics.Dimension.value = [0.01, 0.01, float(self.data["h_Obs" + idxObst])] # length and width are disregarded because the size is set via countour values; non-zero values must be used to avoid CarMaker error
        myTestRun.Traffic.Traffic[self.trafficObjID].Basics.Fr12CoM.value = AP_CENTER_OF_MASS
        myTestRun.Traffic.Traffic[self.trafficObjID].Basics.Contour.Contour.value = self.data["con_Obs" + idxObst]
        myTestRun.Traffic.Traffic[self.trafficObjID].Basics.Contour.Mirror.value = 1
        myTestRun.Traffic.Traffic[self.trafficObjID].Init.Orientation.value = [0.0, 0.0, float(self.data["or_Obs" + idxObst])]
        
        if self.is_number_tryexcept(self.data["h_grd_Obs" + idxObst]) == True:
            myTestRun.Traffic.Traffic[self.trafficObjID].Basics.Offset.value = str(self.data["h_grd_Obs" + idxObst]) + " 0.0"
        else:
            myTestRun.Traffic.Traffic[self.trafficObjID].Basics.Offset.value = "0.0 0.0"

        myTestRun.Traffic.Traffic[self.trafficObjID].DetectMask.value = "1 1"
        myTestRun.Traffic.Traffic[self.trafficObjID].Init.Road.value = str(self.data["stX_Obs" + idxObst]) + " " + str(self.data["stY_Obs" + idxObst])
        self.trafficObjID = self.trafficObjID + 1


    def fillWheelstopperData(self, myTestRun):
        myTestRun.Traffic.add()         #add new traffic object
        myTestRun.Traffic.Traffic[self.trafficObjID].Name.value = "Whs" + '{:02d}'.format(self.trafficObjID)
        myTestRun.Traffic.Traffic[self.trafficObjID].Info.value = "Wheelstopper"
        myTestRun.Traffic.Traffic[self.trafficObjID].Movie.Geometry.value = " " # TODO: get textures for wheelstopper
        myTestRun.Traffic.Traffic[self.trafficObjID].Color.value = AP_COLOR_YELLOW
        myTestRun.Traffic.Traffic[self.trafficObjID].Basics.Dimension.value = [0.01, 0.01, self.wheelstopperHeight[AP_WHS_DEFAULT_TYPE]] # TODO: get wheelstopper height
        myTestRun.Traffic.Traffic[self.trafficObjID].Basics.Fr12CoM.value = AP_CENTER_OF_MASS
        myTestRun.Traffic.Traffic[self.trafficObjID].Basics.Contour.Contour.value = "\t0.0 0.0\n\t0.0 " + str(self.wheelstopperWidth[AP_WHS_DEFAULT_TYPE] / 2) + "\n\t" + str(self.wheelstopperLength[AP_WHS_DEFAULT_TYPE]) + " " + str(self.wheelstopperWidth[AP_WHS_DEFAULT_TYPE] / 2) + "\n\t" + str(self.wheelstopperLength[AP_WHS_DEFAULT_TYPE]) + " 0.0"
        myTestRun.Traffic.Traffic[self.trafficObjID].Basics.Contour.Mirror.value = 1
        
        if self.data["Category"].find("Perpendicular") != -1:
            orientationZ = 0.0
            startX = self.u_Rd - self.latHPark / 2 - self.wheelstopperLength[AP_WHS_DEFAULT_TYPE] / 2
            startY = self.v_Rd + self.CMSign * (self.latVPark - self.wheelstopperWidth[AP_WHS_DEFAULT_TYPE] / 2 - float(self.data["d_ws"]))
        elif self.data["Category"].find("Parallel") != -1:
            orientationZ = -self.CMSign * 90.0 # the wheelstopper is perpendicular to the road
            startX = self.u_Rd - self.latHPark + self.wheelstopperWidth[AP_WHS_DEFAULT_TYPE] / 2 + float(self.data["d_ws"])
            startY = self.v_Rd + self.CMSign * (self.latVPark / 2 + self.wheelstopperLength[AP_WHS_DEFAULT_TYPE] / 2)
        elif self.data["Category"].find("Angled") != -1:
            theta = 360 - (self.data["alpha"] - 90)
            orientationZ = (-1) * self.CMSign * theta
            theta = math.radians(theta)
            offX = (-self.wheelstopperLength[AP_WHS_DEFAULT_TYPE] / 2) * math.cos(theta) - self.data["d_ws"] * math.sin(theta)
            offY = (-1) * self.CMSign * ((-self.wheelstopperLength[AP_WHS_DEFAULT_TYPE] / 2) * math.sin(theta) + self.data["d_ws"] * math.cos(theta))
            startX = self.u_P + offX
            startY = self.v_P + offY - self.CMSign * self.wheelstopperWidth[AP_WHS_DEFAULT_TYPE] / 2
        else:
            print("Function Wheelstopper: Category not handled.")
            sys.exit(0)
        
        startX = round(startX, 3)
        startY = round(startY, 3)
        
        myTestRun.Traffic.Traffic[self.trafficObjID].Init.Orientation.value = [0.0, 0.0, orientationZ]
        myTestRun.Traffic.Traffic[self.trafficObjID].DetectMask.value = "1 1"
        myTestRun.Traffic.Traffic[self.trafficObjID].Init.Road.value = str(startX) + " " + str(startY)
        self.trafficObjID = self.trafficObjID + 1
        
        
    def fillOdometryData(self, myTestRun):
        myTestRun.Traffic.add()
        myTestRun.Traffic.Traffic[self.trafficObjID].ObjectKind.value = "Movable"
        myTestRun.Traffic.Traffic[self.trafficObjID].Name.value = "Odo"
        myTestRun.Traffic.Traffic[self.trafficObjID].Info.value = "Odometry Box"
        myTestRun.Traffic.Traffic[self.trafficObjID].Movie.Geometry.value = " "
        myTestRun.Traffic.Traffic[self.trafficObjID].Color.value = AP_COLOR_WHITE
        myTestRun.Traffic.Traffic[self.trafficObjID].Basics.Dimension.value = [0.1, 0.2, 1.5]
        myTestRun.Traffic.Traffic[self.trafficObjID].Basics.Offset.value = "0.2 0.0"
        myTestRun.Traffic.Traffic[self.trafficObjID].Basics.Fr12CoM.value = AP_CENTER_OF_MASS
        myTestRun.Traffic.Traffic[self.trafficObjID].Basics.Contour.Mirror.value = 1
        myTestRun.Traffic.Traffic[self.trafficObjID].Basics.Contour.Contour.value = "\t0.0 0.0\n\t0.0 0.1\n\t0.1 0.0"
        myTestRun.Traffic.Traffic[self.trafficObjID].Init.Orientation.value = [0.0, 0.0, 0.0]
        myTestRun.Traffic.Traffic[self.trafficObjID].FreeMotion.value = 1
        myTestRun.Traffic.Traffic[self.trafficObjID].DetectMask.value = "1 1"
        myTestRun.Traffic.Traffic[self.trafficObjID].Route.value = "0 0"
        myTestRun.Traffic.Traffic[self.trafficObjID].Init.v.value = 0.0
        myTestRun.Traffic.Traffic[self.trafficObjID].Init.Road.value = "0 0"
        myTestRun.Traffic.Traffic[self.trafficObjID].Man.TreatAtEnd.value = "FreezePos"
        myTestRun.Traffic.Traffic[self.trafficObjID].Man.N.value = "0"
        self.trafficObjID = self.trafficObjID + 1

    
    def fillManeuverGlobalSettings(self, myTestRun):
        myTestRun.DrivMan.Cmds.value = ManCmdMoveOdoArrow
        
    
    def fillManeuverInit(self, myTestRun, index):
        myTestRun.DrivMan.DrivManList.add()
        myTestRun.DrivMan.DrivManList[index].Info.value = "Init"
        myTestRun.DrivMan.DrivManList[index].Label.value = ""
        myTestRun.DrivMan.DrivManList[index].TimeLimit.value = 0.5
        myTestRun.DrivMan.DrivManList[index].Cmds.value = ManCmdResetSteerWhl + "\n" + \
                                                          ManCmdDisableIPGLat + "\n" + \
                                                          "\t\"\"\n" + \
                                                          ManCmdEnableVEDODO + "\n" + \
                                                          ManCmdConfigureLSCA + "\n" + \
                                                          ManCmdEnableLimitFOV + "\n" + \
                                                          ManCmdDisableLatency + "\n" + \
                                                          ManCmdDisableSI
        myTestRun.DrivMan.DrivManList[index].LongDyn.value = "Manual"
        myTestRun.DrivMan.DrivManList[index].Clutch.value = "0 0 0.2 abs"
        myTestRun.DrivMan.DrivManList[index].Gas.value = "0 0 0.2 abs"
        myTestRun.DrivMan.DrivManList[index].Brake.value = "1 0 0.2 abs"
        myTestRun.DrivMan.DrivManList[index].BrakePark.value = "0 0 0.2 abs"
        myTestRun.DrivMan.DrivManList[index].LatDyn.value = "Driver " + str(self.latOffset)

        
    def fillManeuverEnableAP(self, myTestRun, index):
        myTestRun.DrivMan.DrivManList.add()
        myTestRun.DrivMan.DrivManList[index].Info.value = "AP activation"
        myTestRun.DrivMan.DrivManList[index].Label.value = ""
        myTestRun.DrivMan.DrivManList[index].TimeLimit.value = 1.0
        myTestRun.DrivMan.DrivManList[index].Cmds.value = ManCmdEnableAP
        myTestRun.DrivMan.DrivManList[index].LongDyn.value = "Manual"
        myTestRun.DrivMan.DrivManList[index].Clutch.value = "0 0 0.2 abs"
        myTestRun.DrivMan.DrivManList[index].Gas.value = "0 0 0.2 abs"
        myTestRun.DrivMan.DrivManList[index].Brake.value = "1 0 0.2 abs"
        myTestRun.DrivMan.DrivManList[index].BrakePark.value = "0 0 0.2 abs"
        myTestRun.DrivMan.DrivManList[index].LatDyn.value = "Driver " + str(self.latOffset)

        
    def fillManeuverVelOverwrite(self, myTestRun, index):
        myTestRun.DrivMan.DrivManList.add()
        myTestRun.DrivMan.DrivManList[index].Info.value = "Disable scanning limitations and velocity overwrite"
        myTestRun.DrivMan.DrivManList[index].Label.value = ""
        myTestRun.DrivMan.DrivManList[index].TimeLimit.value = 0.5
        myTestRun.DrivMan.DrivManList[index].Cmds.value = ManCmdDisableLimitFOV + "\n" + \
                                                          ManCmdDisableLatency + "\n" + \
                                                          "\t\"\"\n" + \
                                                          ManCmdVelOverwrite
        myTestRun.DrivMan.DrivManList[index].LatDyn.value = "Driver " + str(self.latOffset)


    def fillManeuverScan(self, myTestRun, index):
        myTestRun.DrivMan.DrivManList.add()
        myTestRun.DrivMan.DrivManList[index].Info.value = "Scanning phase"
        myTestRun.DrivMan.DrivManList[index].Label.value = ""
        
        if self.scanOn == AP_SCAN_ON_STRAIGHT:
            dist_sRoad = AP_STR_SCAN_DRV_DIST
        else:
            dist = self.scanDist - AP_ANG_SCAN_BRK_DIST
            dist_sRoad = round(dist * math.cos(math.radians(self.data["psi_Ego_0"])), 3)
        
        myTestRun.DrivMan.DrivManList[index].Cmds.value = ManCmdPreventRoll + "\n" + \
                                                          "\t\"\"\n" + \
                                                          "\t# Scanning with constant velocity complete" + \
                                                          "\n\tEval first() ? Qu::StartPos = Vhcl.sRoad" + \
                                                          "\n\tEval Vhcl.sRoad >= StartPos + " + str(dist_sRoad) + " ? ManJump(\"+1\")"
        myTestRun.DrivMan.DrivManList[index].LongDyn.value = "VelControl 10 0.0 1.0 0.0 1 0.0"

        
    def fillManeuverStop(self, myTestRun, index):
        myTestRun.DrivMan.DrivManList.add()
        myTestRun.DrivMan.DrivManList[index].Info.value = "Stopping phase"
        myTestRun.DrivMan.DrivManList[index].Label.value = ""
        myTestRun.DrivMan.DrivManList[index].Cmds.value = ManCmdScanBrkDone
        
        if self.scanOn == AP_SCAN_ON_STRAIGHT:
            stopDecel = 0
            stopDist = AP_STR_SCAN_BRK_DIST - AP_STR_SCAN_BRK_TOL
        else:
            stopDecel = AP_SCAN_DECEL
            stopDist = 0
        
        myTestRun.DrivMan.DrivManList[index].LongDyn.value = "Stop " + str(stopDecel) + " " + str(stopDist)

        
    def fillManeuverHMIAndReverse(self, myTestRun, index):
        myTestRun.DrivMan.DrivManList.add()
        myTestRun.DrivMan.DrivManList[index].Info.value = "HMI input and shift to R"
        myTestRun.DrivMan.DrivManList[index].Label.value = ""
        myTestRun.DrivMan.DrivManList[index].TimeLimit.value = 1.4
        
        switchDirection = ""
        if self.data["Maneuver"].find('forward') != - 1 and self.data["Category"].find("Angled") == -1:
            switchDirection = "\t\n" + ManCmdSwitchDirection

        myTestRun.DrivMan.DrivManList[index].Cmds.value = ManCmdShiftInR + "\n" + \
                                                          "\t\"\"\n" + \
                                                          ManCmdStartSelection + \
                                                          switchDirection + \
                                                          "\t\n" + \
                                                          ManCmdStartParking
        myTestRun.DrivMan.DrivManList[index].LongDyn.value = "Manual"
        myTestRun.DrivMan.DrivManList[index].Clutch.value = "0 0 0.2 abs"
        myTestRun.DrivMan.DrivManList[index].Gas.value = "0 0 0.2 abs"
        myTestRun.DrivMan.DrivManList[index].Brake.value = "1 0 0.2 abs"
        myTestRun.DrivMan.DrivManList[index].BrakePark.value = "0 0 0.2 abs"

        
    def fillManeuverAPCtrl(self, myTestRun, index):
        myTestRun.DrivMan.DrivManList.add()
        myTestRun.DrivMan.DrivManList[index].Info.value = "AP control"
        myTestRun.DrivMan.DrivManList[index].Label.value = ""
        myTestRun.DrivMan.DrivManList[index].EndCondition.value = "AP.headUnitVisualizationPort.screen_nu == 5 || AP.headUnitVisualizationPort.screen_nu == 6"
        myTestRun.DrivMan.DrivManList[index].Cmds.value = ManCmdStopTRNoSteer + "\n" + \
                                                          "\t\"\"\n" + \
                                                          ManCmdStopTRTime
        myTestRun.DrivMan.DrivManList[index].LongDyn.value = "Manual"

        
    def fillManeuverEnd(self, myTestRun, index):
        myTestRun.DrivMan.DrivManList[index].Info.value = "End testrun"
        myTestRun.DrivMan.DrivManList[index].Label.value = "END"
        myTestRun.DrivMan.DrivManList[index].TimeLimit.value = 0.1 # TODO: this should be set to minimum duration required for STET
        myTestRun.DrivMan.DrivManList[index].LongDyn.value = "Manual"

        
    def fillManeuverData(self, myTestRun):
        # Manipulate Driver Maneuvers        
        index = 0
        
        # Global settings / Preparation
        self.fillManeuverGlobalSettings(myTestRun)
        
        # Testrun Init
        self.fillManeuverInit(myTestRun, index)
        index += 1
        
        # Enable AP function
        self.fillManeuverEnableAP(myTestRun, index)
        index += 1
        
        # Check if scanning phase is possible
        if self.scanOn != AP_SCAN_OFF: # use scanning maneuvers
            self.fillManeuverScan(myTestRun, index)
            index += 1
            self.fillManeuverStop(myTestRun, index)
            index += 1
        else: # load entire testrun map with no sensor limitations
            self.fillManeuverVelOverwrite(myTestRun, index)
            index += 1
        
        # Select Reverse gear and start parking
        self.fillManeuverHMIAndReverse(myTestRun, index)
        index += 1
        
        # Monitor AP algo (end when parking is successfull / no steering request for time threshold / parking takes too long)
        self.fillManeuverAPCtrl(myTestRun, index)
        index += 1
        
        # Testrun end
        self.fillManeuverEnd(myTestRun, index)

        
    def fillRoadData(self, myTestRun):
        # road curvature
        if self.data["kappa"] == 0:
            myTestRun.Road.Definition.Links[0].Segments[0].Type = 'Straight'
            myTestRun.Road.Definition.Links[0].Segments[0].Dim1 = AP_ROAD_LENGTH
            myTestRun.Road.Definition.Links[0].Segments[0].Dim2 = 0
        else:
            if self.data["kappa"] > 0:
                myTestRun.Road.Definition.Links[0].Segments[0].Type = 'TurnLeft'
            else:
                myTestRun.Road.Definition.Links[0].Segments[0].Type = 'TurnRight' 
            myTestRun.Road.Definition.Links[0].Segments[0].Dim1 = 1/abs(self.data["kappa"])
            myTestRun.Road.Definition.Links[0].Segments[0].Dim2 = AP_ROAD_LENGTH * 180 * abs(self.data["kappa"]) / math.pi # Arc circle legnth L = pi * R * alfa / 180
            
        # elevation profile
        if self.data["hill"] != 0.0:
            if self.data["grad"] != 0:
                print("Longitudinal elevation and hill not handled simultaneously - fillRoadData()")
                sys.exit(0)
            # r = 1 / hill (circle radius)
            # l = road length (length of arc of circle)
            # θ = arc circle angle (rad)
            # l = r x θ <=> θ = l / r
            # trigonometric equations inside right triangle
            # r = hypotenuse
            # adjacent cathetus = cos(θ/2) * r
            # CM hill height = r - adjacent cathetus
            radius = 1 / self.data["hill"]
            hillHeight = radius - math.cos((AP_ROAD_LENGTH / radius) / 2) * radius
            myTestRun.Road.Definition.Links[0].ElevationProfile.ProfilePoints[1].Value = hillHeight
            grade = hillHeight / (AP_ROAD_LENGTH / 2) * 2 # 2 x hill slope (%) - TODO: multiplication by 2 based on observation; check how to determine actual value
            myTestRun.Road.Definition.Links[0].ElevationProfile.ProfilePoints[0].Grade = grade
            myTestRun.Road.Definition.Links[0].ElevationProfile.ProfilePoints[1].Grade = 0
            myTestRun.Road.Definition.Links[0].ElevationProfile.ProfilePoints[2].Grade = (-1) * grade
        else:
            myTestRun.Road.Definition.Links[0].ElevationProfile.ProfilePoints[1].Value = (self.data["grad"] * AP_ROAD_LENGTH / 100) / 2 # catalogue value is in %
            myTestRun.Road.Definition.Links[0].ElevationProfile.ProfilePoints[2].Value = self.data["grad"] * AP_ROAD_LENGTH / 100 # catalogue value is in %
            myTestRun.Road.Definition.Links[0].ElevationProfile.ProfilePoints[0].Grade = self.data["grad"] / 100
            myTestRun.Road.Definition.Links[0].ElevationProfile.ProfilePoints[1].Grade = self.data["grad"] / 100
            myTestRun.Road.Definition.Links[0].ElevationProfile.ProfilePoints[2].Grade = self.data["grad"] / 100
            myTestRun.Road.Definition.Links[0].ElevationProfile.ProfilePoints[1].Offset = AP_ROAD_LENGTH / 2
            myTestRun.Road.Definition.Links[0].ElevationProfile.ProfilePoints[2].Offset = AP_ROAD_LENGTH
        
        myTestRun.Road.Definition.Links[0].SlopeProfile.ProfilePoints[0].Value     = self.data["slope"] / 100 # catalogue value is in %
        myTestRun.Road.Definition.Links[0].SlopeProfile.ProfilePoints[1].Value     = self.data["slope"] / 100 # catalogue value is in %
        myTestRun.Road.Definition.Links[0].SlopeProfile.ProfilePoints[1].Offset    = AP_ROAD_LENGTH
        
        myTestRun.Road.Definition.Links[0].CamberProfile.ProfilePoints[0].Value    = self.data["camber"]
        myTestRun.Road.Definition.Links[0].CamberProfile.ProfilePoints[1].Value    = self.data["camber"]
        
        # lane sections
        myTestRun.Road.Definition.Links[0].LaneSections[0].LanesLeft[0].WidthStart  = self.data["w_ul"]
        myTestRun.Road.Definition.Links[0].LaneSections[0].LanesLeft[0].WidthEnd    = self.data["w_ul"]
        myTestRun.Road.Definition.Links[0].LaneSections[0].LanesLeft[0].Type        = AP_LANE_DRIVING
        myTestRun.Road.Definition.Links[0].LaneSections[0].LanesRight[0].WidthStart = self.data["w_ul"]
        myTestRun.Road.Definition.Links[0].LaneSections[0].LanesRight[0].WidthEnd   = self.data["w_ul"]
        myTestRun.Road.Definition.Links[0].LaneSections[0].LanesRight[0].Type       = AP_LANE_DRIVING
        myTestRun.Road.Definition.Links[0].LaneSections[0].addLane("left", AP_LW_PEDESTRIAN, AP_LW_PEDESTRIAN, AP_LANE_PEDESTRIAN)
        myTestRun.Road.Definition.Links[0].LaneSections[0].addLane("right", AP_LW_PEDESTRIAN, AP_LW_PEDESTRIAN, AP_LANE_PEDESTRIAN)
        myTestRun.Road.Definition.Links[0].LaneSections[0].RoadMarking.Type         = AP_RM_CONTINUOUS
        
        # parking area          
        if self.data["Category"].find("Perpendicular") != -1 or self.data["Category"].find("Parallel") != -1 or self.data["Category"].find("Angled") != -1:
            pSizeX = self.latHPark
            pSizeY = self.latVPark
        else:
            print("Scenario catogory not handled - fillRoadData()")
            sys.exit(0)
            
        if self.data["Category"].find("Left") != -1:
            pSide = "left"
            pOppSide = "right"
            oppSideRMPos = AP_RM_POS_LEFT
        elif self.data["Category"].find("Right") != -1:
            pSide = "right"
            pOppSide = "left"
            oppSideRMPos = AP_RM_POS_RIGHT
        else:
            # print(("Can not determine parking side for category {}.".format(self.data["Category"]))) #TODO: Check if this message is required
            pSide = "right"
            pOppSide = "left"
            oppSideRMPos = AP_RM_POS_RIGHT
            
        # create parking slots
        parkStart = self.u_Rd - 2 * self.latHPark - self.xOffPStart_Ang
        for idx in range(1, 4):
            myTestRun.Road.Definition.Links[0].addLaneSection()
            myTestRun.Road.Definition.Links[0].LaneSections[idx].Start    = parkStart + (idx - 1) * pSizeX + AP_PARK_ADD_WIDTH[idx-1]
            myTestRun.Road.Definition.Links[0].LaneSections[idx].LanesLeft[0].WidthStart   = self.data["w_ul"]
            myTestRun.Road.Definition.Links[0].LaneSections[idx].LanesLeft[0].WidthEnd     = self.data["w_ul"]
            myTestRun.Road.Definition.Links[0].LaneSections[idx].LanesLeft[0].Type         = AP_LANE_DRIVING
            myTestRun.Road.Definition.Links[0].LaneSections[idx].LanesLeft[0].RoadMarking  = None
            myTestRun.Road.Definition.Links[0].LaneSections[idx].LanesRight[0].WidthStart  = self.data["w_ul"]
            myTestRun.Road.Definition.Links[0].LaneSections[idx].LanesRight[0].WidthEnd    = self.data["w_ul"]
            myTestRun.Road.Definition.Links[0].LaneSections[idx].LanesRight[0].Type        = AP_LANE_DRIVING
            myTestRun.Road.Definition.Links[0].LaneSections[idx].LanesRight[0].RoadMarking = None
            myTestRun.Road.Definition.Links[0].LaneSections[idx].addLane(pSide, pSizeY + AP_PARK_ADD_DEPTH, pSizeY + AP_PARK_ADD_DEPTH, AP_LANE_PARKING)
            myTestRun.Road.Definition.Links[0].LaneSections[idx].addLane(pOppSide, AP_LW_PEDESTRIAN, AP_LW_PEDESTRIAN, AP_LANE_PEDESTRIAN, True, Position = oppSideRMPos, RMType = AP_RM_CONTINUOUS)
            myTestRun.Road.Definition.Links[0].LaneSections[idx].RoadMarking.Type          = AP_RM_DOTTED
        
        # end of parking area
        myTestRun.Road.Definition.Links[0].addLaneSection()
        myTestRun.Road.Definition.Links[0].LaneSections[4].Start    = self.u_Rd + pSizeX + AP_PARK_ADD_WIDTH[3] + self.xOffPEnd_Ang
        myTestRun.Road.Definition.Links[0].LaneSections[4].LanesLeft[0].WidthStart  = self.data["w_ul"]
        myTestRun.Road.Definition.Links[0].LaneSections[4].LanesLeft[0].WidthEnd    = self.data["w_ul"]
        myTestRun.Road.Definition.Links[0].LaneSections[4].LanesLeft[0].Type        = AP_LANE_DRIVING
        myTestRun.Road.Definition.Links[0].LaneSections[4].LanesRight[0].WidthStart = self.data["w_ul"]
        myTestRun.Road.Definition.Links[0].LaneSections[4].LanesRight[0].WidthEnd   = self.data["w_ul"]
        myTestRun.Road.Definition.Links[0].LaneSections[4].LanesRight[0].Type       = AP_LANE_DRIVING
        myTestRun.Road.Definition.Links[0].LaneSections[4].addLane("left", AP_LW_PEDESTRIAN, AP_LW_PEDESTRIAN, AP_LANE_PEDESTRIAN)
        myTestRun.Road.Definition.Links[0].LaneSections[4].addLane("right", AP_LW_PEDESTRIAN, AP_LW_PEDESTRIAN, AP_LANE_PEDESTRIAN)
        myTestRun.Road.Definition.Links[0].LaneSections[4].RoadMarking.Type         = AP_RM_CONTINUOUS
        
        # bump
        if self.data["bump"] != "none":
            myTestRun.Road.Definition.Links[0].addBump()
            myTestRun.Road.Definition.Links[0].Bumps[0].Type = 'Beam'
            myTestRun.Road.Definition.Links[0].Bumps[0].StartOffset = self.u_Rd - self.latHPark / 2 - (self.bumpRampUp[self.data["bump"]] + self.bumpPlateau[self.data["bump"]] + self.bumpRampDown[self.data["bump"]]) / 2
            myTestRun.Road.Definition.Links[0].Bumps[0].Reference = 0 # 0 = Center, 2 = Lane 0 Left, -2 = Lane 0 Right
            myTestRun.Road.Definition.Links[0].Bumps[0].Height = self.bumpHeight[self.data["bump"]]
            myTestRun.Road.Definition.Links[0].Bumps[0].RampUp = self.bumpRampUp[self.data["bump"]]
            myTestRun.Road.Definition.Links[0].Bumps[0].Plateau = self.bumpPlateau[self.data["bump"]]
            myTestRun.Road.Definition.Links[0].Bumps[0].RampDown = self.bumpRampDown[self.data["bump"]]
            myTestRun.Road.Definition.Links[0].Bumps[0].Width = self.data["w_ul"] * 2
            if self.bumpHeight[self.data["bump"]] < 0.0:
                myTestRun.Road.Definition.Links[0].Bumps[0].Material = "Textures/Infrastructure/Concrete_1.jpg"

        
    def getEvaluationCriteria(self, myTestRun):
        # evalCriteria = "\n"
        evalCriteria = "\n\t\"\""
        evalCriteria += "\n\t" + "--- Evaluation Criteria ---"
        evalCriteria += "\n\t" + "n_strokes_max = " + str(round(self.data["n_strokes_max"], 2))
        evalCriteria += "\n\t" + "v_max = " + str(round(self.data["v_max"], 2)) + " m/s"
        evalCriteria += "\n\t" + "t_sim_max = " + str(round(self.data["t_sim_max"], 2)) + " s"
        if self.data["Scope Base"] == "yes":
            val = 1
        else:
            val = 0
        evalCriteria += "\n\t" + "Scope Base = " + str(val) + " (" + self.data["Scope Base"] + ")"
        if self.data["Non-/Use Case"] == "Use Case":
            val = 1
        else:
            val = 0
        evalCriteria += "\n\t" + "Non-/Use Case = " + str(val) + " (" + self.data["Non-/Use Case"] + ")"
        evalCriteria += "\n\t" + "Maneuver = " + self.data["Maneuver"]
        
        if self.data["Use Case ID"].find("AP_UC_") != -1:
            (xOTP, yOTP, psiOTP) = self.getOptimalTargetPose(myTestRun)
            evalCriteria += "\n\t--- Optimal Target Pose ---"
            evalCriteria += "\n\tOTP_x = " + str(xOTP) + " m"
            evalCriteria += "\n\tOTP_y = " + str(yOTP) + " m"
            evalCriteria += "\n\tOTP_psi = " + str(psiOTP) + " °"
        
        return evalCriteria

        
    def getOptimalTargetPose(self, myTestRun):
        (xOTP, yOTP, psiOTP) = (0, 0, 0)
        # Parallel Parking
        # AP_UC_001 - Parallel Parking Between Two Aligned Vehicles
        if self.data["Use Case ID"].find("AP_UC_001") != -1:
            psiOTP = self.getAnglePx("1")
            if psiOTP != 0: # assumption: psiOTP = 0°
                print("Test scenario {}: Scenario does not comply with the use case ID.".format(self.data["Situation ID"]))
                sys.exit(0)
            
            minXFront = min([v for k, v in self.corners_P1.items() if k.startswith('x')])
            maxXRear = max([v for k, v in self.corners_P2.items() if k.startswith('x')])
            distFrontRear = minXFront - maxXRear
            yOTP = self.CMSign * (min([abs(v) for k, v in self.corners_P1.items() if k.startswith('y')]) + self.data["w_Ego"] / 2)
            if distFrontRear >= (self.data["l_Ego"] + AP_G_PAR_SLOT_MIN_OFFSET_L_M) and distFrontRear <= (self.data["l_Ego"] + 2 * AP_G_DIST_CMF_SSIDE_HIGH_OBST_PAR_M): # AP_UC_001_02 Short Parking Slot
                xOTP = (distFrontRear - self.data["l_Ego"]) / 2 + self.data["o_Ego"] + maxXRear
            elif distFrontRear > (self.data["l_Ego"] + 2 * AP_G_DIST_CMF_SSIDE_HIGH_OBST_PAR_M) and distFrontRear <= (self.data["l_Ego"] + AP_G_PAR_SLOT_MAX_OFFSET_L_M): # AP_UC_001_01 Long Parking Slot
                xOTP = minXFront - AP_G_DIST_CMF_SSIDE_HIGH_OBST_PAR_M - self.data["l_Ego"] + self.data["o_Ego"]
                
        # AP_UC_002_01 - Parking behind one vehicle and next to Curbstone or Wall # TODO: complete this OTP
        elif self.data["Use Case ID"].find("AP_UC_002_01") != -1:
            minXP1 = min([v for k, v in self.corners_P1.items() if k.startswith('x')])
            minAbsYP1 = min([abs(v) for k, v in self.corners_P1.items() if k.startswith('y')]) # the farthest point on the road of the parked vehicle = road side edge
            xOTP = minXP1 - AP_G_DIST_CMF_FRONT_HIGH_OBST_M - self.data["l_Ego"] + self.data["o_Ego"]
            # AP_UC_003
            d1 = 0
            if self.limHeight[self.data["lim_P_End"]] <= self.limHeight["curb_tb"]: # less than body traversable
                d1 = AP_G_DIST_CMF_LSIDE_TRAV_PAR_M
            elif self.limHeight[self.data["lim_P_End"]] <= self.limHeight["curb_ntd"]: # less than non-traversable with openable doors
                d1 = AP_G_DIST_CMF_LSIDE_DOOR_OPEN_PAR_M
            else: # non-traversable or wall
                d1 = AP_G_DIST_CMF_LSIDE_HIGH_OBST_PAR_M
            # yOTP = self.CMSign * (self.data["w_ul"] + self.latVPark - d1 - self.data["w_Ego"] / 2)
            yOTP = self.v_P - self.CMSign * (d1 + self.data["w_Ego"] / 2)
            if abs(yOTP - self.CMSign * (self.data["w_Ego"] / 2 + AP_G_DIST_MIN_NO_DELIMITER_M)) < minAbsYP1: # keep a lateral distance of AP_G_DIST_MIN_NO_DELIMITER_M to the road side edge
                yOTP = self.CMSign * (minAbsYP1 + AP_G_DIST_MIN_NO_DELIMITER_M + self.data["w_Ego"] / 2)
            
        # AP_UC_002_02 - Parking behind one vehicle and small obstacle # TODO: complete this OTP
        elif self.data["Use Case ID"].find("AP_UC_002_02") != -1:
            # assume cylindrical obstacle
            if self.data["exists_P1"] == "true": # parking behind vehicle P1
                minXFront = min([v for k, v in self.corners_P1.items() if k.startswith('x')])
                maxXRear = self.data["stX_Obs1"] + self.data["r_Obs1"]
                psiOTP = self.getAnglePx("1")
                yRef = self.corners_P1["yRef"]
                x1_Rd = self.corners_P1["x1"]
                y1_Rd = self.corners_P1["y1"]
                x2_Rd = self.corners_P1["x2"]
                y2_Rd = self.corners_P1["y2"]
            elif self.data["exists_P2"] == "true": # parking in front of vehicle P2
                minXFront = self.data["stX_Obs1"] - self.data["r_Obs1"]
                maxXRear = max([v for k, v in self.corners_P2.items() if k.startswith('x')])
                psiOTP = self.getAnglePx("2")
                yRef = self.corners_P2["yRef"]
                x1_Rd = self.corners_P2["x1"]
                y1_Rd = self.corners_P2["y1"]
                x2_Rd = self.corners_P2["x2"]
                y2_Rd = self.corners_P2["y2"]
            else:
                print("Test scenario {}: Scenario does not comply with the use case ID.".format(self.data["Situation ID"]))
                sys.exit(0)
            distFrontRear = minXFront - maxXRear
            # if distFrontRear >= (self.data["l_Ego"] + AP_G_PAR_SLOT_MIN_OFFSET_L_M) and distFrontRear <= (self.data["l_Ego"] + AP_G_DIST_CMF_FRONT_HIGH_OBST_M + AP_G_DIST_CMF_REAR_HIGH_OBST_M): # 1504044 Short Parking Slot # TODO: get exact definition
                # xOTP = round((distFrontRear - self.data["l_Ego"]) / 2 + self.data["o_Ego"] + maxXRear, 3)
            # else: # # 1504042 Long Parking Slot # TODO: get exact definition
                # # assume that second side edge is behind P1 # TODO: clarify second side edge position
                # xOTP = round(minXFront - AP_G_DIST_CMF_SSIDE_HIGH_OBST_PAR_M - self.data["l_Ego"] + self.data["o_Ego"], 3)
            if distFrontRear >= (self.data["l_Ego"] + AP_G_PAR_SLOT_MIN_OFFSET_L_M) and distFrontRear <= (self.data["l_Ego"] + 2 * AP_G_DIST_CMF_SSIDE_HIGH_OBST_PAR_M): # AP_UC_001_02 Short Parking Slot
                xOTP = (distFrontRear - self.data["l_Ego"]) / 2 + self.data["o_Ego"] + maxXRear
            elif distFrontRear > (self.data["l_Ego"] + 2 * AP_G_DIST_CMF_SSIDE_HIGH_OBST_PAR_M) and distFrontRear <= (self.data["l_Ego"] + AP_G_PAR_SLOT_MAX_OFFSET_L_M): # AP_UC_001_01 Long Parking Slot
                xOTP = minXFront - AP_G_DIST_CMF_SSIDE_HIGH_OBST_PAR_M - self.data["l_Ego"] + self.data["o_Ego"]
                
            # Do not delete this - "current" implementation in code    
            # yOTPOff = math.sin(math.radians(psiOTP)) * self.data["o_Ego"]
            # yOTP = round(yRef + yOTPOff, 3)
            yOTPRd = (xOTP - x1_Rd) * (y2_Rd - y1_Rd) / (x2_Rd - x1_Rd) + y1_Rd
            yOTP = yOTPRd + self.CMSign * (abs(self.data["w_Ego"] / 2 * math.cos(math.radians(psiOTP))))
            
            
        # AP_UC_003 - Parallel parking between two vehicles next to curbstones or walls
        elif self.data["Use Case ID"].find("AP_UC_003") != -1:
            minXFront = min([v for k, v in self.corners_P1.items() if k.startswith('x')])
            maxXRear = max([v for k, v in self.corners_P2.items() if k.startswith('x')])
            distFrontRear = minXFront - maxXRear
            # psiOTP is 0 by default
            # Longitudinal position as in AP_UC_001
            if distFrontRear >= (self.data["l_Ego"] + AP_G_PAR_SLOT_MIN_OFFSET_L_M) and distFrontRear <= (self.data["l_Ego"] + 2 * AP_G_DIST_CMF_SSIDE_HIGH_OBST_PAR_M): # AP_UC_001_02 Short Parking Slot
                xOTP = (distFrontRear - self.data["l_Ego"]) / 2 + self.data["o_Ego"] + maxXRear
            elif distFrontRear > (self.data["l_Ego"] + 2 * AP_G_DIST_CMF_SSIDE_HIGH_OBST_PAR_M) and distFrontRear <= (self.data["l_Ego"] + AP_G_PAR_SLOT_MAX_OFFSET_L_M): # AP_UC_001_01 Long Parking Slot
                xOTP = minXFront - AP_G_DIST_CMF_SSIDE_HIGH_OBST_PAR_M - self.data["l_Ego"] + self.data["o_Ego"]
            d1 = 0
            if self.limHeight[self.data["lim_P_End"]] <= self.limHeight["curb_tb"]: # wheel traversable or body traversable curbside limiter
                d1 = AP_G_DIST_CMF_LSIDE_TRAV_PAR_M
            elif self.limHeight[self.data["lim_P_End"]] <= self.limHeight["curb_ntd"]: # door openable curbside limiter
                d1 = AP_G_DIST_CMF_LSIDE_DOOR_OPEN_M
            else: # high curbstone or wall
                d1 = AP_G_DIST_CMF_LSIDE_HIGH_OBST_M
            yOTP = self.v_P - self.CMSign * (d1 + self.data["w_Ego"] / 2)
            '''
            yOTP = round(self.CMSign * (min([abs(v) for k, v in self.corners_P1.items() if k.startswith('y')]) + self.data["w_Ego"] / 2), 3)
            if self.limHeight[self.data["lim_P_End"]] <= self.limHeight["curb_ntd"]:
                yOTP = round(self.v_P - self.CMSign * (AP_G_DIST_CMF_LSIDE_DOOR_OPEN_PAR_M + self.data["w_Ego"] / 2), 3)
            else: 
                yOTP = round(self.CMSign * (self.data["w_ul"] + self.latVPark - AP_G_DIST_CMF_LSIDE_HIGH_OBST_PAR_M - self.data["w_Ego"] / 2), 3)
            if abs(yOTP - self.CMSign * (self.data["w_Ego"] / 2  + AP_G_DIST_MIN_NO_DELIMITER_M)) < abs(self.roadSideEdge):
                yOTP = self.roadSideEdge + self.CMSign * (AP_G_DIST_MIN_NO_DELIMITER_M + self.data["w_Ego"] / 2)
            '''
                
        # AP_UC_007 - Parallel Parking based on Parking Slot Markings no vehicles
        elif self.data["Use Case ID"].find("AP_UC_007") != -1:
            # psiOTP is 0 by default
            xOTP = self.u_P - self.data["l_Ego"] / 2 + self.data["o_Ego"] # TODO: check if a maximum distance to parking limiter on second side must be imposed
            # AP_UC_007_01 - Parking based on Slot Markings only
            if self.data["lim_P_End"] == "none" and self.data["lim_Rd_P"] == "line":
                dR = AP_G_DIST_CMF_LANE_M
                yOTP = self.v_Rd + self.CMSign * (dR + self.data["w_Ego"] / 2)
            else:
                if self.data["lim_P_End"] == "line" and self.data["lim_Rd_P"] == "none":
                    dC = AP_G_DIST_CMF_PARKSLOT_MARKER_M
                else: # markings on road side and curbside edges, or curbstone on curbside edge
                    if self.data["lim_P_End"] == "line":
                        dMin = 0 # TODO: check if this value may be negative
                        dMax = AP_G_DIST_CMF_PARKSLOT_MARKER_M
                    # AP_UC_007_02 - Parking Slot with Slot Markings and Curbs or Wall
                    elif self.data["lim_P_End"] == "curb_t" or self.data["lim_P_End"] == "curb_tb":
                        dMin = AP_G_DIST_MIN_LSIDE_TRAV_PAR_M
                        dMax = AP_G_DIST_CMF_LSIDE_TRAV_PAR_M
                    elif self.data["lim_P_End"] == "curb_ntd":
                        dMin = AP_G_DIST_MIN_LSIDE_DOOR_OPEN_PAR_M
                        dMax = AP_G_DIST_CMF_LSIDE_DOOR_OPEN_PAR_M
                    else: # high curbstone or wall
                        dMin = AP_G_DIST_MIN_LSIDE_HIGH_OBST_PAR_M
                        dMax = AP_G_DIST_CMF_LSIDE_HIGH_OBST_PAR_M
                        
                    dW = self.latVPark - self.data["w_Ego"]
                    cC = dMax
                    cR = AP_G_DIST_CMF_LANE_M
                    dC = dW * (cC / (cC + cR))
                    if dC > dMax:
                        dC = dMax
                    if dC < dMin:
                        dC = dMin
                yOTP = self.v_P - self.CMSign * (dC + self.data["w_Ego"] / 2)
                
        # AP_UC_008 - Parallel Parking based on Parking Slot Markings with vehicles
        # OTP for this UC is set based on discussion with Tobias; requirements are outdated
        elif self.data["Use Case ID"].find("AP_UC_008_01") != -1: # adjecent vehicle do not overlap the parking slot
            # psiOTP is 0 by default
            xOTP = self.u_P - self.data["l_Ego"] / 2 + self.data["o_Ego"] # TODO: check if a maximum distance to parking limiter on second side must be imposed
            
            minYP1 = min([abs(v) for k, v in self.corners_P1.items() if k.startswith('y')])
            minYP2 = min([abs(v) for k, v in self.corners_P2.items() if k.startswith('y')])
            eR = self.CMSign * (minYP1 + minYP2) / 2
            yOTP = eR + self.CMSign * (self.data["w_Ego"] / 2)
            curbRef = 0
            if self.data["lim_P_End"] != "line" and self.limHeight[self.data["lim_P_End"]] >= self.limHeight["curb_t"]:
                curbRef = self.v_P
                    
            # convention: description contains "double curbside limiter" when line marking is doubled by a curb / wall on curbside edge
            # convention: if so, obstacle 1 is used to describe the curb / wall
            # convention: this obstacle is rectangular, parallel to road axis and uses (u_P, v_P) as reference (area_Obs1 = parking)
            elif self.data["Description"].find("double curbside limiter") != -1:
                curbRef = self.v_P - self.CMSign * (self.data["v_Obs1"] + self.data["w_Obs1"])
                
            # the vehicle must keep a distance of at least AP_G_DIST_MIN_LSIDE_HIGH_OBST_PAR_M to high obstacle on curb side edge
            # high obstacle parameter is the only released value (other heights are not handled)
            if curbRef != 0:
                if abs(yOTP + self.CMSign * (self.data["w_Ego"] / 2)) + AP_G_DIST_MIN_LSIDE_HIGH_OBST_PAR_M > abs(curbRef):
                    yOTP = curbRef - self.CMSign * (self.data["w_Ego"] / 2 + AP_G_DIST_MIN_LSIDE_HIGH_OBST_PAR_M)
            
            # the vehicle must keep a distance of at least AP_G_DIST_MIN_PARKSLOT_MARKER_M to marking on curb side edge
            if self.data["lim_P_End"] == "line":
                if abs(yOTP + self.CMSign * (self.data["w_Ego"] / 2)) + AP_G_DIST_MIN_PARKSLOT_MARKER_M > abs(self.v_P):
                    yOTP = self.v_P - self.CMSign * (self.data["w_Ego"] / 2 + AP_G_DIST_MIN_PARKSLOT_MARKER_M)
                    
            # TODO: implement the checks for maximum distance to curb side limiters when requirements are clarified
                
        # Perpendicular Parking
        # AP_UC_018 - Perpendicular Parking Between Two Vehicles
        elif self.data["Use Case ID"].find("AP_UC_018") != -1 or self.data["Use Case ID"].find("AP_UC_019") != -1:
            minXP1 = min([v for k, v in self.corners_P1.items() if k.startswith('x')])
            maxXP2 = max([v for k, v in self.corners_P2.items() if k.startswith('x')])
            minYP1 = min([abs(v) for k, v in self.corners_P1.items() if k.startswith('y')])
            xOTP = (minXP1 + maxXP2) / 2
            # AP_UC_018: Backward Perpendicular Parking
            if self.data["Use Case ID"].find("AP_UC_018") != -1:
                yOTP = self.CMSign * (minYP1 + self.data["l_Ego"] - self.data["o_Ego"])
                psiOTP = (-1) * self.CMSign * 90
            # AP_UC_019: Forward Perpendicular Parking
            elif self.data["Use Case ID"].find("AP_UC_019") != -1:
                yOTP = self.CMSign * (minYP1 + self.data["o_Ego"])
                psiOTP = self.CMSign * 90
                
        # AP_UC_021 - Perpendicular Parking With a Curbstone or a Wall at the Curb Side Edge
        elif self.data["Use Case ID"].find("AP_UC_021") != -1:
            '''
            minXP1 = min([v for k, v in self.corners_P1.items() if k.startswith('x')])
            maxXP2 = max([v for k, v in self.corners_P2.items() if k.startswith('x')])
            xOTP = (minXP1 + maxXP2) / 2
            '''
            psiP1 = self.getAnglePx("1")
            psiP2 = self.getAnglePx("2")
            # get the general direction of the parked vehicles, without the orientation (backwards / forwards) - range[-45..135]
            if psiP1 > 135 or psiP1 < -45:
                psiP1 = psiP1 - np.sign(psiP1) * 180
            if psiP2 > 135 or psiP2 < -45:
                psiP2 = psiP2 - np.sign(psiP2) * 180
            avgPsi = (psiP1 + psiP2) / 2
            
            if self.data["Category"].find("Right") != -1: # right
                # Backward Perpendicular Parking
                if self.data["Maneuver"].find('backward') != - 1:
                    psiOTP = avgPsi
                else:
                    psiOTP = avgPsi + 180
            else: # left
                # Backward Perpendicular Parking
                if self.data["Maneuver"].find('backward') != - 1:
                    psiOTP = avgPsi - 180
                else:
                    psiOTP = avgPsi
                    
            minYP1 = min([abs(v) for k, v in self.corners_P1.items() if k.startswith('y')])
            minYP2 = min([abs(v) for k, v in self.corners_P2.items() if k.startswith('y')])
            eR = self.CMSign * (minYP1 + minYP2) / 2
            
            midRoadX = (self.corners_P2["x2"] + self.corners_P1["x1"]) / 2
            midRoadY = (self.corners_P2["y2"] + self.corners_P1["y1"]) / 2
            midCurbX = (self.corners_P2["x3"] + self.corners_P1["x4"]) / 2
            midCurbY = (self.corners_P2["y3"] + self.corners_P1["y4"]) / 2
                
            # AP_UC_021_02 - with Overhang
            # backwards
            if self.data["Maneuver"].find('backward') != - 1:
                egoYOff = (self.data["l_Ego"] - self.data["o_Ego"]) * abs(math.sin(math.radians(psiOTP)))
            # forwards
            else:
                egoYOff = self.data["o_Ego"] * abs(math.sin(math.radians(psiOTP)))
            roadYOff = self.data["w_Ego"] / 2 * abs(math.cos(math.radians(psiOTP)))
            yOTP = eR + self.CMSign * (egoYOff + roadYOff)
                
            # wheel closest point to curbstone
            whlCenterYOff = self.data["w_Ego"] / 2 * abs(math.cos(math.radians(psiOTP)))
            whlTyreYOff = AP_VW_PASSAT_TYRE_RAD * abs(math.sin(math.radians(psiOTP)))
            whlClosestY = yOTP + self.CMSign * (whlCenterYOff + whlTyreYOff)
            # the vehicle must keep a distance of at least AP_G_MIN_SAF_DIST_TRAV_OBST_M from the wheel to the curbstone
            # this should never happen because of AP_G_PERP_SLOT_MIN_OFFSET_L_M
            if abs(whlClosestY) + AP_G_MIN_SAF_DIST_TRAV_OBST_M > abs(self.v_P):
                yOTP -= self.CMSign * (abs(whlClosestY) + AP_G_MIN_SAF_DIST_TRAV_OBST_M - abs(self.v_P)) # move the vehicle closer to road
                
            xOTP = self.get_interp2d_x(midRoadX, midRoadY, midCurbX, midCurbY, yOTP)
            
        # AP_UC_024 - Perpendicular Parking based on Parking Slot Markings
        elif self.data["Use Case ID"].find("AP_UC_024") != -1:
            # use cases where the line markers are not orthogonal will be manually generated
            # backwards
            if self.data["Maneuver"].find('backward') != - 1:
                psiOTP = (-1) * self.CMSign * 90
                egoYOff = self.data["o_Ego"]
            # forwards
            else:
                psiOTP = self.CMSign * 90
                egoYOff = self.data["l_Ego"] - self.data["o_Ego"]
                
            yOTP = self.v_P - self.CMSign * (egoYOff + AP_G_DIST_CMF_PARKSLOT_MARKER_M)
            # distance to the road side edge shall not be higher than AP_G_PERP_SLOT_MAX_OFFSET_L_M / 2
            if abs(yOTP - self.CMSign * (self.data["l_Ego"] - egoYOff)) > abs(self.v_Rd) + AP_G_PERP_SLOT_MAX_OFFSET_L_M / 2:
                yOTP = self.v_Rd + self.CMSign * (self.data["l_Ego"] - egoYOff + AP_G_PERP_SLOT_MAX_OFFSET_L_M / 2)
            # distance to the curb side edge shall not be smaller than AP_G_DIST_MIN_PARKSLOT_MARKER_M
            if abs(yOTP + self.CMSign * (egoYOff + AP_G_DIST_MIN_PARKSLOT_MARKER_M)) > abs(self.v_P):
                yOTP = self.v_P - self.CMSign * (egoYOff + AP_G_DIST_MIN_PARKSLOT_MARKER_M)
            
            # AP_UC_024_01 - Perpendicular Parking in Slots Enclosed by Parallel Parking Slot Markings Only
            xOTP = self.u_Rd - self.latHPark / 2
            
        # AP_UC_027 - Perpendicular Parking based on Parking Slot Markings
        elif self.data["Use Case ID"].find("AP_UC_027") != -1:
            # use cases where the line markers are not orthogonal will be manually generated
            # backwards
            if self.data["Maneuver"].find('backward') != - 1:
                psiOTP = (-1) * self.CMSign * 90
                egoYOff = self.data["o_Ego"]
            # forwards
            else:
                psiOTP = self.CMSign * 90
                egoYOff = self.data["l_Ego"] - self.data["o_Ego"]
                
            yOTP = self.v_P - self.CMSign * (egoYOff + AP_G_DIST_CMF_PARKSLOT_MARKER_M)
            # distance to the road side edge shall not be higher than AP_G_PERP_SLOT_MAX_OFFSET_L_M / 2
            if abs(yOTP - self.CMSign * (self.data["l_Ego"] - egoYOff)) > abs(self.v_Rd) + AP_G_PERP_SLOT_MAX_OFFSET_L_M / 2:
                yOTP = self.v_Rd + self.CMSign * (self.data["l_Ego"] - egoYOff + AP_G_PERP_SLOT_MAX_OFFSET_L_M / 2)
            # distance to the curb side edge shall not be smaller than AP_G_DIST_MIN_PARKSLOT_MARKER_M
            if abs(yOTP + self.CMSign * (egoYOff + AP_G_DIST_MIN_PARKSLOT_MARKER_M)) > abs(self.v_P):
                yOTP = self.v_P - self.CMSign * (egoYOff + AP_G_DIST_MIN_PARKSLOT_MARKER_M)
            
            # AP_UC_027_01 - Perpendicular Parking With Parking Slot Markings and Vehicles within Lines
            xOTP = self.u_Rd - self.latHPark / 2
            minXP1 = min([v for k, v in self.corners_P1.items() if k.startswith('x')])
            maxXP2 = max([v for k, v in self.corners_P2.items() if k.startswith('x')])
            
            # if distance between parked vehicles does not allow for door opening on both sides (case 3)
            if minXP1 - maxXP2 < self.data["w_Ego"] + 2 * AP_G_DIST_LSIDE_MIN_DOOR_OPEN_M:
                xOTP = (minXP1 - maxXP2) / 2 + maxXP2
            else:
                # if distance between ego vehicle and P1 is smaller than AP_G_DIST_LSIDE_MIN_DOOR_OPEN_M 
                if xOTP + self.data["w_Ego"] / 2 + AP_G_DIST_LSIDE_MIN_DOOR_OPEN_M > minXP1:
                    xOTP = minXP1 - AP_G_DIST_LSIDE_MIN_DOOR_OPEN_M - self.data["w_Ego"] / 2
                # if distance between ego vehicle and P2 is smaller than AP_G_DIST_LSIDE_MIN_DOOR_OPEN_M
                if xOTP - self.data["w_Ego"] / 2 - AP_G_DIST_LSIDE_MIN_DOOR_OPEN_M < maxXP2:
                    xOTP = maxXP2 + AP_G_DIST_LSIDE_MIN_DOOR_OPEN_M + self.data["w_Ego"] / 2
            
            # if distance between ego vehicle and second side edge marker is smaller than AP_G_DIST_MIN_PARKSLOT_MARKER_M
            if xOTP + self.data["w_Ego"] / 2 + AP_G_DIST_MIN_PARKSLOT_MARKER_M > self.u_Rd:
                xOTP = self.u_Rd - AP_G_DIST_MIN_PARKSLOT_MARKER_M - self.data["w_Ego"] / 2
            # if distance between ego vehicle and first side edge marker is smaller than AP_G_DIST_MIN_PARKSLOT_MARKER_M
            if xOTP - self.data["w_Ego"] / 2 - AP_G_DIST_MIN_PARKSLOT_MARKER_M < self.u_Rd - self.latHPark:
                xOTP = self.u_Rd - self.latHPark + AP_G_DIST_MIN_PARKSLOT_MARKER_M + self.data["w_Ego"] / 2
                
            # if distance between ego vehicle and P1 is smaller than AP_G_DIST_MIN_LSIDE_HIGH_OBST_PERP_M
            if xOTP + self.data["w_Ego"] / 2 + AP_G_DIST_MIN_LSIDE_HIGH_OBST_PERP_M > minXP1:
                xOTP = minXP1 - AP_G_DIST_MIN_LSIDE_HIGH_OBST_PERP_M - self.data["w_Ego"] / 2
            # if distance between ego vehicle and P2 is smaller than AP_G_DIST_MIN_LSIDE_HIGH_OBST_PERP_M
            if xOTP - self.data["w_Ego"] / 2 - AP_G_DIST_MIN_LSIDE_HIGH_OBST_PERP_M < maxXP2:
                xOTP = maxXP2 + AP_G_DIST_MIN_LSIDE_HIGH_OBST_PERP_M + self.data["w_Ego"] / 2
            
        # AP_UC_037 - Perpendicular Parking Based on Wheel Stoppers
        elif self.data["Use Case ID"].find("AP_UC_037") != -1:
            # backwards
            if self.data["Maneuver"].find('backward') != - 1:
                psiOTP = (-1) * self.CMSign * 90
                egoYOff = self.data["o_Ego"]
                wsWhlOff = 0
            # forwards
            else:
                psiOTP = self.CMSign * 90
                egoYOff = self.data["l_Ego"] - self.data["o_Ego"]
                wsWhlOff = self.data["wb_Ego"]
                
            yOTP = self.v_P - self.CMSign * (egoYOff + AP_G_DIST_CMF_PARKSLOT_MARKER_M)
            # distance to the road side edge shall not be higher than AP_G_PERP_SLOT_MAX_OFFSET_L_M / 2
            if abs(yOTP - self.CMSign * (self.data["l_Ego"] - egoYOff)) > abs(self.v_Rd) + AP_G_PERP_SLOT_MAX_OFFSET_L_M / 2:
                yOTP = self.v_Rd + self.CMSign * (self.data["l_Ego"] - egoYOff + AP_G_PERP_SLOT_MAX_OFFSET_L_M / 2)
            # ego wheels shall not go beyond the wheelstopper
            if abs(yOTP + self.CMSign * (wsWhlOff + AP_VW_PASSAT_TYRE_RAD)) > abs(self.v_P - self.CMSign * (self.data["d_ws"] + self.wheelstopperWidth[AP_WHS_DEFAULT_TYPE] / 2)):
                yOTP = self.v_P - self.CMSign * (self.data["d_ws"] + self.wheelstopperWidth[AP_WHS_DEFAULT_TYPE] / 2 + wsWhlOff + AP_VW_PASSAT_TYRE_RAD)
            # ego wheels shall not go farther than AP_G_MAX_DIST_WHEEL_STOPPER_M from the wheelstopper
            if abs(yOTP + self.CMSign * (wsWhlOff + AP_VW_PASSAT_TYRE_RAD)) < abs(self.v_P - self.CMSign * (self.data["d_ws"] + self.wheelstopperWidth[AP_WHS_DEFAULT_TYPE] / 2 + AP_G_MAX_DIST_WHEEL_STOPPER_M)):
                yOTP = self.v_P - self.CMSign * (self.data["d_ws"] + self.wheelstopperWidth[AP_WHS_DEFAULT_TYPE] / 2 + AP_G_MAX_DIST_WHEEL_STOPPER_M + wsWhlOff + AP_VW_PASSAT_TYRE_RAD)
            
            # AP_UC_024_01 - Perpendicular Parking in Slots Enclosed by Parallel Parking Slot Markings Only
            xOTP = self.u_Rd - self.latHPark / 2
            
        # AP_UC_060 - Angular Parking Between Two Misaligned Vehicles
        elif self.data["Use Case ID"].find("AP_UC_060") != -1:
            psiP1 = self.getAnglePx("1")
            psiP2 = self.getAnglePx("2")
            # get the general direction of the parked vehicles, without the orientation (backwards / forwards)
            # TODO: check if this also works with reversed fishbone parking
            # if psiP1 >= 90:
                # psiP1 -= 180
            # if psiP2 >= 90:
                # psiP2 -= 180
            # check if parked vehicles angles are in valid range - ignore those outside of valid range
            # assumption is that at least one parked vehicle is in range
            # if psiP1 < math.degrees(math.pi / 2 - AP_G_ANG_MAX_ORI_ANGLE_RAD) or psiP1 > math.degrees(math.pi / 2 - AP_G_PERP_MAX_ORI_ANGLE_RAD):
                # psiOTP = psiP2
            # elif psiP2 < math.degrees(math.pi / 2 - AP_G_ANG_MAX_ORI_ANGLE_RAD) or psiP2 > math.degrees(math.pi / 2 - AP_G_PERP_MAX_ORI_ANGLE_RAD):
                # psiOTP = psiP1
            # else:
                # psiOTP = (psiP1 + psiP2) / 2
            # if (psiP1 < math.degrees(math.pi / 2 - AP_G_ANG_MAX_ORI_ANGLE_RAD) and psiP1 < math.degrees(-math.pi / 2 - AP_G_ANG_MAX_ORI_ANGLE_RAD)) or # almost parallel
               # (psiP1 > math.degrees(math.pi / 2 - AP_G_PERP_MAX_ORI_ANGLE_RAD) and  psiP1 < math.degrees(math.pi / 2 + AP_G_PERP_MAX_ORI_ANGLE_RAD)) or # almost perpendicular
               # psiP1 < math.degrees(-math.pi / 2 + AP_G_ANG_MAX_ORI_ANGLE_RAD):
            tempPsiP1 = abs(psiP1)
            tempPsiP2 = abs(psiP2)
            if tempPsiP1 > 180:
                tempPsiP1 -= 180
            if tempPsiP2 > 180:
                tempPsiP2 -= 180
            # check if parked vehicles angles are in valid range - ignore those outside of valid range
            # assumption is that at least one parked vehicle is in range
            if tempPsiP1 < math.degrees(math.pi / 2 - AP_G_ANG_MAX_ORI_ANGLE_RAD) or tempPsiP1 > math.degrees(math.pi / 2 + AP_G_ANG_MAX_ORI_ANGLE_RAD) or (tempPsiP1 > math.degrees(math.pi / 2 - AP_G_PERP_MAX_ORI_ANGLE_RAD) and tempPsiP1 < math.degrees(math.pi / 2 + AP_G_PERP_MAX_ORI_ANGLE_RAD)):
                psiOTP = psiP2
            elif tempPsiP2 < math.degrees(math.pi / 2 - AP_G_ANG_MAX_ORI_ANGLE_RAD) or tempPsiP2 > math.degrees(math.pi / 2 + AP_G_ANG_MAX_ORI_ANGLE_RAD) or (tempPsiP2 > math.degrees(math.pi / 2 - AP_G_PERP_MAX_ORI_ANGLE_RAD) and tempPsiP2 < math.degrees(math.pi / 2 + AP_G_PERP_MAX_ORI_ANGLE_RAD)):
                psiOTP = psiP1
            else:
                psiOTP = (psiP1 + psiP2) / 2              
            
            minYP1 = min([abs(v) for k, v in self.corners_P1.items() if k.startswith('y')])
            minYP2 = min([abs(v) for k, v in self.corners_P2.items() if k.startswith('y')])
            eR = self.CMSign * (minYP1 + minYP2) / 2
            
            midRoadX = (self.corners_P2["x2"] + self.corners_P1["x1"]) / 2
            midRoadY = (self.corners_P2["y2"] + self.corners_P1["y1"]) / 2
            midCurbX = (self.corners_P2["x3"] + self.corners_P1["x4"]) / 2
            midCurbY = (self.corners_P2["y3"] + self.corners_P1["y4"]) / 2
                
            # backwards
            if self.data["Maneuver"].find('backward') != - 1:
                egoYOff = (self.data["l_Ego"] - self.data["o_Ego"]) * abs(math.sin(math.radians(psiOTP)))
            # forwards
            else:
                egoYOff = self.data["o_Ego"] * abs(math.sin(math.radians(psiOTP)))
            roadYOff = self.data["w_Ego"] / 2 * abs(math.cos(math.radians(psiOTP)))
            yOTP = eR + self.CMSign * (egoYOff + roadYOff)
            
            xOTP = self.get_interp2d_x(midRoadX, midRoadY, midCurbX, midCurbY, yOTP)
            
            
        
        xOTP = round(xOTP, 3)
        yOTP = round(yOTP, 3)
        psiOTP = round(psiOTP, 3)
        return (xOTP, yOTP, psiOTP)
    
    
#********************************************************************
# Auxiliary functions
#********************************************************************

        
    def is_number_tryexcept(self, string):
        try:
            float(string)
            return True
        except ValueError:
            return False
            
    def get_interp2d_x(self, x1, y1, x2, y2, y):
        x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
        return x
        
    def get_point_side(self, x, y, x1, y1, x2, y2):
        # -1 - point on left side of the line
        # 0 - point on line
        # 1 - point on right side of the line
        position = np.sign((x2 - x1) * (y - y1) - (y2 - y1) * (x - x1)) * (-1) * self.CMSign # TODO: check logic behind (-1) * self.CMSign (required for left side); check validity for alpha > 90°
        return position
        