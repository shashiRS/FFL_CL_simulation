#====================================================================
# System Imports
#====================================================================
import sys, os
import numpy as np
import operator
# import cm_eval_erg_parser
import math
import timeit
from collections import OrderedDict
try:
    import cm_eval_erg_parser
except:
    from pyBase.pycmEval import cm_eval_erg_parser
#====================================================================
# System Imports
#====================================================================
INIT_DISTANCE = 999.9

"""
TODO add if needed - code taken from QVA eval

def qva_point_to_line_orientation( a1x, a1y, a2x, a2y,  b1x, b1y, eps ):

    POINT_ON_LINE = 0;
    POINT_RIGHT   = 1;
    POINT_LEFT    = -1;

    v1x = a2x-a1x; #A1A2
    v1y = a2y-a1y;
    v2x = b1x-a1x; #A1B1
    v2y = b1y-a1y;

    det = v2x*v1y - v2y*v1x;

    if( abs(det) <= eps):
        return POINT_ON_LINE; #on the line


    if(det > eps):
        return POINT_RIGHT; #right side
    else:
        return POINT_LEFT; #left side


def qva_line_intersections( a1x, a1y, a2x, a2y,  b1x, b1y, b2x, b2y, eps ):

    is_valid = 0; is_x=0; is_y=0;

    x1 = a1x; y1 = a1y; x2 = a2x; y2 = a2y;
    x3 = b1x; y3 = b1y; x4 = b2x; y4 = b2y;

    p1 = qva_point_to_line_orientation(a1x, a1y, a2x, a2y, b1x, b1y, eps);
    p2 = qva_point_to_line_orientation(a1x, a1y, a2x, a2y, b2x, b2y, eps);

    if ( not (p1 != 0 and p1 != p2) ):
        return [is_valid, is_x, is_y]

    p1 = qva_point_to_line_orientation(b1x, b1y, b2x, b2y, a1x, a1y, eps);
    p2 = qva_point_to_line_orientation(b1x, b1y, b2x, b2y, a2x, a2y, eps);

    if ( not (p1 != 0 and p1 != p2)):
        return [is_valid, is_x, is_y]

    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    ltf = x1 * y2 - y1 * x2; rtf = x3 * y4 - y3 * x4;

    if ( abs(denom) < eps ):
        return [is_valid, is_x, is_y]


    is_valid  = 1;
    is_x = ((ltf * (x3 - x4) - (x1 - x2) * rtf) / denom);
    is_y = ((ltf * (y3 - y4) - (y1 - y2) * rtf) / denom);

    return [is_valid, is_x, is_y]

def qva_trj_intersections_py( xdata1, ydata1, xdata2, ydata2 ):

    eps = 0.00001;

    for c_idx1 in range(len(xdata1)-1):
        for c_idx2 in range(len(xdata2)-1):
            a1x = xdata1[c_idx1];
            a1y = ydata1[c_idx1];
            a2x = xdata1[c_idx1+1];
            a2y = ydata1[c_idx1+1];
            b1x = xdata2[c_idx2];
            b1y = ydata2[c_idx2];
            b2x = xdata2[c_idx2+1];
            b2y = ydata2[c_idx2+1];

            ret = qva_line_intersections( a1x, a1y, a2x, a2y,  b1x, b1y, b2x, b2y, eps );
            is_valid = ret[0]
            is_x     = ret[1]
            is_y     = ret[2]

            if is_valid == 1:
               return [is_valid, is_x, is_y, c_idx1, c_idx2 ]

    return [0, is_x, is_y, c_idx1, c_idx2 ]
"""

def approx_eq(a, b):
    return abs(a - b) <= 0.001

class cErgCalculator(object):
    def __init__(self, currErgFilePath, signal_mapping, cacheWholeFile = True):
        try:
            self.currErgFile = cm_eval_erg_parser.cErgFile(currErgFilePath, cacheWholeFile)
        except FileNotFoundError:
            print(f'{currErgFilePath} not found.')
        except:
            print(f'Exception occured while parsing {currErgFilePath}.')
        else:
            try:
                self.TimeVec                = np.array(self.currErgFile.readAllSignalValues("Time"))
                self.currHypCrossingProb    = np.array(self.currErgFile.readAllSignalValues("QVA.HypCrossing.Prob"))
                self.currHypTTC = np.array(self.currErgFile.readAllSignalValues(signal_mapping["acdc_hyp_ttc"]))
                self.prebrake = np.array(self.currErgFile.readAllSignalValues(signal_mapping["prebrake_signame"]))
                self.TTC = np.array(self.currErgFile.readAllSignalValues(signal_mapping["acdc_hyp_ttc"]))
                self.prefill  = np.array(self.currErgFile.readAllSignalValues(signal_mapping["prefill_signame"]))
                self.fcw = np.array(self.currErgFile.readAllSignalValues(signal_mapping["FCW_signame"]))
                self.ego_velocity = (np.array(self.currErgFile.readAllSignalValues(signal_mapping["ego_speed"])))*3.6
                self.collision_sensor = np.array(self.currErgFile.readAllSignalValues(signal_mapping["collision_detection"]))
                self.car_distance = np.array(self.currErgFile.readAllSignalValues(signal_mapping["car_dist"]))
                self.car_distance_AD = np.array(self.currErgFile.readAllSignalValues(signal_mapping["car_dist"]))
                self.ego_acceleration = np.array(self.currErgFile.readAllSignalValues(signal_mapping["ego_acceleration"]))
                self.traffic_long_velocity = (np.array(self.currErgFile.readAllSignalValues(signal_mapping["target_speed"])))*3.6
                self.loctr_accel_request = np.array(self.currErgFile.readAllSignalValues(signal_mapping["loctr_accel_request"]))
                self.EPF_Left_fLatDistObstacleToBorder = np.array(self.currErgFile.readAllSignalValues(signal_mapping["EPF_Left_fLatDistObstacleToBorder"]))
                self.EPF_Left_fLatDistEgoToBorder = np.array(self.currErgFile.readAllSignalValues(signal_mapping["EPF_Left_fLatDistEgoToBorder"]))
                self.EPF_Right_fLatDistObstacleToBorder = np.array(self.currErgFile.readAllSignalValues(signal_mapping["EPF_Right_fLatDistObstacleToBorder"]))
                self.EPF_Right_fLatDistEgoToBorder = np.array(self.currErgFile.readAllSignalValues(signal_mapping["EPF_Right_fLatDistEgoToBorder"]))
                self.DynamicPrewarn = np.array(self.currErgFile.readAllSignalValues(signal_mapping["DynamicPrewarn"]))
                self.rt_range_ttc = np.array(self.currErgFile.readAllSignalValues(signal_mapping["rt_range_ttc"]))
                self.car_Length = np.array(self.currErgFile.readAllSignalValues(signal_mapping["car_length"]))

                # Ground Truth Signals
                self.fDistX = np.array(self.currErgFile.readAllSignalValues(signal_mapping["gt_long"]))
                self.fDistY = np.array(self.currErgFile.readAllSignalValues(signal_mapping["gt_lat"]))

                # CEM Signals
                self.CEM_fDistX = np.array(self.currErgFile.readAllSignalValues(signal_mapping["CEM_fDistX"]))
                self.CEM_fDistY = np.array(self.currErgFile.readAllSignalValues(signal_mapping["CEM_fDistY"]))
                self.CEM_fVabsX = np.array(self.currErgFile.readAllSignalValues(signal_mapping["CEM_fVabsX"]))
                self.CEM_fVabsY = np.array(self.currErgFile.readAllSignalValues(signal_mapping["CEM_fVabsY"]))
                self.CEM_eClassification = np.array(self.currErgFile.readAllSignalValues(signal_mapping["CEM_eClassification"]))

                try:
                # Fetch CEM Signals
                    self.CEM_Coordinates_0_fPosX = np.array(self.currErgFile.readAllSignalValues(signal_mapping["aShapePointCoordifPosX_0"]))
                    self.CEM_Coordinates_0_fPosY = np.array(self.currErgFile.readAllSignalValues(signal_mapping["aShapePointCoordifPosY_0"]))
                    self.CEM_Coordinates_1_fPosX = np.array(self.currErgFile.readAllSignalValues(signal_mapping["aShapePointCoordifPosX_1"]))
                    self.CEM_Coordinates_1_fPosY = np.array(self.currErgFile.readAllSignalValues(signal_mapping["aShapePointCoordifPosY_1"]))
                    self.CEM_Coordinates_2_fPosX = np.array(self.currErgFile.readAllSignalValues(signal_mapping["aShapePointCoordifPosX_2"]))
                    self.CEM_Coordinates_2_fPosY = np.array(self.currErgFile.readAllSignalValues(signal_mapping["aShapePointCoordifPosY_2"]))
                    self.CEM_Coordinates_3_fPosX = np.array(self.currErgFile.readAllSignalValues(signal_mapping["aShapePointCoordifPosX_3"]))
                    self.CEM_Coordinates_3_fPosY = np.array(self.currErgFile.readAllSignalValues(signal_mapping["aShapePointCoordifPosY_3"]))
                except:
                    pass
                self.CEM_prob_existence = np.array(self.currErgFile.readAllSignalValues(signal_mapping["CEM_prob_existence"]))
                self.CEM_fAabsX = np.array(self.currErgFile.readAllSignalValues(signal_mapping["CEM_fAabsX"]))
                self.CEM_fAabsY = np.array(self.currErgFile.readAllSignalValues(signal_mapping["CEM_fAabsY"]))
                self.CEM_fYaw = np.array(self.currErgFile.readAllSignalValues(signal_mapping["CEM_fYaw"]))

                # Fetch BirdEye Signal from ERG
                try:
                    self.bird_eye_signal = np.array(self.currErgFile.readAllSignalValues("BirdEyeImage"))
                except:
                    pass

                #Yaw for ego & traffic
                self.egoYaw = np.array(self.currErgFile.readAllSignalValues("Car.Fr1.rz"))

                #Collision relative velocity
                self.heading_angle = np.array(self.currErgFile.readAllSignalValues(signal_mapping['heading_angle']))
                self.ego_heading_angle = np.array(self.currErgFile.readAllSignalValues(signal_mapping['ego_heading_angle']))
                self.target_vel_x = np.array(self.currErgFile.readAllSignalValues(signal_mapping['target_vel_x']))
                self.target_vel_y = np.array(self.currErgFile.readAllSignalValues(signal_mapping['target_vel_y']))
                self.ego_vel_x = np.array(self.currErgFile.readAllSignalValues(signal_mapping['ego_vel_x']))
                self.ego_vel_y = np.array(self.currErgFile.readAllSignalValues(signal_mapping['ego_vel_y']))

                # MoCo related signals from ERG
                self.f_LongDistMin_0 = np.array(self.currErgFile.readAllSignalValues(signal_mapping['f_LongDistMin_0']))
                self.f_RelLongDist_0 = np.array(self.currErgFile.readAllSignalValues(signal_mapping['f_RelLongDist_0']))
                self.f_LongIntrusionMax_0 = np.array(self.currErgFile.readAllSignalValues(signal_mapping['f_LongIntrusionMax_0']))
                self.f_LongIntrusionMin_0 = np.array(self.currErgFile.readAllSignalValues(signal_mapping['f_LongIntrusionMin_0']))
                self.f_AbsLongVelo_tar = np.array(self.currErgFile.readAllSignalValues(signal_mapping['f_AbsLongVelo_tar']))
                self.f_LongJerkLimMax_0 = np.array(self.currErgFile.readAllSignalValues(signal_mapping['f_LongJerkLimMax_0']))
                self.f_LongJerkLimMin_0 = np.array(self.currErgFile.readAllSignalValues(signal_mapping['f_LongJerkLimMin_0']))
                self.f_LongAccelLimMin_0 = np.array(self.currErgFile.readAllSignalValues(signal_mapping['f_LongAccelLimMin_0']))
                self.f_LongAccelLimMax_0 = np.array(self.currErgFile.readAllSignalValues(signal_mapping['f_LongAccelLimMax_0']))
                self.accelReq = np.array(self.currErgFile.readAllSignalValues(signal_mapping["accelReq"]))
                self.velTrans_x_mu = np.array(self.currErgFile.readAllSignalValues(signal_mapping['velTrans_x_mu']))
                self.accelTrans_x_mu = np.array(self.currErgFile.readAllSignalValues(signal_mapping['accelTrans_x_mu']))
                self.longTrajReqAcc_0 = np.array(self.currErgFile.readAllSignalValues(signal_mapping["longTrajReqAcc_0"]))
                self.longTrajReqVel_0 = np.array(self.currErgFile.readAllSignalValues(signal_mapping["longTrajReqVel_0"]))
                self.b_aebActivated = np.array(self.currErgFile.readAllSignalValues(signal_mapping["b_aebActivated"]))
                self.targetYaw = np.array(self.currErgFile.readAllSignalValues("Traffic.XTarget.rz"))

                #for AMP
                self.Brek_Index = []
                self.BreaksTime = []
                self.Number_Traf_Obj_atBTime = []
                self.Traf_X_pos_Brea_Time = []
                self.Traf_Y_pos_Brea_Time = []
                self.min_distan_y = []
                self.min_distan_x = []
                self.Number_Traf_Obj=np.array(self.currErgFile.readAllSignalValues(signal_mapping["Num_Traffic_Part"]))
                self.TTC_All = np.array(self.currErgFile.readAllSignalValues(signal_mapping["acdc_hyp_ttc"]))
                self.Traf_0_X_Pos =np.array(self.currErgFile.readAllSignalValues(signal_mapping["Traf_0_X_Pos"]))
            except:
                try:
                    self.TimeVec = np.array(self.currErgFile.readAllSignalValues("Time"))
                    self.currHypCrossingProb = np.array(self.currErgFile.readAllSignalValues("QVA.HypCrossing.Prob"))
                    self.ego_velocity = (np.array(self.currErgFile.readAllSignalValues("Car.v"))) * 3.6
                    self.ego_acceleration = np.array(self.currErgFile.readAllSignalValues("Car.ax"))
                    self.traffic_long_velocity = (np.array(
                        self.currErgFile.readAllSignalValues("Traffic.T00.LongVel"))) * 3.6
                    self.prebrake = np.array(self.currErgFile.readAllSignalValues("_MEDIC.PreBrake.bPreBrakeDecelEnabled"))
                except:
                    self.TimeVec                = np.array(self.currErgFile.readAllSignalValues("Time"))

    def calcEgoRect(self, egoLength, egoWidth):
        """
        Calculate ego rectangle and its shape points which is necessary e.g. for collision detection. Usually this function does not need to be called manually
        as every calculation function will call it implicitly if ego rectangle is not calculated yet.
        
        :param egoLength: Length of the ego vehicle
        :param egoWidth: Width of the ego vehicle
        """
        if not hasattr(self, 'egotx'):
            self.egotx                  = np.array(self.currErgFile.readAllSignalValues("Car.Fr1.tx"))
        if not hasattr(self, 'egoty'):
            self.egoty                  = np.array(self.currErgFile.readAllSignalValues("Car.Fr1.ty"))
        self.egoLength              = egoLength
        self.egoWidth               = egoWidth
        #self.egoTheta               = np.arctan2(np.array(self.currErgFile.readAllSignalValues("Car.Fr1.vy")),
        #                                                   np.array(self.currErgFile.readAllSignalValues("Car.Fr1.vx")))    does not work properly
        if not hasattr(self, 'egovx'):
            self.egovx                  = np.array(self.currErgFile.readAllSignalValues("Car.Fr1.vx"))
        if not hasattr(self, 'egovy'):
            self.egovy                  = np.array(self.currErgFile.readAllSignalValues("Car.Fr1.vy"))
        if not hasattr(self, 'egoTheta'):
            self.egoTheta               = np.zeros(shape=(len(self.egotx))) #cause it's always zero
            #self.egoTheta               = np.arctan2( self.egovy, self.egovx )
            self.egoTheta               = np.array(self.currErgFile.readAllSignalValues("Car.Fr1.rz"))
        self.egoRect                = ({"FL":(self.egotx - self.egoWidth/2.0*np.sin(self.egoTheta) + self.egoLength*np.cos(self.egoTheta),
                                              self.egoty + self.egoWidth/2.0*np.cos(self.egoTheta) + self.egoLength*np.sin(self.egoTheta)),
                                        "FR":(self.egotx + self.egoWidth/2.0*np.sin(self.egoTheta) + self.egoLength*np.cos(self.egoTheta),
                                              self.egoty - self.egoWidth/2.0*np.cos(self.egoTheta) + self.egoLength*np.sin(self.egoTheta)),
                                        "RR":(self.egotx + self.egoWidth/2.0*np.sin(self.egoTheta),
                                              self.egoty - self.egoWidth/2.0*np.cos(self.egoTheta)),
                                        "RL":(self.egotx - self.egoWidth/2.0*np.sin(self.egoTheta),
                                              self.egoty + self.egoWidth/2.0*np.cos(self.egoTheta))})

    def calcEgoRect2(self, egoLength, egoWidth):
        if not hasattr(self, 'egotx'):
            self.egotx                  = np.array(self.currErgFile.readAllSignalValues("Car.Fr1.tx"))
        if not hasattr(self, 'egoty'):
            self.egoty                  = np.array(self.currErgFile.readAllSignalValues("Car.Fr1.ty"))
        self.egoLength              = egoLength
        self.egoWidth               = egoWidth
        #self.egoTheta               = np.arctan2(np.array(self.currErgFile.readAllSignalValues("Car.Fr1.vy")),
        #                                                   np.array(self.currErgFile.readAllSignalValues("Car.Fr1.vx")))    does not work properly
        if not hasattr(self, 'egovx'):
            self.egovx                  = np.array(self.currErgFile.readAllSignalValues("Car.Fr1.vx"))
        if not hasattr(self, 'egovy'):
            self.egovy                  = np.array(self.currErgFile.readAllSignalValues("Car.Fr1.vy"))
        if not hasattr(self, 'egoTheta'):
            self.egoTheta               = np.zeros(shape=(len(self.egotx))) #cause it's always zero
            #self.egoTheta               = np.arctan2( self.egovy, self.egovx )
            self.egoTheta               = np.array(self.currErgFile.readAllSignalValues("Car.Fr1.rz"))
        self.egoRect                = [{"FL":(self.egotx - self.egoWidth/2.0*np.sin(self.egoTheta) + self.egoLength*np.cos(self.egoTheta),
                                              self.egoty + self.egoWidth/2.0*np.cos(self.egoTheta) + self.egoLength*np.sin(self.egoTheta)),
                                        "FR":(self.egotx + self.egoWidth/2.0*np.sin(self.egoTheta) + self.egoLength*np.cos(self.egoTheta),
                                              self.egoty - self.egoWidth/2.0*np.cos(self.egoTheta) + self.egoLength*np.sin(self.egoTheta)),
                                        "RR":(self.egotx + self.egoWidth/2.0*np.sin(self.egoTheta),
                                              self.egoty - self.egoWidth/2.0*np.cos(self.egoTheta)),
                                        "RL":(self.egotx - self.egoWidth/2.0*np.sin(self.egoTheta),
                                              self.egoty + self.egoWidth/2.0*np.cos(self.egoTheta))}]
        return self.egoRect

    def calcTargetRects(self, targetLength, targetWidth, targetName, sections):
        """
        Calculate target rectangle and its shape points which is necessary e.g. for collision detection. Usually this function does not need to be called manually
        as every calculation function will call it implicitly if target rectangle is not calculated yet.
        
        :param targetLength: Length of the target vehicle
        :param targetWidth: Width of the target vehicle
        :param targetName: Name of the target vehicle as defined in the testrun
        :param sections: Number of sections the target vehicle is separated in. May be useful for the detection with which part the of the target vehicle the \
        ego vehicle collides, e.g. hood, cabin, trunk. If this is not relevant, choose 1.

        :return: target rectangle shape points (dict)
        """

        self.targettx               = np.array(self.currErgFile.readAllSignalValues("Traffic." + targetName + ".tx"))
        self.targetty               = np.array(self.currErgFile.readAllSignalValues("Traffic." + targetName + ".ty"))
        self.targetTheta            = np.array(self.currErgFile.readAllSignalValues("Traffic." + targetName + ".rz"))
        targetRect = []
        for currElement in range(sections):
            targetRect.append({"FL":(self.targettx - targetWidth/2.0*np.sin(self.targetTheta) + float(currElement+1)/sections * targetLength*np.cos(self.targetTheta),
                                     self.targetty + targetWidth/2.0*np.cos(self.targetTheta) + float(currElement+1)/sections * targetLength*np.sin(self.targetTheta)),
                               "FR":(self.targettx + targetWidth/2.0*np.sin(self.targetTheta) + float(currElement+1)/sections * targetLength*np.cos(self.targetTheta),
                                     self.targetty - targetWidth/2.0*np.cos(self.targetTheta) + float(currElement+1)/sections * targetLength*np.sin(self.targetTheta)),
                               "RR":(self.targettx + targetWidth/2.0*np.sin(self.targetTheta) + float(currElement  )/sections * targetLength*np.cos(self.targetTheta),
                                     self.targetty - targetWidth/2.0*np.cos(self.targetTheta) + float(currElement  )/sections * targetLength*np.sin(self.targetTheta)),
                               "RL":(self.targettx - targetWidth/2.0*np.sin(self.targetTheta) + float(currElement  )/sections * targetLength*np.cos(self.targetTheta),
                                     self.targetty + targetWidth/2.0*np.cos(self.targetTheta) + float(currElement  )/sections * targetLength*np.sin(self.targetTheta))})
        return targetRect

    def calcMinDist(self, egoLength, egoWidth, targetLength, targetWidth, targetName, minDistList=False):
        """
        Calculate minimum distance between the ego vehicle and a target vehicle.
        
        :param egoLength: Length of the ego vehicle
        :param egoWidth: Width of the ego vehicle
        :param targetLength: Length of the target vehicle
        :param targetWidth: Width of the target vehicle
        :param targetName: Name of the target vehicle as defined in the testrun

        return: Minimum distance
        """
        def calcMinDistSegments(rectPoints, rectSegments):

            def testIfPointOnRect(rectSegmentPrevX, rectSegmentPrevY, rectSegmentX, rectSegmentY, pointOnSegX, pointOnSegY, rectPointEgoX, rectPointEgoY):
                if    (min(rectSegmentPrevX, rectSegmentX) < pointOnSegX) and (max(rectSegmentPrevX, rectSegmentX) > pointOnSegX) \
                  and (min(rectSegmentPrevY, rectSegmentY) < pointOnSegY) and (max(rectSegmentPrevY, rectSegmentY) > pointOnSegY):
                    distPoint = math.sqrt( (pointOnSegX-rectPointEgoX)**2 + (pointOnSegY-rectPointEgoY)**2 )
                    return distPoint
                return INIT_DISTANCE

            vTestIfPointOnRect = np.vectorize(testIfPointOnRect)
            minDistPntSeg = np.empty(shape=(len(rectPoints['RL'][0])), dtype=float)
            minDistPntSeg.fill(INIT_DISTANCE)
            cornerList = ['FL', 'FR', 'RR', 'RL']
            for keyEgo in cornerList:
                keyObjPrev = cornerList[-1]
                for keyObj in cornerList:
                    directionVector = [rectSegments[keyObj][0] - rectSegments[keyObjPrev][0],
                                       rectSegments[keyObj][1] - rectSegments[keyObjPrev][1] ]
                    normVector      = [-directionVector[1], directionVector[0]]
                    xMinusR0        = [rectPoints[keyEgo][0]-rectSegments[keyObj][0],
                                       rectPoints[keyEgo][1]-rectSegments[keyObj][1] ]
                    factorForCalc   = (xMinusR0[0]*normVector[0] + xMinusR0[1]*normVector[1])/(normVector[0]**2 + normVector[1]**2)
                    pointOnSeg      = [rectPoints[keyEgo][0] - factorForCalc*normVector[0],
                                       rectPoints[keyEgo][1] - factorForCalc*normVector[1] ]
                    distPoints = vTestIfPointOnRect(rectSegments[keyObjPrev][0], rectSegments[keyObjPrev][1], rectSegments[keyObj][0], rectSegments[keyObj][1],
                                                pointOnSeg[0], pointOnSeg[1], rectPoints[keyEgo][0], rectPoints[keyEgo][1])
                    minDistPntSeg = np.minimum(minDistPntSeg, distPoints)
                    keyObjPrev = keyObj
            return minDistPntSeg

        if not hasattr(self, 'egoRect'):
            self.calcEgoRect(egoLength, egoWidth)
        if not hasattr(self, 'targetRectComplete'):
            self.targetRectComplete = self.calcTargetRects(targetLength, targetWidth, targetName, 1)
        self.minDist = INIT_DISTANCE
        # check MinDist of Points
        for keyEgo in self.egoRect:
            for keyObj in self.targetRectComplete[0]:
                distpoints = np.sqrt( (self.targetRectComplete[0][keyObj][0]-self.egoRect[keyEgo][0])**2 + (self.targetRectComplete[0][keyObj][1]-self.egoRect[keyEgo][1])**2 )
                self.minDist = np.minimum(distpoints, self.minDist)
        self.minDist = np.minimum(calcMinDistSegments(self.egoRect, self.targetRectComplete[0]), self.minDist)
        self.minDist = np.minimum(calcMinDistSegments(self.targetRectComplete[0], self.egoRect), self.minDist)

        if minDistList:
            return self.minDist
        else:
            return min(self.minDist)

    def calcCollisionOccured(self, egoLength, egoWidth, targetLength, targetWidth, targetName, minTime=0):
        """
        Calculate if a collision happened between the ego vehicle and a target vehicle.
        Note: This function comes from a time when Carmaker did not have the CollisionSensor. It is recommended to use the signal of the CollisionSensor
        instead.
        
        :param egoLength: Length of the ego vehicle
        :param egoWidth: Width of the ego vehicle
        :param targetLength: Length of the target vehicle
        :param targetWidth: Width of the target vehicle
        :param targetName: Name of the target vehicle as defined in the testrun
        :param minTime: Simulation time at which the evaluation for a collision starts, e.g. time > 0.1s

        :return: True if collision occured, false if not
        """
        if not hasattr(self, 'egoRect'):
            self.calcEgoRect(egoLength, egoWidth)
        if not hasattr(self, 'targetRectComplete'):
            self.targetRectComplete = self.calcTargetRects(targetLength, targetWidth, targetName, 1)
        self.collisionOccured = self.currSectionCalcCollisionOccuredPy(self.egoRect, self.targetRectComplete[0])
        self.collisionOccured[0:minTime*100] = np.zeros([minTime*100], 'bool')
        if max(self.collisionOccured):
            return True
        return False

    def calcCollision_coord(self,egoLength, egoWidth,targetLength, targetWidth, targetName, timeOfColl, COM):
        """
        Calculate the collision time of the ego vehicle and a target vehicle
        
        :param egoLength: Length of the ego vehicle
        :param egoWidth: Width of the ego vehicle
        :param targetLength: Length of the target vehicle
        :param targetWidth: Width of the target vehicle
        :param targetName: Name of the target vehicle as defined in the testrun

        :return: Collision time
        """


        self.egoRectComplete = self.calcEgoRect2(egoLength, egoWidth)
        self.targetRectComplete = self.calcTargetRects(targetLength, targetWidth, targetName, 1)
        egoRect, tarRect= self.egoRectComplete[0], self.targetRectComplete[0]
        timeOfColl = timeOfColl * 100
        Abbr = {'FL': 'Front Left', 'FR': 'Front Right', 'RR': 'Rear Right', 'RL': 'Rear Left',
                'Front': 'Front Side', 'Left': 'Left Side', 'Right': 'Right Side', 'Rear': 'Rear Side'}
        def get_distance(coord1, coord2):
            return math.sqrt(((coord2[0] - coord1[0]) ** 2 + (coord2[1] - coord1[1]) ** 2))

        for rects in [egoRect, tarRect]:
            for key, val in rects.items():
                rects[key] = (round(val[0][int(timeOfColl)], 2), round(val[1][int(timeOfColl)],2))

        tar_quant = [FL_tar, FR_tar, RR_tar, RL_tar] = [tarRect['FL'], tarRect['FR'], tarRect['RR'], tarRect['RL']]
        ego_quant = [FL_ego, FR_ego, RR_ego, RL_ego] = [egoRect['FL'],egoRect['FR'], egoRect['RR'],egoRect['RL']]
        # print 'EGO: ', ego_quant, 'TARGET: ', tar_quant

        equations = [egoEquation, tarEquation] = [
            {'FL_FR_ego': [FL_ego, FR_ego, '', '','Ego_Front'], 'FR_RR_ego': [FR_ego, RR_ego, '', '','Ego_Right'],
             'RR_RL_ego': [RR_ego, RL_ego, '', '','Ego_Rear'], 'RL_FL_ego': [RL_ego,FL_ego, '', '','Ego_Left']},
            {'FL_FR_tar': [FL_tar, FR_tar, '', '', 'Target_Front'], 'FR_RR_tar': [FR_tar, RR_tar, '', '', 'Target_Right'],
             'RR_RL_tar': [RR_tar, RL_tar, '', '', 'Target_Rear'], 'RL_FL_tar': [RL_tar,FL_tar, '', '', 'Target_Left']}]
        # Populating line equation calculation
        for equation in equations:
            for side,val in equation.items():
                y1, y2, x1, x2 = float(val[0][1]), float(val[1][1]), float(val[0][0]), float(val[1][0])
                if x2-x1 != 0:
                    m = (y2 - y1) / (x2-x1)
                    c = y2 - (m * x2)  # m: slope and c: intercept calculation
                    val[2], val[3] = round(m, 2), round(c, 2)
                else:
                    m = 'undefined'
                    c = 'undefined'
                    val[2], val[3] = m,c

        coll_dict = {}      #coll_dict : {min dist: coordinates, ego, target }
        corners_map = {0: 'FL', 1: 'FR', 2:'RR',3: 'RL'}

        # Center of Mass Calculation
        def calcCOM(vehicle, value):
            if vehicle == 'Tar':
                calData = [tarEquation['RL_FL_tar'], tar_quant[2], tar_quant[3], tar_quant[0], tar_quant[1]]
            else:
                calData = [egoEquation['RL_FL_ego'], ego_quant[2], ego_quant[3], ego_quant[0], ego_quant[1]]
            coord0 = (((calData[1][0] + calData[2][0])*0.5), ((calData[1][1] + calData[2][1])*0.5))
            coord1 = (((calData[3][0] + calData[4][0]) * 0.5), ((calData[3][1] + calData[4][1]) * 0.5))
            slope = calData[0][2]
            if not isinstance(slope, str):
                dx = (value / math.sqrt(1 + (slope * slope)))
                dy = slope * dx
                ax = coord0[0] + dx
                ay = coord0[1] + dy
                bx = coord0[0] - dx
                by = coord0[1] - dy
                if (ax > coord0[0] and ax < coord1[0]) or (ay > coord0[1] and ay < coord1[1]):
                    coord = (ax,ay)
                else:
                    coord = (bx,by)
            else:
                coord = (0,value)
            return (round(coord[0],2), round(coord[1],2))
        trafficCOM = COM[1]
        egoCOM = COM[0]
        tarCoG = calcCOM('Tar', trafficCOM)
        egoCog = calcCOM('Ego', egoCOM)
        # print 'tar:', tarCoG, 'Ego: ', egoCog

        for equation2 in equations:
            vehicle1 = equation2
            if equations.index(equation2)==0:
                quantList1, quantList2, prefix1, prefix2   = tar_quant, ego_quant, 'Target: ', 'Ego: '
            if equations.index(equation2) == 1:
                quantList1, quantList2, prefix1, prefix2 = ego_quant, tar_quant, 'Ego: ' , 'Target: '

            # target corners distance calculation: tar_quant
            for corner in quantList1:
                corner_name = corners_map[quantList1.index(corner)]
                # prefix2 + Abbr[val[4].split('_')[1]], prefix1 + Abbr[corner_name]
                for key,val in vehicle1.items():
                    # print 'calculation for: ', prefix2 + Abbr[val[4].split('_')[1]], prefix1 + Abbr[corner_name]
                    m, c = val[2], val[3]
                    x1, x2 = val[0][0], val[1][0]
                    y1, y2 = val[0][1], val[1][1]
                    if not isinstance(m, str):
                        # slope = 0
                        if m == 0.00 or m == -0.00 or m == 0 or m == 0.0 or m == -0.0:
                            range_x = list(np.arange(min(x1, x2), max(x1, x2), 0.01))
                            range_xy = [(round(n,2),round(y1,2)) for n in range_x]
                        else:
                            if abs(x2-x1) > abs(y2-y1):
                                range_x = list(np.arange(min(x1, x2), max(x1, x2), 0.01))
                                range_xy = [(round(n, 2),round((n * m + c), 2)) for n in range_x]
                            else:
                                range_y = list(np.arange(min(y1, y2), max(y1, y2), 0.01))
                                range_xy = [(round(((n - c) / m), 2), round(n, 2)) for n in range_y]
                    # Undefined slope
                    else:
                        range_y = list(np.arange(min(y1, y2), max(y1, y2), 0.01))
                        range_xy = [(round(x1,2),round(n,2)) for n in range_y]

                    distances = [get_distance(corner, line_xy) for line_xy in range_xy]
                    min_dist = (min(distances), range_xy[distances.index(min(distances))])
                    coll_dict[min_dist[0]] = [min_dist[1], prefix2 + Abbr[val[4].split('_')[1]],
                                              prefix1 + Abbr[corner_name]]

                # Check corner collision
                if equations.index(equation2) == 0:
                    for corner2 in quantList2:
                        # calculate target corner to ego corner each distance
                        coll_dict[get_distance(corner, corner2)] = [
                            corner2, prefix2 + Abbr[corners_map[quantList2.index(corner2)]],
                            prefix1 + Abbr[corner_name]]

        selected = sorted(coll_dict[min(coll_dict.keys())])

        # Checking side collision (slope of line comparison)
        sides = {}      # Side name: slope
        side_collision = False
        for selection in selected:
            if selected.index(selection) ==2:
                pass
            else:
                prefix_side = selection.split(': ')[0] + '_'
                if selection.split(': ')[1].split(' ')[1] == 'Side':
                    side1 = prefix_side + selection.split(': ')[1].split(' ')[0]
                    sides[side1] = ''
                else:
                    side2 = prefix_side + selection.split(': ')[1].split(' ')[0]
                    side3 = prefix_side + selection.split(': ')[1].split(' ')[1]
                    sides[side2], sides[side3] = '', ''

        for key1,val1 in sides.items():
            equation3 = egoEquation if 'Ego_' in key1 else tarEquation
            for key2, val2 in equation3.items():
                if val2[4] == key1:
                    sides[key1] = val2[2]
        side_collision = True if sides[sides.keys()[0]] == sides[sides.keys()[1]] or \
                                 sides[sides.keys()[1]] == sides[sides.keys()[2]] or \
                                 sides[sides.keys()[0]] == sides[sides.keys()[2]] else False
        if side_collision and sides[sides.keys()[0]] == sides[sides.keys()[1]]:
            selected = sorted([sides.keys()[0], sides.keys()[1], selected[2]])
        elif side_collision and sides[sides.keys()[1]] == sides[sides.keys()[2]]:
            selected = sorted([sides.keys()[1], sides.keys()[2], selected[2]])
        elif side_collision and sides[sides.keys()[0]] == sides[sides.keys()[2]]:
            selected = sorted([sides.keys()[0], sides.keys()[2], selected[2]])

        if side_collision:
            selected[0], selected[1] = selected[0].split('_')[0] + ': ' + selected[0].split('_')[1] + ' Side', \
                                       selected[1].split('_')[0] + ': ' + selected[1].split('_')[1] + ' Side'
        # print "Collision ", selected[2], selected[0] + '<br />' + selected[1] + "\n\n"
        return [selected[2], selected[0] + '<br />' + selected[1], tarCoG, egoCog]

    def calcCollCoord_wrtET(self,egoLength, egoWidth,targetLength, targetWidth, targetName, timeOfColl, collision_details):
        self.egoRectComplete = self.calcEgoRect2(egoLength, egoWidth)
        self.targetRectComplete = self.calcTargetRects(targetLength, targetWidth, targetName, 1)
        egoRect, tarRect= self.egoRectComplete[0], self.targetRectComplete[0]
        [collisionCoord, collisionPositions, tarCoG, egoCog] = collision_details
        def get_refCoord(egoLength, egoWidth,targetLength, targetWidth):
            egoRef = [RearMid_ego, FrontMid_ego, FL_ego, FR_ego, RL_ego, RR_ego] = [(0,0),(0,egoLength),
                                                        (-1*egoWidth/2,egoLength),(egoWidth/2,egoLength),
                                                        (-1*egoWidth/2,0),(egoWidth/2,0)]
            tarRef = [RearMid_tar,FrontMid_tar, FL_tar, FR_tar, RL_tar, RR_tar] = [(0,0),(0,targetLength),
                                                        (-1*targetWidth/2,targetLength),(targetWidth/2,targetLength),
                                                        (-1*targetWidth/2,0),(targetWidth/2,0)]
            return [egoRef, tarRef]

        refCoordDefault = get_refCoord(egoLength, egoWidth,targetLength, targetWidth)

        def get_position(coord1,coord2, coord3, coord4):    #coord4: impact point
            return'left' if coord1 >= coord4[0] <= coord2[0] else 'right'
        def get_distance(coord1, coord2):
            return math.sqrt(((coord2[0] - coord1[0])**2 + (coord2[1] - coord1[1])**2))
        def get_coord_FR(position, distance, coord):
            return (-1*distance, coord[1]) if position =='left' else (distance, coord[1])
        def get_coord_LR(distance, coord):
            return (coord[0], distance)

        for rects in [egoRect, tarRect]:
            for key, val in rects.items():
                rects[key] = (val[0][int(timeOfColl * 100)], val[1][int(timeOfColl * 100)])

        ego_quant = [FL_ego, FR_ego, RR_ego, RL_ego] = [egoRect['FL'],egoRect['FR'], egoRect['RR'],egoRect['RL']]
        xy_refEgo_rear = ((RL_ego[0] + RR_ego[0])/2 , (RL_ego[1] + RR_ego[1])/2)
        xy_refEgo_front = ((FL_ego[0] + FR_ego[0]) / 2, (FL_ego[1] + FR_ego[1]) / 2)

        ego_quantAll = [ego_quant,xy_refEgo_rear, xy_refEgo_front]

        tar_quant = [FL_tar, FR_tar, RR_tar, RL_tar] = [tarRect['FL'], tarRect['FR'], tarRect['RR'], tarRect['RL']]
        xy_refTar_rear = ((RL_tar[0] + RR_tar[0])/2 , (RL_tar[1] + RR_tar[1])/2)
        xy_refTar_front = ((FL_tar[0] + FR_tar[0]) / 2, (FL_tar[1] + FR_tar[1]) / 2)
        tar_quantAll = [tar_quant, xy_refTar_rear, xy_refTar_front]

        Abbr = {'corner': ['Front Left', 'Front Right', 'Rear Right', 'Rear Left'],
                'side_FR': ['Front Side', 'Rear Side'],
                'side_RL': ['Left Side', 'Right Side']}

        collision_dict = {}         # eg: collision_dict = {'Ego': 'Front Side', 'Tar': 'Rear Left'}
        for positions in collisionPositions.split('<br />'):
            collision_dict[positions.split(': ')[0]] = positions.split(': ')[1]

        refCoords= {'Ego': '', 'Target': ''}
        for key,val in collision_dict.items():
            veh_data = [refCoordDefault[0],ego_quantAll] if key == 'Ego' else [refCoordDefault[1],tar_quantAll]
            if val in Abbr['side_FR']:  #FR: FrontRear
                if val.split(' ')[0] =='Front':
                    left, mid, right, mid_def, coll   = veh_data[1][0][0], veh_data[1][2], veh_data[1][0][1],\
                                                        veh_data[0][1], collisionCoord
                if val.split(' ')[0] =='Rear':
                    left, mid, right, mid_def, coll = veh_data[1][0][3], veh_data[1][1], veh_data[1][0][2],\
                                                      veh_data[0][2], collisionCoord
                position = get_position(left, mid, right, coll)
                distance = get_distance(mid, coll)
                req_coord = get_coord_FR(position, distance, mid_def)

            if val in Abbr['side_RL']:  #RL: RightLeft
                if val.split(' ')[0] == 'Left':
                    RL, RL_def, coll  = veh_data[1][0][3], veh_data[0][4], collisionCoord #RL: Rear:Right/Left
                if val.split(' ')[0] == 'Right':
                    RL, RL_def, coll = veh_data[1][0][2], veh_data[0][5], collisionCoord
                distance = get_distance(RL, coll)
                req_coord = get_coord_LR(distance, RL_def)
            if val in Abbr['corner']:
                data = veh_data[0]
                Corner_coord = {'Front Left': data[2], 'Front Right': data[3],
                                'Rear Right': data[5], 'Rear Left': data[4]}
                req_coord = Corner_coord[val]
            refCoords[key] = (round(req_coord[0],2), round(req_coord[1],2))
        # print "Reference coordinates: ", refCoords
        return refCoords

    def calcCollisionTime(self, egoLength, egoWidth, targetLength, targetWidth, targetName):
        if not hasattr(self, 'collisionOccured'):
            self.calcCollisionOccured(egoLength, egoWidth, targetLength, targetWidth, targetName)
        self.currCollisionIdx   = np.argmax(self.collisionOccured)
        self.collisionTime      = self.TimeVec[self.currCollisionIdx]
        return self.collisionTime

    def getQuantityAtTime(self, quantityToCalc, currTime):
        """
        Get a quantity at a certain time
        
        :param quantityToCalc: Quanitity name (str)
        :param currTime: Time at which the value of the quantity shall be given

        :return: Value of the quantity
        """


        if hasattr(self, quantityToCalc):
            self.currQuantity       = getattr(self, quantityToCalc)
        else:
            self.currQuantity       = np.array(self.currErgFile.readAllSignalValues(quantityToCalc))

        # Rounding currTime to 2 digits as we have rounded TimeVec, to resolve issue for few cases in EuNCAP
        timeVecList = np.array(np.where(self.TimeVec >= round(currTime, 2))).tolist()[0]
        if timeVecList:
            self.currTimeIdx            = timeVecList[0]
            self.currQuantityAtTime     = self.currQuantity[self.currTimeIdx]

            return self.currQuantityAtTime
        else:
            return int(currTime*100)

    def calcValueOccurred(self, quantityToCalc, currValue, condition, roundDeciamal=False, decimalPlace=2):
        """
        Check if a condition of a quantity is true at any time.
        Ex: calcValueOccurred("Car.vx", "10", ">") checks if Car.vx is grater than 10 any time in the testrun.
        
        :param quantityToCalc: Quanitity name (str)
        :param currValue: Value that shall be checked
        :param condition: Condition that shall be checked: ">", "<", ">=", "<=", "==", "!="
        :param roundDeciamal: Boolean, specifying if values are to be rounded.
        :param decimalPlace: Decimal places to which values are to be rounded.

        :return: True, if the condition is true at any time. False, if not.
        """

        ops = {">":operator.gt, "<":operator.lt, ">=":operator.ge, "<=":operator.le, "==":operator.eq, "!=":operator.ne}
        if hasattr(self, quantityToCalc):
            self.currQuantity       = getattr(self, quantityToCalc)
        else:
            self.currQuantity       = np.array(self.currErgFile.readAllSignalValues(quantityToCalc))
        if roundDeciamal:
            self.currQuantity = np.round(self.currQuantity, decimalPlace)
            currValue = round(currValue, decimalPlace)
        if any(ops[condition](self.currQuantity, currValue)):
            return True
        return False

    def calcValueOccurred_between_interval(self, quantityToCalc, currValue, condition, interval):
        """
                Check if a condition of a quantity falls under particular interval.
                :param quantityToCalc: Quanitity name (str)
                :param currValue: By default 2
                :param condition:  Condition that shall be checked: ">", "<", ">=", "<=", "==", "!="
                :param interval: [Start Range, End Range] List with index 0 as start_time and index 1 as end_time
                :return: True, if the condition is true at any time. False, if not.
                """
        ops = {">": operator.gt, "<": operator.lt, ">=": operator.ge, "<=": operator.le, "==": operator.eq,
               "!=": operator.ne}
        self.currQuantity       = self.currErgFile.readAllSignalValues(quantityToCalc)

        if self.currQuantity == []: # To handle signals that are not present
            return None

        try:
            time_indices = np.array(np.where((np.array(self.TimeVec) > interval[0]) & (np.array(self.TimeVec) < interval[1]))).tolist()[0]
            start_index = time_indices[0]
            end_index = time_indices[-1]
            for i in range(start_index, end_index):
                if ops[condition](self.currQuantity[i], currValue):
                    return True
            return False
        except:
            error_msg = 'ERROR'
            return error_msg

    def calcValueCollisionOccured(self, quantityToCalc):   # To Calculate collision occured or no (Ranishree)
        if hasattr(self, quantityToCalc):
            self.currQuantity = getattr(self, quantityToCalc)
        else:
            self.currQuantity = np.array(self.currErgFile.readAllSignalValues(quantityToCalc))
        collision_list = list(set(self.currQuantity))
        if len(collision_list) > 1:
            return True
        return False

    def calcValueOccurred_AMP(self, quantityToCalc, currValue, condition):
        ops = {">":operator.gt, "<":operator.lt, ">=":operator.ge, "<=":operator.le, "==":operator.eq, "!=":operator.ne}
        Num_Brek_Index=0
        j1=0
        Critical1 = 0
        Critical2 = 0
        Critical3 = 0
        Brek_Index=[]
        currQuantityIdx=[]
        Start_vel=[]
        After_Brake_Vel=[]
        length_time_vec = len(self.TimeVec)
        Duration = self.TimeVec[length_time_vec-1]
        Brake_Duration =[]

        for i in range(len(self.currHypTTC)):
            if self.currHypTTC[i] <= 1.5 and self.currHypTTC[i] > 1:
                Critical1 = Critical1 + 1
            elif self.currHypTTC[i] <= 1 and self.currHypTTC[i] > 0.5:
                Critical2 = Critical2 + 1
            elif self.currHypTTC[i] > 0 and self.currHypTTC[i] <= 0.5:
                Critical3 = Critical3 + 1
        if hasattr(self, quantityToCalc):
            self.currQuantity       = getattr(self, quantityToCalc)
        else:
            self.currQuantity       = np.array(self.currErgFile.readAllSignalValues(quantityToCalc))

            self.lenght_currQuantity = len(self.currQuantity)
            for i in range(self.lenght_currQuantity):
                if self.currQuantity[i] > 0:
                    Brek_Index.append(i)
                    Num_Brek_Index=Num_Brek_Index+1
            if Num_Brek_Index>0:
                currQuantityIdx.append(Brek_Index[0])
                for i1 in range(len(Brek_Index)-1):
                    if i1+2 == Num_Brek_Index :
                        Brake_Duration.append(self.TimeVec[Brek_Index[i1]] - self.TimeVec[currQuantityIdx[j1]])
                        Start_vel.append(self.ego_velocity[currQuantityIdx[j1]])
                        After_Brake_Vel.append(self.ego_velocity[Brek_Index[i1]])
                        break
                    self.Differ = Brek_Index[i1+1] - Brek_Index[i1]
                    if self.Differ > 1:
                        j1 = j1 + 1
                        currQuantityIdx.append(Brek_Index[i1+1])
                        Brake_Duration.append(self.TimeVec[Brek_Index[i1]] - self.TimeVec[currQuantityIdx[j1-1]])
                        Start_vel.append(self.ego_velocity[currQuantityIdx[j1-1]])
                        After_Brake_Vel.append(self.ego_velocity[Brek_Index[i1]])

                self.Howmany_Times = j1+1
                self.BreaksTime = [0 for x in range(len(currQuantityIdx))]
                self.Number_Traf_Obj_atBTime = [0 for x in range(len(currQuantityIdx))]
                TTC_At_Break = [0 for x in range(len(currQuantityIdx))]
                self.min_distan_x = [0 for x in range(len(currQuantityIdx))]
                self.min_distan_y = [0 for x in range(len(currQuantityIdx))]
                self.Ego_Vel_At_Break=[0 for x in range(len(currQuantityIdx))]
                self.Traf_Obj_x = [[0 for x in range(10)] for y in range(len(self.Traf_0_X_Pos))]
                self.Traf_Obj_y = [[0 for x in range(10)] for y in range(len(self.Traf_0_X_Pos))]

                self.Traf_Y_pos_Brea_Time = [[0 for x in range(10)] for y in range(len(currQuantityIdx))]
                self.Traf_X_pos_Brea_Time = [[0 for x in range(10)] for y in range(len(currQuantityIdx))]
                Distance_bet_Ego_Tar = [[0 for x in range(5)] for y in range(len(currQuantityIdx))]
                TTc_for_Near_TP = [[0 for x in range(5)] for y in range(len(currQuantityIdx))]
                TP_X_Position = [[0 for x in range(5)] for y in range(len(currQuantityIdx))]
                TP_Y_Position = [[0 for x in range(5)] for y in range(len(currQuantityIdx))]
                Traf_Y_Positio = [[0 for x in range(5)] for y in range(len(currQuantityIdx))]
                Traf_X_Positio = [[0 for x in range(5)] for y in range(len(currQuantityIdx))]
                On_Ego_Lane = [0 for x in range(len(currQuantityIdx))];
                Traf_Y_pos_Brea_Time1 = [[0 for x in range(10)] for y in range(len(currQuantityIdx))]
                Traf_X_pos_Brea_Time1 = [[0 for x in range(10)] for y in range(len(currQuantityIdx))]
                for i2 in range(len(currQuantityIdx)):
                    self.BreaksTime[i2] = self.TimeVec[currQuantityIdx[i2]]
                    self.Ego_Vel_At_Break[i2] = self.ego_velocity[currQuantityIdx[i2]]
                    self.Number_Traf_Obj_atBTime[i2] = self.Number_Traf_Obj[currQuantityIdx[i2]]
                    TTC_At_Break[i2] = self.TTC_All[currQuantityIdx[i2]]

                    for num_Obj in range(self.Number_Traf_Obj_atBTime[i2]):
                        x_signal_name_String = "_MultiSensor.Out.sensdata_0.tpout.TrafPart_" + str(num_Obj) + ".GeomProp.Position.f_X"
                        y_signal_name_String = "_MultiSensor.Out.sensdata_0.tpout.TrafPart_" + str(num_Obj) + ".GeomProp.Position.f_Y"
                        self.Traf_Obj_x[num_Obj] = np.array(self.currErgFile.readAllSignalValues(x_signal_name_String))
                        self.Traf_Obj_y[num_Obj] = np.array(self.currErgFile.readAllSignalValues(y_signal_name_String))
                    for i3 in range(self.Number_Traf_Obj_atBTime[i2]):
                        if i3 < 5 :
                            self.Traf_X_pos_Brea_Time[i2][i3] = self.Traf_Obj_x[i3][currQuantityIdx[i2]]
                            self.Traf_Y_pos_Brea_Time[i2][i3] = self.Traf_Obj_y[i3][currQuantityIdx[i2]]
                    Traf_X_Positio1 = []
                    Traf_Y_Positio1 = []
                    for i4 in range(len(self.Traf_Y_pos_Brea_Time[i2])):
                        if self.Traf_Y_pos_Brea_Time[i2][i4]!=0:
                            Traf_Y_pos_Brea_Time1[i2][i4] = self.Traf_Y_pos_Brea_Time[i2][i4]
                            if self.Traf_Y_pos_Brea_Time[i2][i4] < 0:
                               Traf_Y_pos_Brea_Time1[i2][i4] = self.Traf_Y_pos_Brea_Time[i2][i4] * (-1)

                            Traf_Y_Positio1.append(Traf_Y_pos_Brea_Time1[i2][i4])
                            Traf_Y_Positio[i2][i4] = (self.Traf_Y_pos_Brea_Time[i2][i4])
                    for i4 in range(len(self.Traf_X_pos_Brea_Time[i2])):

                        if self.Traf_X_pos_Brea_Time[i2][i4] != 0:
                            Traf_X_pos_Brea_Time1[i2][i4] = self.Traf_X_pos_Brea_Time[i2][i4]
                            if self.Traf_X_pos_Brea_Time[i2][i4] < 0:
                                Traf_X_pos_Brea_Time1[i2][i4] = self.Traf_X_pos_Brea_Time[i2][i4] * (-1)

                            Traf_X_Positio1.append(Traf_X_pos_Brea_Time1[i2][i4])
                            Traf_X_Positio[i2][i4] = self.Traf_X_pos_Brea_Time[i2][i4]
                    try:
                        self.min_distan_y[i2] = min(Traf_Y_Positio1)
                        self.min_distan_x[i2] = min(Traf_X_Positio1)
                    except:
                        pass
                    #To find Any traffi prt in Ego lane with thresou;o of 1.5 left and right
                    if self.min_distan_y[i2] < 1.5:

                        On_Ego_Lane[i2] = 1
                    else:
                        On_Ego_Lane[i2] = 0
                    #nearrest TP

                    for i5 in range(len(Traf_Y_Positio[i2])):
                        TP_X_Position[i2][i5]=Traf_X_Positio[i2][i5]
                        TP_Y_Position[i2][i5]=Traf_Y_Positio[i2][i5]
                        Distance_bet_Ego_Tar[i2][i5] = math.sqrt((Traf_Y_Positio[i2][i5]**2)+(Traf_X_Positio[i2][i5]**2))
                    TTc_for_Near_TP =TTC_At_Break

                return 1,self.BreaksTime,self.Howmany_Times,TP_X_Position,TP_Y_Position,On_Ego_Lane,TTc_for_Near_TP,str("{:.2f}".format(Duration)),Critical1,Critical2,Critical3,Brake_Duration,Start_vel,After_Brake_Vel
            else:
                return 0,0,0,0,0,"No",0,str("{:.2f}".format(Duration)),Critical1,Critical2,Critical3,Brake_Duration,0,0



    def calcQuantityMin(self, quantityName):
        """
        Check the minimum value of a quantity
        
        :param quantityName: Quanitity name (str)

        :return: Minimum value of the quantity
        """

        if hasattr(self, quantityName):
            self.currQuantity = getattr(self, quantityName)
        else:
            self.currQuantity = np.array(self.currErgFile.readAllSignalValues(quantityName))
        minValue              = np.min(self.currQuantity)
        return minValue

    def calcQuantityMax(self, quantityName):
        """
        Check the maximum value of a quantity
        
        :param quantityName: Quanitity name (str)

        :return: Maximum value of the quantity
        """

        if hasattr(self, quantityName):
            self.currQuantity = getattr(self, quantityName)
        else:
            self.currQuantity = np.array(self.currErgFile.readAllSignalValues(quantityName))
        maxValue              = max(self.currQuantity)
        return maxValue

    def valOccuredQuantityMax(self, quantityName, condition, currValue):
        ops = {">": operator.gt, "<": operator.lt, ">=": operator.ge, "<=": operator.le, "==": operator.eq,
               "!=": operator.ne}
        if hasattr(self, quantityName):
            self.currQuantity = getattr(self, quantityName)
        else:
            self.currQuantity = np.array(self.currErgFile.readAllSignalValues(quantityName))
        maxValue = np.max(self.currQuantity)
        if ops[condition](maxValue, float(currValue)):
            return True
        return False

    def calcTimeAtValue(self, quantityToCalc, currValue, condition):
        """
        Check the time when a condition is true for the first time.
        
        :param quantityToCalc: Quanitity name (str)
        :param currValue: Value that shall be checked
        :param condition: Condition that shall be checked: ">", "<", ">=", "<=", "==", "!="

        :return: Time when the condition is true the first time
        """

        ops = {">":operator.gt, "<":operator.lt, ">=":operator.ge, "<=":operator.le, "==":operator.eq, "!=":operator.ne}
        if hasattr(self, quantityToCalc):
            self.currQuantity       = getattr(self, quantityToCalc)
        else:
            self.currQuantity       = np.array(self.currErgFile.readAllSignalValues(quantityToCalc))
        self.currQuantityIdx        = np.argmax(ops[condition](self.currQuantity, currValue))
        self.currTimeValue          = self.TimeVec[self.currQuantityIdx]
        return self.currTimeValue

    def calcIdxAtValue(self, quantityToCalc, currValue, condition):
        """
        Check the index when a condition is true for the first time.
        
        :param quantityToCalc: Quanitity name (str)
        :param currValue: Value that shall be checked
        :param condition: Condition that shall be checked: ">", "<", ">=", "<=", "==", "!="

        :return: Index in quantity values array when the condition is true the first time
        """

        ops = {">":operator.gt, "<":operator.lt, ">=":operator.ge, "<=":operator.le, "==":operator.eq, "!=":operator.ne}
        if hasattr(self, quantityToCalc):
            self.currQuantity       = getattr(self, quantityToCalc)
        else:
            self.currQuantity       = np.array(self.currErgFile.readAllSignalValues(quantityToCalc))
        self.currQuantityIdx        = np.argmax(ops[condition](self.currQuantity, currValue))
        return self.currQuantityIdx

    def calcIdxAtTime(self, currValue):
        """
        Check the index of a specific time
        
        :param currValue: Time for that the index shall be returned

        :return: Index in time array for given time. If the given time is between two times in the array, the upper value is taken
        """

        if hasattr(self, 'TimeVec'):
            self.currQuantity       = getattr(self, 'TimeVec')
        else:
            self.currQuantity       = np.array(self.currErgFile.readAllSignalValues('TimeVec'))
        self.currQuantityIdx        = np.argmax(operator.eq(self.currQuantity, currValue))
        return self.currQuantityIdx

    #Func to return value at specific index.
    def calcValueAtIdx(self, quantityToCalc, index):
        if hasattr(self, quantityToCalc):
            self.currQuantity = getattr(self, quantityToCalc)
        else:
            self.currQuantity = np.array(self.currErgFile.readAllSignalValues(quantityToCalc))
        return self.currQuantity[index]

    def calcOccurancesOfSignal(self, quantityToCalc, currValue, condition, startTime=0.0, Type=''):
        """
        Check if the given condition is true and find all occurances.
        
        :param quantityToCalc: Name of the quantity (str)
        :param currValue: Value that shall be checked
        :param condition: Condition that shall be checked: ">", "<", ">=", "<=", "==", "!="
        :param startTime: Simulation time at which the evaluation for the condition shall start, e.g. time > 0.1s
        :param Type: Parameter that can be given that is written in the returned dictionary as additional information

        :return: List of dictionaries of the occurances with start time, end time and type
        """

        ops = {">":operator.gt, "<":operator.lt, ">=":operator.ge, "<=":operator.le, "==":operator.eq, "!=":operator.ne}
        if hasattr(self, quantityToCalc):
            self.currQuantity       = getattr(self, quantityToCalc)
        else:
            self.currQuantity       = np.array(self.currErgFile.readAllSignalValues(quantityToCalc))
        timeIdx                     = np.argmax(ops['>='](self.TimeVec, startTime))

        occurancesList = []
        occuranceActive = False
        durationStartTime = 0
        durationEndTime = 0
        for currIdx,currTime in enumerate(self.TimeVec):
            if currIdx < timeIdx: # only log occurances after start time
                continue
            if not occuranceActive: # get start time
                if ops[condition](self.currQuantity[currIdx], currValue):
                    durationStartTime = currTime
                    occuranceActive   = True
            else: # get end time
                if not ops[condition](self.currQuantity[currIdx], currValue):
                    durationEndTime   = currTime
                    occuranceActive   = False
                    occurancesList.append({'startTime': durationStartTime, 'endTime': durationEndTime, 'type': Type})

        # if occurance does not end
        if durationStartTime > durationEndTime:
            occurancesList.append({'startTime': durationStartTime, 'endTime': None, 'type': Type})

        return occurancesList

    def calcValueOccurredMultipleCond(self, quantityToCalc, currValue1, condition1, currValue2, condition2):
        """
        Check if the two given conditions are true.
        
        :param quantityToCalc: Name of the quantity (str)
        :param currValue1: First value that shall be checked
        :param condition1: Condition that shall be checked for first value: ">", "<", ">=", "<=", "==", "!="
        :param currValue2: Second value that shall be checked
        :param condition2: Second condition that shall be checked: ">", "<", ">=", "<=", "==", "!="

        :return: True if both conditions are true at any time, false if not.
        """

        ops = {">":operator.gt, "<":operator.lt, ">=":operator.ge, "<=":operator.le, "==":operator.eq, "!=":operator.ne}
        if hasattr(self, quantityToCalc):
            self.currQuantity       = getattr(self, quantityToCalc)
        else:
            self.currQuantity       = np.array(self.currErgFile.readAllSignalValues(quantityToCalc))
        #self.currQuantityIdx        = np.argmax(ops[condition](self.currQuantity, currValue))
        self.currQuantityIdx        = [idx for idx,X in enumerate(list(self.currQuantity)) if (ops[condition1](X, currValue1) and ops[condition2](X, currValue2) and X != 0)]
        if self.currQuantityIdx != [] and self.currQuantityIdx != [0]:
            return True
        return False

    def calcTimeAtValueMultipleCond(self, quantityToCalc, currValue1, condition1, currValue2, condition2):
        """
        Check the time when both given conditions are true the first time.
        
        :param quantityToCalc: Name of the quantity (str)
        :param currValue1: First value that shall be checked
        :param condition1: Condition that shall be checked for first value: ">", "<", ">=", "<=", "==", "!="
        :param currValue2: Second value that shall be checked
        :param condition2: Second condition that shall be checked: ">", "<", ">=", "<=", "==", "!="

        :return: Time when both conditions are true the first time
        """

        ops = {">":operator.gt, "<":operator.lt, ">=":operator.ge, "<=":operator.le, "==":operator.eq, "!=":operator.ne}
        if hasattr(self, quantityToCalc):
            self.currQuantity       = getattr(self, quantityToCalc)
        else:
            self.currQuantity       = np.array(self.currErgFile.readAllSignalValues(quantityToCalc))
        self.currQuantityIdx        = [idx for idx,X in enumerate(list(self.currQuantity)) if (ops[condition1](X, currValue1) and ops[condition2](X, currValue2) and X != 0)][0]
        self.currTimeValue          = self.TimeVec[self.currQuantityIdx]
        return self.currTimeValue


    def calcPercCabinHitAtCollision(self, egoLength, egoWidth, targetLength, targetWidth, targetName, sections, percCabin):
        if not hasattr(self, 'collisionTime'):
            self.calcCollisionTime(egoLength, egoWidth, targetLength, targetWidth, targetName)

        # Calculate Target Rect call:
        targetRectInSections = self.calcTargetRects(targetLength, targetWidth, targetName, sections)
        
        calcCollisionOccuredAtSectionsList = []
        targetRectAtCollision              = []
        egoRectAtCollision = ({'FL':[np.array(self.egoRect['FL'][0][self.currCollisionIdx]), np.array(self.egoRect['FL'][1][self.currCollisionIdx])],
                               'FR':[np.array(self.egoRect['FR'][0][self.currCollisionIdx]), np.array(self.egoRect['FR'][1][self.currCollisionIdx])],
                               'RL':[np.array(self.egoRect['RL'][0][self.currCollisionIdx]), np.array(self.egoRect['RL'][1][self.currCollisionIdx])],
                               'RR':[np.array(self.egoRect['RR'][0][self.currCollisionIdx]), np.array(self.egoRect['RR'][1][self.currCollisionIdx])]})
        calcCollisionOccuredAtSections     = 0
        for currRectIdx in range(sections):
            currtargetRectAtCollision = ({'FL':[np.array(targetRectInSections[currRectIdx]['FL'][0][self.currCollisionIdx]), np.array(targetRectInSections[currRectIdx]['FL'][1][self.currCollisionIdx])],
                                          'FR':[np.array(targetRectInSections[currRectIdx]['FR'][0][self.currCollisionIdx]), np.array(targetRectInSections[currRectIdx]['FR'][1][self.currCollisionIdx])],
                                          'RL':[np.array(targetRectInSections[currRectIdx]['RL'][0][self.currCollisionIdx]), np.array(targetRectInSections[currRectIdx]['RL'][1][self.currCollisionIdx])],
                                          'RR':[np.array(targetRectInSections[currRectIdx]['RR'][0][self.currCollisionIdx]), np.array(targetRectInSections[currRectIdx]['RR'][1][self.currCollisionIdx])]})
            targetRectAtCollision.append(currtargetRectAtCollision)
            calcCollisionOccuredAtSectionsList.append( self.currSectionCalcCollisionOccuredPy(egoRectAtCollision, currtargetRectAtCollision, currIdx=self.currCollisionIdx) )
            if currRectIdx >= percCabin[0]*(sections-1) and currRectIdx <= percCabin[1]*(sections-1):
                calcCollisionOccuredAtSections += calcCollisionOccuredAtSectionsList[-1]
        self.percCabinHit = (calcCollisionOccuredAtSections/((percCabin[1]*(sections-1)-percCabin[0]*(sections-1)+1)))*100
        ### WORKAROUND
        ## Calc percCabinHit for modified angle -> otherwise non 90 degree cases usually lead to 0 percCabinHit
        #ego angle + 90 deg for perfect hit condition
        ## targetThetaMod = self.egoTheta[self.currCollisionIdx] + math.pi/2 #+ or - ??
        targetThetaMod = self.targetTheta[self.currCollisionIdx] ###DELETE MAYBE
        usedSection = 0
        for idx, val in enumerate(calcCollisionOccuredAtSectionsList):
            if idx > 0:
                if val and targetRectAtCollision[idx]['FL'][0]<targetRectAtCollision[idx-1]['FL'][0]:
                    usedSection = idx
        #usedSection = np.argmax(calcCollisionOccuredAtSectionsList)
        #difference from actual angle to ego angle + 90 deg
        angleModified  = self.targetTheta[self.currCollisionIdx] - targetThetaMod  #+ or - ??
        hitPointTargetRect = targetRectInSections[np.argmax(calcCollisionOccuredAtSectionsList)]
        if abs(angleModified) > -1e3: #1e-3
            targetRectMod = []
            calcCollisionOccuredAtSectionsListMod = []
            calcCollisionOccuredAtSectionsMod     = 0
            for currElement in range(sections):  
                #new Rectangles
                targetRectMod =     ({"FL":(self.targettx[self.currCollisionIdx] - targetWidth/2.0*np.sin(targetThetaMod) + float(usedSection+1)/sections * targetLength*np.cos(targetThetaMod),
                                            self.targetty[self.currCollisionIdx] + targetWidth/2.0*np.cos(targetThetaMod) + float(currElement+1)/sections * targetLength*np.sin(targetThetaMod)),
                                      "FR":(self.targettx[self.currCollisionIdx] + targetWidth/2.0*np.sin(targetThetaMod) + float(usedSection+1)/sections * targetLength*np.cos(targetThetaMod),
                                            self.targetty[self.currCollisionIdx] - targetWidth/2.0*np.cos(targetThetaMod) + float(currElement+1)/sections * targetLength*np.sin(targetThetaMod)),
                                      "RR":(self.targettx[self.currCollisionIdx] + targetWidth/2.0*np.sin(targetThetaMod) + float(usedSection  )/sections * targetLength*np.cos(targetThetaMod),
                                            self.targetty[self.currCollisionIdx] - targetWidth/2.0*np.cos(targetThetaMod) + float(currElement  )/sections * targetLength*np.sin(targetThetaMod)),
                                      "RL":(self.targettx[self.currCollisionIdx] - targetWidth/2.0*np.sin(targetThetaMod) + float(usedSection  )/sections * targetLength*np.cos(targetThetaMod),
                                            self.targetty[self.currCollisionIdx] + targetWidth/2.0*np.cos(targetThetaMod) + float(currElement  )/sections * targetLength*np.sin(targetThetaMod))})
                calcCollisionOccuredAtSectionsListMod.append( self.currSectionCalcCollisionOccuredPy(egoRectAtCollision, targetRectMod, currIdx=self.currCollisionIdx) )
                if currElement >= math.floor(percCabin[0]*(sections)) and currElement < math.ceil(percCabin[1]*(sections)):
                    calcCollisionOccuredAtSectionsMod += calcCollisionOccuredAtSectionsListMod[-1]
            self.percCabinHitMod = np.array((calcCollisionOccuredAtSectionsMod/(math.ceil(percCabin[1]*(sections)-math.floor(percCabin[0]*(sections)))))*100)
        else:
            self.percCabinHitMod = np.array(self.percCabinHit)
        return self.percCabinHit, self.percCabinHitMod


    def currSectionCalcCollisionOccuredPy(self, rectA, rectB, currIdx=None):
        def sortProjPoints(VecA, VecB, tmp):
            if tmp < VecA:
                VecA = tmp
            if tmp > VecB:
                VecB = tmp
            return VecA, VecB

        overlap = 1
        iAxes   = 2
        # Build an iterable representation of all rect points
        EMPPointIterable = []
        Rect1            = []
        Rect2            = []
        Rect1.append( rectA["FL"] )
        Rect1.append( rectA["FR"] )
        Rect1.append( rectA["RL"] )
        Rect1.append( rectA["RR"] )
        Rect2.append( rectB["FL"] )
        Rect2.append( rectB["FR"] )
        Rect2.append( rectB["RL"] )
        Rect2.append( rectB["RR"] )
        EMPPointIterable.append( Rect1 )
        EMPPointIterable.append( Rect2 )
        vSortProjPoints = np.vectorize(sortProjPoints)
        
        # Angle of rects
        angleOfRect = []
        #angleOfRect.append( np.arctan2((rectA["FL"][1] - rectA["RL"][1]), (rectA["FL"][0] - rectA["RL"][0])) )
        #angleOfRect.append( np.zeros(shape=(len(self.egoRect['RL'][0]))) )
        #if currIdx:
        #    angleOfRect.append( np.arctan2( self.egovy[currIdx], self.egovx[currIdx] ) )
        #else:
        #angleOfRect.append( np.arctan2( self.egovy, self.egovx ) )
        angleOfRect.append( np.arctan2( (rectA["FL"][1] - rectA["RL"][1]), (rectA["FL"][0] - rectA["RL"][0]) ) )
        angleOfRect.append( np.arctan2( (rectB["FL"][1] - rectB["RL"][1]), (rectB["FL"][0] - rectB["RL"][0]) ) )
        collisionList = []
        #if currIdx:
        #    collision     = np.ones(shape=1, dtype=bool)
        #else:
        collision     = np.ones(shape=(rectA['RL'][0].size), dtype=bool)
        for currAngleOfRect in angleOfRect:
            tmpAngleCos = np.cos( -currAngleOfRect )
            tmpAngleSin = np.sin( -currAngleOfRect )

            minOnAxisList = []
            maxOnAxisList = []
            # Iterate through Rectangles
            for currRect in EMPPointIterable:
                firstPoint = True
                minOnAxis = []
                maxOnAxis = []
                # Iterative through points of rects 
                for currPoint in currRect:
                    tmp = []
                    tmp.append( (tmpAngleCos * currPoint[0]) - (tmpAngleSin * currPoint[1]) )
                    tmp.append( (tmpAngleSin * currPoint[0]) + (tmpAngleCos * currPoint[1]) )

                    for c1 in range(iAxes):
                        if firstPoint:
                            minOnAxis.append( tmp[c1] )
                            maxOnAxis.append( tmp[c1] )
                        else:
                            minOnAxis[c1], maxOnAxis[c1] = vSortProjPoints(minOnAxis[c1], maxOnAxis[c1], tmp[c1])
                    firstPoint = False

                minOnAxisList.append(minOnAxis)
                maxOnAxisList.append(maxOnAxis)

            collisionX =     (((minOnAxisList[1][0] <= maxOnAxisList[0][0]) * (minOnAxisList[1][0] >= minOnAxisList[0][0])) #Check: Min Point Rect2 is between Rect1 Points
                            + ((maxOnAxisList[1][0] <= maxOnAxisList[0][0]) * (maxOnAxisList[1][0] >= minOnAxisList[0][0])) #Check: Max Point Rect2 is between Rect1 Points
                            + ((maxOnAxisList[1][0] >= maxOnAxisList[0][0]) * (minOnAxisList[1][0] <= minOnAxisList[0][0]))) #Check: Rect1 is surrounded by Rect2
            collisionY =     (((minOnAxisList[1][1] <= maxOnAxisList[0][1]) * (minOnAxisList[1][1] >= minOnAxisList[0][1])) #Check: Min Point Rect2 is between Rect1 Points
                            + ((maxOnAxisList[1][1] <= maxOnAxisList[0][1]) * (maxOnAxisList[1][1] >= minOnAxisList[0][1])) #Check: Max Point Rect2 is between Rect1 Points
                            + ((maxOnAxisList[1][1] >= maxOnAxisList[0][1]) * (minOnAxisList[1][1] <= minOnAxisList[0][1]))) #Check: Rect1 is surrounded by Rect2
            collisionList.append( collisionX * collisionY )
            
            #if currIdx:
            #    testForOverlap = collisionList[-1]
            #else:
            testForOverlap = np.amax(collisionList[-1])
            if testForOverlap:
                #Overlap exists!
                pass
                #EMPTY on purpose
            else:
                overlap = 0
                break
        for element in collisionList:
            collision *= element
        return collision

    def calcQuantityUniqueValue(self, quantityName):
        if hasattr(self, quantityName):
            self.currQuantity = getattr(self, quantityName)
        else:
            self.currQuantity = np.array(self.currErgFile.readAllSignalValues(quantityName))
        unique_val = np.unique(self.currQuantity)
        return np.sort(unique_val)[::-1]

    def calcFirstLastIndexofOccurence(self, quantityToCalc, currValue, condition=None):
        ops = {">": operator.gt, "<": operator.lt, ">=": operator.ge, "<=": operator.le, "==": approx_eq,
               "!=": operator.ne}
        currValue = round(currValue, 3)
        if hasattr(self, quantityToCalc):
            self.currQuantity = getattr(self, quantityToCalc)
        else:
            self.currQuantity = np.round(np.array(self.currErgFile.readAllSignalValues(quantityToCalc)), 3)
        if not condition:
            try:
                startIndex = np.where(self.currQuantity == currValue)[0][0]
                lastIndex = np.where(self.currQuantity == currValue)[0][-1]
            except:
                startIndex = None
                lastIndex = None
        else:
            startIndex = None
            lastIndex = None
            try:
                startIndex = np.where(ops[condition](self.currQuantity, currValue))[0][0]
                lastIndex = np.where(ops[condition](self.currQuantity, currValue))[0][-1]
            except:
                startIndex = None
                lastIndex = None

        return startIndex, lastIndex

    def calcIndexofAllOccurences(self, quantityToCalc, currValue):
        if hasattr(self, quantityToCalc):
            self.currQuantity = getattr(self, quantityToCalc)
        else:
            self.currQuantity = np.array(self.currErgFile.readAllSignalValues(quantityToCalc))
        try:
            count = 0
            occurences = OrderedDict()
            array_indices = np.where(self.currQuantity == currValue)
            if array_indices:
                index = np.array(array_indices).tolist()[0]
                nums = sorted(set(index))
                gaps = [[s, e] for s, e in zip(nums, nums[1:]) if s + 1 < e]
                edges = iter(nums[:1] + sum(gaps, []) + nums[-1:])
                for i in list(zip(edges, edges)):
                    occurences["Occurence" + str(count)] = i
                    count += 1
        except Exception as e:
            occurences = OrderedDict()

        return occurences

