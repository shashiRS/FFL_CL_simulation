#********************************************************************
# System Imports
#********************************************************************
import sys
import imp
imp.reload(sys)
# sys.setdefaultencoding('utf8')
import cmTestRunSimple
import math


#====================================================================
# Tire Circumference Estimation Factories
#====================================================================
TCE_ROAD_LENGTH     = 10000 # m


#====================================================================
# Tire Circumference Estimation Factories
#====================================================================

def FactoryTestRunTCE(testRun, vEgo=15, rRadius=0, rGrad=0, rSlope=0, rCamber=0):
    # testRun.addReplacement("Vehicle = ", "{0}".format("VW_Golf_VII_14TSI.car")) # TODO: remove this after switch for VW Passat
    testRun.addReplacement("DrivMan.1.Info = Velocity = ", "{0} {1}".format(str(vEgo), " km/h"))
    testRun.addReplacement("DrivMan.1.LongDyn = Driver 1 0 ", "{0}".format(str(vEgo)))
    
    if rRadius == 0: # if it's a straight road
        testRun.addReplacement("Road.Link.0.Seg.0.Type = ", "{0}".format("Straight"))
        testRun.addReplacement("Road.Link.0.Seg.0.Param = ", "{0} {1}".format(str(TCE_ROAD_LENGTH), "0 0 0 0 0 0 0"))
    else: # it's a curved road
        arcLength = abs((TCE_ROAD_LENGTH * 180) / (math.pi * rRadius)) # Arc circle legnth L = pi * R * alfa / 180
        if rRadius > 0: # if it's a left turn road
            testRun.addReplacement("Road.Link.0.Seg.0.Type = ", "{0}".format("TurnLeft"))
            testRun.addReplacement("Road.Link.0.Seg.0.Param = ", "{0} {1} {2}".format(str(abs(rRadius)), str(arcLength), "0 0 0 0 0 0"))
        else: # if it's a right turn road
            testRun.addReplacement("Road.Link.0.Seg.0.Type = ", "{0}".format("TurnRight"))
            testRun.addReplacement("Road.Link.0.Seg.0.Param = ", "{0} {1} {2}".format(str(abs(rRadius)), str(arcLength), "0 0 0 0 0 0"))
            
    # longitudinal slope (rGrad [%])
    # longitudinal slope value = rGrad * TCE_ROAD_LENGTH / 100
    # longitudinal slope gradient = rGrad / 100
    testRun.addReplacement("	0.00 ", "{0} {1}".format("0", str(rGrad  / 100.0))) # start point
    testRun.addReplacement("	10000.00 ", "{0} {1}".format(str(rGrad * TCE_ROAD_LENGTH / 100), str(rGrad / 100.0))) # end point
    
    # lateral slope (rSlope [%])
    # lateral slope value = rGrad / 100
    testRun.addReplacement("	0.000 ", "{0} {1}".format(str(rSlope / 100.0), "-999")) # start point
    testRun.addReplacement("	10000.000 ", "{0} {1}".format(str(rSlope / 100.0), "-999")) # end point
    
    # camber (rCamber [m/m])
    testRun.addReplacement("	0.0000 ", "{0} {1}".format(str(rCamber), "-999")) # start point
    testRun.addReplacement("	10000.0000 ", "{0} {1}".format(str(rCamber), "-999")) # end point
    
    
def FactoryTestRunTCECity(testRun, vEgo=30):
    testRun.addReplacement("DrivMan.1.Info = Velocity = ", "{0} {1}".format(str(vEgo), " km/h"))
    testRun.addReplacement("DrivMan.1.LongDyn = Driver 1 0 ", "{0}".format(str(vEgo)))