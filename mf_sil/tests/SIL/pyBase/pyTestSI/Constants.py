#====================================================================
# Constants
#====================================================================
# Requirement

# CarMaker
EGO_LENGTH                  = 4.787 # m
EGO_WIDTH                   = 1.832 # m
VHCL_SROAD_STARTOFF         = 3.882 # m - Default Vhcl.sRoad value for 0m initial offset (offset of the front wheels)

# Limiter heights
LIM_HEIGHT_LINE             = 0.001 # m
LIM_HEIGHT_CURBT            = 0.03 # m
LIM_HEIGHT_CURBTB           = 0.12 # m
LIM_HEIGHT_CURBNTD          = 0.15 # m
LIM_HEIGHT_CURBNT           = 0.25 # m
LIM_HEIGHT_WALL             = 3.0 # m
LIM_WIDTH                   = 0.15 # m

# CarMaker scenario
P_scenarioEnd_s             = 3 # s - time considered to stop the CarMaker scenario after the ego vehicle passed the parking spot
P_scenarioEnd_d             = 6 # m - distance considered to stop the CarMaker scenario after the ego vehicle passed the parking spot
P_collisionTime             = 10 # s - time considered from the start of the CarMaker scenario to the moment when the ego vehicle the traffic object collide
P_circNumSeg                = 20 # number of segments considered for a circular obstacle contour (set as divisor of 180°)
P_scanDist                  = 35 # m - distance traveled for the scanning phase (in SI scenarios, the vehicle will keep constant velocity after this distance)
P_curbLimReductionFact      = 0.5 # % - reduction factor of curb side limiter size from latHPark