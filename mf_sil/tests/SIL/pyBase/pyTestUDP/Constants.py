#====================================================================
# Constants
#====================================================================
# Requirement

# CarMaker
EGO_LENGTH                  = 4.787 # m
EGO_WIDTH                   = 1.832 # m
EGO_OVERHANG                = 1.096 # m
EGO_WHEELBASE               = 2.786 # m
VHCL_SROAD_STARTOFF         = 3.882 # m - Default Vhcl.sRoad value for 0m initial offset (offset of the front wheels)

# CarMaker scenario
P_minDrivingDist            = 7 # m - minimum distance traveled before colliding with a static pole
P_scanningTime              = 8.5 # s - time traveled before colliding with the obstacle
P_collisionPointX           = 60 # m - longitudinal offset of collision point between ego vehicle and obstacle
P_stopAndGoDummyStart       = 40 # m - longitudinal offset of the dummy car for the stop and go scenario
P_limiterWidth              = 0.15 # m - parking limiter width (e.g. line, curb, wall)
P_parkSlotMiddle            = 40 # m - longitudinal offset for the parking slot middle
CM_CIRC_OBST_NUM_SEG        = 20   # number of segments considered for a circular obstacle contour (set as divisor of 180°)