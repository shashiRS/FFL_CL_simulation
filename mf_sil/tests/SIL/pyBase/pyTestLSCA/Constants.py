#====================================================================
# Constants
#====================================================================
# Requirement

# CarMaker
EGO_LENGTH                  = 4.787 # m
EGO_WIDTH                   = 1.832 # m
EGO_WIDTH_MIRRORS           = 2.083 # m
EGO_OVERHANG                = 1.096 # m
EGO_WHEELBASE               = 2.786 # m
VHCL_SROAD_STARTOFF         = 3.882 # m - Default Vhcl.sRoad value for 0m initial offset (offset of the front wheels)
DYN_OBJ_EXTENSION           = 0.25 # m - SI extenstion for dynamic objects (CM name must differ from 'Txx'); considered on each side

# CarMaker scenario
P_minDrivingDist            = 7 # m - minimum distance traveled before colliding with a static pole
P_drivingTime               = 4.6 # s - time traveled before colliding with a static pole