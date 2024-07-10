# ============================================================================ #
# Imports
# ============================================================================ #
import os
import sys
import logging

sys.path.append( os.path.join(os.path.dirname(__file__), '..') )
import cm_eval_erg_parser

# ============================================================================ #
# Parameters:
# ============================================================================ #
erg_file_str = os.path.join( os.path.dirname(__file__), 'ergFiles' , 'Examples_DriverAssistanceSystems_ACCandPreCrash_TrafficJam_withCrash_101345.erg' )

# ============================================================================ #
# Preparation:
# ============================================================================ #
logging.basicConfig(level=logging.INFO)

# load erg file
erg_obj = cm_eval_erg_parser.cErgFile( erg_file_str )

# ============================================================================ #
# Tests:
# ============================================================================ #

# Test 1:
# -------
logging.info('Start with Test: \'Reading the whole file!\'')

reading = True
while reading:
    dataPackage = erg_obj.readNextTimeStep()
    
    if dataPackage == '':
        reading = False
    else:
        #print dataPackage
        #print '{0:5>}: {1:3f}'.format('Time', dataPackage['Time'])
        pass

logging.info('Test Succeeded!')


# Test 2:
# -------
logging.info('Starting Test: \'jump to time 40sec: 20ms steps-> 40/0.02=2000\'')
time_value = 0.0
if erg_obj.jumpToRecordById(2000):
    curr_id = erg_obj.getCurrentRecordId()
    dataPackage = erg_obj.readNextTimeStep()
    time_unit = erg_obj.getUnitOfValue('Time')
    time_value = dataPackage['Time']
    #print '{0:5}: {1:3f}'.format('Time', time_value)
if 40.00001 >= time_value >= 39.99999:
    logging.info('Test succeeded!')
else:
    logging.error('TEST Failed!')

# see which values are available:
# print "\n".join(erg_obj.getArrayOfAvailableValues())
