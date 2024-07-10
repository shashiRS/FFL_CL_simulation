#====================================================================
# SYSTEM IMPORTS
#====================================================================
import os
import sys

#====================================================================
# OWN IMPORTS
#====================================================================
sys.path.append( os.path.join(os.path.dirname(__file__), '..'))
import fct_bsig_runner

#====================================================================
# PARAMETER
#====================================================================
EXAMPLE_FCT_DLL_PATH        = os.path.abspath(os.path.join(os.path.dirname(__file__), "fct_sim_ext.dll")) 
EXAMPLE_FCT_DATA_TYPES_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "fct_data_types.py")) 
EXAMPLE_BSIG_PATH           = os.path.abspath(os.path.join(os.path.dirname(__file__), "example.bsig"))
EXAMPLE_DEVICE              = "ARS"

def TestFctBsigRunnerARS():
    runnerDevice = fct_bsig_runner.cFctBsigRunnerDevice(device=EXAMPLE_DEVICE)
    runnerSimVfb = fct_bsig_runner.cFctBsigRunnerSimVFB(device=EXAMPLE_DEVICE)
    runnerReSim  = fct_bsig_runner.cFctBsigRunnerResim(fctSimExtDllPath=EXAMPLE_FCT_DLL_PATH, fctDataTypesPath=EXAMPLE_FCT_DATA_TYPES_PATH, device=EXAMPLE_DEVICE)
    
    runnerDevice.AddRequestedOutputURL("pFCTVehOutput->pHEADOutputCustom->sWarnings.sAcuteDynamicWarning.eSignal")
    runnerSimVfb.AddRequestedOutputURL("pFCTVehOutput->pHEADOutputCustom->sWarnings.sAcuteDynamicWarning.eSignal")
    runnerReSim.AddRequestedOutputURL("pFCTVehOutput->pHEADOutputCustom->sWarnings.sAcuteDynamicWarning.eSignal")
    
    outputDevice = runnerDevice.Run(EXAMPLE_BSIG_PATH)
    outputSimVfb = runnerSimVfb.Run(EXAMPLE_BSIG_PATH)
    outputReSim  = runnerReSim.Run(EXAMPLE_BSIG_PATH)
    
    print("Runner Device Output at cycle 3888 is: {0} (should be 4)".format(outputDevice["pFCTVehOutput->pHEADOutputCustom->sWarnings.sAcuteDynamicWarning.eSignal"][3888]))
    print("Runner SimVfb Output at cycle 3888 is: {0} (should be 4)".format(outputSimVfb["pFCTVehOutput->pHEADOutputCustom->sWarnings.sAcuteDynamicWarning.eSignal"][3888]))
    print("Runner ReSim  Output at cycle 3888 is: {0} (should be 4)".format(outputReSim["pFCTVehOutput->pHEADOutputCustom->sWarnings.sAcuteDynamicWarning.eSignal"][3888]))

if __name__ == "__main__":
    TestFctBsigRunnerARS()
