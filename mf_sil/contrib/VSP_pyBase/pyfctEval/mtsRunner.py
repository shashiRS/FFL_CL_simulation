#====================================================================
# System Imports
#====================================================================
import os
import sys
import time
import logging
import argparse
import subprocess
import xml.dom.minidom
import shutil

#====================================================================
# Local Parameters
#====================================================================
EXTENSION_REC_FILE   = ".rec"
EXTENSION_RREC_FILE  = ".rrec"
EXTENSION_LOCK_FILE  = ".lock"
EXTENSION_ERROR_FILE = ".error"
EXTENSION_BSIG_FILE  = ".bsig"
EXTENSION_LABEL_FILE = ".lbl"
MTSProcessName     = "measapp.exe"

maximumWaitSeconds        = 15 * 60 * 60
maximumWaitForBsigSeconds = 3 * 60
waitForMTSToStartSeconds  = 2 * 20
sleepWaitSeconds          = 5

def createErrorFile(file, args, msg):
    errorFilePath = os.path.join(args.outputDir, file['name'] + EXTENSION_ERROR_FILE)
    with open(errorFilePath, 'a') as errorFile:
        errorFile.write(msg)

def allFilesInFolder(folder, extensions):
    fileList = []
    for root, dirs, files in os.walk(folder):
        for file in files:
            if file.endswith(tuple(extensions)):
                fileNameWithoutExtension = os.path.splitext(os.path.basename(file))[0]
                fileFullPath = os.path.join(root, file)
                fileRelPath = os.path.relpath(root, folder)
                fileList.append({"name":fileNameWithoutExtension, "fullPath":fileFullPath})
    return fileList

def getRecFilesFromBpl(bpl_file):
    dom_impl = xml.dom.minidom.getDOMImplementation()
    batch_dom = xml.dom.minidom.parse( bpl_file )
    batch_entries = batch_dom.getElementsByTagName( "BatchEntry" )
    
    
    fileList = [];
    for b in batch_entries:
        fileFullPath             = b._attrs['fileName'].nodeValue
        if os.path.exists(fileFullPath):  
            logging.info('append from bpl: ' + fileFullPath)
            fileNameWithoutExtension = os.path.splitext(os.path.basename(fileFullPath))[0]
            fileList.append({"name":fileNameWithoutExtension, "fullPath":fileFullPath})
        else:
            logging.warning('File not found: ' + fileFullPath)

    return fileList

def processRecordingInMTS(file, args):
    # Create *.lock File
    lockFilePath = os.path.join(args.outputDir, file['name'] + EXTENSION_LOCK_FILE)

    with open(lockFilePath, 'a') as lockFile:
        lockFile.write("Locked by {0}".format(os.getenv('COMPUTERNAME')))

    # Start MTS with all the Parameter Glory
    measappExePath = os.path.join(args.mtsDir, "measapp.exe")
    bsigFilePath   = os.path.join(args.outputDir, file['name'] + EXTENSION_BSIG_FILE)
    
    if not os.path.isfile(measappExePath):
        logging.error("Invalid mts path "  + measappExePath)
    mtsStartString = '{0} -lc"{1}" -lr"{2}" -show"minimized" -pal -eab -silent -norestart'.format(measappExePath, args.mtsCfgPath, file['fullPath'])
    logging.info("Starting MTS with: {0}".format(mtsStartString))
    mtsPID = subprocess.Popen(mtsStartString).pid

    # Wait maximum time for MTS Process until it shuts down
    startTime = int(time.time())
    logging.info("Wait {0} seconds for MTS to start up".format(waitForMTSToStartSeconds))
    time.sleep(waitForMTSToStartSeconds)
    while True:
        if str(mtsPID).encode() not in subprocess.Popen('tasklist', stdout=subprocess.PIPE).communicate()[0]:
            # MTS exited on its own, yaay
            # Delete *.lock file
            logging.info("Measapp.exe Shutdown detected. Checking Creation of Bsig file...")
            try:
                os.remove(lockFilePath)
            except OSError as exc:
                logging.error(exc)
            # Bsig file should have been created. If not, create error file
            bsigFilePath = os.path.join(args.outputDir, file['name'] + EXTENSION_BSIG_FILE)
            if os.path.exists(bsigFilePath):
                logging.info("Bsig File found.")
            else:
                logging.info("Bsig File not found. Creating error file.")
                createErrorFile(file, args, "Bsig File was not created")
            break 
        if (int(time.time()) > (startTime + maximumWaitSeconds)):
            logging.info("Maximum wait time exceeded. Killing MTS, removing lock, creating error file")
            # MTS exceeded maximum Wait time
            # Kill MTS
            os.system('taskkill /f /PID {}'.format(mtsPID))
            # Remove *.lock file
            try:
                os.remove(lockFilePath)
            except OSError as exc:
                logging.error(exc)
            # Create *.error file
            createErrorFile(file, args, "Maximum wait time exceeded")
            break
        if (int(time.time()) > (startTime + maximumWaitForBsigSeconds)) and not os.path.exists(bsigFilePath.replace('\\', '\\\\').replace('\t', '\\t').replace('\n', '\\n')):
            logging.info("Found no bsig after some time. Killing MTS, removing lock, creating error file")
            # Kill MTS
            os.system('taskkill /f /PID {}'.format(mtsPID))
            # Remove *.lock file
            try:
                os.remove(lockFilePath)
            except OSError as exc: 
                logging.error(exc)
            # Create *.error file
            createErrorFile(file, args, "bsig not created after some minutes")
            break        
        
        time.sleep(sleepWaitSeconds)

def SetBSIGOutputDirInMTSConfig(path_mtsCfg, path_bsigOut):
    new_mtsCfg = os.path.join( path_bsigOut, os.path.basename(path_mtsCfg) )
    if not os.path.isdir(path_bsigOut):
        os.makedirs(path_bsigOut)
        
    with open(path_mtsCfg, mode='r') as oldFile:
        with open(new_mtsCfg, mode='w') as newFile:
            for line in oldFile:
                if 'Output Directory=' in line:
                    newFile.write( 'Output Directory="' + path_bsigOut.replace('\\', '\\\\') + '\\\\"' + '\n')
                else:
                    newFile.write(line)
                
    return new_mtsCfg

def singleRun(args):
    if os.path.isdir(args.recFileInput):    
        # Retrieve all Rec File Paths from Rec File Directory
        recFiles = allFilesInFolder(args.recFileInput, [EXTENSION_REC_FILE, EXTENSION_RREC_FILE])
    elif os.path.isfile(args.recFileInput):
        # It should be a bpl file
        if os.path.basename(args.recFileInput).split('.')[1] != 'bpl':
            logging.error("Invalid rec file input (need *.bpl file)" + args.recFileInput)
            sys.exit(0)
        recFiles = getRecFilesFromBpl(args.recFileInput)
    else:
        logging.error("Invalid rec file input " + args.recFileInput)
        sys.exit(0)

    # Find Rec file that desperately needs processing
    for currRecFile in recFiles:
        lockedFiles    = allFilesInFolder(args.outputDir, EXTENSION_LOCK_FILE)
        errorFiles     = allFilesInFolder(args.outputDir, EXTENSION_ERROR_FILE)
        bsigFiles      = allFilesInFolder(args.outputDir, EXTENSION_BSIG_FILE)

        # Copy label file
        labelFilePath = os.path.join(currRecFile['fullPath'].split(EXTENSION_RREC_FILE)[0] + EXTENSION_LABEL_FILE)
        logging.info("Looking for Label File {0}".format(labelFilePath))
        if os.path.isfile(labelFilePath):
            logging.info("Label File found")
            labelFileTargetPath = os.path.join(args.outputDir, currRecFile['name'] + EXTENSION_LABEL_FILE)
            shutil.copyfile(labelFilePath, labelFileTargetPath)
            logging.info("Label File copied to bsig location {0}".format(labelFileTargetPath))

        # Check output Directory for any file with the same name ( *.lock, *.error, *.bsig )
        logging.info("Checking Recording: {0}".format(currRecFile['fullPath']))
        if any(currLockedFile['name'] == currRecFile['name'] for currLockedFile in lockedFiles): 
            logging.info("Skipping current Recording because of Lock File found")
            continue
        if any(currErrorFile['name'] == currRecFile['name'] for currErrorFile in errorFiles): 
            logging.info("Skipping current Recording because of Error File found")
            continue
        if any(currBsigFile['name'] == currRecFile['name'] for currBsigFile in bsigFiles): 
            logging.info("Skipping current Recording because of Bsig File found")
            continue

        # Found Recording to be processed
        logging.info("Recording is suitable for processing")
        processRecordingInMTS(currRecFile, args)

def run(args):
    if (args.infiniteRun == True):
        while True:
            singleRun(args)
            time.sleep(5*60)
    else:
        singleRun(args)

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    # Argument Parser
    parser = argparse.ArgumentParser()
    parser.add_argument("recFileInput", help="Directory of Rec Files to be processed or bpl file")
    parser.add_argument("outputDir",  help="Directory where generated bsig files will be transferred to")
    parser.add_argument("mtsDir",     help="MTS Directory")
    parser.add_argument("mtsCfgPath", help="Path to the MTS Config")
    parser.add_argument("-i", "--infiniteRun", action="store_true", default=False)
    
    args = parser.parse_args()

    logging.info("Remote Runner started")
    logging.info("Recording Directory: {0}".format(args.recFileInput))
    logging.info("Output Directory: {0}".format(args.outputDir))
    logging.info("MTS Directory: {0}".format(args.mtsDir))
    logging.info("MTS Config Path: {0}".format(args.mtsCfgPath))

    # Set bsig Output Directory in MTS Config
    args.mtsCfgPath = SetBSIGOutputDirInMTSConfig(args.mtsCfgPath, args.outputDir)

    # Main Loop
    logging.info("Starting Main Loop")
    run(args)
    logging.info("Finished Main Loop")