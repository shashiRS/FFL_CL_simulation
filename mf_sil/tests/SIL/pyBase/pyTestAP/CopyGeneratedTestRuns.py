import os
import sys
import shutil , stat
from pyBase_path_cfg import *

TestRunFolder_list = ["\\01_Perpendicular_Left", "\\02_Perpendicular_Right", "\\03_Parallel_Left", "\\04_Parallel_Right", "\\05_Angled_Left", "\\06_Angled_Right"]


def checkIfPathNotExist(currentPath):
    if not os.path.isdir(currentPath):
        return 1
    else:
        return 0

def checkIfFileExist(currentPath, file):
    if not os.path.isfile(currentPath + file):
        return 0
    else:
        return 1

def saveTestsNameInList(currentPath):
    testRun = []
    for path, subdirs, files in os.walk(currentPath):
        for name in files:
            testRun.append(os.path.join(path, name).replace(currentPath,""))
    return testRun

def saveTestRunInString(currentPath, testRunName):
    try:
        File = open(currentPath + testRunName, 'r')
        FileData = File.read()
        File.close()
        #return in a string all testRun text without the Date(line3)
        return FileData[:FileData.index("FileCreator")] + FileData[FileData.index("Description"):]
    except:
        print("Cannot open testrun: " + testRunName)

def copyTestRun(origPath, destPath,testRun):
    try:  
        shutil.copyfile(origPath + testRun, destPath + testRun)
    except: 
        print("Cannot copy testrun: " + testRun)

def __main__():
    if(checkIfPathNotExist(APGENERATED_TEMP_PATH)):
        print("Temp path with generated testruns does not exist: {}".format(APGENERATED_TEMP_PATH))
        sys.exit()
    if(checkIfPathNotExist(APGENERATED_PATH)):
        print("Final path for generated testruns does not exist: {}".format(APGENERATED_TEMP_PATH))
        sys.exit()
    
    #remove read only flag
    os.chmod(APGENERATED_PATH, stat.S_IWRITE )
    
    #check if the scenarios type directories exist, if not -> create it
    for folderName in TestRunFolder_list:
        if(checkIfPathNotExist(APGENERATED_PATH + folderName)):
            os.mkdir(APGENERATED_PATH + folderName)

    tempTestRuns_list = []
    finalTestRuns_list = []

    #save in a list the testruns from the temp path
    tempTestRuns_list = saveTestsNameInList(APGENERATED_TEMP_PATH)
    #save in a list the testruns from final path
    finalTestRuns_list = saveTestsNameInList(APGENERATED_PATH)
    
    #delete TestRuns from final path that were not regenerated this time
    for fTest in finalTestRuns_list:
        if not checkIfFileExist(APGENERATED_TEMP_PATH, fTest):
            try:
                os.remove(APGENERATED_PATH + fTest)
            except:
                print("Cannot remove file {} from {}".format(fTest, APGENERATED_PATH))

    #copy new TestRuns and modified TestRuns to final path
    for fTest in tempTestRuns_list:
        copy = False
        #verify if the TestRun exists in final path
        if checkIfFileExist(APGENERATED_PATH, fTest):
            #save in a string all testRun text without the Date (line 3)
            newStringTestRun = saveTestRunInString(APGENERATED_TEMP_PATH, fTest)
            oldStringTestRun = saveTestRunInString(APGENERATED_PATH, fTest)
            #check if the data from TestRun are the same (without line 3 - Data)
            if oldStringTestRun == newStringTestRun:
                copy = False
            else:
                copy = True 
        else:
            copy = True
        if copy == True:
            copyTestRun(APGENERATED_TEMP_PATH, APGENERATED_PATH, fTest)

__main__()        