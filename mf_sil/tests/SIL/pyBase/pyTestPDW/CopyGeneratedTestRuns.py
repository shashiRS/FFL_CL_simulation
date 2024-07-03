import os
import sys
import shutil , stat
from pyBase_path_cfg import *

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
    args = sys.argv
    currentPath = PDWGENERATED_PATH
    tempPath = PDWGENERATED_TEMP_PATH
    if len(args) == 2 and args[1] == "-factory": # copy factory generated scenarios
        currentPath = PDWFACTORYGENERATED_PATH
    if(checkIfPathNotExist(tempPath)):
        print("Temp path with generated testruns does not exist: {}".format(tempPath))
        sys.exit()
    if(checkIfPathNotExist(currentPath)):
        print("Final path for generated testruns does not exist: {}".format(tempPath))
        sys.exit()
    
    #remove read only flag
    os.chmod(currentPath, stat.S_IWRITE )

    tempTestRuns_list = []
    finalTestRuns_list = []

    #save in a list the testruns from the temp path
    tempTestRuns_list = saveTestsNameInList(tempPath)
    #save in a list the testruns from final path
    finalTestRuns_list = saveTestsNameInList(currentPath)
    
    #delete TestRuns from final path that were not regenerated this time
    for fTest in finalTestRuns_list:
        if not checkIfFileExist(tempPath, fTest):
            try:
                os.remove(currentPath + fTest)
            except:
                print("Cannot remove file {} from {}".format(fTest, currentPath))

    #copy new TestRuns and modified TestRuns to final path
    for fTest in tempTestRuns_list:
        copy = False
        #verify if the TestRun exists in final path
        if checkIfFileExist(currentPath, fTest):
            #save in a string all testRun text without the Date (line 3)
            newStringTestRun = saveTestRunInString(tempPath, fTest)
            oldStringTestRun = saveTestRunInString(currentPath, fTest)
            #check if the data from TestRun are the same (without line 3 - Data)
            if oldStringTestRun == newStringTestRun:
                copy = False
            else:
                copy = True 
        else:
            copy = True
        if copy == True:
            copyTestRun(tempPath, currentPath, fTest)

__main__()        