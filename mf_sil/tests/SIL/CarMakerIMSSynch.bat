@ECHO OFF
echo Update the CarMaker folder projects with files that are not in GITHub for Movie visualization
echo Create Sandbox, only if run for first time
si createsandbox --project=/ADAS/Projects/Function_Projects/AUP.1xx/AUP.100/06_Algorithm/06_Simulation/05_CarMaker_IMS_GIT_Sync/project.pj --cwd=../SIL/CarMaker --recurse --hostname=ims-adas --port=7001
echo Resynch the project
si resync --sandbox=CarMaker/project.pj --cwd=../SIL/CarMaker --recurse --overwriteChanged --hostname=ims-adas --port=7001

set /p DUMMY=Hit ENTER to continue...