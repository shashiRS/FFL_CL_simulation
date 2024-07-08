@ECHO off
cls
setlocal EnableDelayedExpansion
set "startTime=%time: =0%"

echo Performing Memory Parking Small Regression
set "input_folder=MP_SmallRegressionTests"
set "output_folder=Small_Regr_MP"

call %~dp0\..\..\..\contrib\Reporting_UI\pl_parking\venv\Scripts\activate.bat
python %~dp0\raport_generator.py regresession %input_folder% %output_folder%
call %~dp0\..\..\..\contrib\Reporting_UI\pl_parking\venv\Scripts\deactivate.bat

:: Below are calculations for the script's runtime
set "endTime=%time: =0%"

rem Get elapsed time:
set "end=!endTime:%time:~8,1%=%%100)*100+1!"  &  set "start=!startTime:%time:~8,1%=%%100)*100+1!"
set /A "elap=((((10!end:%time:~2,1%=%%100)*60+1!%%100)-((((10!start:%time:~2,1%=%%100)*60+1!%%100), elap-=(elap>>31)*24*60*60*100"

rem Convert elapsed time to HH:MM:SS:CC format:
set /A "cc=elap%%100+100,elap/=100,ss=elap%%60+100,elap/=60,mm=elap%%60+100,hh=elap/60+100"

echo Start:    %startTime%
echo End:      %endTime%
echo Elapsed:  %hh:~1%%time:~2,1%%mm:~1%%time:~2,1%%ss:~1%%time:~8,1%%cc:~1%

pause