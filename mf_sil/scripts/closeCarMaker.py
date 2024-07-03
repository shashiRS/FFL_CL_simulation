import psutil

PROCNAME = "CarMaker.win64.exe"

for proc in psutil.process_iter():
    # check whether the process name matches
    if proc.name() == PROCNAME:
        proc.kill()
        
print("All CarMaker proccesses were closed!")