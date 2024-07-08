@ECHO OFF
tasklist | find /i "HIL.exe" >nul && taskkill /im HIL.exe /F >nul
