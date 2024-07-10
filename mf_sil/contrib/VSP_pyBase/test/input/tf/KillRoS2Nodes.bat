@echo off
tasklist /FI "IMAGENAME eq ars540_node.exe" 2>NUL | find /I /N "ars540_node.exe">NUL
if "%ERRORLEVEL%"=="0" taskkill /f /im ars540_node.exe
tasklist /FI "IMAGENAME eq coh_node.exe" 2>NUL | find /I /N "coh_node.exe">NUL
if "%ERRORLEVEL%"=="0" taskkill /f /im coh_node.exe
tasklist /FI "IMAGENAME eq eml_node.exe" 2>NUL | find /I /N "eml_node.exe">NUL
if "%ERRORLEVEL%"=="0" taskkill /f /im eml_node.exe
tasklist /FI "IMAGENAME eq mfc_aptiv_node.exe" 2>NUL | find /I /N "mfc_aptiv_node.exe">NUL
if "%ERRORLEVEL%"=="0" taskkill /f /im mfc_aptiv_node.exe
tasklist /FI "IMAGENAME eq rmf_node.exe" 2>NUL | find /I /N "rmf_node.exe">NUL
if "%ERRORLEVEL%"=="0" taskkill /f /im rmf_node.exe
tasklist /FI "IMAGENAME eq sef_node.exe" 2>NUL | find /I /N "sef_node.exe">NUL
if "%ERRORLEVEL%"=="0" taskkill /f /im sef_node.exe
tasklist /FI "IMAGENAME eq srr_aptiv_node.exe" 2>NUL | find /I /N "srr_aptiv_node.exe">NUL
if "%ERRORLEVEL%"=="0" taskkill /f /im srr_aptiv_node.exe
tasklist /FI "IMAGENAME eq tpf2_node.exe" 2>NUL | find /I /N "tpf2_node.exe">NUL
if "%ERRORLEVEL%"=="0" taskkill /f /im tpf2_node.exe
tasklist /FI "IMAGENAME eq val_node.exe" 2>NUL | find /I /N "val_node.exe">NUL
if "%ERRORLEVEL%"=="0" taskkill /f /im val_node.exe
tasklist /FI "IMAGENAME eq fdp_base_node.exe" 2>NUL | find /I /N "fdp_base_node.exe">NUL
if "%ERRORLEVEL%"=="0" taskkill /f /im fdp_base_node.exe
tasklist /FI "IMAGENAME eq mapbw11_node.exe" 2>NUL | find /I /N "mapbw11_node.exe">NUL
if "%ERRORLEVEL%"=="0" taskkill /f /im mapbw11_node.exe
                                                    
exit