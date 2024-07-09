@echo off
REM configure pip's index and certifacate for intranet resources via 
REM env-vars.
set PIP_INDEX_URL=https://eu.artifactory.conti.de/artifactory/api/pypi/c_adas_cip_pypi_v/simple
set PIP_CERT=%~dp0\conti_certificates.crt
py -3 %~dp0\get-pip.py 

REM make pip configuration permanent by using "pip config ..."
REM the configuration is stored in C:\Users\<USER>\AppData\Roaming\pip\pip.ini
pip config set global.cert %PIP_CERT%
pip config set global.index-url %PIP_INDEX_URL%
set PIP_INDEX_URL=
set PIP_CERT=