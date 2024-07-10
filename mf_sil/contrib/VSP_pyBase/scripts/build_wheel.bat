@echo off

set PYBASE_ROOT=%~dp0\..

if exist %PYBASE_ROOT%\vsp_pybase\ (
    rmdir %PYBASE_ROOT%\vsp_pybase /S /Q
)

if exist %PYBASE_ROOT%\adas_vsp_pybase.egg-info\ (
    rmdir %PYBASE_ROOT%\adas_vsp_pybase.egg-info /S /Q
)

if exist %PYBASE_ROOT%\build\ (
    rmdir %PYBASE_ROOT%\build /S /Q
)

if exist %PYBASE_ROOT%\dist\ (
    rmdir %PYBASE_ROOT%\dist /S /Q
)

mkdir %PYBASE_ROOT%\vsp_pybase

xcopy %PYBASE_ROOT%\pycmCommon %PYBASE_ROOT%\vsp_pybase\pycmCommon\ /S /E
xcopy %PYBASE_ROOT%\pycmEval %PYBASE_ROOT%\vsp_pybase\pycmEval\ /S /E
xcopy %PYBASE_ROOT%\pyUtils %PYBASE_ROOT%\vsp_pybase\pyUtils\ /S /E
copy %PYBASE_ROOT%\__init__.py %PYBASE_ROOT%\vsp_pybase\__init__.py

call %PYBASE_ROOT%\.venv\Scripts\activate
python -m build -w
deactivate
