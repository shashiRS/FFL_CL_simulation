@echo off
cd ..
echo local BRICKS build
echo building Brake model in debug mode
py scripts/cip.py sync && py scripts/cip.py build --variant OCTAGON --platform vs2017_debug
cd scripts