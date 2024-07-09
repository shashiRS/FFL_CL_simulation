@echo off
cd ..
echo local BRICKS build
echo building Brake model in release mode
py scripts/cip.py sync && py scripts/cip.py build --variant SMOKE_TEST --platform vs2017
cd scripts