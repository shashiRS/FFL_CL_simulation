@ECHO OFF
echo Delete previous erg files
SET CurrentErgPath=..\..\..\CarMaker\SimOutput\PDW\
del /s /q %CurrentErgPath%*.* >nul