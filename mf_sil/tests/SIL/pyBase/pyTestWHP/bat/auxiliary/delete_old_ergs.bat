@ECHO OFF
echo Delete previous erg files
SET CurrentErgPath=..\..\..\CarMaker\SimOutput\WHP\
del /s /q %CurrentErgPath%*.* >nul