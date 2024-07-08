@ECHO OFF

SETLOCAL EnableDelayedExpansion

SET CurrentErgPath=..\..\..\CarMaker\SimOutput\PDW\
DIR /A /B /S %CurrentErgPath% | findstr . >NUL || GOTO EXIT

SET ArchiveErgPath=d:\ArchiveERG\PDW\
SET Today=%date:~10,4%%date:~4,2%%date:~7,2%
SET TodayArchivePath=%ArchiveErgPath%%Today%

IF not exist %TodayArchivePath%* (
    SET newdir=%TodayArchivePath%
    GOTO COPY
)

IF exist %TodayArchivePath% (
    REN %TodayArchivePath% %Today%_1 
    SET newdir=%TodayArchivePath%_2
    GOTO COPY
)

SET /a maxSuffix=0
CD %ArchiveErgPath%
FOR /d %%d in (%Today%_*) DO (
    SETLOCAL EnableDelayedExpansion
    FOR /F "tokens=2 delims=_" %%s in ("%%d") do (
        IF %%s GTR !maxSuffix! SET /a maxSuffix=%%s
    )
)
SET /a maxSuffix=%maxSuffix%+1
SET newdir=%TodayArchivePath%_%maxSuffix%
GOTO COPY

:COPY
MKDIR %newdir%
XCOPY %CurrentErgPath%* %newdir% >nul
ECHO Archive output erg files in %newdir%

:EXIT