%% vCUS smoke testing analysis script
% Copyies all .erg files from C:\_repos\mf_sil\tests\SIL\CarMaker\SimOutput
% to the current folder (hard copied) and performs a smoke test analysis
% including the generation of a report

run('C:\_repos\vCUS-Phen-Mod\vCUS_Phen_Mat\uspOutput_Debug\cmenv_R2023a.m');

clear all;
clc;

selpath_origin = uigetdir('C:\_repos\mf_sil\tests\SIL\CarMaker\SimOutput','Select the SimOutput folder of your CM directory (location of .erg-files)');
f = waitbar(0,'Please wait, copying .erg files...');

copyfile([selpath_origin '\*.erg'], pwd);
copyfile([selpath_origin '\*.erg.info'], pwd);

refDir = dir('*.erg');
numFiles = numel(refDir);
S = dir(fullfile(pwd,'*.erg'));

for FileIdx = 1:numFiles
    waitbar((FileIdx-1)/numFiles, f, 'Processing Data...');

    %% Run process
    clear dataStru;
    dataStru = cmread([S(FileIdx).folder,'\', S(FileIdx).name]);

    summaryMatx = [];
    summaryMatx_USP = [];
    summaryMatx_fEcho = [];
    summaryMatx_DrvDet = [];
    
    directEchosXArr = [];
    directEchosYArr = [];
            %matrix columns: sampPntID, Rx_ID, xPos, yPos, xSensPos, ySensPos,
                            %TO_ID, tStamps, ToF 
    
    %field name example ReflectionData_6_NPPos_y
    
    %read field names
    allFields   = fieldnames(dataStru);%cell array with strings
    
    %strings with field names
    %field name example SensData_6_Target_NPPos_x_0
    
    % Names of Reflection Data
    rootStrg    = 'ReflectionData_';
    npXcoord    = '_NPPos_x';
    npYcoord    = '_NPPos_y';
    npZcoord    = '_NPPos_z';
    % npZcoord = '_Target_NPPos_z_';
    ussPosX     = '_RxPos_x';
    ussPosY     = '_RxPos_y';
    ussPosZ     = '_RxPos_z';
    % ussPosZ = '_SensPos_z';
    rxID        = '_Rx';
    toId        = '_TOid';
    timeStmp    = '_TimeTag';
    tof         = '_TimeOF';
    amp         = '_Amp';
    txID        = '_Tx';
    ussTxPosX     = '_TxPos_x';
    ussTxPosY     = '_TxPos_y';
    ussTxPosZ     = '_TxPos_z';
    cycle           = '_CycleNo';
    
    % Names of DrvDetectionList
    DrvDetRootStr      = 'AP_UsDrvDetectionList_detections_';
    DrvDetrelEcuTimestamp_us     = '__relEcuTimestamp_us';
    DrvDetsensorTimestamp_us     = '__sensorTimestamp_us';
    
    % Names USP Point content
    uspRootStr      = 'AP_uspPointListOutput_points_';
    uspXpos         = '__Xposition_m';
    uspYpos         = '__Yposition_m';
    direction         = '__direction';
    directionVariance = '__directionVariance';
    heightConfidence  = '__heightConfidence';
    pointCountInTrack = '__pointCountInTrack';
    probabilityHigh   = '__probabilityHigh';
    rawMeasRange      = '__rawMeasRange_m';
    sensorDirection   = '__sensorDirection';
    sensorMask        = '__sensorMask';
    timestamp_us      = '__timestamp_us';
    trackCurvature    = '__trackCurvature';
    trackId           = '__trackId';
    varTrackVurvature = '__varTrackVurvature';
    varXposition      = '__varXposition_m';
    varYposition      = '__varYposition_m';
    xSensorPos        = '__xSensorPos_m';
    xyPositionCovar   = '__xyPositionCovar_m';
    ySensorPos        = '__ySensorPos_m';
    
    % Names FilteredEchoes
    fEchoRootStr                = 'AP_usFilteredEchoOutput_echoes_';
    fEchorelEcuTimestamp_us     = '__relEcuTimestamp_us';
    fEchoreltof_us     = '__tof_us';
    
    
    %#### make a summary matrix with data from all echoes and plot ground truth NP
    %and USS positions:
    
    %get echoes max Reflection Data
    reflData_fields = contains(allFields,rootStrg);
    npX_fields      = contains(allFields,npXcoord);
    nEchoes         = sum(reflData_fields & npX_fields);
    npsPerSamp      = [];
    
    %get USP Points max
    uspPoints_fields = contains(allFields, uspRootStr);
    xPos_fields     = contains(allFields, uspXpos);
    nPoints          = sum(uspPoints_fields & xPos_fields);
    PointsPerSamp      = [];
    
    %get FilteredEchoes max
    fEcho_fields    = contains(allFields,fEchoRootStr);
    fEchoRETS       = contains(allFields,fEchorelEcuTimestamp_us);
    nfEchoes         = sum(fEcho_fields & fEchoRETS);
    nfsPerSamp      = [];
    
    %% Filtered Echoes
    totalfEcho_number  = 0;
    for ifEcho  = 1:nPoints
        nfEchoes_EcuTimestamp_us_strg   = [fEchoRootStr num2str(ifEcho-1) fEchorelEcuTimestamp_us];
        nfEchoes_EcuTimestamp_us_vec    = dataStru.(nfEchoes_EcuTimestamp_us_strg).data';
    
        nfEchoes_tof_us_strg   = [fEchoRootStr num2str(ifEcho-1) fEchoreltof_us];
        nfEchoes_tof_us_vec    = dataStru.(nfEchoes_tof_us_strg).data';
    
        dataBool_fEcho 	= (nfEchoes_tof_us_vec ~= 0);
        nfsPerSamp  = [nfsPerSamp, dataBool_fEcho]; %#ok<AGROW>
    
        %get Rx positions for current echo across all sample points
        nSamps          = length(nfEchoes_EcuTimestamp_us_vec);
        sampsID         = (1:nSamps)';
        pointsId        = ones(sum(dataBool_fEcho), 1) * (ifEcho-1);
        tempMatx_fEcho    = [sampsID(dataBool_fEcho), nfEchoes_tof_us_vec(dataBool_fEcho)];
        summaryMatx_fEcho = [summaryMatx_fEcho; tempMatx_fEcho]; %#ok<AGROW>
    
        totalfEcho_number = totalfEcho_number + sum(dataBool_fEcho);
    end
    
    %% USP Points
    totalUSPPoints_number  = 0;
    dechoes_cnt = 1;
    for iPoints  = 1:nPoints
        uspXpos_strg   = [uspRootStr num2str(iPoints-1) uspXpos];
        uspYpos_strg   = [uspRootStr num2str(iPoints-1) uspYpos];
        uspXpos_vec    = dataStru.(uspXpos_strg).data';
        uspYpos_vec    = dataStru.(uspYpos_strg).data';
    
        xSensorPos_strg   = [uspRootStr num2str(iPoints-1) xSensorPos];
        ySensorPos_strg   = [uspRootStr num2str(iPoints-1) ySensorPos];
        xSensorPos_vec    = dataStru.(xSensorPos_strg).data';
        ySensorPos_vec    = dataStru.(ySensorPos_strg).data';
    
        sensorMask_strg   = [uspRootStr num2str(iPoints-1) sensorMask];
        sensorMask_vec    = dataStru.(sensorMask_strg).data';
    
        timestamp_us_strg   = [uspRootStr num2str(iPoints-1) timestamp_us];
        timestamp_us_vec    = dataStru.(timestamp_us_strg).data';
        timeUSP_            = dataStru.Time.data';
    
        dataBool_USP 	= (uspXpos_vec ~= 0) & (uspYpos_vec ~= 0);
        PointsPerSamp  = [PointsPerSamp, dataBool_USP]; %#ok<AGROW>
    
        %get Rx positions for current echo across all sample points
        nSamps          = length(uspXpos_vec);
        sampsID         = (1:nSamps)';
        pointsId        = ones(sum(dataBool_USP), 1) * (iPoints-1);
        tempMatx_USP    = [sampsID(dataBool_USP), uspXpos_vec(dataBool_USP), uspYpos_vec(dataBool_USP), xSensorPos_vec(dataBool_USP), ySensorPos_vec(dataBool_USP), sensorMask_vec(dataBool_USP), timestamp_us_vec(dataBool_USP), timeUSP_(dataBool_USP)];
        summaryMatx_USP = [summaryMatx_USP; tempMatx_USP]; %#ok<AGROW>
    
        % Save data with only direct reflections
        % idX = find(sensorMask_vec==1|sensorMask_vec==2|sensorMask_vec==4|sensorMask_vec==8|sensorMask_vec==16|sensorMask_vec==32|sensorMask_vec==64|sensorMask_vec==128|sensorMask_vec==256|sensorMask_vec==512|sensorMask_vec==1024|sensorMask_vec==2048);
        % idX = find(sensorMask_vec~=1&sensorMask_vec~=2&sensorMask_vec~=4&sensorMask_vec~=8&sensorMask_vec~=16&sensorMask_vec~=32&sensorMask_vec~=64&sensorMask_vec~=128&sensorMask_vec~=256&sensorMask_vec~=512&sensorMask_vec~=1024&sensorMask_vec~=2048);   
        idX = find(sensorMask_vec>32);
        directechoes_X = 0;
        directechoes_Y = 0;
        for idX_search=1:size(idX,1)
            directechoes_X(idX_search) = uspXpos_vec(idX(idX_search));
            directechoes_Y(idX_search) = uspYpos_vec(idX(idX_search));
        end
        directEchosXArr = [directEchosXArr,directechoes_X];
        directEchosYArr = [directEchosYArr,directechoes_Y];
    
        totalUSPPoints_number = totalUSPPoints_number + sum(dataBool_USP);
    end
    
    %% Reflection Data
    totalNP_number  = 0;
    for iEcho  = 1:nEchoes
        toID_strg   = [rootStrg num2str(iEcho-1) toId];
        toID_vec    = dataStru.(toID_strg).data';
        
        %structure field names for NP coordinates
        thisFieldX  = [rootStrg num2str(iEcho-1) npXcoord];
        thisFieldY  = [rootStrg num2str(iEcho-1) npYcoord];
        thisFieldZ  = [rootStrg num2str(iEcho-1) npZcoord];
        
        %NP position data
        thisX       = dataStru.(thisFieldX).data';
        thisY       = dataStru.(thisFieldY).data';   
        thisZ       = dataStru.(thisFieldZ).data';   
        dataBool 	= (thisX ~= 0) & (thisY ~= 0);
        npsPerSamp  = [npsPerSamp, dataBool]; %#ok<AGROW>
        
        if max(dataBool) == 0 %no echoes in all sample for this iEcho 
            continue;
        end
        
        %**********Sensor positions
        %string field name for Rx coordinates
        thisSensX   = [rootStrg num2str(iEcho-1) ussPosX];
        thisSensY	= [rootStrg num2str(iEcho-1) ussPosY];
        thisSensZ	= [rootStrg num2str(iEcho-1) ussPosZ];
        
        %string field name for Tx coordinates
        thisTxSensX  = [rootStrg num2str(iEcho-1) ussTxPosX];
        thisTxSensY	= [rootStrg num2str(iEcho-1) ussTxPosY];
        thisTxSensZ	= [rootStrg num2str(iEcho-1) ussTxPosZ];
        
        %get Rx positions for current echo across all sample points
        ussX                    = dataStru.(thisSensX).data';
        nSamps                  = length(ussX);
    
        ussY                    = dataStru.(thisSensY).data';
        ussZ                    = dataStru.(thisSensZ).data';
        
        %get Tx positions for current echo across all sample points
        tx_ussX                = dataStru.(thisTxSensX).data';
        tx_ussY                = dataStru.(thisTxSensY).data';
        tx_ussZ                = dataStru.(thisTxSensZ).data';
        
        %plot USS positions
      % plot(ussX(dataBool), ussY(dataBool), 'ok', 'MarkerFaceColor', 'k', 'MarkerSize', markSize-3);
        %**********
        
        %Amp, Rx ID, timestamp and ToF structure field names
        RxID_strg   = [rootStrg num2str(iEcho-1) rxID];
        TxID_strg   = [rootStrg num2str(iEcho-1) txID];
        Cycle_strg   = [rootStrg num2str(iEcho-1) cycle];
        tStamp_strg = [rootStrg num2str(iEcho-1) timeStmp];
        tof_strg    = [rootStrg num2str(iEcho-1) tof];
        amp_strg    = [rootStrg num2str(iEcho-1) amp];
        %corresponding vectors
        RxID_vec    = dataStru.(RxID_strg).data';
        TxID_vec    = dataStru.(TxID_strg).data';
        Cycle_vec    = dataStru.(Cycle_strg).data';
        tStamp_vec	= dataStru.(tStamp_strg).data';
        tof_vec     = dataStru.(tof_strg).data';
        amp_vec     = dataStru.(amp_strg).data';
        time_       = dataStru.Time.data';
    
        %summarize data:
        %thisSens    = zeros(sum(boolXY), 1) + (iUSS-1); 
        sampsID     = (1:nSamps)';
        echoId      = ones(sum(dataBool), 1) * (iEcho-1);
        tempMatx    = [sampsID(dataBool), toID_vec(dataBool), thisX(dataBool), thisY(dataBool),  thisZ(dataBool), tStamp_vec(dataBool),...
                        tof_vec(dataBool), amp_vec(dataBool),RxID_vec(dataBool), ussX(dataBool), ussY(dataBool), ussZ(dataBool),...
                         TxID_vec(dataBool), tx_ussX(dataBool), tx_ussY(dataBool), tx_ussZ(dataBool), echoId, Cycle_vec(dataBool), time_(dataBool)];
        summaryMatx = [summaryMatx; tempMatx]; %#ok<AGROW>
        %matrix columns: sampPntID, Rx_ID, xPos, yPos, xSensPos, ySensPos,
                        %TO_ID, tStamps, ToF, amp
    
        
    
        %plot Nearest points
        %plot(thisX(dataBool), thisY(dataBool), '*g', 'MarkerFaceColor', 'g', 'MarkerSize', markSize-2);
        %axis equal;
        totalNP_number = totalNP_number + sum(dataBool);
    end%for NPs
    
    
    
    % Plotting
    fig(FileIdx) = figure(FileIdx);
        x0=10;
        y0=10;
        width=1200;
        height=1000;
        set(gcf,'position',[x0,y0,width,height])
    hold on;
    grid on;

    title(S(FileIdx).name, 'Interpreter', 'none');
    
    % GT Data
    plot(summaryMatx(:,3),summaryMatx(:,4),['.','b']);
    
    % USP Points
    plot(summaryMatx_USP(:,2),summaryMatx_USP(:,3),['o','r']);
    
    % Only specific echoes
    %plot(directEchosXArr,directEchosYArr,['o','r']);   
    
    plot(summaryMatx(:,14),summaryMatx(:,15),['o','g']);
    plot(summaryMatx_USP(:,4),summaryMatx_USP(:,5),['+','k']);
    
    %legend('GT - Reflection Data','USP Output - Direct Echoes only');
    %legend('GT - Reflection Data','USP Output');
    legend('GT - Reflection Data','USP Output','GT - Sensor Poses','USP Sensor Poses');
    axis equal;
    
    % figure;
    % hold on;
    % grid on;
    % %plot(summaryMatx(:,19), summaryMatx(:,6),'.');
    % %plot(summaryMatx_USP(:,8), summaryMatx_USP(:,7),['o','r']);
    % plot(summaryMatx(:,6),'.');
    % plot(summaryMatx_USP(:,7),['o','r']);
    
    % figure;
    % hold on;
    % grid on;
    % plot(summaryMatx_fEcho(:,2),'.');
    % plot(summaryMatx(:,7),['r','.']);
end

waitbar(1, f, 'Creating report...');
exportgraphics(fig(1),'vCUS_SmokeTest-Report.pdf');
for FileIdx = 2:numFiles
    exportgraphics(fig(FileIdx),'vCUS_SmokeTest-Report.pdf','Append',true)
end

close(f);
close all;