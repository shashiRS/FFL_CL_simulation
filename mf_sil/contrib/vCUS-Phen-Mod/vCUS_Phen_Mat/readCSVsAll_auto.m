function readCSVsAll_auto()

%Validation tool for USS phenomenological model March 2022 
%!!! Uses parallel processing toolbox !!!

%run the tool from \..\vCUS-Phen-Mod\vCUS_Phen_Mat

%******
%INPUT: choose one or all folder(s) in \..\vCUS-Phen-Mod\vCUS_Phen_CM\SimOutput\Validation
%containing CSV files generated in debug mode by the USS phen mod. 

%In turn, CSV files contain usRaw data, header:
% UsRaw_timestamp_us_u64	UsRaw_tof_8us_u14	UsRaw_nfd_nu_u1 UsRaw_virtualisation_nu_u1	
%UsRaw_rxSensorId_nu_u8	UsRaw_measTag_ms_u8	UsRaw_amplitude_nu_u8	UsRaw_timeDomainConfidenceLevel_nu_u4
%vCUS_Tx	vCUS_TOid	vCUS_ToF	vCUS_Fr0_position_NP_x	vCUS_Fr0_position_NP_y	vCUS_Fr0_position_NP_z	
%vCUS_Fr0_position_Sens_Tx_x	vCUS_Fr0_position_Sens_Tx_y	vCUS_Fr0_position_Sens_Tx_z	vCUS_Fr0_position_Sens_x	
%vCUS_Fr0_position_Sens_y	vCUS_Fr0_position_Sens_z

%tool loads all CSVs in a folder, one folder at a time, and checks all usRaw timestamps
%of echoes against previous burst times to calculate ToF, compares these to
%vCUS ToFs. Also, calculates TOFs based on LoFs computed from sensor and
%reflection point coordinates.

%*******
%OUTPUT: the tool makes and saves five figures containing- all TOs in 3D,
%all TOs in 2d (Bird's eye view), subplots of all individual TOs in 2D, histogram
%of differences between usRaw and vCUS ToFs as well as TO- and USS-based
%averages of these diferences, a simlar figure containing differences
%between ToFs from LoFs vs. vCUS ToFs
% In addition it saves a .mat file containing a summary table and a
% structure with outgoing burst times for all 12 sensors (sensor number as structure
%field numbers). Also saves 2 csv files for easy data overview
%For a general quick overview, for each function run, it saves a log text file with the number of
%timestamp miss-matches per simulation folder
%the log file is saved in folder Validation, and all results in ValidationResults
%==========================================================================

% use parfor loop or vectorized version
% starting the parallel pool takes about 30s on i7 2.8 GHz 4 cores with 24GB RAM
%
% a 45s simulation is processed by the vectorized version in 6s :useParF = false;
% and using parfor in 30s to open it +1s the parfor loop: useParF = true
%
% uses parfor for processing multiple simulations on the same parallel pool
% otherwise (for a sg sim) takes this value:
useParF = false;

%usRaw - vCUS ToF match tolerance: see in getUssRawTof
TemperatureK	= 293.15 ; %Kelvin
DistanceToTof   = 2*((((TemperatureK) - 273.15) * 0.6) + 331.5); 
timeFactor      = 8;%microsecs

% get data folder manually
%dataFld         = uigetdir(pwd,'Sim Validation Folder');

%get all data folders in validation
nowDir      = pwd;
dataPath    = [nowDir,'\..\vCUS_Phen_CM\SimOutput\Validation'];
cd(dataPath);

try
    %prompt for data folder
%     answer = questdlg('Please load simulation data from:', ...
%         'Data selection', ...
%         'Single simulation','All sims in Validation', 'Cancel', ...
%                                                 'Single simulation');
    % Handle response
%     switch answer
%         case 'Single simulation'
%             fullPath                = uigetdir(dataPath, 'Select sim data folder');
%             [dataDir, nameFolds, ~] = fileparts(fullPath);
%             nameFolds               = {nameFolds};
%             nDataFlds               = length(nameFolds);
%         case 'All sims in Validation'
            %fullPath    = uigetdir(dataPath, 'Select Validation folder');
            dataDir     = dataPath;
            allContent  = dir(dataDir);
            isSub       = [allContent(:).isdir];
            nameFolds   = {allContent(isSub).name}';
            %folders made this century?
            nameFolds   = nameFolds(logical(contains(nameFolds, '20')),:);
            nDataFlds   = length(nameFolds);
%         case 'Cancel'
%             hEnd = figure;
%             spy ; pause(0.5); close(hEnd);
%             cd(nowDir);
%             return;
%     end
catch
    cd(nowDir);
    uiwait(msgbox("Failed to open folder(s), plese try again.", "Error","error"));
    return;
end

%summary basic results file
dt      = datestr(now,'yyyy-mm-dd_HH-MM-SS-AM');
logFid  = fopen( [dataPath filesep 'validationSummary-' dt '.txt'], 'wt' );

%open parpool for multiple sims
%in auto mode only process the last folder
startFld = nDataFlds;
endFld   = nDataFlds;
if (endFld -  startFld)> 1 
    useParF = true;
    parpool; 
end

for iFld = startFld:endFld
    thisFld = [dataDir filesep nameFolds{iFld}];
    disp(['### Processing simulation ' num2str(iFld) ' of ' ...
                                                       num2str(nDataFlds)]);
     %delete mesage opened previosly
	if (iFld >startFld && ishandle(hInfo)); delete(hInfo); end
     
    %start processing ...
    [hInfo, nTofWrong, wrongBursts] = ...
                        processFolder(thisFld, timeFactor, DistanceToTof, useParF);
    
    if (~ishandle(hInfo) && max(isnan([nTofWrong, wrongBursts])) > 0)               
        msgbox(['No CSV file in folder ' nameFolds{iFld}], "Error","error");
        fprintf( logFid, '??? No CSVs in folder %s\n', nameFolds{iFld});
        fprintf( logFid, ' \n');
        continue;
    end
                    
    fprintf( logFid, ...
        '###The number of usRaw-vCUS ToF mismatches in simulation %s is: %d\n', ...
                                               nameFolds{iFld}, nTofWrong);
    fprintf( logFid, ...
        '****The number of echo-to-burst mismatches in simulation %s is: %d\n', ...
                                               nameFolds{iFld}, wrongBursts);
    fprintf( logFid, ' \n');
    cd(dataDir)
end%for iFld
fclose(logFid);
% if nDataFlds > 1
%     delete(hInfo);
% end

papoObj = gcp('nocreate');
if ~isempty(papoObj)
    delete(papoObj);
end

cd(nowDir);
clear all; %#ok<CLALL>
end%func

%==========================================================================
function [hBox, nOfMismatches, nOfBurstMismatches] = ...
                     processFolder(dataFld, timeScale, Lof2Tof, useParF)
%loads all CSV file in the current sim
%inputs: data folder containing sim data, timeScale = 8 us, conversion
%factor for LoF to ToF
%output: msg box handle and summary of missmatches for ToFs
                 
cd(dataFld);
FileListStru    = dir('TimeStamps_*.csv');
filesList       = {FileListStru(:).name}';
nFiles          = numel(filesList);

if nFiles == 0
    [hBox, nOfMismatches, nOfBurstMismatches] = deal(nan);
    return;
end

%make results folder
[rootPath, fldname, ~]  = fileparts(pwd);
rootPath                = [rootPath filesep 'ValidationResults'];
if ~exist(rootPath, 'dir')
       mkdir(rootPath)
end

resultFld               = [fldname 'Results'];
resFld                  = [rootPath filesep resultFld];
if ~exist(resFld, 'dir')
       mkdir(resFld)
end

%make log file
logFid = fopen( [resFld filesep 'validationLog.txt'], 'wt' );
dt = datestr(now,'yyyy-mm-dd HH:MM:SS.FFF AM');
fprintf( logFid, 'processing time : %s\n',dt);

fWb = waitbar(0,'%','Name',['Loading ' fldname ' CSV data...']);
fWb.Position = [441 295.8750 fWb.Position(3)*1.25 56.2500];

allDataInFld    = [];
filesCell = [];
rowInFile = [];
%dataInFile      = []; 
for iFile = 1:nFiles
    
    waitbar(iFile/nFiles,fWb,sprintf(['%3.2f %% of ' num2str(nFiles) ...
                                        ' CSV files'], 100*iFile/nFiles));
    fileNow         = [dataFld filesep filesList{iFile}];
    tempTable       = readtable(fileNow);
    tablenRows      = height(tempTable);
    
    %store file names and rows
    tempFileName                    = strings;
    tempFileName(1:tablenRows,1)    = deal(filesList{iFile});
    filesCell                       = cat(1, filesCell, tempFileName);
    temRows                         = 2:tablenRows+1;
    rowInFile                       = [rowInFile; temRows']; %#ok<AGROW>
    
    %get table data
    %UsRaw
    timestamp_us_u64    = tempTable.UsRaw_timestamp_us_u64;%in microsecs
    tof_8us_u14         = tempTable.UsRaw_tof_8us_u14;
    nfd_nu_u1           = tempTable.UsRaw_nfd_nu_u1;
    virtualis_nu_u1     = tempTable.UsRaw_virtualisation_nu_u1;
    rx_Id               = tempTable.UsRaw_rxSensorId_nu_u8;
    burst_Flag          = tempTable.UsRaw_measTag_ms_u8;
   	%vCUS_
    vCUS_Tx_Id          = tempTable.vCUS_Tx;
    vCUS_ToF            = tempTable.vCUS_ToF;
    
    %NP coordinates 
    npPosition          = [tempTable.vCUS_Fr0_position_NP_x, ...
                            tempTable.vCUS_Fr0_position_NP_y, ...
                            tempTable.vCUS_Fr0_position_NP_z];
                        
    %get tx id for cases when tx = rx
    vCUS_Tx_Id(vCUS_Tx_Id == -1) = rx_Id(vCUS_Tx_Id == -1);
    
    txPosition        = [tempTable.vCUS_Fr0_position_Sens_Tx_x, ...
                            tempTable.vCUS_Fr0_position_Sens_Tx_y, ...
                            tempTable.vCUS_Fr0_position_Sens_Tx_z];
    rxPosition        = [tempTable.vCUS_Fr0_position_Sens_x, ...
                            tempTable.vCUS_Fr0_position_Sens_y, ...
                            tempTable.vCUS_Fr0_position_Sens_z];
   
    %combine bit info 14 + 1 + 1 to 16 bit unsigned short
    bit14Binary     = dec2bin(tof_8us_u14,14);%if binary starts with 0 this could be 13 bits , add 0s at start to make it 14 bits?
    unsignedShort   = strcat(num2str(virtualis_nu_u1), num2str(nfd_nu_u1), bit14Binary);
    decimal_us      = bin2dec(unsignedShort);
    
    %unsigned LL / 8 -> unsigned short
    tmpTimeTag      = timeScale * floor(timestamp_us_u64./timeScale);
    binary_tTT_us   = dec2bin(tmpTimeTag);
    bin_tTT_us16    = binary_tTT_us(:, end-15:end);%get least sig bits
    decim_tTT_us16  = bin2dec(bin_tTT_us16);
    
    matchBitConversion = max(decim_tTT_us16 ~= decimal_us);
    
    if matchBitConversion 
        disp(['****** failed bit conversion on file: ' fileNow]);
        fprintf(logFid, '****** failed bit conversion on file: %s\n',fileNow);
        %continue
    end
    
    %check for data loss and log it
    if (max(tmpTimeTag ~= decim_tTT_us16))
        %disp(['###### typecast to us16 dataloss in file : ' fileNow]);
        fprintf( logFid, '###### typecast to us16 dataloss in file : %s\n',fileNow);
    end
    dataInFile = [timestamp_us_u64, rx_Id, vCUS_Tx_Id, burst_Flag, ...
                    vCUS_ToF, tempTable.vCUS_TOid, txPosition, rxPosition, npPosition];
    allDataInFld = [allDataInFld ; dataInFile]; %#ok<AGROW>

end %for iFile

fclose(logFid);
delete(fWb);

%sort on first column (on timestamps)
allDataInFld = sortrows(allDataInFld);

%initial Tx position
burstTxId   = allDataInFld(allDataInFld(:,4) >= 127, 3);
burstTxPos  = allDataInFld(allDataInFld(:,4) >= 127, 7:9);
sensIds     = sort(unique(burstTxId));
startIndxs  = arrayfun(@(a) find(burstTxId == a,1),sensIds);
startTxPos  = burstTxPos(startIndxs,:);
startTxPos  = [startTxPos; startTxPos(1,:)];

filesCell(allDataInFld(:,4) >= 127, :) = [];
rowInFile(allDataInFld(:,4) >= 127, :) = [];

%data in allDataInFld: timestamp_us_u64, rx_Id, vCUS_Tx_Id, burst_Flag,
%   vCUS_ToF, vCUS_TOid, Tx sensor (x,y,z), Rx sensor (x,y,z) 
[matchedDataTable, transmitStru] = getUssRawTof(allDataInFld, useParF);
% tableHead   = ["usRaw_ToF", "rx_Id", "tx_Id", "vCUS_ToF", ...
%                "usRaw_vCus_diff", "isMatch", "TO", "Tx_x", "Tx_y", "Tx_z", ...
%                "Rx_x", "Rx_y", "Rx_z", "NP_x", "NP_y", "NP_z","BackwardsMatchBurstNo"];

matxData    = matchedDataTable{:,:};
%dist Tx to NP + Rx to NP
LoF         = sqrt(sum( (matxData(:, 14:16) - matxData(:, 8:10)) .^ 2, 2)) + ...
                sqrt(sum((matxData(:, 14:16) - matxData(:, 11:13)) .^ 2, 2));
recalcToF   = floor( 1000000*LoF ./ Lof2Tof);

%calculate vCUS ToF - recalcToF differences
recalcTofDiffs = matchedDataTable.vCUS_ToF - recalcToF;
recalcToFMatx = [recalcTofDiffs, matchedDataTable.rx_Id, ...
                                                    matchedDataTable.TO];

[tofDifPerUSS, stdvforUSS]  = meansAndSd(recalcToFMatx(:,1:2));
[tofDifPerTO, stdvforTO]    = meansAndSd(recalcToFMatx(:,[1,3]));
%plotting histogram
[figHand1] = plotHist(recalcTofDiffs, tofDifPerUSS, stdvforUSS, tofDifPerTO, ...
                          stdvforTO, 'vCUS vs. Recalculated ToF differences');

newMatData = [matxData, recalcToF, filesCell, rowInFile];
tableHdr   = ["usRaw_ToF", "rx_Id", "tx_Id", "vCUS_ToF", ...
               "usRaw_vCus_diff", "isMatch", "TO", "Tx_x", "Tx_y", "Tx_z", ...
               "Rx_x", "Rx_y", "Rx_z", "NP_x", "NP_y", "NP_z",...
               "BackwardsMatchBurstNo", "TofFromLof", ...
               "fileNames", "rowInFile"];
sumTable   = array2table(newMatData,"VariableNames",tableHdr);

%plot NPs for each TO
dataTO      = [matxData(:, 7), matxData(:, 14:16), matxData(:,2)];
sensPos     = matxData(:, 11:13);
[hFigSubpTOs, hAllTOs3D, hAllTOs2D ] = plotAllTOs(dataTO, sensPos, startTxPos);       

%calculate means of usRaw-vCUS ToF differences
dataMeans = [matchedDataTable.usRaw_vCus_diff, matchedDataTable.rx_Id, ...
                                                    matchedDataTable.TO];

[meansPerUSS, stdvPerUSS]   = meansAndSd(dataMeans(:,1:2));
[meansPerTO, stdvPerTO]     = meansAndSd(dataMeans(:,[1,3]));

tofDiffs    = matchedDataTable.usRaw_vCus_diff;
%plotting histogram
[figHand] = plotHist(tofDiffs, meansPerUSS, stdvPerUSS, meansPerTO, ...
                                stdvPerTO, 'UsRaw-vCUS ToF differences');

fBox = msgbox('Saving data, please wait some more...');
%save 
diffFig  = [resFld filesep 'SimTofDiffs.fig'];
diffPng  = [resFld filesep 'SimTofDiffs.png'];
set(figHand1, 'visible', 'on'); 
saveas(figHand1,diffFig);
%figHand1.Position = [0.7 0.05  0.4 0.4];
print(diffPng,'-dpng','-r600');
close(figHand1);

resultsFig  = [resFld filesep 'usRaw_vCusDiffs.fig'];
resultsPng  = [resFld filesep 'usRaw_vCusDiffs.png'];
set(figHand, 'visible', 'on'); 
saveas(figHand,resultsFig);
%figHand.Position = [0.7 0.05  0.4 0.4];
print(resultsPng,'-dpng','-r600');
close(figHand);

trafObjFig  = [resFld filesep 'TrafObjsSubplots.fig'];
trafObjPng  = [resFld filesep 'TrafObjsSubplots.png'];
set(hFigSubpTOs, 'visible', 'on');
saveas(hFigSubpTOs,trafObjFig);
%hFigSubpTOs.Position = [0.7 0.05  0.4 0.4];
print(trafObjPng,'-dpng','-r600');
close(hFigSubpTOs);

TO2dFig  = [resFld filesep 'TrafObjs2D.fig'];
TO2dPng  = [resFld filesep 'TrafObjs2D.png'];
set(hAllTOs2D, 'visible', 'on');
saveas(hAllTOs2D,TO2dFig);
%hAllTOs2D.Position = [0.7 0.05  0.4 0.4];
print(TO2dPng,'-dpng','-r600');
close(hAllTOs2D);

% montagePng  = [resFld filesep 'Summary.png'];
% dirOutput   = dir(fullfile(resFld,'*.png'));
% fileNames   = string({dirOutput.name});
% fMntg       = figure;
% cd(resFld);
% montage(fileNames,'Size',[2 2]);
% saveas(fMntg,montagePng);
% close(fMntg);

TO3dFig  = [resFld filesep 'TrafObjs3D.fig'];
set(hAllTOs3D, 'visible', 'on');
saveas(hAllTOs3D,TO3dFig);
close(hAllTOs3D);

resultsCSV    = [resFld filesep 'Results.csv'];
writetable(sumTable,resultsCSV);

%save some data on differences
MatlabToFs = recalcToF(recalcTofDiffs ~=0);
recalcToF_files	= filesCell(recalcTofDiffs ~=0);
recalcToF_rows	= rowInFile(recalcTofDiffs ~=0);
vCusTofVals     = matchedDataTable.vCUS_ToF(recalcTofDiffs ~=0);
vCusDiffs       = recalcTofDiffs(recalcTofDiffs ~=0);
vCusDifsTable	= table(recalcToF_files, recalcToF_rows,MatlabToFs,...
                                                   vCusTofVals, vCusDiffs);
vCusDifsFile  = [resFld filesep 'vCusVsRecalculated_ToFs.csv'];
writetable(vCusDifsTable, vCusDifsFile);

usRawVcusFiles  = filesCell(tofDiffs ~=0);
usRawVcusRows   = rowInFile(tofDiffs ~=0);
usRawTStamps    = matchedDataTable.usRaw_ToF(tofDiffs ~=0);
usRawVcusDifs   = tofDiffs(tofDiffs ~=0);
usRawVcusTable  = table(usRawVcusFiles, usRawVcusRows,usRawTStamps, ...
                                                            usRawVcusDifs);
vCusVsUsRawFile  = [resFld filesep 'usRawVsVcus_ToFs.csv'];
writetable(usRawVcusTable, vCusVsUsRawFile);

dataFile  = [resFld filesep 'SummaryData.mat'];
save(dataFile,'sumTable','transmitStru');

delete(fBox);
CreateStruct.Interpreter    = 'tex';
CreateStruct.WindowStyle    = 'modal';
msgInBox                    = [string(strrep(['Sim ' fldname ' completed!'] ...
                                    , '_','-')) ; "See ValidationResults"];
hBox                        = msgbox(msgInBox,"Value",CreateStruct);
pause(1);

nOfBurstMismatches = sum(isnan(matchedDataTable.BackwardsMatchBurstNo));
nOfMismatches = sum(abs(matchedDataTable.usRaw_vCus_diff) > 0);

end%func

%==========================================================================
function [dataTable, transmitStru] = getUssRawTof(dataMatx, useParF)
%check timestamps: calculate ToFs from usRaw timestamps and match them to the 
%vCUS timestamps. return data table and structure containing outgoing burst
%times for all sensors (sensor as structure field number)

tofTolerance = 1; % microsecs

%get transmisstion timestamps and Tx sensor id at transmission for this sim
timestamp_u64       = dataMatx(:,1);
tx_Id               = dataMatx(:,3);
transmitTstamps     = timestamp_u64(dataMatx(:,4) >= 127);
transmitTx          = tx_Id(dataMatx(:,4) >= 127);
transmitSensId      = unique(tx_Id);
nUSSs             	= max(transmitSensId);

for iUSS = 1:nUSSs
    transmitStru(iUSS).tStampU64 = transmitTstamps(transmitTx == iUSS); %#ok<AGROW>
end

%remove transmission bursts:
indxOutBurst                = dataMatx(:,4) >= 127;
dataMatx(indxOutBurst, :)   = [];
timestamp_u64(indxOutBurst)	= [];
tx_Id(indxOutBurst)         = [];
%make data readable again
rx_Id           = dataMatx(:,2);
tof             = dataMatx(:,5);
toId            = dataMatx(:,6);

%look at all received burst timestamps and calculate tof
nTstamps        = length(timestamp_u64);
%mismCnt         = 0;

fBox = msgbox(['Please wait, processing ' num2str(nTstamps) ... 
                                                    ' time stamps...']);
% fWb = waitbar(0,'Aiming for 100:','Name',['Processing ' num2str(nTstamps) ... 
%                                                     ' time stamps...']);

%=========try some vectorization==========
%this does not calculate differences for missmatches only finds which
%time stamps do not match with a burst
if ~useParF
    %out matx data: usRaw_ToF, rx_Id, tx_Id, vCUS_ToF, usRaw_vCus_diff, isMatch
    %tic;  
    outMatx     = nan(nTstamps, 17);
    dataMat     = [];
    allIndxs	= (1:length(timestamp_u64))';
    for iUss = 1:nUSSs
        %get this sensor transmitting times
        txBurstTimes    = transmitStru(iUss).tStampU64;
        if isempty(txBurstTimes)
            continue;
        end

        nowIndxs        = allIndxs(tx_Id == iUss);
        nowTimeStmp     = timestamp_u64(tx_Id == iUss);
        nowTof          = tof(tx_Id == iUss);
        nowRx           = rx_Id(tx_Id == iUss); 
        n_tStamps       = length(nowIndxs);
        txNow           = ones(n_tStamps, 1) * (iUss);

        %find ToFs = differences: echoTimeStamps - burstTimeStamps
        %matrix with element by element diffs between vectors
        difMatx         = bsxfun(@minus, nowTimeStmp', txBurstTimes);

        %***find which ones are matching vCusToFs no matter how many bursts ago
        boolEqualDiffs  = bsxfun(@eq, difMatx, nowTof');%matrix of bools
        %matching values
        usRawTof        = difMatx(boolEqualDiffs);

        %bool vectors of matching and mismatching  ToFs
        matchBool       = sum(boolEqualDiffs,1) == 1;
        misMatchBool    = sum(boolEqualDiffs,1) ~= 1;

        %calculate usRaw ToFs as echo - one_burst_ago
        [nowDifs_1previous, oneBrstBack]  = oneByOneDiffs(txBurstTimes, nowTimeStmp);

        %indices of mmatch and mismatch
        matchIndx       = nowIndxs(matchBool);
        misMtchIndx     = nowIndxs(misMatchBool);
        tempMisMatch    = nowDifs_1previous(misMatchBool);

        %all indices and usRaw ToFs, mismaches tof filled with nans
        tmp_usRawTofs   = [usRawTof; tempMisMatch];
        tmpIndxs        = [matchIndx; misMtchIndx];

        %*********
        %derive the rank of the matched burst: echo match to 1,2,3 burst ago
        %to get an idea of the logic use to generate boolEqualDiffs and oneBrstBack
            %burstT = 0     7    11
            %echoT = 2     5     8    12    14
            %ToF   = [2,5,1,5,3];

        %where were the matches found (boolEqualDiffs) xor the location of 
        %one burst ago relative to current echo (oneBrstBack)
        C           = xor(boolEqualDiffs,oneBrstBack); 
        RankCell    = arrayfun(@(x) find(C(:,x) == 1,1, 'last') - ...
                       find(C(:,x) == 1,1, 'first'), 1:n_tStamps, 'UniformOutput',false);
        RankCell(cellfun('isempty',RankCell)) = {0};
        rankVec         = cell2mat(RankCell);%elements are 0 for 1 Burst Back, 1 for 2 BB, 2 for 3BB
        nowRanks        = (matchBool + rankVec)';%rank of match to burst
        %*********

        %summary data
        tempMat     = [tmpIndxs, tmp_usRawTofs];
        tempMat     = [sortrows(tempMat) , ~misMatchBool', nowRx, txNow];
        %tof diffs: vcus vs usRaw
        usRawVsVcus = tempMat(:,2) - nowTof;
        tempMat     = [tempMat, usRawVsVcus, nowRanks]; %#ok<AGROW>
        dataMat     = [dataMat; tempMat ]; %#ok<AGROW>

    end%for
    
    %sort to initial indices and remove them and add vCUS tofs
    dataMat = sortrows(dataMat);
    dataMat = [dataMat(:, 2:end), tof];

    %match to previous version of the output matrix
    % matrix columns tmp_usRawTofs, ~misMatchBool',  nowRx, txNow, usRawVsVcus, nowRanks, tof
    % new column order 1 3 4, 7, 5, 2
    outMatx(:,1:6)         = dataMat(:, [1, 3, 4, 7, 5, 2]);

    %matchToBurst - to which previous burst did the echo match (first second third...?)
    matchToBurst    = dataMat(:, 6);

    %adds TO id, Tx, Rx coordinates
    outMatx(:,7:17)  = [toId, dataMatx(:, end-8:end), matchToBurst];
    %disp(['vectorized duration is: ' num2str(toc)]);
end% if ~useParF
%=========vectorized version done==========


%alternative to vectorized: do element wise check in parfor loop
%=====================================
if useParF
    %out matx data: usRaw_ToF, rx_Id, tx_Id, vCUS_ToF, usRaw_vCus_diff, isMatch
    outMatx         = nan(nTstamps, 6);
    %tic;          
    checkPool = gcp('nocreate');
    if isempty(checkPool)
        parpool;
    end
%    disp(['opened parallel pool in: ' num2str(toc)]);
    matchToBurst = nan(nTstamps,1);
    parfor iTstmp = 1 : nTstamps
        nowTx           = tx_Id(iTstmp);
        txBurstTimes    = transmitStru(nowTx+1).tStampU64;
        nowRx           = rx_Id(iTstmp);
        nowTimeStmp     = timestamp_u64(iTstmp);
        nowTof          = tof(iTstmp);

        %find closest anterior timestamp for sent burst from current Tx 
        sentTimeStmps   = txBurstTimes(txBurstTimes < nowTimeStmp);

        %echo from 1 burst ago
        if isempty(sentTimeStmps)
            disp('no previous burst')
        end
        usRawTof        = nowTimeStmp - sentTimeStmps(end);

        %check if usRaw_tof matches vCUS_tof
        isMatch         = (abs(usRawTof - nowTof) < tofTolerance);
        if isMatch; matchToBurst(iTstmp) = 1;end
        if ((~isMatch) && (numel(sentTimeStmps) >= 2))
            %if tofs donn't match check echo from 2 bursts ago
            usRawTof = nowTimeStmp - sentTimeStmps(end-1);
            isMatch  = (abs(usRawTof - nowTof) < tofTolerance);
            if isMatch; matchToBurst(iTstmp) = 2;end
            %take this data no matter if it's a match or not
            outMatx(iTstmp, :) = [usRawTof, nowRx, nowTx, nowTof, ...
                                                (usRawTof - nowTof), isMatch];
        else 
            %if tofs match or there is only one sending burst, take the first
            %one (matching or not)
            outMatx(iTstmp, :) = [usRawTof, nowRx, nowTx, nowTof, ...
                                                (usRawTof - nowTof), isMatch];
        end%if

        %waitbar(iTstmp/nTstamps,fWb,sprintf('Aiming for 100: %3.2f %%', 100*iTstmp/nTstamps));
    end%for 

    %adds TO id, Tx, Rx coordinates
    outMatx  = [outMatx, toId, dataMatx(:, end-8:end), matchToBurst];
    %disp(['total parfor duration is: ' num2str(toc)]);
end% if useParF2
%=========parfor version done==========

delete(fBox);
%make a table
tableHead   = ["usRaw_ToF", "rx_Id", "tx_Id", "vCUS_ToF", ...
               "usRaw_vCus_diff", "isMatch", "TO", "Tx_x", "Tx_y", "Tx_z", ...
               "Rx_x", "Rx_y", "Rx_z", "NP_x", "NP_y", "NP_z","BackwardsMatchBurstNo"];
dataTable   = array2table(outMatx,"VariableNames",tableHead);

end %function

%==========================================================================
function [figHsubP, hFigAllTOs, h2dAllTOs] = plotAllTOs(dataMatx, xyzSenso, startTxCord)
%plot traffic objects

%dataMatx header:   "TO", "NP_x", "NP_y", "NP_z","rx_Id",
maxDisplayCube  = 20;%meters
figSize         = 0.55;

%get number of TOs
tosVec      = dataMatx(:,1);
tos         = unique(tosVec);
nTOs        = numel(tos);
nDots       = numel(tosVec);
maxTO       = max(tos)+1;

%**********generate some colors - the complicated way;
% cmap1               = hot(maxTO);
% cmap2               = winter(maxTO) ;
% % Combine them into one tall colormap.
% combinedColorMap    = [cmap1; cmap2];
% % Pick rows at random.
% randomRows          = randi(size(combinedColorMap, 1), [maxTO, 1]);
% % Extract the rows from the combined color map.
% randomColors        = combinedColorMap(randomRows, :);
%**********

%random colors
randomColors        = rand(maxTO,3);
colorFact           = zeros(nDots,3);
indTo               = zeros(nDots,1);
szDots              = zeros(nDots,1) +10;

%make grid for subplots of individual TOs
if isprime(nTOs)
    dispTOs = nTOs + 1;
else
    dispTOs = nTOs;
end
rowSubp = floor(sqrt(dispTOs));
colSubP = ceil(dispTOs/rowSubp);

%assign colors to TOs & start plotting traffic objects
figHsubP     = figure('Name','Detected TOs','units','normalized', ...
                    'outerposition',[0.25 0.05  figSize figSize], 'visible','off');
hold on;
for iTo = 1:nTOs
    toId = tos(iTo);
    colorFact(tosVec == toId, :) = colorFact(tosVec == toId, :) ...
                                                    + randomColors(toId+1, :);
    indTo(tosVec == toId, :)     = indTo(tosVec == toId, :) + iTo;
    colorNow = colorFact(tosVec == toId, :);
    %avoid white TOs
    if sum(colorNow(1,:)) > 2.9
        colorNow = abs(colorNow - rand(1,3));
        colorFact(tosVec == toId, :) = colorNow;
    end
    
    zsNow    = szDots(tosVec == toId, :);
    xyzNpNow = dataMatx(tosVec == toId, 2:4);
    subplot(rowSubp, colSubP, iTo);
    %show traffic objects in 2D (x,y)
    
    scatter(xyzNpNow(:,1), xyzNpNow(:,2), zsNow, colorNow);
    pbaspect([1 1 1]);
    title(['TO ' num2str(toId)]);
    xlabel('X [m]');
    ylabel('Y [m]');

end
hold off;

%3D Traffic objects and sensors
hFigAllTOs        = figure('Name','Detected TOs','units','normalized', ...
                    'outerposition',[0.25 0.05  figSize figSize], 'visible','off');

szSens = zeros(size(xyzSenso,1),1)+2;
hScat = scatter3(xyzSenso(:,1), xyzSenso(:,2), xyzSenso(:,3), szSens);
hold on;
plot3(startTxCord(:,1), startTxCord(:,2), startTxCord(:,3),'o-', ...
    'color',[1,0.5,0],...
    'LineWidth',1.5,...
    'MarkerSize',6,...
    'MarkerEdgeColor',[0,0.5,1],...
    'MarkerFaceColor',[0,0.5,1]);
hScat.MarkerEdgeColor = [0,0,0];
hScat.MarkerFaceColor = [0,0,0];
xlabel ('X [m]');
ylabel ('Y [m]');
zlabel ('Z [m]');
scatter3(dataMatx(:,2), dataMatx(:,3), dataMatx(:,4), szDots, colorFact);
title('Sensors and TOs positions');
zlim([0 4]);%show from above ~4 meters
view(-120,35);
set(gca,'Color','[0.7 0.7 0.7]')
%add boxes to TOs
cloudSource                 = dataMatx(:,2:4);
[cloudMirror, mirrIndx]     = mirrorDataPnts(cloudSource, indTo);
nDataP                      = size(cloudMirror,1);
dataNoise               	= (rand(nDataP,3)-0.5)/10;%add noise to datap:bigger boxes
ptCloud                     = pointCloud(cloudMirror + dataNoise);
[cubStru]                   = boxTos(ptCloud, nTOs, mirrIndx);
for i = 1:nTOs
    thisCube = cubStru(i).cubModel;
    %do not display v. large cubes (i.e., moving TOs tracked through the entire sim.)
    if max(thisCube.Dimensions) > maxDisplayCube
        continue;
    end
        
    plot(thisCube);
end
legend('USS positions', 'USS positions @ start', 'Color: TOs');
hold off;

%2D traffic objects and sensors
h2dAllTOs  = figure('Name','Detected TOs','units','normalized', ...
                   'outerposition',[0.25 0.05  figSize figSize], 'visible', 'off');

hScat = scatter(xyzSenso(:,1), xyzSenso(:,2), szSens);
hold on;
plot(startTxCord(:,1), startTxCord(:,2), 'o-', 'color',[1,0.5,0],...
    'LineWidth',1.5,...
    'MarkerSize',6,...
    'MarkerEdgeColor',[0,0.5,1],...
    'MarkerFaceColor',[0,0.5,1]);
%show vehicle and sensors last position
hScat.MarkerEdgeColor = [0,0,0];
hScat.MarkerFaceColor = [0,0,0];
scatter(dataMatx(:,2), dataMatx(:,3),szDots, colorFact);
title('Sensors and TOs positions');
legend('USS positions', 'USS positions @ start', 'Color: TOs');
xlabel ('X [m]');
ylabel ('Y [m]');
hold off;

end %func

%==========================================================================
function [means, stDevs] = meansAndSd(mat2colums)
%make mean of values of the 1st column based on indices in second
[ud,~,iy]	= unique( mat2colums(:,2));  
means       = [ud, accumarray(iy,mat2colums(:,1),[],@mean)];
stDevs      = [ud, accumarray(iy,mat2colums(:,1),[],@std)];
end

%==========================================================================
function [hFig] = plotHist(valuesVec, mean1, std1, mean2, std2, figName)
%plot histograms 

figSize = 0.55;

%freedman-diaconis bin size: h=2×IQR×n^−1/3 - doesn't always work
IQR         = prctile(valuesVec,75) - prctile(valuesVec,25);
binWidth    = 2 * IQR * (numel(valuesVec)^(-1/3));
binEdges    = min(valuesVec) : binWidth : (max(valuesVec)+binWidth);

hFig        = figure('Name', figName,'units','normalized', ...
                     'outerposition',[0.25 0.05  figSize figSize], 'visible','off');
subplot(1,2,1)
if (binWidth ~= 0)
    histogram(valuesVec,binEdges);
    [N,~] = histcounts(valuesVec,binEdges);
    if max(N)>10000; set(gca, 'YScale', 'log'); end
else
    histogram(valuesVec);
    [N,~] = histcounts(valuesVec);
    if max(N)>10000; set(gca, 'YScale', 'log'); end
end%if
xlabel('ToF differences [microSec]');
ylabel('Count [# echoes]');

%plot
    %means per USS
errhigh = + std1(:,2);
errlow  = - std1(:,2);
subplot(2,2,2)
bar(mean1(:,1)+1,mean1(:,2))                
hold on
er              = errorbar(mean1(:,1)+1, mean1(:,2), errlow, errhigh);    
er.Color        = [0 0 0];                            
er.LineStyle    = 'none';  
xticks(min(mean1(:,1)+1):max(mean1(:,1)+1));
hold off
xlabel('Sensor ID');
ylabel('Mean ToF differences [us]');

    %means per TO
errhigh = + std2(:,2);
errlow  = - std2(:,2);
subplot(2,2,4)
bar(mean2(:,1), mean2(:,2))                
hold on
xticks(min(mean2(:,1)):max(mean2(:,1)));
er              = errorbar(mean2(:,1), mean2(:,2), errlow, errhigh);    
er.Color        = [0 0 0];                            
er.LineStyle    = 'none';  
hold off
xlabel('TO ID');
ylabel('Mean ToF differences [us]');

sgtitle(figName);
end

%==========================================================================
function [cubStru] = boxTos(datapoints, nTOs, toIndx)
% makes boxes around traffic objects, input: cuboid object with datapoints,
% number of trafic objects, indices of TOs- 1 to nTos same length as data
% output: cuboid object

%no clustering for now
% Z           = linkage(datapoints,'ward');
% nClusters   = floor(nTOs * 1.25);
% c           = cluster(Z,'Maxclust',nClusters);

cubStru = struct([]);
for iTo = 1:nTOs
    indx                    = find(toIndx == iTo);
    cubStru(iTo).cubModel   = pcfitcuboid(datapoints,indx);
end
    

end%func

%==========================================================================
function [dataAndMirrored, newInd]   = mirrorDataPnts(dataPnts, indxs)
%function takes datapoints nx3 matrix containing n datapoints and xyz
%coordinates and if the Z coordinates for a given object (indicated by
%indxs) are in the same horizontal plane (not much difference between z
%coordinates), it mirrors the datapoints at 0.2 m above ground (z = 0);

nTos        = max(indxs);
mirrLevel   = 0.15;% meters, mirror at this z level
maxToThick  = 0.1;%meters, max object thickness, only mirror if t.obj is thin
newInd      = indxs;
for iTo = 1:nTos
    zNow        = dataPnts(indxs == iTo, 3);
    zDifNow     = max(zNow) - min(zNow);
    isFlat      = (zDifNow < maxToThick) && (min(zNow) > mirrLevel);
    tempData    = dataPnts(indxs == iTo, :);
    nTemp       = size(tempData,1);
    mirrorData  = [tempData(:,1:2), (zeros(nTemp, 1) + mirrLevel)];
    if isFlat
        %add mirrored data to data
        dataPnts = [dataPnts; mirrorData]; %#ok<AGROW>
        newInd   = [newInd; (zeros(nTemp, 1) + iTo)]; %#ok<AGROW>
    end%if
end%for
dataAndMirrored = dataPnts;
end%func

%==========================================================================
function [difs, boolBSmallerA]  = oneByOneDiffs(burstT, echoT)
%calculates element-wise differences between A and B such that subtracts
%from a number in A the first smaller number in B. To test, use A = 2:3:15
%B = 1:3:9

%input: column vectors
%output row vector

A = echoT';
B = burstT';

%calculate matrix of elements in B smaller than A
blSmall = bsxfun(@le, B', A);
nRows   = length(B);
boolBSmallerA = cell2mat(arrayfun(@(x) blSmall(x, :) - blSmall(x+1, :), ...
                                       1:nRows-1, 'UniformOutput',false)');
boolBSmallerA = logical([boolBSmallerA ; blSmall(end, :)]);

%get element-wise diffs between A and B 
difs          = cell2mat(arrayfun(@(x) A(boolBSmallerA(x,:)) - B(x), ...
                                       1:nRows, 'UniformOutput',false))';
end