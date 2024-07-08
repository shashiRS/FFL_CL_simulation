function [summaryMatx, npsPerSampVec, hPlot] = readReflData(dataStru, markSize, figHan, hPlot)
%function gets sensor position from and nearest points from SensData data
%structure in IPG control (contained in dataStru); markSize- for data
%plotting

%outputs a summary matrix and data structure containing the data 

summaryMatx = [];
        %matrix columns: sampPntID, Rx_ID, xPos, yPos, xSensPos, ySensPos,
                        %TO_ID, tStamps, ToF 

%field name example ReflectionData_6_NPPos_y

%read field names
allFields   = fieldnames(dataStru);%cell array with strings

%strings with field names
%field name example SensData_6_Target_NPPos_x_0
rootStrg    = 'ReflectionData_';
npXcoord    = '_NPPos_x';
npYcoord    = '_NPPos_y';
% npZcoord = '_Target_NPPos_z_';
ussPosX     = '_RxPos_x';
ussPosY     = '_RxPos_y';
% ussPosZ = '_SensPos_z';
rxID        = '_Rx';
toId        = '_TOid';
timeStmp    = '_TimeTag';
tof         = '_TimeOF';
amp         = '_Amp';

%this fig is made in the parent func
%hFigCM = gcf; % current figure handle
if ~ishandle(figHan)
    warndlg('figure does not exists, exiting..');
    return;
end


%#### make a summary matrix with data from all echoes and plot ground truth NP
%and USS positions:

%get echoes max
reflData_fields = contains(allFields,rootStrg);
npX_fields      = contains(allFields,npXcoord);
nEchoes         = sum(reflData_fields & npX_fields);
npsPerSamp      = [];

totalNP_number  = 0;
figure(figHan);
hold on;
for iEcho  = 1:nEchoes
    
    toID_strg   = [rootStrg num2str(iEcho-1) toId];
    toID_vec    = dataStru.(toID_strg).data';
    
    %structure field names for NP coordinates
    thisFieldX  = [rootStrg num2str(iEcho-1) npXcoord];
    thisFieldY  = [rootStrg num2str(iEcho-1) npYcoord];
    %NP position data
    thisX       = dataStru.(thisFieldX).data';
    thisY       = dataStru.(thisFieldY).data';  
    %dataBool 	= ~isnan(thisX) & ~isnan(thisY);
    dataBool 	= (thisX ~= 0) & (thisY ~= 0);
    npsPerSamp  = [npsPerSamp, dataBool]; %#ok<AGROW>
    
    if max(dataBool) == 0 %no echoes in all sample for this iEcho 
        continue;
    end
    
   % fprintf('nPnts in echo %d is %d.\n', iEcho, sum(dataBool));

    %**********Sensor positions
    %string field name for Rx coordinates
    thisSensX   = [rootStrg num2str(iEcho-1) ussPosX];
    thisSensY	= [rootStrg num2str(iEcho-1) ussPosY];
    
    %get Rx positions for current echo across all sample points
    ussX                    = dataStru.(thisSensX).data';
    nSamps                  = length(ussX);

    ussY                    = dataStru.(thisSensY).data';
    %plot USS positions
    hPlot(3) = plot(ussX(dataBool), ussY(dataBool), 'ok', 'MarkerFaceColor', 'k', 'MarkerSize', markSize-3);
    %**********
    
    %Amp, Rx ID, timestamp and ToF structure field names
    RxID_strg   = [rootStrg num2str(iEcho-1) rxID];
    tStamp_strg = [rootStrg num2str(iEcho-1) timeStmp];
    tof_strg    = [rootStrg num2str(iEcho-1) tof];
    amp_strg    = [rootStrg num2str(iEcho-1) amp];
    %corresponding vectors
    RxID_vec    = dataStru.(RxID_strg).data';
    tStamp_vec	= dataStru.(tStamp_strg).data';
    tof_vec     = dataStru.(tof_strg).data';
    amp_vec     = dataStru.(amp_strg).data';

    %summarize data:
    %thisSens    = zeros(sum(boolXY), 1) + (iUSS-1); 
    sampsID     = (1:nSamps)';
    tempMatx    = [sampsID(dataBool), RxID_vec(dataBool), thisX(dataBool), thisY(dataBool), ...
                    ussX(dataBool), ussY(dataBool), toID_vec(dataBool), tStamp_vec(dataBool),...
                    tof_vec(dataBool), amp_vec(dataBool)];
    summaryMatx = [summaryMatx; tempMatx]; %#ok<AGROW>
    %matrix columns: sampPntID, Rx_ID, xPos, yPos, xSensPos, ySensPos,
                    %TO_ID, tStamps, ToF, amp 

    %plot Nearest points
    hPlot(4) = plot(thisX(dataBool), thisY(dataBool), '*g', 'MarkerFaceColor', 'g', 'MarkerSize', markSize-2);
    %axis equal;
    totalNP_number = totalNP_number + sum(dataBool);
end%for NPs

set(gca,'DataAspectRatio',[1 1 1]);
hold off;
disp(['The total number of vCUS points is: ' num2str(totalNP_number)]);

npsPerSampVec =  sum(npsPerSamp, 2);
end%func