function [summaryMatx, npsPerSampVec] = readVCUSData_to_CSV(dataStru)
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


%#### make a summary matrix with data from all echoes and plot ground truth NP
%and USS positions:

%get echoes max
reflData_fields = contains(allFields,rootStrg);
npX_fields      = contains(allFields,npXcoord);
nEchoes         = sum(reflData_fields & npX_fields);
npsPerSamp      = [];

totalNP_number  = 0;
%figure();
%hold on;
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

    %summarize data:
    %thisSens    = zeros(sum(boolXY), 1) + (iUSS-1); 
    sampsID     = (1:nSamps)';
    echoId      = ones(sum(dataBool), 1) * (iEcho-1);
    tempMatx    = [sampsID(dataBool), toID_vec(dataBool), thisX(dataBool), thisY(dataBool),  thisZ(dataBool), tStamp_vec(dataBool),...
                    tof_vec(dataBool), amp_vec(dataBool),RxID_vec(dataBool), ussX(dataBool), ussY(dataBool), ussZ(dataBool),...
                     TxID_vec(dataBool), tx_ussX(dataBool), tx_ussY(dataBool), tx_ussZ(dataBool), echoId, Cycle_vec(dataBool)];
    summaryMatx = [summaryMatx; tempMatx]; %#ok<AGROW>
    %matrix columns: sampPntID, Rx_ID, xPos, yPos, xSensPos, ySensPos,
                    %TO_ID, tStamps, ToF, amp 

    %plot Nearest points
    %plot(thisX(dataBool), thisY(dataBool), '*g', 'MarkerFaceColor', 'g', 'MarkerSize', markSize-2);
    %axis equal;
    totalNP_number = totalNP_number + sum(dataBool);
end%for NPs

%set(gca,'DataAspectRatio',[1 1 1]);
%hold off;
disp(['The total number of vCUS points is: ' num2str(totalNP_number)]);


%get unique readings in one 40 ms cycle, not repeating echoes every 1 ms
%(i.e., for a sample rate of i KHz);

summaryMatxSort     = sortrows(summaryMatx,10) ;
[~,ia,~]            = unique(summaryMatxSort(:,2:end),'rows', 'first');
summaryMatxUni    	= summaryMatxSort(sort(ia), :);

npsPerSampVec =  sum(npsPerSamp, 2);

tableHead = ["SapleID (1ms)", "TO_ID", "NP_x (m)", "NP_y (m)", "NP_z (m)", ...
                 "TimeTag (us)", "ToF (us)", "Amp", "RxID",  "Rx_x (m)", "Rx_y (m)", "Rx_z (m)", "Tx_ID", ...
                "Tx_x (m)", "Tx_y (m)", "Tx_z (m)", "EchoId", "Cnt_40msCycle"];
a2t = array2table(summaryMatxUni,"VariableNames",tableHead);
writetable(a2t,'PerpendicularWall_uni.csv') ;

a2t = array2table(summaryMatxSort,"VariableNames",tableHead);
writetable(a2t,'PerpendicularWall_all.csv') ;

end%func