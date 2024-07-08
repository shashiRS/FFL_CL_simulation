function mfSilValidation()


%limitations: v large TO, i.e. which are tracked throughtout the sim could
%have v large bounding boxes which may include other TOs

%first!!!!!!!!!!!
%set path to c:\LegacyApp\IPG\carmaker\win64-10.1\Templates\Car\src\
%run cmenv 

%addpath("c:\LegacyApp\IPG\carmaker\win64-10.1\Templates\Car\src\");
%cmenv 

isLoadFunc = exist('cmread'); 
cannotLoad = isLoadFunc ~= 3;

if cannotLoad
    %check if dir exists
    path2cmenv = 'c:\LegacyApp\IPG\carmaker\win64-11.1.2\Templates\Car\src\';
    pathDirCheck = exist(path2cmenv);
    if pathDirCheck ~= 7
        path2cmenv = uigetdir(matlabroot,'Select path to cmenv function');
    end
    
    %load CarMaker environment
    if isfolder(path2cmenv)
        addpath(path2cmenv);
    else
        warndlg ('Please restart, path not found!')
        return;
    end
    
    if exist('cmenv') == 2
        cmenv;
    else
        warndlg ('Please restart, CarMaker env func not found!')
        return;
    end
end

close all;%close all open figs
boxTolerance    = 0.5;%m
densityMapRes   = 0.5; %m reflection point density map bins
dotPlotSize     = 4;%min is 4
USP_summaryMatx = [];
%matrix columns: samplePntID, USPpointID, timestamp, xPos, yPos, xSensPos, ySensPos

do_vCUS_stuff = 0;%close all validation figures and plot vCUS amplitude


%a = cmread(’d:\DSUsers\uie73459\06_GitHubMfSiL\mf_sil\tests\SIL\CarMaker\SimOutput\Parallel_Right_In_Scan_On_vCUS_UDP.erg’);

%extract data from CM output in matrices; output from mf_sil calling vCUS_phen_mod

%for configuration of the output see 
% d:\DSUsers\uie73459\06_GitHubMfSiL\mf_sil\tests\SIL\CarMaker\Data\Config\SiLValidation_Output
%data struture should contain 
    %AP_uspPointListOutput_numberOfPoints_nu_u8
    %AP_uspPointListOutput_points_0__xPosition_m_f32 - ....

%temp read file
thisFld = pwd;
dataDir = 'd:\DSUsers\uie73459\06_GitHubMfSiL\mf_sil\tests\SIL\CarMaker\SimOutput\';
if ~isfolder(dataDir)
    dataDir =  uigetdir(matlabroot,'Select path to data file(s)');
end

if ~isfolder(dataDir)
    warndlg('Restart func, no data folder was found!');
    return;
end
cd(dataDir)
%
[FileName,PathName] = uigetfile('*.erg','Select the CM data file');
cd(thisFld);
if FileName == 0
    warndlg('Restart func, no data file was found!');
    return;
end

askFile     = exist([PathName FileName], 'file');
fileExists	= askFile == 2;
if fileExists
    dataStru	=  cmread([PathName FileName]);  
else
    warndlg('Restart func, no data file was found!');
    return;
end

allFields = fieldnames(dataStru);%cell array with strings

if max(strcmp(allFields,'AP_uspPointListOutput_numberOfPoints_nu_u8'))
    %get number of data points per sampling point 
    nDataPntsVec = dataStru.AP_uspPointListOutput_numberOfPoints_nu_u8.data';
else 
    hwrn = warndlg('Number of data points is not available, exiting..');
    %pause(1);
    delete(hwrn);
    return;
end

nSampPnts = length(nDataPntsVec);

rootStrg            = 'AP_uspPointListOutput_points_';
usp_xPosStrg        = '__xPosition_m_f32';
usp_yPosStrg        = '__yPosition_m_f32';
usp_sens_xPosStrg   = '__xSensorPos_m_f32';
usp_sens_yPosStrg   = '__ySensorPos_m_f32';
usp_tStamp          = '__timestamp_us_u64';
usp_range           = '__rawMeasRange_m_f32';

%how many dataP did usp output across all sample points
maxPntsUSP_out     = max(nDataPntsVec);

%check how many USP datapoints were read 
bool_pntLists   = contains(allFields,rootStrg);
bool_coords   	= contains(allFields,'__yPosition_m_f32');
n_savedPnts   	= sum(bool_pntLists & bool_coords);
 
if (maxPntsUSP_out > n_savedPnts)
    hwrn = warndlg('Loaded less data points than USP output');
    %pause(3);
    delete(hwrn);
end

%which is higher the n of USP output points or the n of points saved from IPG control
%read whatever is smaller
nPnts = min(maxPntsUSP_out, n_savedPnts);

%generate matrices
[matxUsp_xNp, matxUsp_yNp, matxUsp_xUss, matxUsp_yUss, matxUsp_tStmp, matxUsp_dist] ...
                                = deal(nan( nSampPnts, nPnts));                           
for iPnt = 1:nPnts %only read the max n of points, see above
    %make structure fields
    thisX       =  [rootStrg num2str(iPnt-1) usp_xPosStrg];
    thisY       =  [rootStrg num2str(iPnt-1) usp_yPosStrg];
    thisSensX   =  [rootStrg num2str(iPnt-1) usp_sens_xPosStrg];
    thisSensY   =  [rootStrg num2str(iPnt-1) usp_sens_yPosStrg];
    thisTstamp  =  [rootStrg num2str(iPnt-1) usp_tStamp];
    thisDist    =  [rootStrg num2str(iPnt-1) usp_range];
    
    %get structure fields in matrices
    matxUsp_xNp(:,iPnt)     = dataStru.(thisX).data';
    matxUsp_yNp(:,iPnt)     = dataStru.(thisY).data';
    matxUsp_xUss(:,iPnt)    = dataStru.(thisSensX).data';
    matxUsp_yUss(:,iPnt)    = dataStru.(thisSensY).data';
    matxUsp_tStmp(:,iPnt) 	= dataStru.(thisTstamp).data';
    matxUsp_dist(:,iPnt) 	= dataStru.(thisDist).data'; 
   
end
%rows are sample points; columns are reflection points
matxUsp_xNp(matxUsp_xNp == 0)       = nan;
matxUsp_yNp(matxUsp_yNp == 0)       = nan;
matxUsp_xUss(matxUsp_xUss == 0)     = nan;
matxUsp_yUss(matxUsp_yUss == 0)     = nan;
matxUsp_tStmp(matxUsp_tStmp == 0)	= nan;
matxUsp_dist(matxUsp_dist == 0)     = nan;
vecSampPnts                         = (1:size(matxUsp_dist,1))';
totalPnts                           = 0;

%**** place data in matrices and plot USP output points and sensor positions 
% and call function to get SensData Nearest points and USS positions and plot it
hFig        = figure('Name', 'vCUS & USP overview', 'visible','off');
hold on;
for iPnt = 1:nPnts
    
    if min(isnan(matxUsp_xNp(:,iPnt))) == 1
        continue;%all coordinates are NANs
    end
    
    h(1) = plot(matxUsp_xNp(:,iPnt), matxUsp_yNp(:,iPnt), 'or', 'MarkerFaceColor', 'r', 'MarkerSize', 2);%dotPlotSize
    %h(2) = plot(matxUsp_xUss(:,iPnt), matxUsp_yUss(:,iPnt), 'ob', 'MarkerFaceColor', 'b', 'MarkerSize', dotPlotSize-1);
    h(2) = h(1);
        
    boolPnts    = (~isnan(matxUsp_xNp(:,iPnt))) & (~isnan(matxUsp_yNp(:,iPnt)));
    tempPntID   = zeros(sum(boolPnts), 1) + (iPnt-1);
    
    %make summary matx
    %matrix columns: samplePntID, USPpointID, timestamp, xPos, yPos, xSensPos, ySensPos
    tempMatx = [vecSampPnts(boolPnts), tempPntID, matxUsp_tStmp(boolPnts, iPnt), ...
                    matxUsp_xNp(boolPnts,iPnt), matxUsp_yNp(boolPnts,iPnt), ...
                    matxUsp_xUss(boolPnts,iPnt), matxUsp_yUss(boolPnts,iPnt)];
    USP_summaryMatx = [USP_summaryMatx ; tempMatx]; %#ok<AGROW>
    
    totalPnts = totalPnts + sum(~isnan(matxUsp_xNp(:,iPnt)) & ~isnan(matxUsp_yNp(:,iPnt)));
    
end
disp(['The total number of USP points is: ' num2str(totalPnts)]);

[sumSensDataMatx, npsPerSampVec, h] = readReflData(dataStru, dotPlotSize, hFig, h);
%sumSensDataMatx columns: sampPntID, Rx_ID, xPos, yPos, xSensPos, ySensPos,
                %TO_ID, tStamps, ToF, amp 

hold off;
%**** USP and sensor data shown 

if do_vCUS_stuff 
    showAmplitudes(sumSensDataMatx, boxTolerance);
    return;
end

%make vector of number of nearest points (ground truth) per sample point
% nNpsPerSamp = npsPerSampVec;
% %**** show number of NPs and USP output points per sample 
% sampPntsh = figure;
% hold on;
% plot    (vecSampPnts, nDataPntsVec, 'o-k', 'MarkerSize', dotPlotSize);
% plot    (vecSampPnts, nNpsPerSamp, 'o-m', 'MarkerSize', dotPlotSize);
% legend  ('USP output','SensData input', 'Location','best');
% ylabel  ('Count');
% xlabel  ('Sample point');
% hold off;
% %**** shown
densityMapCorr = {};
%make bounding boxes
[densityMapCorr, outOfTosUsp, corrCell, ellipseKPICell, nOutOfElli] = ...
    getToBoxAndStats(sumSensDataMatx, h, USP_summaryMatx, hFig, boxTolerance, densityMapRes);


%#############################
%make 2D normalized density maps for reflection points and USS
%positions
%reflection point density map correlation: ground truth vs USP
NPs_xy          = sumSensDataMatx(:, 3:4);
UDP_xy          = USP_summaryMatx(:, 4:5);
[reflPntCorr]   = doMapCorrelations(NPs_xy, UDP_xy, densityMapRes, 'Reflection');

%USS positions density map correlation: ground truth vs USP
UssGT_xy            = sumSensDataMatx(:, 5:6);
UssUDP_xy       	= USP_summaryMatx(:, 6:7);
[ussPositionCorr]   = doMapCorrelations(UssGT_xy, UssUDP_xy, densityMapRes, 'USS coord. ');
%#############################


%summary basic results file
dt      = datestr(now,'yyyy-mm-dd_HH-MM-SS-AM');
fileStr = [PathName FileName '_validation_' dt '.txt'];
logFid  = fopen( fileStr, 'wt' );

nEntrys = size(corrCell,1);
for iEntry = 1:nEntrys
    fprintf( logFid, [corrCell{iEntry} ' \n']);
    %disp([corrCell{iEntry}])
end

msgCorr1 = sprintf(['The 2D normalized density map correlation of vCUS reflection'...
                          ' points vs. USP is: %2.2f  \n'] , reflPntCorr);
msgCorr2 = sprintf(['The 2D normalized density map correlation of USS postions,' ...
                          ' vCUS vs. USP is: %2.2f \n'] , ussPositionCorr); 
fprintf (logFid, ' \n');
fprintf (logFid, [outOfTosUsp '%%\n']);
fprintf ('\n')
fprintf ([outOfTosUsp '%%']);
fprintf ('\n\n')                      
fprintf (logFid, ' \n');
fprintf (logFid, msgCorr1);
%disp    (msgCorr1)
fprintf (logFid, msgCorr2);
%disp    (msgCorr2);
fprintf (logFid, ' \n');
for iLn = 2 : size(densityMapCorr, 1)
    thisStrg = [densityMapCorr{1, 2} ': ' densityMapCorr{iLn, 1} ...
                            ' ' num2str(cell2mat(densityMapCorr{iLn, 2}))];
    fprintf (logFid, thisStrg);
    fprintf (logFid, ' \n');
end
fprintf (logFid, ' \n');

%print ellipse kpis
nLineEllip = size(ellipseKPICell, 1);
if nLineEllip > 2
    fprintf (logFid, 'Ellipse KPIs are not recommended for multiple TOs simulations! \n');
end

for iLn = 1 : nLineEllip
    thisStrg = sprintf( '%-15s%-25s%-30s', ellipseKPICell{iLn, 1},  num2str(ellipseKPICell{iLn, 2}) ...
                            , num2str(ellipseKPICell{iLn, 3}));

    fprintf (logFid, thisStrg);
    fprintf (logFid, ' \n');
end
fprintf (logFid, ' \n');
outOfEllipseUsp = sprintf(['The number of USP points outside ellipses '...
                                     ' is: %.2f %s'], nOutOfElli, '%');
fprintf (logFid, outOfEllipseUsp);
 
fclose  (logFid);
if ispc
    winopen(fileStr);
end
 showDLG();
end%function
%==========================================================================

function showDLG()
	d       = dialog('Position',[300 300 250 100],'Name',' ');

    txt     = uicontrol('Parent',d,...
               'Style','text',...
               'Position',[20 40 210 40],...
               'String','Validation done!');

    btn1	= uicontrol('Parent',d,...
               'Position',[30 20 70 25],...
               'String','Close all figs',...
               'Callback','close all');
    btn2    = uicontrol('Parent',d,...
               'Position',[150 20 70 25],...
               'String','OK',...
               'Callback','delete(gcf)');
end

%==========================================================================
function [polyg, percInBox, objSize]  = dotsInBox(clustNowMat, boxTolerance, allData)
%function: makes bounding box for data in clustNowMat, with a tolerance of
%boxTolerance around min/max of x&y of the datapoints in clustNowMat,
%checks which points from allData are inside the box

%outputs bounding box and percent of points in box

bt      = boxTolerance;
minX    = min(clustNowMat(:,1));
miny    = min(clustNowMat(:,2));
maxx    = max(clustNowMat(:,1));
maxy    = max(clustNowMat(:,2));

polyg   = [minX-bt miny-bt ; minX-bt maxy+bt ; maxx+bt maxy+bt ; maxx+bt miny-bt ; minX-bt miny-bt] ; 
in      = inpolygon(allData(:,1), allData(:,2), polyg(:,1), polyg(:,2)); 

percInBox   = 100*sum(in)/size(allData,1);
objSize     = sqrt(sum(([minX, miny] - [maxx, maxy]) .^ 2, 2));
end

%==========================================================================
function [densityCorr, outOfTosUsp, corrCell, ellipseKPI,uspOutEllipse] = getToBoxAndStats(dataMtx, h, USP_summaryMatx, hFig, boxTolerance, densityMapRes)
%use dataMtx- data from ReflectionData structure from IPG control(output from vCUS)
%    function splits TOs if made of two parts more than toMinSplit m apart
%makes boxes around (split) TOs and checks how many USP datapoints are
%inside the boxes
%makes cross-correlograms of distances between NP clusters and USP
%datapoints (displacement of NP TO points and USP points)

%dataMtx columns: sampPntID, Rx_ID, xPos, yPos, xSensPos, ySensPos,
                    %TO_ID, tStamps, ToF 

%keep origunal data
data_vCUS_orig = dataMtx;

%sort on timestamps
dataMtx = sortrows(dataMtx, 8);

%could make the data smaller: remove repeating NPs
toCoords    = dataMtx(:, 3:4);
[~,indx,~]  = unique(toCoords, 'rows');
dataMtx     = dataMtx(indx, :);

%get TO id and count
toVec       = dataMtx(:,7);
toIds       = unique(toVec);
nTos        = numel(toIds);

[allToIds, newTOmatx] = splitTOs(dataMtx, toIds, nTos);

%now do boxes with split TOs   
%setup data for cross-correlograms(CRC) and CRC plots
toIds           = unique(allToIds);
nTos            = numel(toIds);
rowSubp         = floor(sqrt(nTos));
colSubP         = ceil(nTos/rowSubp);
hhist           = figure('Name','USP positions displacement to TO clusters');
percentUSPpnts  = zeros(nTos,1);
polygsMatx      = zeros(5,2,nTos);
clustList       = {};
axs             = zeros(nTos, 1);
clustSizes      = zeros(nTos, 1);
rho             = zeros(nTos, 1);
nNpsInClust     = zeros(nTos, 1);
toFig           = figure('Name','Traffic Objects','visible','off');
corrCell        = cell(nTos, 1);
densityCorr 	= {'TO#', 'USP-vCus normalized density map corr'};

%get box of all UPS points
[polygAllUSP, ~,~]  = ...
            dotsInBox(USP_summaryMatx(:, 4:5), boxTolerance, USP_summaryMatx(:, 4:5));


%make cross-correlogram of distances between USP output points and NP clusters
for iTo = 1:nTos
    thisTO              = toIds{iTo};
    boolCoords          = matches(allToIds, thisTO);
    nowCoords           = newTOmatx(boolCoords, :);%vCus NPs for the current TO
    nNpsThisClust       = size(nowCoords,1);
    nNpsInClust(iTo)    = nNpsThisClust;
    
    idealClustDist  = [];
    clustDist       = [];
	autocorrDist    = [];
    
    %getPolygon around current cluster & plot in the first dots figure
    [polygsMatx(:,:,iTo), percentUSPpnts(iTo), clustSizes(iTo)]  = ...
            dotsInBox(nowCoords, boxTolerance, USP_summaryMatx(:, 4:5));
   
    %figure(hFig);
    %set(hFig, 'visible', 'off');
    set(0, 'CurrentFigure', hFig);
    hold on;
    h(5) = plot(polygsMatx(:,1, iTo), polygsMatx(:,2, iTo), '-k');
    hold off;
    
    %get all USP poits distances relative to current cluster points
    
    %make hist for each iNP and add to counts per bin (preset bin edges for each TO)
    %avoid out of memory for large TOs
    ployDiagLength  = sqrt(sum((polygAllUSP(1,:) - polygAllUSP(3,:) ) .^ 2, 2));% this is the max posible dist between two points
    nBins           = floor(ployDiagLength/0.2);%take bins of 20 cm;
    bnEdges         = 0:0.2:0.2*nBins;
    [ N, Nideal, cntAuto]     = deal(zeros(1,nBins));
    for iNp = 1:nNpsThisClust
        thisNP          = nowCoords(iNp,:);
        %clustDist       = [clustDist; sqrt(sum((thisNP - USP_summaryMatx(:, 4:5)) .^ 2, 2))]; %#ok<AGROW>
        clustDist   =  sqrt(sum((thisNP - USP_summaryMatx(:, 4:5)) .^ 2, 2));%this should be a function
        [Ni,edges]	= histcounts(clustDist,bnEdges);
        if max(edges-bnEdges) == 0
            N = N + Ni;
        else
           disp('Something went wrong with binning!!!!')
        end
        
        %get ideal distances
        %idealClustDist  = [idealClustDist; sqrt(sum((thisNP - newTOmatx) .^ 2, 2))]; %#ok<AGROW>
        idealClustDist      = sqrt(sum((thisNP - newTOmatx) .^ 2, 2));
        [Nideal_i,edgesIdl]	= histcounts(idealClustDist,bnEdges);
        if max(edgesIdl-bnEdges) == 0
            Nideal = Nideal + Nideal_i;
        else
           disp('Something went wrong with autoBinning!!!!')
        end
                
        %autocorrDist    = [autocorrDist; sqrt(sum((thisNP - nowCoords) .^ 2, 2))]; %#ok<AGROW>
        autocorrDist = sqrt(sum((thisNP - nowCoords) .^ 2, 2));
        [cntAutoi, edgeAuto] = histcounts(autocorrDist, bnEdges);
        if max(edgeAuto-bnEdges) == 0
            cntAuto = cntAuto + cntAutoi;
        else
           disp('Something went wrong with autoBinning!!!!')
        end
    end
    
    %remove trailing zeros
    indexZero   = find(N ~= 0, 1, 'last');
	N           = N(1 : indexZero);
    Nideal      = Nideal(1 : indexZero);
    cntAuto     = cntAuto(1 : indexZero);
	%maxDist     = max(clustDist);
	%nBins       = floor(maxDist/0.2);%take bins of 20 cm
	%[N,edges]	= histcounts(clustDist,nBins);
    
    %do ideal clust hist and autocorr hist
    normCntIdeal = Nideal/sum(Nideal);
    normCntAuto = cntAuto/cntAuto(1) * normCntIdeal(1); 
    normCnt     = N/sum(N);
    binMiddle   = edges(1:end-1) + (edges(2)-edges(1))/2;
    binMiddle   = binMiddle(1 : indexZero);
    namStrg     = ['TO ' thisTO];
    clustList   = cat(2, clustList, namStrg);
    
    %ideal vs USP cross correlogram correlation
    rho(iTo) = corr(normCnt', normCntIdeal');
    
    %plot histograms of USP points displacement relative to clusers
    figure(hhist);
    hold on;
    axs(iTo) = subplot(rowSubp, colSubP, iTo);
    hold on;
    bar(binMiddle, normCnt);    
    plot(binMiddle, normCntIdeal, '-c', 'LineWidth',1.5);
    plot(binMiddle, normCntAuto, '--m', 'LineWidth',1);
    plot([0, clustSizes(iTo)], [0,0], '-r', 'LineWidth',2);
    plot(0,0,'.w');
    
    cluStrg         = ['TO' thisTO];
    annStrg         = sprintf('Corr: %2.2f', rho(iTo));
    corrCell(iTo)   = {[cluStrg ' ' annStrg]};
    legend({[cluStrg ' vs. USP'],  [cluStrg ' vs. NPs'], ...
               [cluStrg ' vs. ' cluStrg], 'TO length', annStrg,}, ...
               'FontSize', 6, 'Location','Best');
    title(namStrg);
    ylabel('Norm count');
    xlabel('Displacement (m)');
    hold off;
    set(0, 'CurrentFigure', toFig);
    hold on;
    hToLegd(iTo) = plot(nowCoords(:,1), nowCoords(:,2), '*'); %#ok<AGROW>
    xlabel('X (m)');
    ylabel('Y (m)');
    hold off;
    
    %get USP - vCUS point density correlation for this TO
    thisPoly        = polygsMatx(:,:,iTo);
    indx_TO_NPs     = inpolygon(dataMtx(:, 3), dataMtx(:, 4), thisPoly(:,1), thisPoly(:,2));
    TO_NP_coord     = dataMtx(indx_TO_NPs, 3:4);
    indx_UDP      	= inpolygon(USP_summaryMatx(:, 4), USP_summaryMatx(:, 5), thisPoly(:,1), thisPoly(:,2));
    box_UDP_coord   = USP_summaryMatx(indx_UDP, 4:5);
    [reflPntCorr]   = doMapCorrelations(TO_NP_coord, box_UDP_coord, densityMapRes, cluStrg);
    densityCorr     = cat(1, densityCorr,  {cluStrg, num2cell(reflPntCorr)});
end%for iTo
    
linkaxes(axs,'xy');

figure(hFig);
set(gca,'DataAspectRatio',[1 1 1]);
legend(h(1:5),'USP refl. pnts.','USP USS position','vCUS USS postition', ...
    'Nearest Points', 'TO box', 'Location','best', 'FontSize', 8);
xlabel('X (m)');
ylabel('Y (m)');
set(hFig, 'visible', 'on');

%show TOs
figure(toFig);
set(gca,'DataAspectRatio',[1 1 1]);
legend(hToLegd(1:iTo),toIds', 'Location','best', 'FontSize', 8);
set(toFig, 'visible', 'on');

[ellipseKPI, uspOutEllipse, idealPerc] = ...
    boundingEllipsesV4(nTos, polygsMatx, data_vCUS_orig(:, 3:4),...
    USP_summaryMatx(:, 4:5), boxTolerance, toIds); 


%plot percent of USP data found in bounding boxes of ground-truth
%nearest points clusters (relative to total N of USP points)
%idealPerc       = 100*[nNpsInClust/size(newTOmatx,1);0];
if sum(idealPerc) ~=100
    disp("Some of the ground truth dots are not in bounding boxes!!!");
end

idealPerc       = [idealPerc; 0];
clustList       = cat(2, clustList, 'OutsideBoxes');
percOutOf       = 100- sum(percentUSPpnts);
percentUSPpnts  = [percentUSPpnts; percOutOf];
figure      ('Name','Percent USP in TO box');
hold        on;
bar         ([percentUSPpnts, idealPerc]);
xticks      (1:nTos+1);
xticklabels (clustList);
ylabel      ('Data points in box (%)');
legend      ('USP points','Ground truth', 'Location','best')
hold        off;

outOfTosUsp = sprintf(['The number of USP points outside TO bounding '...
                                     'boxes is: %.2f %s'], percOutOf, '%');
                                                       
end%func 

function showAmplitudes(sumSensDataMatx, boxTolerance)
%************************************************
%vCUS stuff: show amplitude per sensor and NP
NPs_xy = sumSensDataMatx(:,3:4);
close all;%close all figures
szDots      = 10;
hHeatAmp    = figure('Name', 'Maximal Amplitude Map');
%hold on;
ampVec      = sumSensDataMatx(:, 10);
minMaxAmp   = [min(ampVec), max(ampVec)];
minmaxX     = [min(NPs_xy(:,1)) - boxTolerance, max(NPs_xy(:,1)) + boxTolerance];
minmaxy     = [min(NPs_xy(:,2)) - boxTolerance, max(NPs_xy(:,2)) + boxTolerance];
ussVec      = sumSensDataMatx(:, 2);
ussIds      = unique(ussVec);
nUss        = numel(ussIds);
rowSubp 	= floor(sqrt(nUss));
colSubP     = ceil(nUss/rowSubp);
for iuss = 1:nUss
    nowUss  = ussIds(iuss);
    allXY	= NPs_xy(ussVec == nowUss, :);
    tempAmp	= ampVec(ussVec == nowUss, :);
    
    %get maximu amplitude for nearest point position
    [tempXY, ~, indx]   = unique(allXY, 'rows');
    nVals               = size(tempXY, 1);
    maxAmpV             = zeros(1,nVals);
    uniIndx             = unique(indx);
    for iVal = 1:nVals
        maxAmpV(iVal) = max(tempAmp(indx == uniIndx(iVal)));
    end
    
    fprintf('sensor number %d has %d reflection points \n', nowUss, numel(tempAmp));
    figure(hHeatAmp);
    sph{iuss} = subplot(rowSubp, colSubP, iuss, 'Parent', hHeatAmp);
    %hold on;
    %plot(tempXY(:,1),tempXY(:,2), 'ok');
    scatter(tempXY(:,1),tempXY(:,2),szDots,maxAmpV,'filled');
    caxis(sph{iuss}, minMaxAmp);
    %colorbar;
    colormap jet;
    namSubplt       = ['RxUSS ' num2str(nowUss)];
    title   (namSubplt);
    xlim    (minmaxX);
    ylim    (minmaxy);
    ylabel  ('y Coordinate (m)');
    xlabel  ('x Coordinate (m)');
end%for
h = axes(hHeatAmp,'visible','off'); 
c = colorbar(h,'Position',[0.93 0.168 0.01 0.7]);  % attach colorbar to h
colormap(c,'jet')
caxis(h,minMaxAmp);             % set colorbar limits
c.Label.String = 'Amplitude';
hold off;
end    