function [elipseKPICell, uspOutside, vCUSInsideBox]  = ...
    boundingEllipsesV4(nBoxs, boxes, vCUS_data, USP_coord, boxAdjust, toIds)
%https://www.mathworks.com/matlabcentral/fileexchange/9542-minimum-volume-enclosing-ellipsoid

%function calculates USP points inside growing elypses, with elypses
%centered on vCUS TOs (ellipse is fitted to each TO from vCUS)

%the ellipse is increases in steps of elipseStepsSize until it encompases
%30% of the USP points 

%the reference ellipse size is the first ellipse which encompases all vCUS
%points for a given TO

%#########otputs######### a list of ellipse sizes which show how large is the ellipse that
%encompases a percentage of USP points (out of total USP) which is 30% (or 50%) of
%the number of vCUS points a given TO has, relative to the total vCUS 

%example: for 50% -> if for vCUS output a TO has 30 points and the total for the simulation is 100,
%           the ref vCUS percentange is 30%; the output is the size of the ellipse
%           which encompases 15% of the USP points (usp in ellipse vs total
%           USP): (15% USP points in ellipse out of total USP ) / (30% vCUS
%           points in that TO out of total vCUS)

%         the area of the ellipse is calculated relative to the area of the
%         ellipse which ecompases the 30% of the vCUS points (the entire current TO)

lineResiduals   = 0.1;%checks if TO is linear: line fit and look at residuals
noiseAmp        = 0.02; %m
refVal          = 0.99;%30%
elipseKPICell   = {'ToID','Ellipse-size-(*RefArea)', ['USP-percPnts-in-' num2str(refVal*100) 'PercDist']};

%get unique NP coordinates
% %sort on timestamps
% dataMtxUnique   = sortrows(vCUS_data, 1);
% %remove repeating NPs
% toCoords        = dataMtxUnique(:, 3:4);
[~,indx,~]      = unique(vCUS_data, 'rows');
if length(indx) < 3
    dataMtxUnique 	= vCUS_data;
else
    dataMtxUnique 	= vCUS_data(indx, :);
end

%matrix holds number of usp points inside ellipse per TO for 21 ellipse
%sizes in steps of 10% increases
%columns TOid, ellipse scale, ellipse area, n USP points in ellipse, n vCUS points in ellipse,
%perc UPS of vCUS reference - in box, perc usp of total USP, perc vCUS in ellipse of total vCUS
%ellipseCell = {};

total_USP   = size(USP_coord, 1);
total_vCUS  = size(vCUS_data, 1);

fdlg = warndlg('Computing ellipses, please wait...');

bA = boxAdjust;

uspInside           = zeros(nBoxs, 1);
vCUSInsideBox       = zeros(nBoxs, 1);
singleTube          = false;
%  parpool;
for iBx = 1:nBoxs
    thisTO_id	= toIds{iBx};
    thisBox     = boxes(:,:,iBx);
    
    %redo box without tollerance -dumb but necessary
    thisBoxX    = thisBox(:,1);
    thisBoxY    = thisBox(:,2);
    smallBoxX   = [thisBoxX(1)+bA; thisBoxX(2)+bA; thisBoxX(3)-bA; thisBoxX(4)-bA; thisBoxX(5)+bA];
    smallBoxY   = [thisBoxY(1)+bA; thisBoxY(2)-bA; thisBoxY(3)-bA; thisBoxY(4)+bA; thisBoxY(5)+bA];
    smallBox    = [smallBoxX, smallBoxY];
    side1 = sqrt(sum((smallBox(1,:) - smallBox(2,:)) .^ 2, 2));
    side2 = sqrt(sum((smallBox(2,:) - smallBox(3,:)) .^ 2, 2));
    if side2<side1
        side = side2;
        side2 = side1;
        side1 = side;
    end
    
    %check if single TO and the object is somewhat simetric 
    if side2/side1 < 2 && nBoxs == 1
        singleTube = true;
    end

    %get TO points (all vCUS points in the bounding box)
    vCUSinBox           = inpolygon(vCUS_data(:,1),vCUS_data(:,2),smallBox(:,1),smallBox(:,2));
    vCUSPerc            = sum(vCUSinBox)/total_vCUS;
    vCUSInsideBox(iBx)  = vCUSPerc*100;
    dataInBox           = vCUS_data(vCUSinBox, :);
    if isempty(vCUSinBox)
        disp(['No vCUS points in: ' thisTO_id]);
        continue;
    end
        
    %get NP coordinates for this TO
    [~,indx,~]      = unique(dataInBox, 'rows');
    
    %***exception if only 2 TO NPs: at least 3 datapoints to get bounding ellipse
    if length(indx) < 3
        if size(dataInBox, 1) > 2
            thisTO 	= dataInBox;
        else
            thisTO 	= [dataInBox; dataInBox; dataInBox];
        end
    else% if at least 3 data points
        thisTO 	= dataInBox(indx, :);
    end
    %***
    
    %###############
    %vCUS coordinates for this TO & vCUS ref ellipse
    vcusX           = thisTO(:,1);
    vcusY           = thisTO(:,2);
    
    %add noise if TO is very flat
    coefficients = polyfit(vcusX, vcusY, 1);%fit line
    yFit = polyval(coefficients, vcusX);
    residualsSum = sum(abs(yFit - vcusY));%check residuals
    if residualsSum < lineResiduals
        %add noise 0.02
        nNoise      = size(thisTO,1);%add noise - adds variablity to bounding ellipse
        vcusX       = thisTO(:,1) + ( -noiseAmp + (2*noiseAmp).*rand(nNoise,1));
        vcusY       = thisTO(:,2) + ( -noiseAmp + (2*noiseAmp).*rand(nNoise,1));
    end
    
    %get TO bounding ellipse
    vCUS_TOdata                 = [vcusX, vcusY];
    [hanOut_cus, polyshp_vCUS]	= MinVolEllipse([vcusX, vcusY]', 0.01, 300, thisTO_id);
    close(hanOut_cus);
    polygCoordsCus	= polyshp_vCUS.Vertices;
    polygCoordsCus	= [polygCoordsCus;polygCoordsCus(1, :)];%#ok<AGROW> %close the polygon
    ellipAreaCus 	= polyarea(polygCoordsCus(:,1),polygCoordsCus(:,2));
    
    %get TO centroid
    [xBoxCenter, yBoxCenter]    = centroid(polyshp_vCUS);
    centro                      = [xBoxCenter, yBoxCenter] ;
    %###############
    
    %distances from centroid to each usp point
    uspToCentro = sqrt(sum((USP_coord - centro) .^ 2, 2));
    
    %if there is a single TO with somewhat simetric shape take the ellipse
    %for the of USP points up to 99% of the distance distribution
            %99% of the points does not work because the rest of 1% can be at
            %the same distance if there are repeats
    if singleTube
        elipseKPICell   = {'ToID','Ellipse-size-(m^2)', ['USP-percPnts-in-' num2str(refVal*100) 'PercDist']};
        newRefVal       = refVal;
        
        [ellipArea, hanOut, percUSPDots, uspInElip, inUSP] = getDataSubsetEllipse(newRefVal, uspToCentro, USP_coord, USP_coord, thisTO_id);
        
        %store and show KPIs
        kpiVal          = ellipArea;
        thisKPI         = { toIds{iBx}, kpiVal, percUSPDots * 100};
        elipseKPICell   = cat(1, elipseKPICell, thisKPI);
        uspInside(iBx)  = uspInElip;
    
        plotEllipse(hanOut, vCUS_TOdata, polygCoordsCus, inUSP, USP_coord)

    else

        %get the ellipse for the proportion of usp points closest to centroid representing 99% of the
        %vCUS proportion for a given TO
        %##############
        
        %first get all the closest USP points representing the same
        %percentage as the vCUS points 
        nEllipUSP100            = ceil(vCUSPerc * total_USP); 
        [dist100, sortIndxUSP]	= sort(uspToCentro);
        dataPSort               = USP_coord(sortIndxUSP, :);
        newUSP100               = dataPSort(1:nEllipUSP100, :);
        dist100                 = dist100(1:nEllipUSP100, :);
        
        %next: take the points up to the 99% of the distances of those nEllipUSP100 points
        %kpi is the area of the ellipse for the points up to the 99% of the
        %distance distribution
        [ellipArea, hanOut, percUSP, uspInElip, inUSP] = getDataSubsetEllipse(refVal, dist100, newUSP100, USP_coord, thisTO_id);
        
        %plot and store data
        areaElip    = ellipArea/ellipAreaCus;
        uspVsVcus   = percUSP/vCUSPerc;
        uspPerc     = percUSP;
        refVcus     = vCUSPerc;%cell2mat(thisBoxCell(:, 7));
        
        newRefVal       = uspVsVcus;
        kpiVal          = areaElip;
        thisKPI         = { toIds{iBx}, kpiVal, newRefVal * 100};
        elipseKPICell   = cat(1, elipseKPICell, thisKPI);
        uspInside(iBx)  = uspInElip;
    
        %plot KPI
        figure('Name', ['TO id: ' toIds{iBx}]);
        hold on;
        plot(areaElip, 100*uspVsVcus, '*r', 'MarkerSize',12);
        plot(areaElip, 100*uspPerc, 'ob', 'MarkerSize',14);
        plot(areaElip, 100*refVcus, '*k');
        plot(kpiVal, newRefVal * 100, 'og', 'MarkerFaceColor', 'g');
        xlabel('x (m)');
        ylabel('y (m)');
        legend({'USP-perc / vCUS-perc', 'USP of total', 'vCUS of total', 'KPI'}, 'Location', 'best');
        ylabel ('Refl. points in ellipse (%)');
        xlabel('Ellipse area (% vCUS TO)');
        hold off;
   
        plotEllipse(hanOut, vCUS_TOdata, polygCoordsCus, inUSP, USP_coord)
                
    end%if noNoise

    
end%for

papoObj = gcp('nocreate');
if ~isempty(papoObj)
    delete(papoObj);
end

uspOutside = 100 * (total_USP - sum(uspInside))/total_USP;

%ellipseCell = cat(1, ellipseCell, thisBoxCell);
close(fdlg);
end%func


function setAxScale(src,event)
   ax = gca;
   scaleV = ax.DataAspectRatio;
   if  any(scaleV ~= 1)
       axis equal
   else
       axis normal
   end
end

function [newUSP] = getDataSubset(firstNDatap, dataP, distances)
        
[~, sortIndxUSP]	= sort(distances);
dataPSort           = dataP(sortIndxUSP, :);
uspEllip         	= dataPSort(1:firstNDatap, :);
[~,uniUSP, ~]       = unique(uspEllip, 'rows');
newUSP              = uspEllip( uniUSP, :);

end
    
function plotEllipse(hanOut, vCUS_TOdata, polygCoordsCus, inUSP,USP_coord)

%plot final ellipse and data
%tempFig = figure('Name', ['Final ellipse TO ' toIds{iBx}]);
%hold on;
set(0, 'CurrentFigure', hanOut);
hold on;
plot(USP_coord(inUSP,1),USP_coord(inUSP,2), 'og');
plot(vCUS_TOdata(:,1),vCUS_TOdata(:,2), '*c');
plot(polygCoordsCus(:,1), polygCoordsCus(:,2), '--m');

legend({'USP for ellipse','USP ellipse', 'Actual USP in Ellipse' ...
        'vCUS in ellipse' , 'vCUS ellipse'}, 'Location', 'best');
xlabel('x (m)');
ylabel('y (m)');
axis equal
ButtonH=uicontrol('Parent',hanOut,'Style','pushbutton','String','Toggle axis','Units','normalized','Position',[0.9 0.95 0.1 0.05],'Visible','on');
ButtonH.Callback = @setAxScale;

hold off
end

function [elipArea, hanOut, percUSPDots, uspInElip, inUSP] = getDataSubsetEllipse(refVal, distances, data2DSubset, allData, thisTO_id)
        
pUspDist        = prctile(distances, refVal * 100);
indxDistcs      = distances < pUspDist;
usp99           = data2DSubset(indxDistcs, :);

%get ellipse for 99 perc of USP points
[~,uniUSP, ~]       = unique(usp99, 'rows');
newUSP              = usp99( uniUSP, :);
[hanOut, polyshp]   = MinVolEllipse(newUSP', 0.001, 300, thisTO_id);

polygCoords         = polyshp.Vertices;
polygCoords         = [polygCoords;polygCoords(1, :)]; %close the polygon
elipArea            = polyarea(polygCoords(:,1),polygCoords(:,2));%ellipse area

%usp in ellip
inUSP               = inpolygon(allData(:,1),allData(:,2),polygCoords(:,1),polygCoords(:,2));
uspInElip           = sum(inUSP);
total_USP           = size(allData, 1);
percUSPDots         = uspInElip/total_USP;

end
    