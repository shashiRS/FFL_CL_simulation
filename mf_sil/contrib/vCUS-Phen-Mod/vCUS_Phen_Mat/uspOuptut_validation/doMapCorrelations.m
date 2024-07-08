
function [reflPntCorr] = doMapCorrelations(grndTruthData, uspData, mapResolution, showWhat)

NPs_xy      = grndTruthData;
USP_xy      = uspData;
minMaxY     = [floor(min([NPs_xy(:, 2) ; USP_xy(:,2) ])), ...
                            ceil(max([NPs_xy(:, 2) ; USP_xy(:, 2) ]))];
minMaxX     = [floor(min([NPs_xy(:, 1) ; USP_xy(:,1) ])), ...
                            ceil(max([NPs_xy(:, 1) ; USP_xy(:, 1) ]))];
nBinsY       = ceil((minMaxY(2) - minMaxY(1)) / mapResolution);
nBinsX       = ceil((minMaxX(2) - minMaxX(1))/mapResolution);

%make correlations of 2D density plots for reflection points
[mapUSP_pnts] = probsMap_2DPoints(minMaxX, minMaxY, nBinsX, nBinsY, ...
                                       mapResolution, USP_xy, [showWhat ' USP points']);
[mapNps]    = probsMap_2DPoints(minMaxX, minMaxY, nBinsX, nBinsY, ...
                                       mapResolution, NPs_xy, [showWhat ' vCUS Points']);
                                            
reflPntCorr = corr2(mapNps,mapUSP_pnts);

end%func


function [N_norm] = probsMap_2DPoints(minMaxX, minMaxY, nBinsX, nBinsY,...  
                                                 mapReso, dataMatx, whatStrg)

x = dataMatx(:,1);
y = dataMatx(:,2);

% Bin the data:
ptsX = linspace(minMaxX(1), minMaxX(2), nBinsX+1);
ptsY = linspace(minMaxY(1), minMaxY(2), nBinsY+1);

N = histcounts2(y(:), x(:), ptsY, ptsX);
N_norm = N/sum(N(:));

xlimits = [ptsX(1)- mapReso/2; ptsX(end) + mapReso/2];
ylimits = [ptsY(1)- mapReso/2; ptsY(end) + mapReso/2];
figure('Name',whatStrg)

% Plot scattered data (for comparison):
axsMap(1) = subplot(1, 2, 1);
scatter(x, y, 'r.');
axis equal;
xlabel ('X (m)');
ylabel ('Y (m)');
set(gca, 'XLim', xlimits, 'YLim', ylimits);

% Plot heatmap:
axsMap(2) = subplot(1, 2, 2);
imagesc(ptsX, ptsY, N_norm);
axis equal;
xlabel ('X (m)');
ylabel ('Y (m)');
set(gca, 'XLim', xlimits, 'YLim', ylimits, 'YDir', 'normal');
c = colorbar('eastoutside');
c.Label.String = 'Normalized density';

linkaxes(axsMap,'xy');
end%func