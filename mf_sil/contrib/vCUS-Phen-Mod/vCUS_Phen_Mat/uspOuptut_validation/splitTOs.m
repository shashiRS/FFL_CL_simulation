function [allToIds, newTOmatx] = splitTOs(dataMtx, toIds, nTos)
hTos = figure('units', 'normalized','position', [0.1,0.5,0.3,0.4]); hold on;
prompt = {};
definput = {};
toDataStru = struct;
for iTo = 1:nTos
    thisTOid    = toIds(iTo);
    thisToMax   = dataMtx(dataMtx(:,7) == thisTOid, 3:4);
    
    toDataStru(iTo).coords = thisToMax;
    plot(thisToMax(:,1), thisToMax(:,2), '.');
    prompt = cat(2, prompt, ['TO' num2str(thisTOid)]);
    definput = cat(2, definput, num2str(1));
end
if nTos > 1
    legend(prompt, 'Location', 'Best');
    dlgtitle = 'Clusters/TO';
    dims = [1 40];
    answer = inputdlg(prompt,dlgtitle,dims,definput);
    %str2double(answer{1});
else
    answer = {'1'};
end
close(hTos);

newTOmatx = [];
allToIds = {};
for iTo = 1:nTos
    nClusts = str2double(answer{iTo});
    thisTOid = toIds(iTo);
    thisData = toDataStru(iTo).coords;
    if nClusts > 1
        idx = clusterdata(thisData,'Maxclust', nClusts);
        %figure;hold on
        for iClu = 1:nClusts
            newTOids        = {};
            %plot(thisData(idx == iClu,1), thisData(idx == iClu,1), '.');            
            thisCluData     = thisData(idx == iClu, :);
            newTOmatx       = [newTOmatx; thisCluData]; %#ok<AGROW>
            newTO           = [num2str(thisTOid) '-' num2str(iClu)];
            [newTOids{1:size(thisCluData, 1),1}]  = deal(num2str(newTO));
            allToIds        = cat(1, allToIds, newTOids);
        end
        
    else
        newTOids    = {};
        newTOmatx   = [newTOmatx; thisData]; %#ok<AGROW>
        [newTOids{1:size(thisData, 1), 1}]  = deal(num2str(thisTOid));
        allToIds    = cat(1, allToIds, newTOids);
    end
end
end

            