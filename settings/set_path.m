function set_path(root,dyn_mode,con_mode,cost_mode)
%SET_PATH Adds the proper locations to the path
%   Detailed explanation goes here

% Move to root directory: /path/intersection_management
cd(root);

% Add Desired Paths
addpath '/Library/gurobi912/mac64/matlab'; % For mac
% addpath 'C:\gurobi950\win64\matlab'; % For Thinkstation (Windows)
% addpath '/opt/gurobi951/linux64/matlab'; % For Thinkstation (Ubuntu)
folders = {'controllers','datastore','dynamics','helpers','settings'};
for ff = 1:length(folders)
    addpath(folders{ff})
    d = dir(folders{ff});
    isub = [d(:).isdir]; %# returns logical vector
    subFolders = {d(isub).name}';
    subFolders(ismember(subFolders,{'.','..'})) = [];
    for fff = 1:length(subFolders)
        if strcmp(subFolders{fff},dyn_mode) || strcmp(subFolders{fff},con_mode) || strcmp(subFolders{fff},cost_mode)
            addpath(strcat(folders{ff},'/',subFolders{fff}));
            d = dir(subFolders{fff});
            isubsub = [d(:).isdir]; %# returns logical vector
            subsubFolders = {d(isubsub).name}';
            subsubFolders(ismember(subsubFolders,{'.','..'})) = [];
            for ffff = 1:length(subsubFolders)
                addpath(strcat(folders{ff},'/',subFolders{fff},'/',subsubFolders{ffff}));
            end
        end
    end
end

end

