function [priority] = get_priority_metric(t,x,settings)
%get_priority_metric - Computes priority metric for QP controller
%
% Syntax:  [priority] = get_priority_metric(t,x,settings)
%
% Inputs:
%    t: time (float)
%    x: state (vector)
%    settings: (struct) contains relevant settings for different cases
%
% Outputs:
%    priority: (vector) weighting params for QP cost function
%
% Other m-files required: others here
% Subfunctions: others
% MAT-files required: MAT files are generated for initial conditions
%
% Author: Mitchell Black
% Email: mblackjr@umich.edu
% Website: http://www.blackmitchell.com
% Jan 2022; Last revision: 31-Jan-2022
%------------- BEGIN CODE --------------% Load control params
switch settings.metric
    case 'None'
        % No priority -- all equal
        settings.power = settings.Na;
        idxLF = 1:1:settings.Na;
        LF    = ones(settings.Na,1);

    case 'FCFS'
        % First Come First Served
        if t > settings.dt
            % Static priority, nothing changes after first assignment
            priority = settings.prior;
            return
        end
        
        LF = 1/2*vecnorm(x(:,1:2)').^2;
        [~,idxLF] = sort(LF,'descend');

    case 'FCFS_V'
        % First Come First Served
        if t > settings.dt
            % Static priority, nothing changes after first assignment
            priority = settings.prior;
            return
        end
        
        LF = (1/2*vecnorm(x(:,1:2)').^2./vecnorm(settings.xdot'));
        [~,idxLF] = sort(LF,'descend');

    case 'HighDev'
        % High Deviation from Nominal Trajectory gets High Priority
        LF = 1/2*vecnorm(settings.xdes' - x(:,1:2)').^2 + 1/2*vecnorm(settings.xdesdot' - settings.xdot').^2;
        [~,idxLF] = sort(LF,'ascend');

    case 'LowDev'
        % Low Deviation from Nominal Trajectory gets High Priority
        LF = 1/2*vecnorm(settings.xdes' - x(:,1:2)').^2 + 1/2*vecnorm(settings.xdesdot' - settings.xdot').^2;
        [~,idxLF] = sort(LF,'descend');

    case 'HighEffort'
        % High Required Effort for safe control gets High Priority
        LF = 1/2*sum(settings.Lgh.^2);
        [~,idxLF] = sort(LF,'ascend');

    case 'LowEffort'
        % Low Required Effort for safe control gets High Priority
        LF = 1/2*sum(settings.Lgh.^2);
        [~,idxLF] = sort(LF,'descend');

        % Not using these right now
%     case 'HighEffortScaled'
%         % High Required (scaled) Effort for safe control gets High Priority
%         LF = 1/2*sum(settings.Lgh.^2);
%         [~,idxLF] = sort(LF,'ascend');
% 
%     case 'LowEffortScaled'
%         % Low Required (scaled) Effort for safe control gets High Priority
%         LF = 1/2*sum(settings.Lgh.^2);
%         [~,idxLF] = sort(LF,'descend');
end

priority = zeros(settings.Na,1);
for aa = 1:settings.Na
    priority(idxLF(aa)) = settings.power^(aa-1);
%     priority(idxLF(aa)) = settings.power*LF(aa)^2/(sum(LF.^2));
end


end

