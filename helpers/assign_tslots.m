function [tSlots] = assign_tslots(t,x,tSlots)
%assign_tslots - Computes optimal departure sequence
% This function seeks to minimize the total delay time for the set of
% vehicles traveling through the intersection.
%
% Syntax:  [departures] = assign_tslots(x)
%
% Inputs:
%    t:        current time in sec -- float
%    x:        current state vector -- ROW vector
%    
% Outputs:
%    departures: list of ordered set of agents awaiting departure
%
% Example: 
%    [departures] = assign_tslots(x);
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% Author: Mitchell Black
% Email: mblackjr@umich.edu
% Website: http://www.blackmitchell.com
% July 2021; Last revision: 13-July-2021
%------------- BEGIN CODE --------------

T_min = 4;
crossing_time = 6;

% intersection_map = struct('N',[[-1.5, 3.0];[ 1.5, 3.0]],...
%                           'S',[[ 1.5,-3.0];[-1.5,-3.0]],...
%                           'E',[[ 3.0, 1.5];[ 3.0,-1.5]],...
%                           'W',[[-3.0,-1.5];[-3.0, 1.5]]);

% Iterate over all agents
for aa = 1:size(x,1)
    
    if within_assignment_zone(x(aa,:)) && tSlots(aa,1) == inf
    
%         % Get Agent's Route
%         origin      = intersection_map.(routes(aa,1))(1,:);
%         destination = intersection_map.(routes(aa,2))(2,:);

        if min(tSlots(:,1)) == inf
            tSlots(aa,1) = t + T_min;
        else
            tSlots(aa,1) = max(tSlots(~isinf(tSlots(:,2)),2));
        end
        
        tSlots(aa,2) = tSlots(aa,1) + crossing_time;
        
    elseif within_assignment_zone(x(aa,:)) && aa > 3
        tSlots(aa,1) = 0.0;
        tSlots(aa,2) = 0.0;
        
    end
        
end


end

function [flag] = within_assignment_zone(x)
%within_assignment_zone - Determines if agent is within assignment zone
%
% Syntax:  flag = within_assignment_zone(x)
%
% Inputs:
%    x:        current state vector for a given agent -- ROW vector
%    
% Outputs:
%    flag: boolean (true if within zone, false otherwise)
%
% Example: 
%    flag = within_assignment_zone(x);
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% Author: Mitchell Black
% Email: mblackjr@umich.edu
% Website: http://www.blackmitchell.com
% July 2021; Last revision: 19-July-2021
%------------- BEGIN CODE --------------

Rzone = 50;
if norm(x(1:2)) < Rzone
    flag = true;
else
    flag = false;
end




end


%------------- END OF CODE --------------
