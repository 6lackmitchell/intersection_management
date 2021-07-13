function [departures] = assign_tslots(t,x,tSlots,routes)
%optimize_departures - Computes optimal departure sequence
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

Na = size(x,1);
intersection_map = struct('N',[[-1.5, 3.0];[ 1.5, 3.0]],...
                          'S',[[ 1.5,-3.0];[-1.5,-3.0]],...
                          'E',[[ 3.0, 1.5];[ 3.0,-1.5]],...
                          'W',[[-3.0,-1.5];[-3.0, 1.5]]);

for aa = 1:Na
    
    % Get Agent's Route
    origin      = intersection_map.(routes(aa,1))(1,:);
    destination = intersection_map.(routes(aa,2))(2,:);
    
    if min(tSlots(:,1)) == inf
        tSlots(aa,1) = 
end

end




%------------- END OF CODE --------------
