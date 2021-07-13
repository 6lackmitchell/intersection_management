function [departures] = optimize_departures(t,x)
%optimize_departures - Computes optimal departure sequence
% This function seeks to minimize the total delay time for the set of
% vehicles traveling through the intersection.
%
% Syntax:  [departures] = optimize_departures(x)
%
% Inputs:
%    t:        current time in sec -- float
%    x:        current state vector -- ROW vector
%    
% Outputs:
%    departures: list of ordered set of agents awaiting departure
%
% Example: 
%    [departures] = optimize_departures(x);
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

for aa = 1:Na
    % Compute nominal/desired departure time (zero acceleration)
    d  = abs(entrance - x(aa,jj));
    v  = sqrt((x(aa,4)*cos(x(aa,3)))^2 + (x(aa,4)*sin(x(aa,3)))^2);
    Td = t + d / v;
    
    
    
    
end

end




%------------- END OF CODE --------------
