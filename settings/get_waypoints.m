function [waypoints] = get_waypoints(N,x0,code)
%GET_WAYPOINTS Computes waypoints for an intersection crossing. 
%   Computes waypoints for an intersection crossing depending on
%the starting position (x0), what kind of turn (code: 0 = right, 1 =
%straight, 2 = left), and resolution (N).

% Define a few constants
lane_width = 3; % m

% Straight through intersection -- no additional waypoints necessary
waypoints = [];
theta0    = pi / (2*(N-1)) * ((1:N) - 1);
R         = 3/2 * lane_width;    
 
if code == 2
    theta = theta0;
    waypoints = x0' + R * [cos(theta) - 1; sin(theta)]';
end
    
end

