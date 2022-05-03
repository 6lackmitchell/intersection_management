function [xd,yd,G,reached] = get_desired_position(t,x,aa)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

% Globally relevant details here
lw       = 3;
L        = 50;
Nturning = 0;
reached  = 0;

% Variables specific to intersection setup
xd0       = [lw/2; -lw/2; -L; L];
xdf       = [lw/2; -lw/2; -L; L];
yd0       = [L; -L; lw/2; -lw/2];
ydf       = [L; -L; lw/2; -lw/2];
goal      = [lw/2 lw; -lw/2 -lw; -lw lw/2; lw -lw/2];
xdf_turn  = [-L;   -lw/2; -L; L];
ydf_turn  = [lw/2; -L; lw/2; -lw/2];
goal_turn = [-lw lw/2; -lw/2 -lw; -lw lw/2;lw -lw/2];
threshold = 0.5;
gain      = 0.1;

if norm(x(1:2)) < lw
    gain = 10;
end


if aa <= Nturning
    
    if x(2) < 0
        xd = xd0(aa);
    elseif x(2) < 1
        xd = xd0(aa) - x(2);
    else
        xd = xdf_turn(aa);
    end

    yd = ydf_turn(aa);

    if norm(x(1:2)-goal_turn(aa,:)) < threshold
        reached = 1;
    end

else
    xd = xdf(aa);
    yd = ydf(aa);

    if norm(x(1:2)-goal(aa,:)) < threshold
        reached = 1;
    end
end

G = gain*eye(4);

end