% Vehicle Parameters
Lr        = 1;
Lf        = 1;
M         = 1;
G         = 9.81; % m/s^2

% Road Parameters
lw        = 3.0;
hlw       = lw / 2;
ien       = lw + 1.0;
far       = 25.0;
SL        = 10.0; % Speed Limit

nLanes    = 1; 
nWay      = 4;
nControls = 2;

wmax      = 4*pi;
amax      = 2 * G;
amax      = 1 * G;
umax      = [wmax; amax];