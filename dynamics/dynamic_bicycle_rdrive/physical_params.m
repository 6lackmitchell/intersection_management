% Vehicle Parameters
Lr        = 1;
Lf        = 1;
M         = 1;

% Road Parameters
lw        = 3.0;
hlw       = lw / 2;
ien       = lw + 1.0;
far       = 25.0;
SL        = 12.0; % Speed Limit

nLanes    = 1; 
nWay      = 4;
nControls = 2;

vmax      = 10;           % 10 mps ~ 25 mph
wmax      = pi / 3;       % 60 deg per second
umax      = [vmax; wmax];

wmax      = 4*pi;
amax      = 2 * 9.81;
umax      = [wmax; amax];