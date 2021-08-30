% Vehicle Parameters
Lr        = 1;
Lf        = 1;
M         = 1;

% Road Parameters
lw        = 3.0;
hlw       = lw / 2;
ien       = lw + 0.5;
far       = 15.0;

nLanes    = 1; 
nWay      = 4;
nControls = 2;

vmax      = 10;           % 10 mps ~ 25 mph
wmax      = pi / 3;       % 60 deg per second
umax      = [vmax; wmax];

amax      = 2 * 9.81;
umax      = [inf; amax];