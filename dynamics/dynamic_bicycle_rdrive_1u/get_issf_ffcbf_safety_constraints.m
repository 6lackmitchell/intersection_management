function [A,b,params] = get_issf_ffcbf_safety_constraints(t,x,settings)
%GET_SAFETY_CONSTRAINTS This is where the safety measures are considered
%   The relevant CBFs are taken into account here.
Lr = 1.0;
Lf = 1.0;

% Unpack PCCA Settings
% agent = settings.AAA;
Na    = settings.Na;

% Add additional settings
settings.('Nu')   = 1;
settings.('Lr')   = Lr;
settings.('Lf')   = Lf;
settings.('lw')   = 3.0;

% A = []; b = []; h = [];
v00 = zeros(Na*settings.Nu,1);
h00 = inf*ones(Na,1);

nConstraints = 9;
A = zeros(nConstraints*Na,settings.Nu*Na);
b = zeros(nConstraints*Na,1);
h = zeros(nConstraints*Na,1);

for aa = 1:Na
    
    settings.('aa') = aa;
    
    [A1,b1,h1]         = get_speed_constraints(t,x,settings);
    [A2,b2,h2]         = get_road_constraints(t,x,settings);
    [A4,b4,h4,v00,h00] = get_collision_avoidance_constraints(t,x,settings);

%     if aa < 4
%         [A3,b3,h3] = get_intersection_constraints(t,x,settings);
%     else
%         A3 = []; b3 = []; h3 = 100;
%     end

    row_idx = nConstraints*(aa-1)+1:nConstraints*(aa-1)+nConstraints;
    A(row_idx,:) = [A1; A2; A4];
    b(row_idx)   = [b1; b2; b4];
    h(row_idx)   = [h1; h2; h4];

%     A = [A; A1; A2; A4];
%     b = [b; b1; b2; b4];
%     h = [h; h1; h2; h4];
    
end

% Organize parameters to be returned
params = struct('h',   h,   ...
                'v00', v00, ...
                'h00', h00);

end

function [A,b,h] = get_road_constraints(t,x,settings)
aa = settings.aa;
lw = settings.lw;
Lr = settings.Lr;

xx = x(aa,:);
A = zeros(2,4); b = ones(2,1);

% vx and vy
vx = xx(4)*(cos(xx(3)) - sin(xx(3))*tan(xx(5)));
vy = xx(4)*(sin(xx(3)) + cos(xx(3))*tan(xx(5)));

% ax and ay
ax_unc = -xx(4)^2/Lr*tan(xx(5))*(sin(xx(3)) + cos(xx(3))*tan(xx(5)));
ay_unc =  xx(4)^2/Lr*tan(xx(5))*(cos(xx(3)) - sin(xx(3))*tan(xx(5)));
ax_con = [cos(xx(3))-sin(xx(3))*tan(xx(5))]';
ay_con = [sin(xx(3))+cos(xx(3))*tan(xx(5))]';

% l1 = 5.0;
% l0 = 10.0;

l1 = 5.0;
l0 = l1^2 / 4;
settings.('l1') = l1;
settings.('l0') = l0;
h = 999*ones(2,1);


if xx(1) < lw && xx(1) > -lw && xx(2) < lw && xx(2) > -lw
    return
elseif xx(1) > 0 && xx(1) < lw
    settings.('vx') = vx;
    settings.('ax_unc') = ax_unc;
    settings.('ax_con') = ax_con;
    [A,b,h] = get_SENEroad_constraint(t,xx,settings);
elseif xx(2) > 0 && xx(2) < lw
    settings.('vy') = vy;
    settings.('ay_unc') = ay_unc;
    settings.('ay_con') = ay_con;
    [A,b,h] = get_ENWNroad_constraint(t,xx,settings);
elseif xx(2) < 0 && xx(2) > -lw
    settings.('vy') = vy;
    settings.('ay_unc') = ay_unc;
    settings.('ay_con') = ay_con;
    [A,b,h] = get_ESWSroad_constraint(t,xx,settings);
elseif xx(1) < 0 && xx(1) > -lw
    settings.('vx') = vx;
    settings.('ax_unc') = ax_unc;
    settings.('ax_con') = ax_con;
    [A,b,h] = get_NWSWroad_constraint(t,xx,settings);
end

end

function [A,b,h] = get_SENEroad_constraint(t,x,settings)
aa = settings.aa;
Nu = settings.Nu;
Na = settings.Na;
% Lr = settings.Lr;
% Lf = settings.Lf;
lw = settings.lw;
l0 = settings.l0;
l1 = settings.l1;
vx = settings.vx;
ax_unc = settings.ax_unc;
ax_con = settings.ax_con;
R = 1;

idx = aa;

h1       = (lw-R) - x(1);
Lfh1     = -vx;
Lf2h1    = -ax_unc;
LgLf1    = -ax_con;

h2       = x(1) - R;
Lfh2     = vx;
Lf2h2    = ax_unc;
LgLf2    = ax_con;

A        = zeros(2,Nu*Na);
b        = zeros(2,1);

A(1,idx) = -LgLf1;
A(2,idx) = -LgLf2;
b(1)     = Lf2h1 + l1*Lfh1 + l0*h1;
b(2)     = Lf2h2 + l1*Lfh2 + l0*h2;
h        = [h1; h2];

A = round(A,12);
b = round(b,12);

end

function [A,b,h] = get_ESWSroad_constraint(t,x,settings)
aa = settings.aa;
Nu = settings.Nu;
Na = settings.Na;
% Lr = settings.Lr;
% Lf = settings.Lf;
lw = settings.lw;
l0 = settings.l0;
l1 = settings.l1;
vy = settings.vy;
ay_unc = settings.ay_unc;
ay_con = settings.ay_con;
R = 1;

idx = aa;

h1       = (lw-R) + x(2);
Lfh1     = vy;
Lf2h1    = ay_unc;
LgLf1    = ay_con;

h2       = -x(2) - R;
Lfh2     = -vy;
Lf2h2    = -ay_unc;
LgLf2    = -ay_con;

A        = zeros(2,Nu*Na);
b        = zeros(2,1);

A(1,idx) = -LgLf1;
A(2,idx) = -LgLf2;
b(1)     = Lf2h1 + l1*Lfh1 + l0*h1;
b(2)     = Lf2h2 + l1*Lfh2 + l0*h2;
h        = [h1; h2];

A = round(A,12);
b = round(b,12);

end

function [A,b,h] = get_ENWNroad_constraint(t,x,settings)
aa = settings.aa;
Nu = settings.Nu;
Na = settings.Na;
% Lr = settings.Lr;
% Lf = settings.Lf;
lw = settings.lw;
l0 = settings.l0;
l1 = settings.l1;
vy = settings.vy;
ay_unc = settings.ay_unc;
ay_con = settings.ay_con;
R = 1;

idx = aa;

h1       = (lw-R) - x(2);
Lfh1     = -vy;
Lf2h1    = -ay_unc;
LgLf1    = -ay_con;

h2       = x(2) - R;
Lfh2     = vy;
Lf2h2    = ay_unc;
LgLf2    = ay_con;

A        = zeros(2,Nu*Na);
b        = zeros(2,1);

A(1,idx) = -LgLf1;
A(2,idx) = -LgLf2;
b(1)     = Lf2h1 + l1*Lfh1 + l0*h1;
b(2)     = Lf2h2 + l1*Lfh2 + l0*h2;
h        = [h1; h2];

A = round(A,12);
b = round(b,12);

end

function [A,b,h] = get_NWSWroad_constraint(t,x,settings)
aa = settings.aa;
Nu = settings.Nu;
Na = settings.Na;
% Lr = settings.Lr;
% Lf = settings.Lf;
lw = settings.lw;
l0 = settings.l0;
l1 = settings.l1;
vx = settings.vx;
ax_unc = settings.ax_unc;
ax_con = settings.ax_con;
R = 1;

idx = aa;

h1       = (lw-R) + x(1);
Lfh1     = vx;
Lf2h1    = ax_unc;
LgLf1    = ax_con;

h2       = -x(1) - R;
Lfh2     = -vx;
Lf2h2    = -ax_unc;
LgLf2    = -ax_con;

A        = zeros(2,Nu*Na);
b        = zeros(2,1);

A(1,idx) = -LgLf1;
A(2,idx) = -LgLf2;
b(1)     = Lf2h1 + l1*Lfh1 + l0*h1;
b(2)     = Lf2h2 + l1*Lfh2 + l0*h2;
h        = [h1; h2];

A = round(A,12);
b = round(b,12);

end

function [A,b,h] = get_speed_constraints(t,x,settings)
aa = settings.aa;
Nu = settings.Nu;
Na = settings.Na;

idx    = aa;
xx     = x(aa,:);

h      = settings.SL - xx(4);
Lfh    = 0;
Lgh    = [-1];

A      = zeros(1,Nu*Na);
A(idx) = -Lgh;
b      = Lfh + 0.25*h^3;

A = round(A,12);
b = round(b,12);

end

function [A,b,H,v00,h00] = get_collision_avoidance_constraints(t,x,settings)
Nu    = settings.Nu;
Na    = settings.Na;
% Nn    = settings.Nn;
Lr    = settings.Lr;
tmax  = settings.lookahead;
% AA    = settings.aa;
% AAA   = settings.AAA;
% uNom  = settings.uNom;

% sl = 1.0;
sw = 1.0;

% A = []; b = []; H = [];
Nc  = factorial(Na-1);
A   = zeros(Nc,Na*Nu);
b   = zeros(Nc,1);
H   = zeros(Nc,1);
v00 = zeros(Nu*Na,1);
h00 = zeros(Na,1);
cc  = 1;

% Loop through every scheduled agent for PCCA
for aa = 1:Na
    
%     Aw = []; bw = []; hw = [];
    nc = Na-aa;
    Aw = zeros(nc,Na*Nu);
    bw = zeros(nc,1);
    hw = zeros(nc,1);
    dd = 1;
    
    % Loop through all other agents for interagent completeness
    for ii = aa+1:Na
        
        xa = x(aa,:);
        xi = x(ii,:);
        
        idx_aa = aa;
        idx_ii = ii;
              
        % dx and dy
        dx  = xa(1) - xi(1);
        dy  = xa(2) - xi(2);
        
        % dvx and dvy
        vax = xa(4)*(cos(xa(3)) - sin(xa(3))*tan(xa(5)));
        vay = xa(4)*(sin(xa(3)) + cos(xa(3))*tan(xa(5)));
        vix = xi(4)*(cos(xi(3)) - sin(xi(3))*tan(xi(5)));
        viy = xi(4)*(sin(xi(3)) + cos(xi(3))*tan(xi(5)));
        dvx = vax - vix;
        dvy = vay - viy;
        
        % Solve for minimizer of h
        kh       = 100.0;
%         tmax     = 1.0;
        eps      = 1e-3;
        tau_star = -(dx*dvx + dy*dvy)/(dvx^2 + dvy^2 + eps);
        Heavy1   = heavyside(tau_star,kh,0);
        Heavy2   = heavyside(tau_star,kh,tmax);
        tau      = tau_star*Heavy1 - (tau_star - tmax)*Heavy2;

        % accelerations -- controlled (con) and uncontrolled (unc)
        axa_unc = -xa(4)^2/Lr*tan(xa(5))*(sin(xa(3)) + cos(xa(3))*tan(xa(5)));
        axi_unc = -xi(4)^2/Lr*tan(xi(5))*(sin(xi(3)) + cos(xi(3))*tan(xi(5)));
        aya_unc =  xa(4)^2/Lr*tan(xa(5))*(cos(xa(3)) - sin(xa(3))*tan(xa(5)));
        ayi_unc =  xi(4)^2/Lr*tan(xi(5))*(cos(xi(3)) - sin(xi(3))*tan(xi(5)));
        axa_con = zeros(1,Na*Nu);
        axi_con = zeros(1,Na*Nu);
        aya_con = zeros(1,Na*Nu);
        ayi_con = zeros(1,Na*Nu);
        axa_con(idx_aa) = cos(xa(3))-sin(xa(3))*tan(xa(5));
        axi_con(idx_ii) = cos(xi(3))-sin(xi(3))*tan(xi(5));
        aya_con(idx_aa) = sin(xa(3))+cos(xa(3))*tan(xa(5));
        ayi_con(idx_ii) = sin(xi(3))+cos(xi(3))*tan(xi(5)); 
        
        % dax and day
        dax_unc = axa_unc - axi_unc;
        day_unc = aya_unc - ayi_unc;
        dax_con = axa_con - axi_con;
        day_con = aya_con - ayi_con;
        
        % taudot
        tau_star_dot_unc = -(dax_unc*(2*dvx*tau_star + dx) + day_unc*(2*dvy*tau_star + dy) + (dvx^2 + dvy^2)) / (dvx^2 + dvy^2 + eps);
        tau_star_dot_con = -(dax_con*(2*dvx*tau_star + dx) + day_con*(2*dvy*tau_star + dy)) / (dvx^2 + dvy^2 + eps);
        Heavy_dot1_unc   = dheavyside(tau_star,kh,0)*tau_star_dot_unc;
        Heavy_dot2_unc   = dheavyside(tau_star,kh,tmax)*tau_star_dot_unc;
        Heavy_dot1_con   = dheavyside(tau_star,kh,0)*tau_star_dot_con;
        Heavy_dot2_con   = dheavyside(tau_star,kh,tmax)*tau_star_dot_con;
        tau_dot_unc      = tau_star_dot_unc*(Heavy1 - Heavy2) + tau_star*(Heavy_dot1_unc - Heavy_dot2_unc);
        tau_dot_con      = tau_star_dot_con*(Heavy1 - Heavy2) + tau_star*(Heavy_dot1_con - Heavy_dot2_con);
        
        % h and hdot (= Lfh + Lgh*u)
        h   = dx^2 + dy^2 + tau^2*(dvx^2 + dvy^2) + 2*tau*(dx*dvx + dy*dvy) - (2*sw)^2;
        Lfh = 2*dx*dvx + 2*dy*dvy + 2*tau*tau_dot_unc*(dvx^2 + dvy^2) + 2*tau^2*(dvx*dax_unc + dvy*day_unc) + 2*tau_dot_unc*(dx*dvx + dy*dvy) + 2*tau*(dvx^2 + dvy^2 + dx*dax_unc + dy*day_unc);
        Lgh = 2*tau*tau_dot_con*(dvx^2 + dvy^2) + 2*tau^2*(dvx*dax_con + dvy*day_con) + 2*tau_dot_con*(dx*dvx + dy*dvy) + 2*tau*(dx*dax_con + dy*day_con);

        % Class K Function
%         kk = 1.5;
        kk = 20;
    
        % Inequalities: Ax <= b
        Aw(dd,:) = -Lgh;
        bw(dd)   = Lfh + kk*h; 
        hw(dd)   = min(h,(dx^2 + dy^2 - (2*sw)^2));

        dd = dd + 1;

    end
    
    A(cc:cc+(nc-1),:) = Aw;
    b(cc:cc+(nc-1))   = bw;
    H(cc:cc+(nc-1))   = hw;

    cc = cc + nc;
    
end

end

function [A,b,h] = get_intersection_constraints(t,x,settings)
aa = settings.aa;
Nu = settings.Nu;
Na = settings.Na;
Lr = settings.Lr;
lw = settings.lw + 1;
tSlots = settings.tSlots;

idx = (-1:0)+aa*Nu;

xx = x(aa,:);
A = []; b = [];
h = 100;

x_intersection = [lw -lw -lw  lw];
y_intersection = [lw  lw -lw -lw];
intersection_empty = sum(inpolygon(x(1:end ~= aa,1),x(1:end ~= aa,2),x_intersection,y_intersection)) == 0;

if t > tSlots(aa,1) || (intersection_empty && ((tSlots(aa,1) - t) < 2.5))
    return
    
elseif xx(1) > 0 && xx(1) < lw && xx(2) < -lw
    h    = -xx(2) - lw;
    Lfh  = -(xx(4)*sin(xx(3)) + xx(4)*cos(xx(3))*tan(xx(5)));
    Lf2h = -(xx(4)^2/Lr * (cos(xx(3))*tan(xx(5)) - sin(xx(3))*tan(xx(5))^2));
    LgLf = -sin(xx(3)) - cos(xx(3))*tan(xx(5));

elseif xx(2) > 0 && xx(2) < lw && xx(1) > lw
    h    = xx(1) - lw;
    Lfh  = xx(4)*cos(xx(3)) - xx(4)*sin(xx(3))*tan(xx(5));
    Lf2h = xx(4)^2/Lr * (sin(xx(3))*tan(xx(5)) - cos(xx(3))*tan(xx(5))^2);
    LgLf = cos(xx(3)) - sin(xx(3))*tan(xx(5));

elseif xx(1) < 0 && xx(1) > -lw && xx(2) > lw
    h    = xx(2) - lw;
    Lfh  = xx(4)*sin(xx(3)) + xx(4)*cos(xx(3))*tan(xx(5));
    Lf2h = xx(4)^2/Lr * (cos(xx(3))*tan(xx(5)) - sin(xx(3))*tan(xx(5))^2);
    LgLf = sin(xx(3)) + cos(xx(3))*tan(xx(5));

elseif xx(2) < 0 && xx(2) > -lw && xx(1) < -lw
    h    = -xx(1) - lw;
    Lfh  = -(xx(4)*cos(xx(3)) - xx(4)*sin(xx(3))*tan(xx(5)));
    Lf2h = -(xx(4)^2/Lr * (sin(xx(3))*tan(xx(5)) - cos(xx(3))*tan(xx(5))^2));
    LgLf = -cos(xx(3)) + sin(xx(3))*tan(xx(5));
else
    disp(t)
    return

end

[A,b] = get_intersection_constraint(idx,h,Lfh,Lf2h,LgLf,Nu,Na);

end

function [A,b] = get_intersection_constraint(idx,h,Lfh,Lf2h,LgLf,Nu,Na)
% l0 = 9.0;
% l0 = 25.0;
% l1 = 10.0;
% 
% % These worked as of 1:53PM Sep 1
% l0 = 5.0;
% l1 = 25.0;

% Experimental
l1 = 2.0;
l0 = l1^2 / 4;

A      = zeros(1,Na*Nu);
A(idx) = -LgLf;
b      = Lf2h + l1*Lfh + l0*h;

A = round(A,12);
b = round(b,12);
end
