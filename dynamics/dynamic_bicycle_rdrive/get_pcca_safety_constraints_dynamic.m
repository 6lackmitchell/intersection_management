function [A,b,h] = get_pcca_safety_constraints_dynamic(t,x,settings)
%GET_SAFETY_CONSTRAINTS This is where the safety measures are considered
%   The relevant CBFs are taken into account here.
Lr = 1.0;
Lf = 1.0;
Na = 4;

% Unpack PCCA Settings
agent  = settings.AAA;
uLast  = settings.uLast;
beta   = atan(Lr/(Lr+Lf)*uLast);

% Add additional settings
settings.('Nu')   = 2;
settings.('Na')   = Na;  
settings.('Lr')   = Lr;
settings.('Lf')   = Lf;
settings.('lw')   = 3.0;
settings.('beta') = beta;

A = []; b = []; h = [];

for aa = 1:Na
    
    if agent >= 4 && aa ~= agent
        continue
    end
    
    settings.('aa') = aa;
    
    [A1,b1,h1] = get_speed_constraints(t,x,settings);
    [A2,b2,h2] = get_road_constraints(t,x,settings);
    A = [A; A1; A2];
    b = [b; b1; b2];
    if aa == agent
        h = [h; h1; h2];
    end

    % Unscheduled vehicles don't consider intersection or interagent safety
    if aa >= 4
        continue
    end

%     [A3,b3] = get_interagent_constraints(t,x,settings);
%     [A3,b3] = get_nonspherical_interagent_constraints(t,x,settings);
    [A3,b3,h3] = get_collision_avoidance_constraints(t,x,settings);
    A = [A; A3];
    b = [b; b3];

    [A4,b4,h4] = get_intersection_constraints(t,x,settings);
    A = [A; A4];
    b = [b; b4];
    
    if aa == agent
        h = [h; h3; h4];
    end
    
end

end

function [A,b,h] = get_road_constraints(t,x,settings)
aa = settings.aa;
lw = settings.lw;
Lr = settings.Lr;

xx = x(aa,:);
A = []; b = [];

% vx and vy
vx = xx(4)*(cos(xx(3)) - sin(xx(3))*tan(xx(5)));
vy = xx(4)*(sin(xx(3)) + cos(xx(3))*tan(xx(5)));

% ax and ay
ax_unc = -xx(4)^2/Lr*tan(xx(5))*(sin(xx(3)) + cos(xx(3))*tan(xx(5)));
ay_unc =  xx(4)^2/Lr*tan(xx(5))*(cos(xx(3)) - sin(xx(3))*tan(xx(5)));
ax_con = [-xx(4)*sin(xx(3))*sec(xx(5))^2; cos(xx(3))-sin(xx(3))*tan(xx(5))]';
ay_con = [ xx(4)*cos(xx(3))*sec(xx(5))^2; sin(xx(3))+cos(xx(3))*tan(xx(5))]';

% l1 = 5.0;
% l0 = 10.0;

l1 = 5.0;
l0 = l1^2 / 4;
settings.('l1') = l1;
settings.('l0') = l0;
h = 999;


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
Lr = settings.Lr;
Lf = settings.Lf;
lw = settings.lw;
l0 = settings.l0;
l1 = settings.l1;
vx = settings.vx;
ax_unc = settings.ax_unc;
ax_con = settings.ax_con;

idx = (-1:0)+aa*Nu;

h1       = lw - x(1);
Lfh1     = -vx;
Lf2h1    = -ax_unc;
LgLf1    = -ax_con;

h2       = x(1);
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
Lr = settings.Lr;
Lf = settings.Lf;
lw = settings.lw;
l0 = settings.l0;
l1 = settings.l1;
vy = settings.vy;
ay_unc = settings.ay_unc;
ay_con = settings.ay_con;

idx = (-1:0)+aa*Nu;

h1       = lw + x(2);
Lfh1     = vy;
Lf2h1    = ay_unc;
LgLf1    = ay_con;

h2       = -x(2);
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
Lr = settings.Lr;
Lf = settings.Lf;
lw = settings.lw;
l0 = settings.l0;
l1 = settings.l1;
vy = settings.vy;
ay_unc = settings.ay_unc;
ay_con = settings.ay_con;

idx = (-1:0)+aa*Nu;

h1       = lw - x(2);
Lfh1     = -vy;
Lf2h1    = -ay_unc;
LgLf1    = -ay_con;

h2       = x(2);
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
Lr = settings.Lr;
Lf = settings.Lf;
lw = settings.lw;
l0 = settings.l0;
l1 = settings.l1;
vx = settings.vx;
ax_unc = settings.ax_unc;
ax_con = settings.ax_con;

idx = (-1:0)+aa*Nu;

h1       = lw + x(1);
Lfh1     = vx;
Lf2h1    = ax_unc;
LgLf1    = ax_con;

h2       = -x(1);
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
run('physical_params.m')
aa = settings.aa;
Nu = settings.Nu;
Na = settings.Na;

idx = (-1:0)+aa*Nu;
xx  = x(aa,:);

h    = SL - xx(4);
Lfh  = 0;
Lgh  = [0 -1];

A      = zeros(1,Nu*Na);
A(idx) = -Lgh;
b = Lfh + h;

A = round(A,12);
b = round(b,12);

end

function [A,b,h] = get_intersection_constraints(t,x,settings)
aa = settings.aa;
Nu = settings.Nu;
Na = settings.Na;
Lr = settings.Lr;
lw = settings.lw;
tSlots = settings.tSlots;
beta = settings.beta;

idx = (-1:0)+aa*Nu;

xx = x(aa,:);
A = []; b = [];

% vx and vy
vx = xx(4)*(cos(xx(3)) - sin(xx(3))*tan(xx(5)));
vy = xx(4)*(sin(xx(3)) + cos(xx(3))*tan(xx(5)));

% ax and ay
ax_unc = -xx(4)^2/Lr*tan(xx(5))*(sin(xx(3)) + cos(xx(3))*tan(xx(5)));
ay_unc =  xx(4)^2/Lr*tan(xx(5))*(cos(xx(3)) - sin(xx(3))*tan(xx(5)));
ax_con = [-xx(4)*sin(xx(3))*sec(xx(5))^2; cos(xx(3))-sin(xx(3))*tan(xx(5))]';
ay_con = [ xx(4)*cos(xx(3))*sec(xx(5))^2; sin(xx(3))+cos(xx(3))*tan(xx(5))]';

if t > tSlots(aa,1)
    h = 999;
    return
    
elseif xx(1) > 0 && xx(1) < lw && xx(2) < -lw
    h    = -xx(2) - lw;
    Lfh  = -vy;
    Lf2h = -ay_unc;
    LgLf = -ay_con;

elseif xx(2) > 0 && xx(2) < lw && xx(1) > lw
    h    = xx(1) - lw;
    Lfh  = vx;
    Lf2h = ax_unc;
    LgLf = ax_con;

elseif xx(1) < 0 && xx(1) > -lw && xx(2) > lw
    h    = xx(2) - lw;
    Lfh  = vy;
    Lf2h = ay_unc;
    LgLf = ay_con;

elseif xx(2) < 0 && xx(2) > -lw && xx(1) < -lw
    h    = -xx(1) - lw;
    Lfh  = -vx;
    Lf2h = -ax_unc;
    LgLf = -ax_con;
 
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
l1 = 5.0;
l0 = l1^2 / 4;

% Experimental
l1 = 1.0;
l0 = l1^2 / 4;

A      = zeros(1,Na*Nu);
A(idx) = -LgLf;
b      = Lf2h + l1*Lfh + l0*h;

A = round(A,12);
b = round(b,12);
end

function [A,b,h] = get_collision_avoidance_constraints(t,x,settings)
Nu    = settings.Nu;
Na    = settings.Na;
uLast = settings.uLast;
Lr    = settings.Lr;
Lf    = settings.Lf;
AA    = settings.aa;
AAA   = settings.AAA;
wHat  = settings.wHat;
beta  = settings.beta;

sl = 1.0;
sw = 0.5;
sw = 1.0;

A = []; b = []; H = [];

% Loop through every scheduled agent for PCCA
for aa = 1:3
    
    % Is this correct?
    if AA > aa
        continue
    end
    
    % Loop through all other agents for interagent completeness
    for ii = 4:Na
        
        % Do not consider self in interagent safety
        if ii == aa
            continue
        end
        
        xa = x(aa,:);
        xi = x(ii,:);
        
        idx_aa = (-1:0)+aa*Nu;
        idx_ii = (-1:0)+ii*Nu;

        Aw = []; bw = []; hw = [];
              
        x_scale = sw / sl;
        
%         Rot
        
        
        % dx and dy
        dx  = (xa(1) - xi(1))*x_scale;
        dy  = xa(2) - xi(2);
        
        % dvx and dvy
        vax = xa(4)*(cos(xa(3)) - sin(xa(3))*tan(xa(5)));
        vay = xa(4)*(sin(xa(3)) + cos(xa(3))*tan(xa(5)));
        vix = xi(4)*(cos(xi(3)) - sin(xi(3))*tan(xi(5)));
        viy = xi(4)*(sin(xi(3)) + cos(xi(3))*tan(xi(5)));
        dvx = (vax - vix)*x_scale;
        dvy = vay - viy;
        
        % Solve for minimizer of h
        tmax = 2.0;
%         tmax = 1.0;
        T    = -(dx*dvx + dy*dvy)/(dvx^2 + dvy^2);
        
        % accelerations -- controlled (con) and uncontrolled (unc)
        axa_unc = -xa(4)^2/Lr*tan(xa(5))*(sin(xa(3)) + cos(xa(3))*tan(xa(5)));
        axi_unc = -xi(4)^2/Lr*tan(xi(5))*(sin(xi(3)) + cos(xi(3))*tan(xi(5)));
        aya_unc =  xa(4)^2/Lr*tan(xa(5))*(cos(xa(3)) - sin(xa(3))*tan(xa(5)));
        ayi_unc =  xi(4)^2/Lr*tan(xi(5))*(cos(xi(3)) - sin(xi(3))*tan(xi(5)));
        axa_con = zeros(1,Na*Nu);
        axi_con = zeros(1,Na*Nu);
        aya_con = zeros(1,Na*Nu);
        ayi_con = zeros(1,Na*Nu);
        axa_con(idx_aa) = [-xa(4)*sin(xa(3))*sec(xa(5))^2; cos(xa(3))-sin(xa(3))*tan(xa(5))]';
        axi_con(idx_ii) = [-xi(4)*sin(xi(3))*sec(xi(5))^2; cos(xi(3))-sin(xi(3))*tan(xi(5))]';
        aya_con(idx_aa) = [ xa(4)*cos(xa(3))*sec(xa(5))^2; sin(xa(3))+cos(xa(3))*tan(xa(5))]';
        ayi_con(idx_ii) = [ xi(4)*cos(xi(3))*sec(xi(5))^2; sin(xi(3))+cos(xi(3))*tan(xi(5))]'; 
        
%         % Assume no other agent control -- EXPERIMENTAL
%         axi_con(idx_ii) = [0; 0]';
%         ayi_con(idx_ii) = [0; 0]'; 

        
        % dax and day
        dax_unc = (axa_unc - axi_unc)*x_scale;
        day_unc = aya_unc - ayi_unc;
        dax_con = (axa_con - axi_con)*x_scale;
        day_con = aya_con - ayi_con;
        
        % Saturate T and assign Tdot
%         if T > tmax
% %             T = tmax;
%             
%             Tdot_unc = 0;
%             Tdot_con = zeros(1,Na*Nu);      
%         elseif T < 0
        if T < 0
            T = 0;
            Tdot_unc = 0;
            Tdot_con = zeros(1,Na*Nu);      
        else
            Tdot_unc = (2*(dvx*dax_unc + dvy*day_unc) - (dvx^2 + dvy^2 + dx*dax_unc + dy*day_unc)*(dvx^2 + dvy^2))/(dvx^2 + dvy^2)^2;
            Tdot_con = (2*(dvx*dax_con + dvy*day_con) - (dx*dax_con + dy*day_con)*(dvx^2 + dvy^2))/(dvx^2 + dvy^2)^2;
        end
        
%         if aa == 1 && ii == 4 && t > 3
%             disp(T)
%         end  
        
        % h and hdot (= Lfh + Lgh*u)
        h   = dx^2 + dy^2 + T^2*(dvx^2 + dvy^2) + 2*T*(dx*dvx + dy*dvy) - sw^2;
        Lfh = 2*dx*dvx + 2*dy*dvy + 2*T*Tdot_unc*(dvx^2 + dvy^2) + 2*T^2*(dvx*dax_unc + dvy*day_unc) + 2*Tdot_unc*(dx*dvx + dy*dvy) + 2*T*(dvx^2 + dvy^2 + dx*dax_unc + dy*day_unc);
        Lgh = 2*T*Tdot_con*(dvx^2 + dvy^2) + 2*T^2*(dvx*dax_con + dvy*day_con) + 2*Tdot_con*(dx*dvx + dy*dvy) + 2*T*(dx*dax_con + dy*day_con);
        
        % PCCA Contribution
        Lfh = Lfh + wHat(AAA,idx_aa)*Lgh(idx_aa)' + wHat(AAA,idx_ii)*Lgh(idx_ii)';
        
        l0  = 3.0;
        l0  = 2.0;
%         l0  = 1.0;

        if T > tmax
            Lfh = Lfh + 10*(T - tmax);
        end

        Aw  = [Aw; -Lgh];
        bw  = [bw; Lfh + l0*h];
        hw  = [hw; min(h,(dx^2 + dy^2 - sw^2))];
    end
    
    A = [A; Aw];
    b = [b; bw];
    H = [H; hw];
    
end

end

function [A,b] = get_nonspherical_interagent_constraints(t,x,settings)
Nu    = settings.Nu;
Na    = settings.Na;
uLast = settings.uLast;
Lr    = settings.Lr;
Lf    = settings.Lf;
AAA   = settings.AAA;
wHat  = settings.wHat;
beta  = settings.beta;

sl = 1.0;
sw = 0.5;

A = []; b = [];

% Loop through every scheduled agent for PCCA
for aa = 1:3
    
    % Loop through all other agents for interagent completeness
    for ii = 1:Na
        
        % Do not consider self in interagent safety
        if ii == aa
            continue
        end
        
        xa = x(aa,:);
        xi = x(ii,:);
        
        idx_aa = (-1:0)+aa*Nu;
        idx_ii = (-1:0)+ii*Nu;

        Aw = []; bw = [];
        T  = 3.0;
        
        % dx and dy
        dx  = xa(1) - xi(1);
        dy  = xa(2) - xi(2);
        
        % theta and rotation matrix
        theta = xa(3) + xa(5);
        Rot   = [cos(theta) sin(theta); -sin(theta) cos(theta)];
        
        % dl and dw
        dl  = Rot(1,:) * [dx; dy];
        dw  = Rot(2,:) * [dx; dy];
        
        % dvx and dvy
        vax = xa(4)*(cos(xa(3)) - sin(xa(3))*tan(xa(5)));
        vay = xa(4)*(sin(xa(3)) + cos(xa(3))*tan(xa(5)));
        vix = xi(4)*(cos(xi(3)) - sin(xi(3))*tan(xi(5)));
        viy = xi(4)*(sin(xi(3)) + cos(xi(3))*tan(xi(5)));
        dvx = vax - vix;
        dvy = vay - viy;
        
        % accelerations -- controlled (con) and uncontrolled (unc)
        axa_unc = -xa(4)^2/Lr*tan(xa(5))*(sin(xa(3)) + cos(xa(3))*tan(xa(5)));
        axi_unc = -xi(4)^2/Lr*tan(xi(5))*(sin(xi(3)) + cos(xi(3))*tan(xi(5)));
        aya_unc =  xa(4)^2/Lr*tan(xa(5))*(cos(xa(3)) - sin(xa(3))*tan(xa(5)));
        ayi_unc =  xi(4)^2/Lr*tan(xi(5))*(cos(xi(3)) - sin(xi(3))*tan(xi(5)));
        axa_con = zeros(1,Na*Nu);
        axi_con = zeros(1,Na*Nu);
        aya_con = zeros(1,Na*Nu);
        ayi_con = zeros(1,Na*Nu);
        axa_con(idx_aa) = [-xa(4)*sin(xa(3))*sec(xa(5))^2; cos(xa(3))-sin(xa(3))*tan(xa(5))]';
        axi_con(idx_ii) = [-xi(4)*sin(xi(3))*sec(xi(5))^2; cos(xi(3))-sin(xi(3))*tan(xi(5))]';
        aya_con(idx_aa) = [ xa(4)*cos(xa(3))*sec(xa(5))^2; sin(xa(3))+cos(xa(3))*tan(xa(5))]';
        ayi_con(idx_ii) = [ xi(4)*cos(xi(3))*sec(xi(5))^2; sin(xi(3))+cos(xi(3))*tan(xi(5))]'; 
        
%         % Assume no other agent control -- EXPERIMENTAL
%         axi_con(idx_ii) = [0; 0]';
%         ayi_con(idx_ii) = [0; 0]'; 

        
        % dax and day
        dax_unc = axa_unc - axi_unc;
        day_unc = aya_unc - ayi_unc;
        dax_con = axa_con - axi_con;
        day_con = aya_con - ayi_con;
        
        % Define Sl -- safe longitudinal distance
        Sl  = sl - max(0,T*(dvx*dx + dvy*dy)/norm([dx dy]));
        
        % Define derivatives
        phidot = xa(4)/Lr*tan(xa(5));
        dldot_unc = -dx*phidot*sin(theta) + dy*phidot*cos(theta) + dvy*sin(theta) + dvx*cos(theta);
        dldot_con = zeros(1,Na*Nu);
        dldot_con(idx_aa) = [-dx*sin(theta)+dy*cos(theta); 0]';
        dwdot_unc = -dvx*sin(theta) + dx*phidot*cos(theta) + dvy*cos(theta) - dy*phidot*sin(theta);
        dwdot_con = zeros(1,Na*Nu);
        dwdot_con(idx_aa) = [dx*cos(theta)-dy*sin(theta); 0]';
        Sldot_unc = -T*(dx*dax_unc + dy*day_unc + dvx^2 + dvy^2 - norm([dvx dvy])*(dx*dvx + dy*dvy)) / norm([dx dy])^2;
        Sldot_con = -T*(dx*dax_con + dy*day_con)/norm([dx dy])^2;
        
%         % Define intermediate variables
%         phidot = xa(4)/Lr*tan(xa(5));
%         f1    = dl;
%         f2    = dw;
%         f1dot_unc = dvx*cos(theta) +dvy*sin(theta) - phidot*(dx*sin(theta) - dy*cos(theta));
%         f1dot_con = zeros(1,Na*Nu);
%         f1dot_con(idx_aa) = [dy*cos(theta) - dx*sin(theta); 0]';
%         pp = dvx*dx + dvy*dy;
%         ppdot_unc = dax_unc*dx + dvx^2 + day_unc*dy + dvy^2;
%         ppdot_con = dax_con*dx + day_con*dy;
%         qq = norm([dx dy]);
%         qqdot = (dx*dvx + dy*dvy) / qq;
% 
%         g1        = Sl;
%         g1dot_unc = -T*(ppdot_unc*qq - pp*qqdot) / qq^2;
%         g1dot_con = -T*ppdot_con / qq;
%         
%         f2dot_unc = dvy*cos(theta) - dvx*sin(theta) - phidot*(dy*sin(theta) + dx*cos(theta));
%         f2dot_con = zeros(1,Na*Nu);
%         f2dot_con(idx_aa) = -[dy*sin(theta) + dx*cos(theta); 0]';
%         g2    = sw;

        
%         % Assume something about unscheduled vehicles
%         if ii < 4
%             Sl_dot_con(idx_ii) = -(dx*axi_con + dy*ayi_con) * T / norm([dx dy]);
%         end
        
        % h and hdot (= Lfh + Lgh*u)
        h   = (dl/Sl)^2 + (dw/sw)^2 - 1;
        Lfh = 2*(dl/Sl)*(dldot_unc*Sl - dl*Sldot_unc)/Sl^2 + 2/sw^2*dw*dwdot_unc;
        Lgh = 2*(dl/Sl)*(dldot_con*Sl - dl*Sldot_con)/Sl^2 + 2/sw^2*dw*dwdot_con;
        
        % PCCA Contribution
        Lfh = Lfh + wHat(AAA,idx_aa)*Lgh(idx_aa)' - wHat(AAA,idx_ii)*Lgh(idx_ii)';
        
        l0  = 10.0;
        Aw  = [Aw; -Lgh];
        bw  = [bw; Lfh + l0*h];
    end
    
    A = [A; Aw];
    b = [b; bw];
    
end

end

function [A,b] = get_interagent_constraints(t,x,settings)
Nu    = settings.Nu;
Na    = settings.Na;
uLast = settings.uLast;
Lr    = settings.Lr;
Lf    = settings.Lf;
AAA   = settings.AAA;
wHat  = settings.wHat;

% k     = 3.0;
RR    = 1.0;
jj    = 1;

% A     = zeros(Na-1,Nu);
% b     = zeros(Na-1,1);
A = []; b = [];

% Loop through every scheduled agent for PCCA
for aa = 1:3
    
    % Loop through all other agents for interagent completeness
    for ii = 1:Na
        
        % Do not consider self in interagent safety
        if ii == aa
            continue
        end
        
        idx_aa = (-1:0)+aa*Nu;
        idx_ii = (-1:0)+ii*Nu;

        Aw = []; bw = [];
    
        % Interagent safety conducted only via acceleration control (for now) 
        for cc = 1:2
            if cc == 1 % Front circle
                cx1 = x(aa,1) + Lf/2*cos(x(aa,3));
                cy1 = x(aa,2) + Lf/2*sin(x(aa,3));
                Lr1 = Lr + Lr/2;
                Lf1 = Lf - Lf/2;

            else       % Rear circle
                cx1 = x(aa,1) - Lr/2*cos(x(aa,3));
                cy1 = x(aa,2) - Lr/2*sin(x(aa,3));
                Lr1 = Lr - Lr/2;
                Lf1 = Lf + Lf/2;
            end

            tan_gamma1 = Lr1 / (Lr1 + Lf1) * uLast(aa); 

            for dd = 1:2
                if dd == 1 % Front circle
                    cx2 = x(ii,1) + Lf/2*cos(x(ii,3));
                    cy2 = x(ii,2) + Lf/2*sin(x(ii,3));
                    Lr2 = Lr + Lr/2;
                    Lf2 = Lf - Lf/2;

                else       % Rear circle
                    cx2 = x(ii,1) - Lr/2*cos(x(ii,3));
                    cy2 = x(ii,2) - Lr/2*sin(x(ii,3));
                    Lr2 = Lr - Lr/2;
                    Lf2 = Lf + Lf/2;
                end

                tan_gamma2 = Lr2 / (Lr2 + Lf2) * uLast(ii);

                dx = cx1 - cx2;
                dy = cy1 - cy2;
                dvx = (x(aa,4)*cos(x(aa,3)) - x(aa,4)*sin(x(aa,3))*tan_gamma1) - (x(ii,4)*cos(x(ii,3)) - x(ii,4)*sin(x(ii,3))*tan_gamma2);
                dvy = (x(aa,4)*sin(x(aa,3)) + x(aa,4)*cos(x(aa,3))*tan_gamma1) - (x(ii,4)*sin(x(ii,3)) + x(ii,4)*cos(x(ii,3))*tan_gamma2);
                dax = x(aa,4)^2/Lr1*(-sin(x(aa,3))*tan_gamma1 - cos(x(aa,3))*tan_gamma1^2) - x(ii,4)^2/Lr2*(-sin(x(ii,3))*tan_gamma2 - cos(x(ii,3))*tan_gamma2^2);
                day = x(aa,4)^2/Lr1*( cos(x(aa,3))*tan_gamma1 - sin(x(aa,3))*tan_gamma1^2) - x(ii,4)^2/Lr2*( cos(x(ii,3))*tan_gamma2 - sin(x(ii,3))*tan_gamma2^2);

                h    = dx^2 + dy^2 - RR^2;
                Lfh  = 2*dx*dvx + 2*dy*dvy;
                Lf2h = 2*dvx^2 + 2*dvy^2 + 2*dx*dax + 2*dy*day;
                LgLf = zeros(1,Nu*Na);
                Acc1 = 0;
                Acc2 =  2*dx*(cos(x(aa,3)) - sin(x(aa,3))*tan_gamma1) + 2*dy*(sin(x(aa,3)) + cos(x(aa,3))*tan_gamma1);
                Add1 = 0;
                Add2 = -2*dx*(cos(x(ii,3)) - sin(x(ii,3))*tan_gamma2) - 2*dy*(sin(x(ii,3)) + cos(x(ii,3))*tan_gamma2);
                    
                LgLf(idx_aa) = [Acc1 Acc2];
                LgLf(idx_ii) = [0 0];
                if ii < 4
                    LgLf(idx_ii) = [Add1 Add2];
                end
                
                % Apply the PCCA Estimate
                Lf2h = Lf2h + wHat(AAA,idx_aa)*[Acc1; Acc2] + wHat(AAA,idx_ii)*[Add1; Add2];

%                 % These work 1:15pm Sep 1
%                 l0 = 10.0;
%                 l1 = 20.0;
% 
%                 l0 = 20.0;
%                 l1 = 20.0;
% 
%                 l0 = 80.0;
%                 l1 = 20.0;
% 
%                 l0 = 200.0;
%                 l1 = 30.0;
% 
%                 % Extremely conservative
%                 l0 = 0.25;
%                 l1 = 1.0;
% 
%                 l1 = 10.0;
%                 l0 = l1^2 / 4;

                l1 = 50.0;
                l0 = l1^2 / 4;

                % Experiment
                a_max         = zeros(Nu*Na,1);
                a_max(idx_aa) = [0; -1*9.81];
                a_max(idx_ii) = [0; -1*9.81];
                max_l1        = 10^6;
                h2dot_max     = Lf2h + LgLf*a_max;
                while (h2dot_max + l1*Lfh + l0*h < 0) && (l1 < max_l1)
                    l1 = 2*l1;
                    l0 = l1^2 / 4;
                end

                Aw  = [Aw; -LgLf];
                bw  = [bw; Lf2h + l1*Lfh + l0*h];
            end

        end
    
        A = [A; Aw];
        b = [b; bw];
        
    end

end

A = round(A,12);
b = round(b,12);
end
