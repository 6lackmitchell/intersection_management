function [A,b,h] = get_ffcbf_safety_constraints_dynamic(t,x,settings)
%GET_SAFETY_CONSTRAINTS This is where the safety measures are considered
%   The relevant CBFs are taken into account here.
Lr = 1.0;
Lf = 1.0;
Na = 10;

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
    
    settings.('aa') = aa;
    
    [A1,b1,h1] = get_speed_constraints(t,x,settings);
    [A2,b2,h2] = get_road_constraints(t,x,settings);
    A = [A; A1; A2];
    b = [b; b1; b2];
%     A = [A; A3];
%     b = [b; b3];
    
    if aa == agent
        h = [h; h1; h2];
    end
    
end

[A3,b3,h3] = get_collision_avoidance_constraints(t,x,settings);

A = [A; A3];
b = [b; b3];
h = [h; h3];


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

function [A,b,H] = get_collision_avoidance_constraints(t,x,settings)
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
for aa = 1:Na
    
    Aw = []; bw = []; hw = [];
    
%     % Is this correct?
%     if AA > aa
%         continue
%     end
    
    % Loop through all other agents for interagent completeness
    for ii = aa:Na
        
        % Do not consider self in interagent safety
        if ii == aa
            continue
        end
        
        xa = x(aa,:);
        xi = x(ii,:);
        
        idx_aa = (-1:0)+aa*Nu;
        idx_ii = (-1:0)+ii*Nu;
              
        x_scale = sw / sl;

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
        kh = 100.0;
        tmax = 4.0;
%         tmax = 5.0;
        eps  = 1e-2;
%         T    = -(dx*dvx + dy*dvy)/(dvx^2 + dvy^2);
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
        
        % taudot
        tau_star_dot_unc = (dax_unc*(2*dvx*tau_star - dx) + day_unc*(2*dvy*tau_star - dy) - (dvx^2 + dvy^2)) / (dvx^2 + dvy^2 + eps);
        tau_star_dot_con = (dax_con*(2*dvx*tau_star - dx) + day_con*(2*dvy*tau_star - dy)) / (dvx^2 + dvy^2 + eps);
        Heavy_dot1_unc   = dheavyside(tau_star,kh,0)*tau_star_dot_unc;
        Heavy_dot2_unc   = dheavyside(tau_star,kh,tmax)*tau_star_dot_unc;
        Heavy_dot1_con   = dheavyside(tau_star,kh,0)*tau_star_dot_con;
        Heavy_dot2_con   = dheavyside(tau_star,kh,tmax)*tau_star_dot_con;
        tau_dot_unc      = tau_star_dot_unc*(Heavy1 - Heavy2) + tau_star*(Heavy_dot1_unc - Heavy_dot2_unc);
        tau_dot_con      = tau_star_dot_con*(Heavy1 - Heavy2) + tau_star*(Heavy_dot1_con - Heavy_dot2_con);
        
        % Saturate T and assign Tdot
%         if T > tmax
% %             T = tmax;
%             
%             Tdot_unc = 0;
%             Tdot_con = zeros(1,Na*Nu);      
%         elseif T < 0
%         if T < 0 || isnan(T)
%             T = 0;
%             Tdot_unc = 0;
%             Tdot_con = zeros(1,Na*Nu);
%         else
%             Tdot_unc = (2*(dvx*dax_unc + dvy*day_unc) - (dvx^2 + dvy^2 + dx*dax_unc + dy*day_unc)*(dvx^2 + dvy^2))/(dvx^2 + dvy^2)^2;
%             Tdot_con = (2*(dvx*dax_con + dvy*day_con) - (dx*dax_con + dy*day_con)*(dvx^2 + dvy^2))/(dvx^2 + dvy^2)^2;
%         end
        
%         if aa == 1 && ii == 4 && t > 3
%             disp(T)
%         end  
        
        % h and hdot (= Lfh + Lgh*u)
        h   = dx^2 + dy^2 + tau^2*(dvx^2 + dvy^2) + 2*tau*(dx*dvx + dy*dvy) - sw^2;
%         Lfh = 2*dx*dvx + 2*dy*dvy + 2*T*Tdot_unc*(dvx^2 + dvy^2) + 2*T^2*(dvx*dax_unc + dvy*day_unc) + 2*Tdot_unc*(dx*dvx + dy*dvy) + 2*T*(dvx^2 + dvy^2 + dx*dax_unc + dy*day_unc);
%         Lgh = 2*T*Tdot_con*(dvx^2 + dvy^2) + 2*T^2*(dvx*dax_con + dvy*day_con) + 2*Tdot_con*(dx*dvx + dy*dvy) + 2*T*(dx*dax_con + dy*day_con);
        Lfh = 2*dx*dvx + 2*dy*dvy + 2*tau*tau_dot_unc*(dvx^2 + dvy^2) + 2*tau^2*(dvx*dax_unc + dvy*day_unc) + 2*tau_dot_unc*(dx*dvx + dy*dvy) + 2*tau*(dvx^2 + dvy^2 + dx*dax_unc + dy*day_unc);
        Lgh = 2*tau*tau_dot_con*(dvx^2 + dvy^2) + 2*tau^2*(dvx*dax_con + dvy*day_con) + 2*tau_dot_con*(dx*dvx + dy*dvy) + 2*tau*(dx*dax_con + dy*day_con);
        
        % PCCA Contribution
        Lfh = Lfh + wHat(AAA,idx_aa)*Lgh(idx_aa)' + wHat(AAA,idx_ii)*Lgh(idx_ii)';
        
        l0  = 3.0;
        l0  = 2.0;
%         l0  = 1.0;

%         if T > tmax
%             Lfh = Lfh + 10*(T - tmax);
%         end

        Aw  = [Aw; -Lgh];
        bw  = [bw; Lfh + l0*h];
        hw  = [hw; min(h,(dx^2 + dy^2 - sw^2))];
    end
    
    A = [A; Aw];
    b = [b; bw];
    H = [H; hw];
    
end

end
