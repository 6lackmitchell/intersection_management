function [A,b,params] = get_ffcbf_safety_constraints_dynamic_darf(t,x,settings)
%GET_SAFETY_CONSTRAINTS This is where the safety measures are considered
%   The relevant CBFs are taken into account here.
Lr = 1.0;
Lf = 1.0;

% Unpack PCCA Settings
agent = settings.AAA;
Na    = settings.Na;

% Add additional settings
settings.('Nu')   = 2;
settings.('Lr')   = Lr;
settings.('Lf')   = Lf;
settings.('lw')   = 3.0;

A = []; b = []; h = [];

for aa = 1:Na
    
    settings.('aa') = aa;
    
    [A1,b1,h1] = get_speed_constraints(t,x,settings);
    [A2,b2,h2] = get_road_constraints(t,x,settings);
    if aa < 4
        [A3,b3,h3] = get_intersection_constraints(t,x,settings);
    else
        A3 = []; b3 = []; h3 = 100;
    end
    A = [A; A1; A2; A3];
    b = [b; b1; b2; b3];
    
    if aa == agent
        h = [h; h1; h2; h3];
    end
    
end

v00 = zeros(Na*settings.Nu,1);
h00 = inf*ones(Na,1);
if agent < 5
    [A4,b4,h4,v00,h00,aval,bval] = get_collision_avoidance_constraints(t,x,settings);

    A = [A zeros(size(A,1),size(settings.gammas,1))];
    A = [A; A4];
    b = [b; b4];
    h = [h; h4];
end




% Organize parameters to be returned
params = struct('h',   h,   ...
                'v00', v00, ...
                'h00', h00, ...
                'avalue',aval, ...
                'bvalue',bval);


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

idx    = (-1:0)+aa*Nu;
xx     = x(aa,:);

h      = SL - xx(4);
Lfh    = 0;
Lgh    = [0 -1];

A      = zeros(1,Nu*Na);
A(idx) = -Lgh;
b      = Lfh + 0.25*h^3;

A = round(A,12);
b = round(b,12);

end

function [A,B,H,v00,h00,aval,bval] = get_collision_avoidance_constraints(t,x,settings)
Nu    = settings.Nu;
Na    = settings.Na;
Nn    = settings.Nn;
Lr    = settings.Lr;
% Lf    = settings.Lf;
% AA    = settings.aa;
AAA   = settings.AAA;
vEst  = settings.vEst;
% wHat  = settings.wHat;
% uNom  = settings.uNom;
gammas = settings.gammas;

sl = 2.0;
sw = 2.0;

A = []; B = []; H = [];
v00 = zeros(Nu*Na,1);
h00 = zeros(Na,1);

% Loop over noncommunicating agents
for ii = 4

    Aw = []; bw = []; hw = [];

    % Loop over communicating agents
    for aa = 1:Na-Nn
        if AAA < 4
            aa = AAA;
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
        kh       = 100.0;
        tmax     = 1.0;
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
        axa_con(idx_aa) = [-xa(4)*sin(xa(3))*sec(xa(5))^2; cos(xa(3))-sin(xa(3))*tan(xa(5))]';
        axi_con(idx_ii) = [-xi(4)*sin(xi(3))*sec(xi(5))^2; cos(xi(3))-sin(xi(3))*tan(xi(5))]';
        aya_con(idx_aa) = [ xa(4)*cos(xa(3))*sec(xa(5))^2; sin(xa(3))+cos(xa(3))*tan(xa(5))]';
        ayi_con(idx_ii) = [ xi(4)*cos(xi(3))*sec(xi(5))^2; sin(xi(3))+cos(xi(3))*tan(xi(5))]'; 
        
        % dax and day
        dax_unc = (axa_unc - axi_unc)*x_scale;
        day_unc =  aya_unc - ayi_unc;
        dax_con = (axa_con - axi_con)*x_scale;
        day_con =  aya_con - ayi_con;
        
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
        h   = dx^2 + dy^2 + tau^2*(dvx^2 + dvy^2) + 2*tau*(dx*dvx + dy*dvy) - sw^2;
        Lfh = 2*dx*dvx + 2*dy*dvy + 2*tau*tau_dot_unc*(dvx^2 + dvy^2) + 2*tau^2*(dvx*dax_unc + dvy*day_unc) + 2*tau_dot_unc*(dx*dvx + dy*dvy) + 2*tau*(dvx^2 + dvy^2 + dx*dax_unc + dy*day_unc);
        Lgh = 2*tau*tau_dot_con*(dvx^2 + dvy^2) + 2*tau^2*(dvx*dax_con + dvy*day_con) + 2*tau_dot_con*(dx*dvx + dy*dvy) + 2*tau*(dx*dax_con + dy*day_con);
        Lgh_copy_aa = Lgh; Lgh_copy_aa(idx_ii) = 0;
        Lgh_copy_ii = Lgh; Lgh_copy_ii(idx_aa) = 0;
        
        if AAA < 4
            idx_adv = idx_ii;
            adv     = ii;
        else
            idx_adv = idx_aa;
            adv     = aa;
        end

        % Class K Constant
        kk   = 5; 
        ke   = 2;
        a0   = Lfh + kk*h;
        ahat = a0 - (1-exp(-1/h))*eps;
        a    = ahat*exp(-eps/abs(ahat)) + a0*(1-exp(-eps/abs(ahat)));
        c    = 1/2; 
        b    = Lgh(idx_adv);
        h00(adv) = h;

        % Two approaches:
        % 1 - assume that v1 = 0 (i.e. no steering input)
%         vv              = exp(-ke/h)*vEst(adv,2)+(1-exp(-ke/h))*(-c*a/b(2));
%         v00(idx_adv(2)) = min([9.81 max([-9.81 vv])]);
        % 2 - distribute c=1/2 between two inputs
        vv1      = exp(-ke/h)*vEst(adv,1)+(1-exp(-ke/h))*(-c/2*a/b(1));
        vv2      = exp(-ke/h)*vEst(adv,2)+(1-exp(-ke/h))*(-c/2*a/b(2));
        h00(adv) = h;
        v00(idx_adv) = min([4*pi 9.81],max([-4*pi -9.81], [vv1 vv2]));



        % Assign constraints
        new_g     = zeros(1,size(gammas,1));
        new_g(aa) = kk*h + Lfh;
        new_A     = [-Lgh_copy_aa -new_g; 
                     -Lgh_copy_ii  new_g];
        new_b     = [0; kk*h + Lfh];

        % Update intermediate constraint matrices/vectors
        Aw  = [Aw; new_A];
        bw  = [bw; new_b]; 
        hw  = [hw; min(h,(dx^2 + dy^2 - sw^2))];
        
        % aval bval
        aval = a;
        bval = sum(b.^2);

        if AAA < 4
            break;
        end

    end

    A = [A; Aw];
    B = [B; bw];
    H = [H; hw];
    
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

x_intersection     = [lw -lw -lw  lw];
y_intersection     = [lw  lw -lw -lw];
intersection_empty = sum(inpolygon(x(1:end ~= aa,1),x(1:end ~= aa,2),x_intersection,y_intersection)) == 0;

if aa ~= 1
    x_lane1            = [lw 0 0 lw];
    y_lane1            = [-lw -lw -10 -10];
    one_gone           = inpolygon(x(1,1),x(1,2),x_lane1,y_lane1) == 0;
else
    one_gone = 1;
end

% if t > tSlots(aa,1) || (intersection_empty && ((tSlots(aa,1) - t) < 2.5))
% if t > tSlots(aa,1) || (intersection_empty && ((tSlots(aa,1) - t) < 1))
% if t > tSlots(aa,1) || (intersection_empty && one_gone && ((tSlots(aa,1) - t) < 2))
if t > tSlots(aa,1) || (intersection_empty && one_gone && ((tSlots(aa,1) - t) < 3))
    return
    
elseif xx(1) > 0 && xx(1) < lw && xx(2) < -lw
    h    = -xx(2) - lw;
    Lfh  = -(xx(4)*sin(xx(3)) + xx(4)*cos(xx(3))*tan(xx(5)));
    Lf2h = -(xx(4)^2/Lr * (cos(xx(3))*tan(xx(5)) - sin(xx(3))*tan(xx(5))^2));
    LgLf = -[0 sin(xx(3)) + cos(xx(3))*tan(xx(5))];

elseif xx(2) > 0 && xx(2) < lw && xx(1) > lw
    h    = xx(1) - lw;
    Lfh  = xx(4)*cos(xx(3)) - xx(4)*sin(xx(3))*tan(xx(5));
    Lf2h = xx(4)^2/Lr * (sin(xx(3))*tan(xx(5)) - cos(xx(3))*tan(xx(5))^2);
    LgLf = [0 cos(xx(3)) - sin(xx(3))*tan(xx(5))];

elseif xx(1) < 0 && xx(1) > -lw && xx(2) > lw
    h    = xx(2) - lw;
    Lfh  = xx(4)*sin(xx(3)) + xx(4)*cos(xx(3))*tan(xx(5));
    Lf2h = xx(4)^2/Lr * (cos(xx(3))*tan(xx(5)) - sin(xx(3))*tan(xx(5))^2);
    LgLf = [0 sin(xx(3)) + cos(xx(3))*tan(xx(5))];

elseif xx(2) < 0 && xx(2) > -lw && xx(1) < -lw
    h    = -xx(1) - lw;
    Lfh  = -(xx(4)*cos(xx(3)) - xx(4)*sin(xx(3))*tan(xx(5)));
    Lf2h = -(xx(4)^2/Lr * (sin(xx(3))*tan(xx(5)) - cos(xx(3))*tan(xx(5))^2));
    LgLf = -[0 cos(xx(3)) - sin(xx(3))*tan(xx(5))];
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
l1 = 1.0;
l1 = 0.1;
l1 = 2.0;
l1 = 10.0;
l1 = 5.0;
l1 = 2.0;
l0 = l1^2 / 4;

A      = zeros(1,Na*Nu);
A(idx) = -LgLf;
b      = Lf2h + l1*Lfh + l0*h;

A = round(A,12);
b = round(b,12);
end
