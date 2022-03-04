function [A,b,params] = get_decentralized_safety_constraints(t,x,settings)
%GET_SAFETY_CONSTRAINTS This is where the safety measures are considered

% Unpack PCCA Settings
% agent = settings.AAA;
Na    = settings.Na;

% Add additional settings
settings.('Nu')   = 1;
settings.('lw')   = 3.0;

v00 = zeros(Na*settings.Nu,1);
h00 = inf*ones(Na,1);

nCons = 4;
nRows = nCons*Na+settings.Ns;
nCols = settings.Nu*Na+settings.Ns;

A  = zeros(nRows,nCols);
b  = zeros(nRows,1);
h  = zeros(nRows,1);
h0 = zeros(nRows,1);

for aa = settings.AAA%1:Na
    
    settings.('aa') = aa;
    
    [A1,b1,h1]         = get_speed_constraints(t,x,settings);
    [A2,b2,h2]         = get_road_constraints(t,x,settings);

    row_idx = nCons*(aa-1)+1:nCons*(aa-1)+nCons;
    A(row_idx,:) = [A1; 0*A2];
    b(row_idx)   = [b1; 0*b2];
    h(row_idx)   = [h1; h2];
    h0(row_idx)  = [h1; h2];

end

[A4,b4,h4,h04,v00,h00] = get_collision_avoidance_constraints(t,x,settings);

row_idx = nCons*Na+1:nCons*Na+factorial(Na-1);

A(row_idx,:) = A4;
b(row_idx)   = b4;
h(row_idx)   = h4;
h0(row_idx)  = h04;
    
% Organize parameters to be returned
params = struct('h',   h,   ...
                'h0',  h0,  ...
                'v00', v00, ...
                'h00', h00);

end

function [A,b,h] = get_road_constraints(t,x,settings)
aa = settings.aa;
lw = settings.lw;
Nu = settings.Nu;
Na = settings.Na;
Ns = settings.Ns;
Lr = 1;

xx = x(aa,:);
A = zeros(2,Nu*Na+Ns);
b = ones(2,1);

% vx and vy
vx = xx(4)*(cos(xx(3)) - sin(xx(3))*tan(xx(5)));
vy = xx(4)*(sin(xx(3)) + cos(xx(3))*tan(xx(5)));

% ax and ay
ax_unc = -xx(4)^2/Lr*tan(xx(5))*(sin(xx(3)) + cos(xx(3))*tan(xx(5)));
ay_unc = -xx(4)^2/Lr*tan(xx(5))*(cos(xx(3)) - sin(xx(3))*tan(xx(5)));
ax_con = [cos(xx(3))-sin(xx(3))*tan(xx(5))]';
ay_con = [sin(xx(3))+cos(xx(3))*tan(xx(5))]';

l1 = 100.0;
l0 = l1^2 / 6;
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
Ns = settings.Ns;
% Lr = settings.Lr;
% Lf = settings.Lf;
lw = settings.lw;
l0 = settings.l0;
l1 = settings.l1;
vx = settings.vx;
ax_unc = settings.ax_unc;
ax_con = settings.ax_con;
R = 1;

idx = Nu*(aa-1)+1:Nu*(aa-1)+Nu;

h1       = (lw-R) - x(1);
Lfh1     = -vx;
Lf2h1    = -ax_unc;
LgLf1    = -ax_con;

h2       = x(1) - R;
Lfh2     = vx;
Lf2h2    = ax_unc;
LgLf2    = ax_con;

A        = zeros(2,Nu*Na+Ns);
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
Ns = settings.Ns;
% Lr = settings.Lr;
% Lf = settings.Lf;
lw = settings.lw;
l0 = settings.l0;
l1 = settings.l1;
vy = settings.vy;
ay_unc = settings.ay_unc;
ay_con = settings.ay_con;
R = 1;

idx = Nu*(aa-1)+1:Nu*(aa-1)+Nu;

h1       = (lw-R) + x(2);
Lfh1     = vy;
Lf2h1    = ay_unc;
LgLf1    = ay_con;

h2       = -x(2) - R;
Lfh2     = -vy;
Lf2h2    = -ay_unc;
LgLf2    = -ay_con;

A        = zeros(2,Nu*Na+Ns);
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
Ns = settings.Ns;
% Lr = settings.Lr;
% Lf = settings.Lf;
lw = settings.lw;
l0 = settings.l0;
l1 = settings.l1;
vy = settings.vy;
ay_unc = settings.ay_unc;
ay_con = settings.ay_con;
R = 1;

idx = Nu*(aa-1)+1:Nu*(aa-1)+Nu;

h1       = (lw-R) - x(2);
Lfh1     = -vy;
Lf2h1    = -ay_unc;
LgLf1    = -ay_con;

h2       = x(2) - R;
Lfh2     = vy;
Lf2h2    = ay_unc;
LgLf2    = ay_con;

A        = zeros(2,Nu*Na+Ns);
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
Ns = settings.Ns;
% Lr = settings.Lr;
% Lf = settings.Lf;
lw = settings.lw;
l0 = settings.l0;
l1 = settings.l1;
vx = settings.vx;
ax_unc = settings.ax_unc;
ax_con = settings.ax_con;
R = 1;

idx = Nu*(aa-1)+1:Nu*(aa-1)+Nu;

h1       = (lw-R) + x(1);
Lfh1     = vx;
Lf2h1    = ax_unc;
LgLf1    = ax_con;

h2       = -x(1) - R;
Lfh2     = -vx;
Lf2h2    = -ax_unc;
LgLf2    = -ax_con;

A        = zeros(2,Nu*Na+Ns);
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
Ns = settings.Ns;

idx    = Nu*(aa-1)+1:Nu*(aa-1)+Nu;
xx     = x(aa,:);

kf      = 10; % Class K gain
hf      = settings.SL - xx(4);
Lfhf    = 0;
Lghf    = [-1];

kr      = kf;
Lfhr    = 0;
if settings.backup
    % Reverse allowed
    hr      = 100;
    Lghr    = [0];
else
    % Reverse not allowed
    hr      = xx(4);
    Lghr    = [1];
end

A        = zeros(2,Nu*Na+Ns);
b        = zeros(2,1);
A(1,idx) = -Lghf;
A(1,Na+idx) = -1; % Slack Parameter for speed limit
A(2,idx) = -Lghr;

b(1)     = Lfhf + kf*hf;
b(2)     = Lfhr + kr*hr;

A = round(A,12);
b = round(b,12);
h = [hf; hr];

end

function [A,b,Ht,H0,v00,h00] = get_collision_avoidance_constraints(t,x,settings)
Nu    = settings.Nu;
Na    = settings.Na;
Ns    = settings.Ns;
tmax  = settings.lookahead;
uNom  = settings.uNom;
AAA   = settings.AAA;
wHat  = settings.wHat;

Lr = 1;
sw = 1.0;

betadot = uNom(1:2:end);

Nc  = factorial(Na-1);
A   = zeros(Nc,Nu*Na+Ns);
b   = zeros(Nc,1);
Ht  = zeros(Nc,1);
H0  = zeros(Nc,1);
v00 = zeros(Nu*Na,1);
h00 = zeros(Na,1);
cc  = 1;
ss  = 1;

% Loop through every scheduled agent for PCCA
for aa = settings.AAA%1:Na
    
%     Aw = []; bw = []; hw = [];
%     nc  = Na-aa; % THIS NEEDS TO BE UNCOMMENTED
    nc  = Na;
    Aw  = zeros(nc,Nu*Na+Ns);
    bw  = zeros(nc,1);
    hw  = zeros(nc,1);
    hw0 = zeros(nc,1);
    dd  = 1;
    
    % Loop through all other agents for interagent completeness
    for ii = 1:Na
        if ii == aa
            continue
        end
%     for ii = aa+1:Na
        
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
%         tmax     = xa(4)/9.81; % Minimum stopping time %1.0;
        eps      = 1e-3;
        tau_star = -(dx*dvx + dy*dvy)/(dvx^2 + dvy^2 + eps);
        Heavy1   = heavyside(tau_star,kh,0);
        Heavy2   = heavyside(tau_star,kh,tmax);
        tau      = tau_star*Heavy1 - (tau_star - tmax)*Heavy2;

        % accelerations -- controlled (con) and uncontrolled (unc)
        axa_unc = -xa(4)^2/Lr*tan(xa(5))*(sin(xa(3)) + cos(xa(3))*tan(xa(5))) - betadot(aa)*xa(4)*sin(xa(3))*sec(xa(5))^2;
        axi_unc = -xi(4)^2/Lr*tan(xi(5))*(sin(xi(3)) + cos(xi(3))*tan(xi(5))) - betadot(ii)*xi(4)*sin(xi(3))*sec(xi(5))^2;
        aya_unc =  xa(4)^2/Lr*tan(xa(5))*(cos(xa(3)) - sin(xa(3))*tan(xa(5))) + betadot(aa)*xa(4)*cos(xa(3))*sec(xa(5))^2;
        ayi_unc =  xi(4)^2/Lr*tan(xi(5))*(cos(xi(3)) - sin(xi(3))*tan(xi(5))) + betadot(ii)*xi(4)*cos(xi(3))*sec(xi(5))^2;
        axa_con = zeros(1,Na*Nu);
        axi_con = zeros(1,Na*Nu);
        aya_con = zeros(1,Na*Nu);
        ayi_con = zeros(1,Na*Nu);
        axa_con(idx_aa) = cos(xa(3))-sin(xa(3))*tan(xa(5));
%         axi_con(idx_ii) = cos(xi(3))-sin(xi(3))*tan(xi(5)); % THIS NEEDS TO BE UNCOMMENTED
        aya_con(idx_aa) = sin(xa(3))+cos(xa(3))*tan(xa(5));
%         ayi_con(idx_ii) = sin(xi(3))+cos(xi(3))*tan(xi(5));  % THIS NEEDS TO BE UNCOMMENTED
        
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

        % Class K Function(s)
        l0   = settings.classk;
        l1   = sqrt(6*l0);
        
        % h and hdot (= Lfh + Lgh*u)
        h0   = dx^2 + dy^2 - (2*sw)^2;
        h    = dx^2 + dy^2 + tau^2*(dvx^2 + dvy^2) + 2*tau*(dx*dvx + dy*dvy) - (2*sw)^2;
        Lfh0 = 2*dx*dvx + 2*dy*dvy;
        Lfh  = 2*(dx*dvx + dy*dvy) + 2*tau*(dvx^2 + dvy^2 + dx*dax_unc + dy*day_unc) + 2*tau_dot_unc*(dx*dvx + dy*dvy + tau*(dvx^2 + dvy^2)) + 2*tau^2*(dvx*dax_unc + dvy*day_unc);        
        Lgh  = 2*tau*tau_dot_con*(dvx^2 + dvy^2) + 2*tau^2*(dvx*dax_con + dvy*day_con) + 2*tau_dot_con*(dx*dvx + dy*dvy) + 2*tau*(dx*dax_con + dy*day_con);
       
        if strcmp(settings.cbf_type,'nominal_cbf')
            % Nominal CBF (Rel-Deg 2)
            H   = h0;
            LfH = l1*Lfh0 + 2*(dvx^2 + dvy^2) + 2*(dx*dax_unc + dy*day_unc);
            LgH = 2*(dx*dax_con + dy*day_con);

        elseif strcmp(settings.cbf_type,'ff_cbf')
            % Future Focused CBF
            l0  = h0;
            H   = h;
            LfH = Lfh;
            LgH = Lgh;

        elseif strcmp(settings.cbf_type,'rv_cbf')
            % Robust-Virtual CBF
            a1    = 0.1;
            kh0   = 1;
            H     = h   + a1*max([tau-1,eps])*h0^(1/kh0);
            LfH   = Lfh + a1*(max([tau-1,eps])*(1/kh0)*h0^(1/kh0-1)*Lfh0 + tau_dot_unc*h0^(1/kh0));
            LgH   = Lgh + a1*tau_dot_con*h0^(1/kh0);
        end
    
        % Inequalities: Ax <= b
        Aw(dd,1:Na*Nu)  = -LgH;
        bw(dd)          = LfH + l0*H; 
        hw(dd)          = H;
        hw0(dd)         = h0;

        dd = dd + 1;
        ss = ss + 1;

    end
    
    A(cc:cc+(nc-1),:) = Aw;
    b(cc:cc+(nc-1))   = bw;
    Ht(cc:cc+(nc-1))  = hw;
    H0(cc:cc+(nc-1))  = hw0;

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
l0 = l1^2 / 6;

A      = zeros(1,Na*Nu);
A(idx) = -LgLf;
b      = Lf2h + l1*Lfh + l0*h;

A = round(A,12);
b = round(b,12);
end
