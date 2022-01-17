% function [A,b,h,v00,h00] = get_ffcbf_safety_constraints_dynamic_rpcca(t,x,settings)
function [A,b,params] = get_ffcbf_safety_constraints_dynamic_ccs(t,x,settings)
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
    [A4,b4,h4,v00,h00] = get_collision_avoidance_constraints(t,x,settings);

    A = [A; A4];
    b = [b; b4];
    h = [h; h4];
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

function [A,b,H,v00,h00] = get_collision_avoidance_constraints(t,x,settings)
Nu    = settings.Nu;
Na    = settings.Na;
Nn    = settings.Nn;
Lr    = settings.Lr;
% Lf    = settings.Lf;
AA    = settings.aa;
AAA   = settings.AAA;
% wHat  = settings.wHat;
uNom  = settings.uNom;

sl = 1.0;
sw = 1.0;

A = []; b = []; H = [];
v00 = zeros(Nu*Na,1);
h00 = zeros(Na,1);

% Loop through every scheduled agent for PCCA
for aa = 1:Na
    
    Aw = []; bw = []; hw = [];
    
    % Loop through all other agents for interagent completeness
    for ii = (Na-Nn+1):Na
        
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
        tau_star_dot_unc = (dax_unc*(2*dvx*tau_star - dx) + day_unc*(2*dvy*tau_star - dy) - (dvx^2 + dvy^2)) / (dvx^2 + dvy^2 + eps);
        tau_star_dot_con = (dax_con*(2*dvx*tau_star - dx) + day_con*(2*dvy*tau_star - dy)) / (dvx^2 + dvy^2 + eps);
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
%         alpha0_h = h^3;
        alpha0_h = 10*h;

        % Class K Function
        u0  = zeros(size(Lgh,2),1);
        if aa == AAA
            u0((-1:0)+AAA*Nu) = uNom;
        end

        uMax = [4*pi; 9.81; 4*pi; 9.81];
        uaa  = u0(idx_aa);
        uii  = u0(idx_ii);
        u0_filtered = filter_nominal_control([uaa; uii],uMax,Lfh,Lgh(idx_aa),alpha0_h);
        u0(idx_aa) = u0_filtered(1:2);
        u0(idx_ii) = u0_filtered(3:4);

        Kh  = max([-Lfh - Lgh*u0,h]);
%         k0  = 1;
%         K   = max([1/max([h,eps])*(-Lfh - Lgh*u0),k0]);

        Aw  = [Aw; -Lgh];
%         bw  = [bw; Lfh + K*h]; 
        bw  = [bw; Lfh + Kh]; 
        hw  = [hw; min(h,(dx^2 + dy^2 - (2*sw)^2))];

    end
    
    A = [A; Aw];
    b = [b; bw];
    H = [H; hw];
    
end

end


% Deprecated
% function [A,b,H,v00,h00] = get_collision_avoidance_constraints(t,x,settings)
% Nu    = settings.Nu;
% Na    = settings.Na;
% Lr    = settings.Lr;
% % Lf    = settings.Lf;
% AA    = settings.aa;
% AAA   = settings.AAA;
% % wHat  = settings.wHat;
% uNom  = settings.uNom;
% 
% sl = 2.0;
% sw = 2.0;
% 
% A = []; b = []; H = [];
% v00 = zeros(Nu*Na,1);
% h00 = zeros(Na,1);
% 
% % Loop through every scheduled agent for PCCA
% for aa = 1:Na
%     
%     Aw = []; bw = []; hw = [];
%     
%     % Loop through all other agents for interagent completeness
%     for ii = aa:Na
%         
%         % Do not consider self in interagent safety
%         if ii == aa
%             continue
%         end
%         
%         xa = x(aa,:);
%         xi = x(ii,:);
%         
%         idx_aa = (-1:0)+aa*Nu;
%         idx_ii = (-1:0)+ii*Nu;
%               
%         x_scale = sw / sl;
% 
%         % dx and dy
%         dx  = (xa(1) - xi(1))*x_scale;
%         dy  = xa(2) - xi(2);
%         
%         % dvx and dvy
%         vax = xa(4)*(cos(xa(3)) - sin(xa(3))*tan(xa(5)));
%         vay = xa(4)*(sin(xa(3)) + cos(xa(3))*tan(xa(5)));
%         vix = xi(4)*(cos(xi(3)) - sin(xi(3))*tan(xi(5)));
%         viy = xi(4)*(sin(xi(3)) + cos(xi(3))*tan(xi(5)));
%         dvx = (vax - vix)*x_scale;
%         dvy = vay - viy;
%         
%         % Solve for minimizer of h
%         kh       = 1000.0;
%         kh       = 100.0;
% %         kh       = 50.0;
% %         kh       = 1.0;
%         kh       = 10.0;
%         tmax     = 1.0;
% %         tmax     = 1.25;
%         eps      = 1e-3;
%         tau_star = -(dx*dvx + dy*dvy)/(dvx^2 + dvy^2 + eps);
%         Heavy1   = heavyside(tau_star,kh,0);
%         Heavy2   = heavyside(tau_star,kh,tmax);
%         tau      = tau_star*Heavy1 - (tau_star - tmax)*Heavy2;
% 
%         % accelerations -- controlled (con) and uncontrolled (unc)
%         axa_unc = -xa(4)^2/Lr*tan(xa(5))*(sin(xa(3)) + cos(xa(3))*tan(xa(5)));
%         axi_unc = -xi(4)^2/Lr*tan(xi(5))*(sin(xi(3)) + cos(xi(3))*tan(xi(5)));
%         aya_unc =  xa(4)^2/Lr*tan(xa(5))*(cos(xa(3)) - sin(xa(3))*tan(xa(5)));
%         ayi_unc =  xi(4)^2/Lr*tan(xi(5))*(cos(xi(3)) - sin(xi(3))*tan(xi(5)));
%         axa_con = zeros(1,Na*Nu);
%         axi_con = zeros(1,Na*Nu);
%         aya_con = zeros(1,Na*Nu);
%         ayi_con = zeros(1,Na*Nu);
%         axa_con(idx_aa) = [-xa(4)*sin(xa(3))*sec(xa(5))^2; cos(xa(3))-sin(xa(3))*tan(xa(5))]';
%         axi_con(idx_ii) = [-xi(4)*sin(xi(3))*sec(xi(5))^2; cos(xi(3))-sin(xi(3))*tan(xi(5))]';
%         aya_con(idx_aa) = [ xa(4)*cos(xa(3))*sec(xa(5))^2; sin(xa(3))+cos(xa(3))*tan(xa(5))]';
%         ayi_con(idx_ii) = [ xi(4)*cos(xi(3))*sec(xi(5))^2; sin(xi(3))+cos(xi(3))*tan(xi(5))]'; 
%         
% %         % Assume no other agent control -- EXPERIMENTAL
% %         axi_con(idx_ii) = [0; 0]';
% %         ayi_con(idx_ii) = [0; 0]'; 
%         
%         % dax and day
%         dax_unc = (axa_unc - axi_unc)*x_scale;
%         day_unc =  aya_unc - ayi_unc;
%         dax_con = (axa_con - axi_con)*x_scale;
%         day_con =  aya_con - ayi_con;
%         
%         % taudot
%         tau_star_dot_unc = (dax_unc*(2*dvx*tau_star - dx) + day_unc*(2*dvy*tau_star - dy) - (dvx^2 + dvy^2)) / (dvx^2 + dvy^2 + eps);
%         tau_star_dot_con = (dax_con*(2*dvx*tau_star - dx) + day_con*(2*dvy*tau_star - dy)) / (dvx^2 + dvy^2 + eps);
%         Heavy_dot1_unc   = dheavyside(tau_star,kh,0)*tau_star_dot_unc;
%         Heavy_dot2_unc   = dheavyside(tau_star,kh,tmax)*tau_star_dot_unc;
%         Heavy_dot1_con   = dheavyside(tau_star,kh,0)*tau_star_dot_con;
%         Heavy_dot2_con   = dheavyside(tau_star,kh,tmax)*tau_star_dot_con;
%         tau_dot_unc      = tau_star_dot_unc*(Heavy1 - Heavy2) + tau_star*(Heavy_dot1_unc - Heavy_dot2_unc);
%         tau_dot_con      = tau_star_dot_con*(Heavy1 - Heavy2) + tau_star*(Heavy_dot1_con - Heavy_dot2_con);
% 
%         % Exp const
%         ke = 1.0;
%         ke = 10.0;
%         ke = 2.0;
% %         ke = 100.0;
% %         ke = 3.0;
%         
%         % h and hdot (= Lfh + Lgh*u)
%         h   = dx^2 + dy^2 + tau^2*(dvx^2 + dvy^2) + 2*tau*(dx*dvx + dy*dvy) - sw^2;
%         Lfh = 2*dx*dvx + 2*dy*dvy + 2*tau*tau_dot_unc*(dvx^2 + dvy^2) + 2*tau^2*(dvx*dax_unc + dvy*day_unc) + 2*tau_dot_unc*(dx*dvx + dy*dvy) + 2*tau*(dvx^2 + dvy^2 + dx*dax_unc + dy*day_unc);
%         Lgh = 2*tau*tau_dot_con*(dvx^2 + dvy^2) + 2*tau^2*(dvx*dax_con + dvy*day_con) + 2*tau_dot_con*(dx*dvx + dy*dvy) + 2*tau*(dx*dax_con + dy*day_con);
% 
%         % MODIFICATIONS
%         Lgh(idx_ii) = (1 - exp(-ke*h))*Lgh(idx_ii);
%         Lfh = exp(-ke*h)*Lfh;
% 
%             
% 
% %         % PCCA Contribution
% %         Lfh = Lfh + wHat(AAA,idx_aa)*Lgh(idx_aa)' + wHat(AAA,idx_ii)*Lgh(idx_ii)';
% 
%         % RPCA Worst Case Control action
%         pWorst = exp(-ke*h);
%         if aa == AAA
%             if h00(ii) == 0
%                 worst_beta  = -sign(Lgh(idx_ii(1)))*4*pi;
%                 worst_acc   = -sign(Lgh(idx_ii(2)))*9.81;
%                 v00(idx_ii) = pWorst*[worst_beta; worst_acc];
%                 h00(ii)     = h;
%             end
%         elseif ii == AAA
%             if h00(aa) == 0
%                 worst_beta  = -sign(Lgh(idx_aa(1)))*4*pi;
%                 worst_acc   = -sign(Lgh(idx_aa(2)))*9.81;
%                 v00(idx_aa) = pWorst*[worst_beta; worst_acc];
%                 h00(aa)     = h;
%             end
%         end
% 
%         % Default worst-case control action
%         uMax        = [4*pi;  9.81];
%         worst_beta  = -sign(Lgh(idx_ii(1)))*uMax(1);
%         worst_acc   = -sign(Lgh(idx_ii(2)))*uMax(2);
%         uWorst      = pWorst*[worst_beta; worst_acc];
%         Phi         = abs(Lgh(idx_ii) * uWorst);
%         
%         % New rPCA
%         l0  = 0.01; % Experimental 1/2 robust scenario -- worked pretty well
% %         ke  = 0.5; % Experimental 1/2 robust scenario -- worked pretty well
% %         ke  = 5.0; % Experimental 1/2 robust scenario -- worked pretty well
% 
%         if sum(isinf(Lgh)) > 0
%             disp(Lgh)
%         end
% %         % Compute parameter for class k function
%         if Lfh + h < 0
%             classk_settings = struct('Nu',Nu,'ke',ke,'knom',l0,'Phi',Phi);
%             [kk,kcode]  = compute_class_k([uNom; pWorst*uWorst],uWorst,repmat(uMax,2,1),h,Lfh,[Lgh(idx_aa) Lgh(idx_ii)],classk_settings);
%             if kcode == 1
%                 disp('Class K Issue')
%             elseif isnan(kk)
%     %             kk = l0;
%                 kk = max([l0,2*(-Lfh + Phi*exp(-ke*h))/h]);
%             else
%     %                 kk
%             end
%         else
%             kk = max([l0,2*(-Lfh + Phi*exp(-ke*h))/h]);
%         end
% 
%         Aw  = [Aw; -Lgh];
%         bw  = [bw; Lfh + kk*h - Phi*exp(-ke*h)]; 
%         hw  = [hw; min(h,(dx^2 + dy^2 - sw^2))];
% 
% %         % h and hdot (= Lfh + Lgh*u) -- Standard CBF
% %         h    = dx^2 + dy^2 - sw^2;
% %         Lfh  = 2*dx*dvx + 2*dy*dvy;
% %         Lf2h = 2*(dvx^2 + dvy^2 + dx*dax_unc + dy*day_unc);
% %         LgLf = 2*(dx*dax_con + dy*day_con);
% % 
% %         % PCCA Contribution
% %         Lf2h = Lf2h + wHat(AAA,idx_aa)*LgLf(idx_aa)' + wHat(AAA,idx_ii)*LgLf(idx_ii)';
% % 
% %         % RPCCA Worst Case Control action
% %         if ii == AAA
% %             worst_beta  = 0;
% %             worst_acc   = -sign(LgLf(idx_aa(2)))*9.81;
% %             v00(idx_aa) = [worst_beta; worst_acc];
% %             h00(aa)     = h;
% %         end
% %     
% %         l1 = 40.0;
% %         l1 = 30.0;
% %         l0 = l1^2 / 4;
% %         
% %         Aw = [Aw; -LgLf];
% %         bw = [bw; Lf2h + l1*Lfh + l0*h];
% %         Hw = [hw; h];
%     end
%     
%     A = [A; Aw];
%     b = [b; bw];
%     H = [H; hw];
%     
% end
% 
% end

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
l0 = l1^2 / 4;

A      = zeros(1,Na*Nu);
A(idx) = -LgLf;
b      = Lf2h + l1*Lfh + l0*h;

A = round(A,12);
b = round(b,12);
end
