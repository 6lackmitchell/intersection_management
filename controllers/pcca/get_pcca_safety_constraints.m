function [A,b] = get_pcca_safety_constraints(t,x,settings)
%GET_SAFETY_CONSTRAINTS This is where the safety measures are considered
%   The relevant CBFs are taken into account here.
Lr = 1.0;
Lf = 1.0;

% Unpack PCCA Settings
uLast  = settings.uLast;
beta   = atan(Lr/(Lr+Lf)*uLast);

% Add additional settings
settings.('Nu')   = 2;
settings.('Na')   = 4; Na = 4; 
settings.('Lr')   = Lr;
settings.('Lf')   = Lf;
settings.('lw')   = 3.0;
settings.('beta') = beta;

A = []; b = [];

for aa = 1:Na
    
    settings.('aa') = aa;
    
    [A1,b1] = get_speed_constraints(t,x,settings);
    [A2,b2] = get_road_constraints(t,x,settings);
    A = [A; A1; A2];
    b = [b; b1; b2];

    % Unscheduled vehicles don't consider intersection or interagent safety
    if aa >= 4
        return
    end

%     [A3,b3] = get_interagent_constraints(t,x,settings);
    [A3,b3] = get_nonspherical_interagent_constraints(t,x,settings);
    A = [A; A3];
    b = [b; b3];

    [A4,b4] = get_intersection_constraints(t,x,settings);
    A = [A; A4];
    b = [b; b4];
    
end

end

function [A,b] = get_road_constraints(t,x,settings)
aa = settings.aa;
lw = settings.lw;

xx = x(aa,:);
A = []; b = [];

if xx(1) < lw && xx(1) > -lw && xx(2) < lw && xx(2) > -lw
    return
elseif xx(1) > 0 && xx(1) < lw
    [A,b] = get_SENEroad_constraint(t,xx,settings);
elseif xx(2) > 0 && xx(2) < lw
    [A,b] = get_ENWNroad_constraint(t,xx,settings);
elseif xx(2) < 0 && xx(2) > -lw
    [A,b] = get_ESWSroad_constraint(t,xx,settings);
elseif xx(1) < 0 && xx(1) > -lw
    [A,b] = get_SWNWroad_constraint(t,xx,settings);
end

end

function [A,b] = get_SENEroad_constraint(t,x,settings)
aa = settings.aa;
Nu = settings.Nu;
Na = settings.Na;
Lr = settings.Lr;
Lf = settings.Lf;
lw = settings.lw;

idx = (-1:0)+aa*Nu;

h1       = lw - x(1);
Lfh1     = -x(4)*cos(x(3));
Lgh1     = [x(4)*sin(x(3)) 0]*Lr/(Lr+Lf);

h2       = x(1);
Lfh2     = x(4)*cos(x(3));
Lgh2     = [-x(4)*sin(x(3)) 0]*Lr/(Lr+Lf);

A        = zeros(2,Nu*Na);
b        = zeros(2,1);

A(1,idx) = -Lgh1;
A(2,idx) = -Lgh2;
b(1)     = Lfh1 + h1;
b(2)     = Lfh2 + h2;

A = round(A,12);
b = round(b,12);

end

function [A,b] = get_ESWSroad_constraint(t,x,settings)
aa = settings.aa;
Nu = settings.Nu;
Na = settings.Na;
Lr = settings.Lr;
Lf = settings.Lf;
lw = settings.lw;

idx = (-1:0)+aa*Nu;

h1       = lw + x(2);
Lfh1     = x(4)*sin(x(3));
Lgh1     = [x(4)*cos(x(3)) 0]*Lr/(Lr+Lf);

h2       = -x(2);
Lfh2     = -x(4)*sin(x(3));
Lgh2     = [-x(4)*cos(x(3)) 0]*Lr/(Lr+Lf);

A        = zeros(2,Nu*Na);
b        = zeros(2,1);

A(1,idx) = -Lgh1;
A(2,idx) = -Lgh2;
b(1)     = Lfh1 + h1;
b(2)     = Lfh2 + h2;

A = round(A,12);
b = round(b,12);

end

function [A,b] = get_ENWNroad_constraint(t,x,settings)
aa = settings.aa;
Nu = settings.Nu;
Na = settings.Na;
Lr = settings.Lr;
Lf = settings.Lf;
lw = settings.lw;

idx = (-1:0)+aa*Nu;

h1       = lw - x(2);
Lfh1     = -x(4)*sin(x(3));
Lgh1     = [-x(4)*cos(x(3)) 0]*Lr/(Lr+Lf);

h2       = x(2);
Lfh2     = x(4)*sin(x(3));
Lgh2     = [x(4)*cos(x(3)) 0]*Lr/(Lr+Lf);

A        = zeros(2,Nu*Na);
b        = zeros(2,1);

A(1,idx) = -Lgh1;
A(2,idx) = -Lgh2;
b(1)     = Lfh1 + h1;
b(2)     = Lfh2 + h2;

A = round(A,12);
b = round(b,12);

end

function [A,b] = get_SWNWroad_constraint(t,x,settings)
aa = settings.aa;
Nu = settings.Nu;
Na = settings.Na;
Lr = settings.Lr;
Lf = settings.Lf;
lw = settings.lw;

idx = (-1:0)+aa*Nu;

h1       = lw + x(1);
Lfh1     = x(4)*cos(x(3));
Lgh1     = [-x(4)*sin(x(3)) 0]*Lr/(Lr+Lf);

h2       = -x(1);
Lfh2     = -x(4)*cos(x(3));
Lgh2     = [x(4)*sin(x(3)) 0]*Lr/(Lr+Lf);

A        = zeros(2,Nu*Na);
b        = zeros(2,1);

A(1,idx) = -Lgh1;
A(2,idx) = -Lgh2;
b(1)     = Lfh1 + h1;
b(2)     = Lfh2 + h2;

A = round(A,12);
b = round(b,12);

end

function [A,b] = get_speed_constraints(t,x,settings)
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

function [A,b] = get_intersection_constraints(t,x,settings)
aa = settings.aa;
Nu = settings.Nu;
Na = settings.Na;
Lr = settings.Lr;
lw = settings.lw;
tSlots = settings.tSlots;
beta = settings.beta;

idx = (-1:0)+aa*Nu;

xx = [x(aa,:) beta(aa)];
A = []; b = [];
h = -10;

if t > tSlots(aa,1)
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
        
        xa = x(aa,:); xa = [xa beta(aa)];
        xi = x(ii,:); xi = [xi beta(ii)];
        
        idx_aa = (-1:0)+aa*Nu;
        idx_ii = (-1:0)+ii*Nu;

        Aw = []; bw = [];
        
        dx  = xa(1) - xi(1);
        dy  = xa(2) - xi(2);
        vax = xa(4)*cos(xa(3)) - xa(4)*sin(xa(3))*tan(xa(5))*Lr/(Lf+Lr);
        vay = xa(4)*sin(xa(3)) + xa(4)*cos(xa(3))*tan(xa(5))*Lr/(Lf+Lr);
        vix = xi(4)*cos(xi(3)) - xi(4)*sin(xi(3))*tan(xi(5))*Lr/(Lf+Lr);
        viy = xi(4)*sin(xi(3)) + xi(4)*cos(xi(3))*tan(xi(5))*Lr/(Lf+Lr);
        dvx = vax - vix;
        dvy = vay - viy;
        th  = xa(3) + xa(5);
        th_dot = xa(4)/Lr*tan(xa(5));
        Rot = [cos(th) sin(th); -sin(th) cos(th)];
        
        axa_unc = -xa(4)^2/Lr * (sin(xa(3)) + cos(xa(3))*tan(xa(5)))*tan(xa(5));
        axi_unc = -xi(4)^2/Lr * (sin(xi(3)) + cos(xi(3))*tan(xi(5)))*tan(xi(5));
        axa_con = [0 cos(xa(3))-sin(xa(3))*tan(xa(5))];
        axi_con = [0 cos(xi(3))-sin(xi(3))*tan(xi(5))];
        
        aya_unc = -xa(4)^2/Lr * (sin(xa(3))*tan(xa(5)) - cos(xa(3)))*tan(xa(5));
        ayi_unc = -xi(4)^2/Lr * (sin(xi(3))*tan(xi(5)) - cos(xi(3)))*tan(xi(5));
        aya_con = [0 sin(xa(3))+cos(xa(3))*tan(xa(5))];
        ayi_con = [0 sin(xi(3))+cos(xi(3))*tan(xi(5))];
        
        dax_unc = axa_unc - axi_unc;
        day_unc = aya_unc - ayi_unc;

        dl  = Rot(1,:) * [dx; dy];
        dw  = Rot(2,:) * [dx; dy];
        
        T   = 1.0;
        Sl  = sl + T * dot([dvx dvy],[dx dy]) / norm([dx dy]);
        
        % Define some derivatives
        dl_dot = (dvx + dy*th_dot)*cos(th) + (dvy - dx*th_dot)*sin(th);
        dw_dot = (dvy - dx*th_dot)*cos(th) - (dvx + dy*th_dot)*sin(th);
        
        Sl_dot_unc = T * ((dot([dax_unc day_unc],[dx dy]) + norm([dvx dvy])^2) * norm([dx dy]) - dot([dvx dvy],[dx dy]) * norm([dvx dvy])) / norm([dx dy])^2; ;
        SL_dot_con = zeros(1,Na*Nu);
        Sl_dot_con(idx_aa) = (dx*axa_con + dy*aya_con) * T / norm([dx dy]);
        Sl_dot_con(idx_ii) = [0 0];
        if ii < 4
            Sl_dot_con(idx_ii) = -(dx*axi_con + dy*ayi_con) * T / norm([dx dy]);
        end
        
        h   = (dl / (Sl))^2 + (dw / sw)^2 - 1;
        Lfh = 2 * (dl / Sl) * (dl_dot*Sl - dl*Sl_dot_unc) / Sl^2 + 2 / sw^2 * dw * dw_dot;
        Lgh = 2 * (dl / Sl) * (-dl*Sl_dot_con) / Sl^2;
        
        l0  = 3;
        Aw  = [Aw; -Lgh];
        bw  = [bw; Lfh + l0*h];
    end
    
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
