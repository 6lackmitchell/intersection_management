function [A,b] = get_safety_constraints_2(t,x,aa,tSlots,uLast)
%GET_SAFETY_CONSTRAINTS This is where the safety measures are considered
%   The relevant CBFs are taken into account here.

Nu = 2;
Na = 6;

Lr = 1.0;
Lf = 1.0;

beta = atan(Lr/(Lr+Lf)*uLast(aa));

[A1,b1] = get_speed_constraints(t,x,aa,Nu,Na);
[A2,b2] = get_road_constraints(t,x,aa,Nu,Na);
[A3,b3] = get_interagent_constraints(t,x,aa,uLast,Nu,Na);

A = [A1; A2; A3];
b = [b1; b2; b3];

if aa >= 4
    return
end

[A4,b4] = get_intersection_constraints(t,x,aa,tSlots,beta,Nu,Na);
A = [A; A4];
b = [b; b4];

% if t < tSlots(aa,1)
%     [A3,b3] = get_intersection_constraints(t,x,aa);
%     A2 = [A2; A3];
%     b2 = [b2; b3];
% end

% [A2,b2] = get_interagent_constraints(t,x,aa);


A = [A1; A2; A3; A4];
b = [b1; b2; b3; b4];


end

function [A,b] = get_road_constraints(t,x,aa,Nu,Na)
lw    = 3;

% Case where safety is computed centrally
if aa == 0
    % Centralized safety
else
    xx = x(aa,:); ii = 1;
    A = []; b = [];

    if xx(1) < lw && xx(1) > -lw && xx(2) < lw && xx(2) > -lw
        return
    elseif xx(1) > 0 && xx(1) < lw
        [A,b] = get_SENEroad_constraint(t,xx,aa,lw,Nu,Na);
    elseif xx(2) > 0 && xx(2) < lw
        [A,b] = get_ENWNroad_constraint(t,xx,aa,lw,Nu,Na);
    elseif xx(2) < 0 && xx(2) > -lw
        [A,b] = get_ESWSroad_constraint(t,xx,aa,lw,Nu,Na);
    elseif xx(1) < 0 && xx(1) > -lw
        [A,b] = get_SWNWroad_constraint(t,xx,aa,lw,Nu,Na);
    end

end

end

function [A,b] = get_SENEroad_constraint(t,x,aa,lw,Nu,Na)

Lr = 1.0;
Lf = 1.0;

h1     = lw - x(1);
Lfh1   = -x(4)*cos(x(3));
Lgh1   = [x(4)*sin(x(3)) 0]*Lr/(Lr+Lf);

h2     = x(1);
Lfh2   = x(4)*cos(x(3));
Lgh2   = [-x(4)*sin(x(3)) 0]*Lr/(Lr+Lf);

A      = zeros(2,Nu);
b      = zeros(2,1);

A(1,:) = -Lgh1;
A(2,:) = -Lgh2;
b(1)   = Lfh1 + h1;
b(2)   = Lfh2 + h2;

A = round(A,12);
b = round(b,12);

end

function [A,b] = get_ESWSroad_constraint(t,x,aa,lw,Nu,Na)
Lr = 1.0;
Lf = 1.0;
h1     = lw + x(2);
Lfh1   = x(4)*sin(x(3));
Lgh1   = [x(4)*cos(x(3)) 0]*Lr/(Lr+Lf);

h2     = -x(2);
Lfh2   = -x(4)*sin(x(3));
Lgh2   = [-x(4)*cos(x(3)) 0]*Lr/(Lr+Lf);

A      = zeros(2,Nu);
b      = zeros(2,1);

A(1,:) = -Lgh1;
A(2,:) = -Lgh2;
b(1)   = Lfh1 + h1;
b(2)   = Lfh2 + h2;

A = round(A,12);
b = round(b,12);

end

function [A,b] = get_ENWNroad_constraint(t,x,aa,lw,Nu,Na)
Lr = 1.0;
Lf = 1.0;
h1     = lw - x(2);
Lfh1   = -x(4)*sin(x(3));
Lgh1   = [-x(4)*cos(x(3)) 0]*Lr/(Lr+Lf);

h2     = x(2);
Lfh2   = x(4)*sin(x(3));
Lgh2   = [x(4)*cos(x(3)) 0]*Lr/(Lr+Lf);

A      = zeros(2,Nu);
b      = zeros(2,1);

A(1,:) = -Lgh1;
A(2,:) = -Lgh2;
b(1)   = Lfh1 + h1;
b(2)   = Lfh2 + h2;

A = round(A,12);
b = round(b,12);

end

function [A,b] = get_SWNWroad_constraint(t,x,aa,lw,Nu,Na)
Lr = 1.0;
Lf = 1.0;
h1     = lw + x(1);
Lfh1   = x(4)*cos(x(3));
Lgh1   = [-x(4)*sin(x(3)) 0]*Lr/(Lr+Lf);

h2     = -x(1);
Lfh2   = -x(4)*cos(x(3));
Lgh2   = [x(4)*sin(x(3)) 0]*Lr/(Lr+Lf);

A      = zeros(2,Nu);
b      = zeros(2,1);

A(1,:) = -Lgh1;
A(2,:) = -Lgh2;
b(1)   = Lfh1 + h1;
b(2)   = Lfh2 + h2;

A = round(A,12);
b = round(b,12);

end

function [A,b] = get_speed_constraints(t,x,aa,Nu,Na)
% this is now an input constraint
% A = []; b = [];
% return
run('physical_params.m')
% SL = 15; % Speed limit in m/s

% Case where safety is computed centrally
if aa == 0
    % Centralized safety
else
    xx = x(aa,:);

    h    = SL - xx(4);
    Lfh  = 0;
    Lgh  = [0 -1];

%     A              = zeros(1,Nu*Na);
%     A(2*aa+(-1:0)) = [-Lgh(1) -Lgh(2)];
    A = -Lgh;
    b = Lfh + h;

end

A = round(A,12);
b = round(b,12);

end

function [A,b] = get_intersection_constraints(t,x,aa,tSlots,beta,Nu,Na)
lw   = 3;
Lr   = 1.0;

% Case where safety is computed centrally
if aa == 0
    % Centralized safety
else
    xx = [x(aa,:) beta];
    A = []; b = [];

%     if t > tSlots(aa,1)%xx(1) < lw && xx(1) > -lw && xx(2) < lw && xx(2) > -lw
%         return
%     elseif xx(1) > 0 && xx(1) < lw && xx(2) < -lw
%         h    = -xx(2) - lw;
%         hdot = -xx(4)*sin(xx(3));
%         Lgh  = [-xx(4)*cos(xx(3)) -sin(xx(3))];
%
%     elseif xx(2) > 0 && xx(2) < lw && xx(1) > lw
%         h    = xx(1) - lw;
%         hdot = xx(4)*cos(xx(3));
%         Lgh  = [-xx(4)*sin(xx(3)) cos(xx(3))];
%
%     elseif xx(1) < 0 && xx(1) > -lw && xx(2) > lw
%         h    = xx(2) - lw;
%         hdot = xx(4)*sin(xx(3));
%         Lgh  = [xx(4)*cos(xx(3)) sin(xx(3))];
%
%     elseif xx(2) < 0 && xx(2) > -lw && xx(1) < -lw
%         h    = -xx(1) - lw;
%         hdot = -xx(4)*cos(xx(3));
%         Lgh  = [xx(4)*sin(xx(3)) -cos(xx(3))];
% %     else
% %         return
%     end
    h = -10;
    if t > tSlots(aa,1)%xx(1) < lw && xx(1) > -lw && xx(2) < lw && xx(2) > -lw
        return
    elseif xx(1) > 0 && xx(1) < lw && xx(2) < -lw
        h    = -xx(2) - lw;
        Lfh  = 0;
        Lgh  = [-sin(xx(3)) 0];
        
        h    = -xx(2) - lw;
        Lfh  = -(xx(4)*sin(xx(3)) + xx(4)*cos(xx(3))*tan(xx(5)));
        Lf2h = -(xx(4)^2/Lr * (cos(xx(3))*tan(xx(5)) - sin(xx(3))*tan(xx(5))^2));
        LgLf = -[0 sin(xx(3)) + cos(xx(3))*tan(xx(5))];
        
        % hdot = -xx(4)*sin(xx(3));
        % Lgh  = [-xx(4)*cos(xx(3)) -sin(xx(3))];

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
        
        
        
        
%         Lfh  = 0;
%         Lgh  = [sin(xx(3)) 0];
%         % hdot = xx(4)*sin(xx(3));
%         % Lgh  = [xx(4)*cos(xx(3)) sin(xx(3))];

    elseif xx(2) < 0 && xx(2) > -lw && xx(1) < -lw
        h    = -xx(1) - lw;
        Lfh  = -(xx(4)*cos(xx(3)) - xx(4)*sin(xx(3))*tan(xx(5)));
        Lf2h = -(xx(4)^2/Lr * (sin(xx(3))*tan(xx(5)) - cos(xx(3))*tan(xx(5))^2));
        LgLf = -[0 cos(xx(3)) - sin(xx(3))*tan(xx(5))];
        
    end

    % [A,b] = get_intersection_constraint(aa,h,hdot,Lgh,Nu,Na);
%     [A,b] = get_intersection_constraint(aa,h,Lfh,Lgh,Nu,Na);
    [A,b] = get_intersection_constraint(aa,h,Lfh,Lf2h,LgLf,Nu,Na);


end

end

% function [A,b] = get_intersection_constraint(aa,h,hdot,Lgh,Nu,Na)
function [A,b] = get_intersection_constraint(aa,h,Lfh,Lf2h,LgLf,Nu,Na)
l0 = 9.0;
l0 = 25.0;
l1 = 10.0;

% These worked as of 1:53PM Sep 1
l0 = 5.0;
l1 = 25.0;

% Experimental
l1 = 5.0;
l0 = l1^2 / 4;

A  = -LgLf;
b  = Lf2h + l1*Lfh + l0*h;

% A              = zeros(1,Nu);
% A(2*aa+(-1:0)) = -Lgh;
% b              = Lfh + k*h;

A = round(A,12);
b = round(b,12);
end

function [A,b] = get_interagent_constraints(t,x,aa,uLast,Nu,Na)
% k     = 3.0;
R     = 1.0;
jj    = 1;

Lr = 1.0;
Lf = 1.0;

% A     = zeros(Na-1,Nu);
% b     = zeros(Na-1,1);
A = []; b = [];

for ii = 1:Na
    if ii == aa
        continue
    end
    
    % One circle for now, possibly 2 later on
    RR = 1.0;
    RR = 1.0;
    
%     for cc = 1:2
%         [Aw,bw] = get_constraint_circle(cx
%     end

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
            
            RR = 1.0;
            dx = cx1 - cx2;
            dy = cy1 - cy2;
            dvx = (x(aa,4)*cos(x(aa,3)) - x(aa,4)*sin(x(aa,3))*tan_gamma1) - (x(ii,4)*cos(x(ii,3)) - x(ii,4)*sin(x(ii,3))*tan_gamma2);
            dvy = (x(aa,4)*sin(x(aa,3)) + x(aa,4)*cos(x(aa,3))*tan_gamma1) - (x(ii,4)*sin(x(ii,3)) + x(ii,4)*cos(x(ii,3))*tan_gamma2);
            dax = x(aa,4)^2/Lr1*(-sin(x(aa,3))*tan_gamma1 - cos(x(aa,3))*tan_gamma1^2) - x(ii,4)^2/Lr2*(-sin(x(ii,3))*tan_gamma2 - cos(x(ii,3))*tan_gamma2^2);
            day = x(aa,4)^2/Lr1*( cos(x(aa,3))*tan_gamma1 - sin(x(aa,3))*tan_gamma1^2) - x(ii,4)^2/Lr2*( cos(x(ii,3))*tan_gamma2 - sin(x(ii,3))*tan_gamma2^2);
            
            h    = dx^2 + dy^2 - RR^2;
            Lfh  = 2*dx*dvx + 2*dy*dvy;
            Lf2h = 2*dvx^2 + 2*dvy^2 + 2*dx*dax + 2*dy*day;
            LgLf = [0 2*dx*(cos(x(aa,3)) - sin(x(aa,3))*tan_gamma1)+2*dy*(sin(x(aa,3)) + cos(x(aa,3))*tan_gamma1)];
            
            % These work 1:15pm Sep 1
            l0 = 10.0;
            l1 = 20.0;
            
            l0 = 20.0;
            l1 = 20.0;
            
            l0 = 80.0;
            l1 = 20.0;
            
            l0 = 200.0;
            l1 = 30.0;
            
            % Extremely conservative
            l0 = 0.25;
            l1 = 1.0;
            
            l1 = 10.0;
            l0 = l1^2 / 4;
            
            l1 = 50.0;
            l0 = l1^2 / 4;
            
            if aa > 3
                l1 = 1.125;
                l1 = 3.0;
                l0 = min(8,l1^2 / 4);
            end
            
            % Experiment
            max_l1    = 10^6;
            h2dot_max = Lf2h + LgLf*[0; -1*9.81];
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

A = round(A,12);
b = round(b,12);
end
