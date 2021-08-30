function [A,b] = get_safety_constraints(t,x,aa,tSlots,k)
%GET_SAFETY_CONSTRAINTS This is where the safety measures are considered
%   The relevant CBFs are taken into account here.

% B     = (x(aa,1)^2 + x(aa,2)^2) - 1.25^2;
% Bdot  = 2*x(aa,1)*x(aa,4)*cos(x(aa,3)) + 2*x(aa,2)*x(aa,4)*sin(x(aa,3));
% L2fB  = (k1+k2)*Bdot + k1*k2*B;
% LfLgB = [-2*x(aa,1)*x(aa,4)*sin(x(aa,3)) + 2*x(aa,2)*x(aa,4)*cos(x(aa,3));
%          2*x(aa,1)*cos(x(aa,3)) + 2*x(aa,2)*sin(x(aa,3))];
% 
% A1    = [-LfLgB(1) -LfLgB(2) 0 0 0 0];
% b1    = L2fB;

A = [];
b = [];
return

Nu = 2;
Na = 6;

[A1,b1] = get_speed_constraints(t,x,aa,Nu,Na);

if aa >= 4
    A = A1; b = b1;
    return
end

[A2,b2] = get_road_constraints(t,x,aa,Nu,Na);
[A3,b3] = get_intersection_constraints(t,x,aa,tSlots,Nu,Na);
[A4,b4] = get_interagent_constraints(t,x,aa,Nu,Na,k);

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
% HO-CBF technique taken from Breeden 2021
a_max = 1;

% h1    = lw - x(1);
% h2    = x(1);
% 
% h1dot = -x(4)*cos(x(3));
% h2dot = -h1dot;
% 
% if h1dot > 0
%     LfH1 = 0;
%     LgH1 = [x(4)*sin(x(3)) -cos(x(3))];
% else
%     h1
% end
% 
% if h2dot > 0
%     
% else
%     
% end

h    = lw*x(1) - x(1)^2;
hdot = lw*x(4)*cos(x(3)) - 2*x(1)*x(4)*cos(x(3));

C    = abs(hdot) / a_max;
H    = h + abs(hdot)*hdot / 2*a_max;
LfH  = hdot - 2*(x(4)*cos(x(3)))^2 * C;
LgH  = [(2*x(1)*x(4)*sin(x(3)) - lw*x(4)*sin(x(3))) (lw*cos(x(3)) - 2*x(1)*cos(x(3)))] * C;

A              = zeros(1,Nu*Na);
A(2*aa+(-1:0)) = [-LgH(1) -LgH(2)];
b              = LfH + H;

A = round(A,12);
b = round(b,12);

end

function [A,b] = get_ESWSroad_constraint(t,x,aa,lw,Nu,Na)

a_max = 1;

h    = -lw*x(2) - x(2)^2;
hdot = -lw*x(4)*sin(x(3)) - 2*x(2)*x(4)*sin(x(3));

C    = abs(hdot) / a_max;
H    = h + abs(hdot)*hdot / 2*a_max;
LfH  = hdot - 2*(x(4)*sin(x(3)))^2 * C;
LgH  = [(-2*x(2)*x(4)*cos(x(3)) - lw*x(4)*cos(x(3))) (-lw*sin(x(3)) - 2*x(2)*sin(x(3)))] * C;

A              = zeros(1,Nu*Na);
A(2*aa+(-1:0)) = [-LgH(1) -LgH(2)];
b              = LfH + H;

A = round(A,12);
b = round(b,12);
end

function [A,b] = get_ENWNroad_constraint(t,x,aa,lw,Nu,Na)

a_max = 1;

h    = lw*x(2) - x(2)^2;
hdot = lw*x(4)*sin(x(3)) - 2*x(2)*x(4)*sin(x(3));

C    = abs(hdot) / a_max;
H    = h + abs(hdot)*hdot / 2*a_max;
LfH  = hdot - 2*(x(4)*sin(x(3)))^2 * C;
LgH  = [(2*x(2)*x(4)*cos(x(3)) - lw*x(4)*cos(x(3))) (lw*sin(x(3)) - 2*x(2)*sin(x(3)))] * C;

A              = zeros(1,Nu*Na);
A(2*aa+(-1:0)) = [-LgH(1) -LgH(2)];
b              = LfH + H;

A = round(A,12);
b = round(b,12);
end

function [A,b] = get_SWNWroad_constraint(t,x,aa,lw,Nu,Na)

a_max = 1;

h    = -lw*x(1) - x(1)^2;
hdot = -lw*x(4)*cos(x(3)) - 2*x(1)*x(4)*cos(x(3));

C    = abs(hdot) / a_max;
H    = h + abs(hdot)*hdot / 2*a_max;
LfH  = hdot - 2*(x(4)*cos(x(3)))^2 * C;
LgH  = [(-2*x(1)*x(4)*sin(x(3)) - lw*x(4)*sin(x(3))) (-lw*cos(x(3)) - 2*x(1)*cos(x(3)))] * C;

A              = zeros(1,Nu*Na);
A(2*aa+(-1:0)) = [-LgH(1) -LgH(2)];
b              = LfH + H;

A = round(A,12);
b = round(b,12);
end

function [A,b] = get_speed_constraints(t,x,aa,Nu,Na)

SL = 11; % Speed limit in m/s

% Case where safety is computed centrally
if aa == 0
    % Centralized safety
else
    xx = x(aa,:);
    
    h    = SL - xx(4);
    Lfh  = 0;
    Lgh  = [0 -1];
    
    A              = zeros(1,Nu*Na);
    A(2*aa+(-1:0)) = [-Lgh(1) -Lgh(2)];
    b    = Lfh + h;
    
end

A = round(A,12);
b = round(b,12);

end

function [A,b] = get_intersection_constraints(t,x,aa,tSlots,Nu,Na)
lw    = 3;

% Case where safety is computed centrally
if aa == 0
    % Centralized safety
else
    xx = x(aa,:);
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
        hdot = -xx(4)*sin(xx(3));
        Lgh  = [-xx(4)*cos(xx(3)) -sin(xx(3))];
        
    elseif xx(2) > 0 && xx(2) < lw && xx(1) > lw
%         wM  = pi / 4;
%         aM  = 2 * 9.81;
%         k   = 1.0;
%         h   = aM*cos(xx(3)) - xx(4)*wM*sin(xx(3)) - (xx(4)*cos(xx(3)))^2/(4*(xx(1) - lw));
%         P   = (4*(xx(1) - lw) - 4*(xx(4)*cos(xx(3)))^3) / (4*(xx(1)-lw))^3;
%         Q   = 2*xx(4)*cos(xx(3))^2 * P;
%         R   = 2*xx(4)^2*sin(xx(3))*cos(xx(3))*P;
%         Lgh = [-aM*sin(xx(3))-xx(4)*wM*cos(xx(3))+R; -wM*sin(xx(3))-Q];
%         
%         A              = zeros(1,8);
%         A(2*aa+(-1:0)) = [-Lgh(1) -Lgh(2)];
%         b              = k*h;
%         A = round(A,12);
%         b = round(b,12);
%         return
        
        h    = xx(1) - lw;
        hdot = xx(4)*cos(xx(3));
        Lgh  = [-xx(4)*sin(xx(3)) cos(xx(3))];
        
    elseif xx(1) < 0 && xx(1) > -lw && xx(2) > lw
        h    = xx(2) - lw;
        hdot = xx(4)*sin(xx(3));
        Lgh  = [xx(4)*cos(xx(3)) sin(xx(3))];
        
    elseif xx(2) < 0 && xx(2) > -lw && xx(1) < -lw
        h    = -xx(1) - lw;
        hdot = -xx(4)*cos(xx(3));
        Lgh  = [xx(4)*sin(xx(3)) -cos(xx(3))];
%     else
%         return
    end
    
    [A,b] = get_intersection_constraint(aa,h,hdot,Lgh,Nu,Na);
    
    
end

end

function [A,b] = get_intersection_constraint(aa,h,hdot,Lgh,Nu,Na)
k     = 1.0;
a_max = 10.0;

C     = abs(hdot) / a_max;
H     = h + hdot / 2 * C;
LfH   = hdot;
LgH   = Lgh * C; 

A              = zeros(1,Nu*Na);
A(2*aa+(-1:0)) = [-LgH(1) -LgH(2)];
b              = LfH + k*H;

A = round(A,12);
b = round(b,12);
end

function [A,b] = get_interagent_constraints(t,x,aa,Nu,Na,k)
% k     = 3.0;
R     = 1.0;
jj    = 1;
A     = zeros(Na-1,Na*Nu);
b     = zeros(Na-1,1);

aM    = 2*9.81;
wM    = pi / 4;

for ii = 1:Na
    if ii == aa 
        continue
    end

%     D    = (x(aa,1) - x(ii,1))^2 + (x(aa,2) - x(ii,2))^2;
%     dx   = x(aa,1)-x(ii,1);
%     dvx  = x(aa,4)*cos(x(aa,3))-x(ii,4)*cos(x(ii,3));
%     dy   = x(aa,2)-x(ii,2);
%     dvy  = x(aa,4)*sin(x(aa,3))-x(ii,4)*sin(x(ii,3));
%     
%     h    = sqrt(D) - R;
%     hdot = (dx*dvx + dy*dvy) / sqrt(D);
%     Lfh  = (sqrt(D) * (dvx^2 + dvy^2) + (dx*dvx + dy*dvy)*hdot) / D;
%     Lgh  = [-dx*x(aa,4)*sin(x(aa,3)) + dy*x(aa,4)*cos(x(aa,3));
%              dx*cos(x(aa,3)) + dy*sin(x(aa,3))] * sqrt(D) / D;
     
    th       = x(aa,3);
    v        = x(aa,4);
    dx       = x(aa,1)-x(ii,1);
    dvx      = x(aa,4)*cos(x(aa,3))-x(ii,4)*cos(x(ii,3));
    dy       = x(aa,2)-x(ii,2);
    dvy      = x(aa,4)*sin(x(aa,3))-x(ii,4)*sin(x(ii,3));
    
    xddot_M  = sqrt((dx*aM*cos(th))^2) + sqrt((dx*wM*v*sin(th))^2);
    yddot_M  = sqrt((dy*aM*sin(th))^2) + sqrt((dy*wM*v*cos(th))^2);
    
    h        = dx^2 + dy^2 - R^2;
    hdot     = 2*(dx*dvx + dy*dvy);
    hddot_M  = 2*(dvx^2 + dvy^2) + 2*xddot_M + 2*yddot_M;
    
    hddot_unc = 2*(dvx^2 + dvy^2);
    hddot_con = 2*[-dx*v*sin(th)+dy*v*cos(th) ...
                    dx*cos(th)+dy*sin(th)];
   
    hddot_M_unc = sign(dx*cos(th))*(dvx*aM*cos(th)) + ...
                  sign(dx*v*sin(th))*(dvx*wM*v*sin(th)) + ...
                  sign(dy*sin(th))*(dvy*aM*sin(th)) + ...
                  sign(dy*v*cos(th))*(dvy*wM*v*cos(th));
    hddot_M_con = [-4*dvx*v*sin(th)+4*dvy*v*cos(th)-sign(dx*cos(th))*dx*aM*sin(th)+sign(dx*v*sin(th))*dx*wM*v*cos(th)+sign(dy*sin(th))*dy*aM*cos(th)-sign(dy*v*cos(th))*dy*wM*v*sin(th)...
                    4*dvx*cos(th)+4*dvy*sin(th)+sign(dx*v*sin(th))*dx*wM*sin(th)+sign(dy*wM*v*cos(th))*dy*wM*cos(th)];
%     hddot_M_unc = 4*dvx*xddot_M+4*dvy*yddot_M;
%     hddot_M_con = 4*[-dvx*v*sin(th)+dvy*v*cos(th)+...
%                      -dx*sign(dx*cos(th))*aM*sin(th)+dx*sign(dx*v*sin(th))*wM*v*cos(th)+...
%                       dy*sign(dy*sin(th))*aM*cos(th)+dy*sign(dy*v*cos(th))*(-wM*v*sin(th)) ...
%                       dvx*cos(th)+dvy*sin(th)+dx*sign(dx*v*sin(th))*wM*sin(th)+dy*sign(dy*v*cos(th))*wM*cos(th)];
    
    H   = 2*hddot_M*h - hdot^2;
    LfH = 2*(hddot_M_unc*h + hddot_M*hdot) - 2*hdot*hddot_unc;
    LgH = 2*(hddot_M_con*h + 0           ) - 2*hdot*hddot_con;
    
%     Lfh  = 2*(dvx^2 + dvy^2);
%     Lgh  = [x(aa,4)*(dy*cos(x(aa,3)) - dx*sin(x(aa,3))); dx*cos(x(aa,3)) + dy*sin(x(aa,3))] * 2;
% 
%     a_max = max(h/5,1);
%     acc_max = 2 * 9.81;
%     a_max = sqrt(2)*R*acc_max;
%     C     = abs(hdot) / a_max;
%     H     = h + hdot / 2 * C;
%     LfH   = hdot + Lfh * C;
%     LgH   = Lgh * C; 
    
    A(jj,2*aa+(-1:0)) = [-LgH(1) -LgH(2)];
    b(jj)             = LfH + k*H;
    
    jj = jj + 1;
end

A = round(A,12);
b = round(b,12);
end
