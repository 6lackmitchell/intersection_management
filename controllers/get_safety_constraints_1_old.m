function [A,b] = get_safety_constraints_1(t,x,aa,tSlots,k)
%GET_SAFETY_CONSTRAINTS This is where the safety measures are considered
%   The relevant CBFs are taken into account here.

Nu = 2;
Na = 6;

[A1,b1] = get_following_distance_constraint(t,x,aa,Nu,Na)
A = A1; b = b1;
return

[A1,b1] = get_speed_constraints(t,x,aa,Nu,Na);


if aa >= 4
    A = A1; b = b1;
    return
end

[A2,b2] = get_intersection_constraints(t,x,aa,tSlots,Nu,Na);
A = [A1; A2]; b = [b1; b2];
return

[A3,b3] = get_road_constraints(t,x,aa,Nu,Na);
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

h1    = lw - x(1);
Lfh1  = 0;
Lgh1  = [-cos(x(3)) 0];

h2    = x(1);
Lfh2  = 0;
Lgh2  = [cos(x(3)) 0];

A                = zeros(2,Nu*Na);
b                = zeros(2,1);

A(1,2*aa+(-1:0)) = -Lgh1;
A(2,2*aa+(-1:0)) = -Lgh2;
b(1)             = Lfh1 + h1;
b(2)             = Lfh2 + h2;

A = round(A,12);
b = round(b,12);

end

function [A,b] = get_ESWSroad_constraint(t,x,aa,lw,Nu,Na)

h1    = lw + x(2);
Lfh1  = 0;
Lgh1  = [sin(x(3)) 0];

h2    = -x(2);
Lfh2  = 0;
Lgh2  = [-sin(x(3)) 0];

A                = zeros(2,Nu*Na);
b                = zeros(2,1);

A(1,2*aa+(-1:0)) = -Lgh1;
A(2,2*aa+(-1:0)) = -Lgh2;
b(1)             = Lfh1 + h1;
b(2)             = Lfh2 + h2;

A = round(A,12);
b = round(b,12);

end

function [A,b] = get_ENWNroad_constraint(t,x,aa,lw,Nu,Na)

h    = lw*x(2) - x(2)^2;
hdot = lw*x(4)*sin(x(3)) - 2*x(2)*x(4)*sin(x(3));

h1    = lw - x(2);
Lfh1  = 0;
Lgh1  = [-sin(x(3)) 0];

h2    = x(2);
Lfh2  = 0;
Lgh2  = [sin(x(3)) 0];

A                = zeros(2,Nu*Na);
b                = zeros(2,1);

A(1,2*aa+(-1:0)) = -Lgh1;
A(2,2*aa+(-1:0)) = -Lgh2;
b(1)             = Lfh1 + h1;
b(2)             = Lfh2 + h2;

A = round(A,12);
b = round(b,12);

end

function [A,b] = get_SWNWroad_constraint(t,x,aa,lw,Nu,Na)

h    = -lw*x(1) - x(1)^2;
hdot = -lw*x(4)*cos(x(3)) - 2*x(1)*x(4)*cos(x(3));

h1    = lw + x(1);
Lfh1  = 0;
Lgh1  = [cos(x(3)) 0];

h2    = -x(1);
Lfh2  = 0;
Lgh2  = [-cos(x(3)) 0];

A                = zeros(2,Nu*Na);
b                = zeros(2,1);

A(1,2*aa+(-1:0)) = -Lgh1;
A(2,2*aa+(-1:0)) = -Lgh2;
b(1)             = Lfh1 + h1;
b(2)             = Lfh2 + h2;

A = round(A,12);
b = round(b,12);

end

function [A,b] = get_speed_constraints(t,x,aa,Nu,Na)
% this is now an input constraint
% A = []; b = [];
% return
SL = 10; % Speed limit in m/s

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
    A = [-Lgh(1) -Lgh(2)];
    b = Lfh + h;

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
        Lfh  = 0;
        Lgh  = [-sin(xx(3)) 0];
        % hdot = -xx(4)*sin(xx(3));
        % Lgh  = [-xx(4)*cos(xx(3)) -sin(xx(3))];

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
        Lfh  = 0;
        Lgh  = [cos(xx(3)) 0];
        % hdot = xx(4)*cos(xx(3));
        % Lgh  = [-xx(4)*sin(xx(3)) cos(xx(3))];

    elseif xx(1) < 0 && xx(1) > -lw && xx(2) > lw
        h    = xx(2) - lw;
        Lfh  = 0;
        Lgh  = [sin(xx(3)) 0];
        % hdot = xx(4)*sin(xx(3));
        % Lgh  = [xx(4)*cos(xx(3)) sin(xx(3))];

    elseif xx(2) < 0 && xx(2) > -lw && xx(1) < -lw
        h    = -xx(1) - lw;
        Lfh  = 0;
        Lgh  = [-cos(xx(3)) 0];
        % hdot = -xx(4)*cos(xx(3));
        % Lgh  = [xx(4)*sin(xx(3)) -cos(xx(3))];
%     else
%         return
    end

    % [A,b] = get_intersection_constraint(aa,h,hdot,Lgh,Nu,Na);
    [A,b] = get_intersection_constraint(aa,h,Lfh,Lgh,Nu,Na);


end

end

% function [A,b] = get_intersection_constraint(aa,h,hdot,Lgh,Nu,Na)
function [A,b] = get_intersection_constraint(aa,h,Lfh,Lgh,Nu,Na)
k     = 1.0;

A              = zeros(1,Nu*Na);
A(2*aa+(-1:0)) = -Lgh;
b              = Lfh + k*h;

A = round(A,12);
b = round(b,12);
end

function [A,b] = get_interagent_constraints(t,x,aa,Nu,Na,k)
% k     = 3.0;
R     = 1.0;
jj    = 1;
A     = zeros(Na-1,Na*Nu);
b     = zeros(Na-1,1);

for ii = 1:Na
    if ii == aa
        continue
    end

    th       = x(aa,3);
    dx       = x(aa,1)-x(ii,1);
    dy       = x(aa,2)-x(ii,2);

    h        = dx^2 + dy^2 - R^2;
    Lfh      = 0;
    Lgh      = [2*dx*cos(th) 2*dy*sin(th)];

    A(jj,2*aa+(-1:0)) = -Lgh;
    b(jj)             = Lfh + k*h;

    jj = jj + 1;
end

A = round(A,12);
b = round(b,12);
end
