function [A,b] = get_safety_constraints_1(t,x,aa,uLast)
%GET_SAFETY_CONSTRAINTS This is where the safety measures are considered
%   The relevant CBFs are taken into account here.

Nu = 2;
Na = 6;

A = [];
b = [];
if aa >= 4
    return
end

[A,b] = get_interagent_constraints(t,x,aa,uLast,Nu,Na);

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
            
            
            
            dvx = x(aa,4)*cos(x(aa,3)) - x(ii,4)*cos(x(ii,3));
            dvy = x(aa,4)*sin(x(aa,3)) - x(ii,4)*sin(x(ii,3));
            
            h    = dx^2 + dy^2 - RR^2;
            Lfh  = 2*dx*dvx + 2*dy*dvy;
            Lgh  = [-2*dx*x(aa,4)*sin(x(aa,3))+2*dy*x(aa,4)*cos(x(aa,3)) 0 1/2]*(Lr1+Lf1)/Lr1;
            l0   = 2.0;

            Aw  = [Aw; -Lgh];
            bw  = [bw; Lfh + l0*h + 1 / (0.1 * x(aa,4) + 0.1)^3];
            
        end
        
    end
    
    A = [A; Aw];
    b = [b; bw];

%     th       = x(aa,3);
%     dx       = x(aa,1)-x(ii,1);
%     dy       = x(aa,2)-x(ii,2);
% 
%     h        = dx^2 + dy^2 - R^2;
%     Lfh      = 0;
%     Lgh      = [2*dx*cos(th) 2*dy*sin(th)];
% 
%     A(jj,2*aa+(-1:0)) = -Lgh;
%     b(jj)             = Lfh + k*h;
% 
%     jj = jj + 1;
end

A = round(A,12);
b = round(b,12);
end
