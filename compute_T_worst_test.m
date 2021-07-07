max_dVdq = 0.08;
d1_max   = 1/ 2 * 0.638;

d1 = d1_max * [0.1; 0.2; 0.3; 0.4; 0.5; 0.6; 0.7; 0.8; 0.9; 1.0];

T = [10; T0 + 2; 6];
phi_inf  = d1 / max_dVdq;
 
for i = 1:length(phi_inf)
%     Tw1 = compute_T_worst(T,2*d1(i));
    Tw2 = compute_T_worst(T,d1_max + d1(i));
%     sum(Tw1)
    sum(Tw2)
end




function [I] = compute_T_worst(T,c3)
    I = zeros(length(T),1);
    mu = 5;
    for t = 1:length(T)
        c1 = mu*pi/(2*T(t));
        c2 = c1;
        k1 = sqrt((4*c1*c2 - c3^2) / (4*c1^2));
        k2 = -c3 / sqrt(4*c1*c2 - c3^2);
        Vbar = c3 / (2*sqrt(c1*c2));
        
        I(t) = mu / (c1*k1) * (pi/2 - atan(k2));
        disp(['T' num2str(t) ': ' num2str(I(t))])
    end
        
end