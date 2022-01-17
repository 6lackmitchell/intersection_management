function [k,code] = compute_class_k(uNom,uWorst,uMax,h,Lfh,Lgh,settings)
%compute_class_k - Computes parameter k for class K function k*h
%Detailed explanation here.
%
% Syntax:  [k] = compute_class_k(u_nom,worst_u,umax,umin,Lfh,Lgh)
%
% Inputs:
%    u_nom:        inputs here
%
% Outputs:
%    k: class k parameter -- float
%
% Example: 
%    k = compute_class_k(u_nom,worst_u,umax,umin,Lfh,Lgh)
%
% Other m-files required: others here
% Subfunctions: others
% MAT-files required: MAT files are generated for initial conditions
%
% Author: Mitchell Black
% Email: mblackjr@umich.edu
% Website: http://www.blackmitchell.com
% Oct 2021; Last revision: 22-Oct-2021
%------------- BEGIN CODE --------------% Load control params
code = 0;
k    = settings.knom;
Nu   = settings.Nu;
ke   = settings.ke;
Phi  = settings.Phi;

% Generate variables
C   = Lfh + Lgh*[uNom(1:2); uWorst];

% Phi = abs(Lgh(end-1:end)*uWorst);

kmax = zeros(2*Nu,1);
kmin = zeros(2*Nu,1);
for ii = 1:2*Nu
    if Lgh(ii) > 0
%         kmin(ii) = (2*Phi*exp(-ke*h) + (uNom(ii)-uMax(ii))/Lgh(ii)*sum(Lgh.^2) - C) / h;
%         kmax(ii) = (2*Phi*exp(-ke*h) + (uNom(ii)+uMax(ii))/Lgh(ii)*sum(Lgh.^2) - C) / h;
        kmin(ii) = (Phi*exp(-ke*h) + (uNom(ii)-uMax(ii))/Lgh(ii)*sum(Lgh.^2) - C) / h;
        kmax(ii) = (Phi*exp(-ke*h) - (uNom(ii)+uMax(ii))/Lgh(ii)*sum(Lgh.^2) - C) / h;
    else
%         kmax(ii) = (2*Phi*exp(-ke*h) + (uNom(ii)-uMax(ii))/Lgh(ii)*sum(Lgh.^2) - C) / h;
%         kmin(ii) = (2*Phi*exp(-ke*h) + (uNom(ii)+uMax(ii))/Lgh(ii)*sum(Lgh.^2) - C) / h;
        kmax(ii) = (Phi*exp(-ke*h) + (uNom(ii)-uMax(ii))/Lgh(ii)*sum(Lgh.^2) - C) / h;
        kmin(ii) = (Phi*exp(-ke*h) - (uNom(ii)+uMax(ii))/Lgh(ii)*sum(Lgh.^2) - C) / h;
    end
end

if isnan(min(kmax)) || isnan(max(kmin))
    disp(h)
end

if min(kmax) < max(kmin)
    code = 1;
    disp(h)
else
    m = 5;
    q = 0.01;

    p = 0.75; % This did fairly well
    p = 0.8; 
%     p = 1.*(1 - exp(-0.05*h));
    p = 2.*(1 - exp(-0.05*h));
    p = m.*(1 - exp(-q*h^2));
%     p = 10.*(1 - exp(-0.1*h));
    if min(kmax) > 0
        kmid = (p*min(kmax) + (m-p)*max(kmin) / (1 + (m-1)*exp(-(2*q)*h)));
    else
        kmid = min(kmax);
    end
%     kmid = mean([min(kmax),max(kmin)]); 
    k = kmid;
%     kmax = min(kmax)
%     kmin = max(kmin)
%     k
%     h

end


end

function [newx_coord,newy_coord] = rotate(matrix,vector)
    z          = vector*matrix;
    newx_coord = z(1);
    newy_coord = z(2);
end

