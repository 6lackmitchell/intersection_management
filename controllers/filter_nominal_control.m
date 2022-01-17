function [ufeas] = filter_nominal_control(uNom,uMax,Lfh,Lgh,alpha0_h)
%filter_nominal_control - modifies nominal control inputs for feasibility
%Detailed explanation here.
%
% Syntax:  [u0] = filter_nominal_control(uNom,uMax,Lfh,Lgh,alpha0_h)
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
% Nov 2021; Last revision: 11-Nov-2021
%------------- BEGIN CODE --------------% 
% Format A matrix
A1 = [Lgh(1)^2+2*Lgh(2)^2 -Lgh(1)*Lgh(2) Lgh(1)^2 Lgh(1)*Lgh(2)];
A2 = [-Lgh(1)*Lgh(2) 2*Lgh(1)^2+Lgh(2)^2 Lgh(1)*Lgh(2) Lgh(2)^2];
A3 = [-Lgh(1)^2 -Lgh(1)*Lgh(2) -Lgh(1)^2-2*Lgh(2)^2 Lgh(1)*Lgh(2)];
A4 = [-Lgh(1)*Lgh(2) -Lgh(2)^2 Lgh(1)*Lgh(2) -2*Lgh(1)^2-Lgh(2)^2];

% Format b matrix (depending on whether Lgh1 and Lgh2 are + or -)
if Lgh(1) > 0
    b1 = Lgh(1)*(Lfh + alpha0_h) + 2*uMax(1)*(Lgh(1)^2+Lgh(2)^2);
    b3 = Lgh(1)*(Lfh + alpha0_h) + 2*uMax(1)*(Lgh(1)^2+Lgh(2)^2);
else
    A1 = -A1; A3 = -A3;
    b1 = -Lgh(1)*(Lfh + alpha0_h) - 2*uMax(1)*(Lgh(1)^2+Lgh(2)^2);
    b3 = -Lgh(1)*(Lfh + alpha0_h) - 2*uMax(1)*(Lgh(1)^2+Lgh(2)^2);
end

if Lgh(2) > 0
    b2 = Lgh(2)*(Lfh + alpha0_h) + 2*uMax(2)*(Lgh(1)^2+Lgh(2)^2);
    b4 = Lgh(2)*(Lfh + alpha0_h) + 2*uMax(2)*(Lgh(1)^2+Lgh(2)^2);
else
    A2 = -A2; A4 = -A4;
    b2 = -Lgh(2)*(Lfh + alpha0_h) - 2*uMax(2)*(Lgh(1)^2+Lgh(2)^2);
    b4 = -Lgh(2)*(Lfh + alpha0_h) - 2*uMax(2)*(Lgh(1)^2+Lgh(2)^2);
end

% Form quadratic program
Q = eye(4); p = -2*uNom;
A = [A1; A2; A3; A4];
b = [b1; b2; b3; b4];

% Solve QP
[ufeas,fval,exitflag] = solve_quadratic_program(Q,p,A,b,[],[],-uMax,uMax);
if exitflag ~= 2
    ufeas = uNom;
end

% % Form linear programming problem
% f = zeros(4,1);
% A = [A1; A2; A3; A4];
% b = [b1; b2; b3; b4];
% 
% % Solve LP
% options = optimoptions('linprog','Display','off');
% [ufeas,fval,exitflag,output] = linprog(f,A,b,[],[],lb,ub,options);

% Maybe LMI later
% % Format Variables for LMI
% 
% 
% % Form LMI
% setlmis([])
% u = lmivar(2,[4,1]);
% lmiterm([1 1 1 0],B);
% lmiterm([1 1 1 u],-A,1);
% 
% % Solve LMI
% [tmin,ufeas] = feasp(lmisys);


end

