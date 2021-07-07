function [u,d] = distributed_cbf_qp(t,x,xo,v_track,cost,dynamics)
%clf_cbf_qp - Controller based on CLF-CBF-QP Formulation
%This controller solves a Quadratic Program with a Control Barrier
%Function (CBF)-based safety constraint and a Control Lyapunov Function
%(CLF)-based performance constraint. 
%
% Syntax:  [u] = clf_cbf_qp(t,x)
%
% Inputs:
%    t - current time in sec
%    x - current state vector -- ROW vector
%
% Outputs:
%    u - control input - ROW vector
%
% Example: 
%    [u(k+1,:),d] = clf_cbf_qp(t,x)
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: OTHER_FUNCTION_NAME1,  OTHER_FUNCTION_NAME2
% Author: Mitchell Black
% Email: mblackjr@umich.edu
% Website: http://www.blackmitchell.com
% May 2021; Last revision: 29-May-2021
%------------- BEGIN CODE --------------
run('control_params.m')
options = optimoptions('quadprog','Display','off');

[f,g] = dynamics.fg;

% Specify u_nom
u_nom = (v_track - x(:,3:4)') / Tc;

% Load Optimization Cost Fcn
[Q,p] = cost(u_nom,q,Nu,0,Ns);

% Control Constraints
Ac  = kron(eye(Nu),[1; -1]);
Ac  = [Ac zeros(2*Nu,Ns)];
bc  = u_max*ones(2*Nu,1);

% Class K Functions -- alpha(B) = a*B
Ak  = [zeros(Ns,Nu) -eye(Ns)];
bk  = ones(Ns,1);

% Safety Constraints -- Traditional CBF Condition
LfB = dBdx(t,x,xo) * f(t,x);
LgB = dBdx(t,x,xo) * g(t,x);
FB  = B(t,x,xo);
As  = [-LgB(:,1:Nu) -eye(length(FB)).*FB];
bs  = LfB;

% 2nd Order CBF w/ Input Constraints
LfB = dBdx(t,x,xo) * f(t,x);
LgB = dBdx(t,x,xo) * g(t,x);

% Constraint Matrix
A = [Ac; Ak; As];
b = [bc; bk; bs];
% A = [Ac; Ak];
% b = [bc; bk];

if sum(any(isnan(A))) || any(isnan(b))
    A
    b
end
    
% Solve Optimization problem
% 1/2*x^T*Q*x + p*x subject to Ax <= b
sol = quadprog(Q,p,A,b,[],[],[],[],[],options);
if isempty(sol)
    A
    b
end
u   = sol(1:Nu);
d   = sol(Nu+1:end);

% if any(FB < 0)
%     A
%     b
%     throw()
% end

end




%------------- END OF CODE --------------
