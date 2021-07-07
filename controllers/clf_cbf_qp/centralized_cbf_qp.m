function [u,d] = distributed_cbf_qp(t,x,u_nom,cost,dynamics)
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
nAgents = size(x,1);
NuA = Nu * nAgents;
NsA = Ns * nAgents;
[f,g]   = dynamics.fg;
idx     = [1:1:nAgents];

% Load Optimization Cost Fcn
[Q,p] = cost(u_nom,q,NuA,0,NsA);

% Control Constraints
Ac  = kron(eye(NuA),[1; -1]);
Ac  = [Ac zeros(2*NuA,NsA)];
bc  = u_max*ones(2*NuA,1);

% Class K Functions -- alpha(B) = a*B
Ak  = [zeros(NsA,NuA) -eye(NsA)];
bk  = ones(NsA,1);

% Safety Constraints
LfB = zeros(NsA,1);
LgB = zeros(NsA,NuA);
FB  = zeros(NsA,1);
for aa = 1:nAgents
    ii1 = aa*Ns-Ns+1:aa*Ns;
    ii2 = aa*Nu-Nu+1:aa*Nu;
    xo  = squeeze(x(idx(idx ~= aa),:));
    LfB(ii1)     = dBdx(t,x(aa,:),xo) * f(t,x(aa,:));
    LgB(ii1,ii2) = dBdx(t,x(aa,:),xo) * g(t,x(aa,:));
    FB(ii1)      = B(t,x(aa,:),xo);
end

As  = [-LgB(:,1:NuA) -eye(length(FB)).*FB];
bs  = LfB;

% Constraint Matrix
A = [Ac; Ak; As];
b = [bc; bk; bs];
% A = [Ac; Ak];
% b = [bc; bk];
% A = [Ac];
% b = [bc];
% A = [];
% b = [];
    
% Solve Optimization problem
% 1/2*x^T*Q*x + p*x subject to Ax <= b
sol = quadprog(Q,p,A,b,[],[],[],[],[],options);
if isempty(sol)
    A
    b
end
u   = sol(1:NuA);
d   = sol(NuA+1:end);

if any(FB < 0)
    A
    b
    error('Safety Violated')
end

end




%------------- END OF CODE --------------
