function [u,d] = centralized_pcca(t,x,settings)
%clf_cbf_qp - Controller based on CLF-CBF-QP Formulation
%This controller solves a Quadratic Program with a Control Barrier
%Function (CBF)-based safety constraint and a Control Lyapunov Function
%(CLF)-based performance constraint. 
%
% Syntax:  [u] = clf_cbf_qp(t,x)
%
% Inputs:
%    t:        current time in sec -- float
%    x:        current state vector -- ROW vector
%    settings: struct containing the following variables
%              dynamics: handle to dynamics function
%
% Outputs:
%    u:    control input - ROW vector
%    data: struct object containing other relevant data recording values 
%
% Example: 
%    [u(k+1,:,:),data] = centralized_pcca(t,x,settings);
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: OTHER_FUNCTION_NAME1,  OTHER_FUNCTION_NAME2
% Author: Mitchell Black
% Email: mblackjr@umich.edu
% Website: http://www.blackmitchell.com
% May 2021; Last revision: 6-July-2021
%------------- BEGIN CODE --------------

% Import and organize parameters
run('control_params.m')
options = optimoptions('quadprog','Display','off');
nAgents = size(x,1);
Nua = Nu * nAgents;
Nsa = Ns * nAgents;
idx = [1:1:nAgents];

cost     = @min_diff_nominal;
dynamics = settings.dynamics;
xGoal    = settings.xGoal;

% Compute nominal control input -- (they use LQR, we use FXT-CLF-QP)
[drift,f,g] = dynamics('single_integrator',t,x,[0]); %#ok<*ASGLU>
settings0   = struct('dyn',{f,g},'xg',xGoal,'cost',cost,'uLast',settings.uLast);
u_nom       = hl_fxt_clf_qp(t,x,settings0);

% % If the true dynamics were double integrator, this needs to be uncommented
% [drift,f,g] = dynamics('double_integrator',t,x,[0]); %#ok<*ASGLU>
% dyn         = struct('fg',{f,g});

% Load Optimization Cost Fcn
q = ones(Nua+Nsa,1);
[Q,p] = cost(u_nom,q,Nua,0,Nsa);

% Control Constraints
Ac  = kron(eye(Nua),[1; -1]);
Ac  = [Ac zeros(2*Nua,Nsa)];
bc  = u_max*ones(2*Nua,1);

% Class K Functions -- alpha(B) = a*B
Ak  = [zeros(Nsa,Nua) -eye(Nsa)];
bk  = ones(Nsa,1);

% % Safety Constraints
% LfB = zeros(Nsa,1);
% LgB = zeros(Nsa,Nua);
% FB  = zeros(Nsa,1);
% for aa = 1:nAgents
%     ii1 = aa*Ns-Ns+1:aa*Ns;
%     ii2 = aa*Nu-Nu+1:aa*Nu;
%     xo  = squeeze(x(idx(idx ~= aa),:));
%     LfB(ii1)     = dBdx(t,x(aa,:),xo) * f(t,x(aa,:));
%     LgB(ii1,ii2) = dBdx(t,x(aa,:),xo) * g(t,x(aa,:));
%     FB(ii1)      = B(t,x(aa,:),xo);
% end
% % Check Safety
% if any(FB < 0)
%     A
%     b
%     error('Safety Violated')
% end
% 
% As  = [-LgB(:,1:Nua) -eye(length(FB)).*FB];
% bs  = LfB;

% Constraint Matrix
% A = [Ac; Ak; As];
% b = [bc; bk; bs];
A = [Ac; Ak];
b = [bc; bk];
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
u   = sol(1:Nua);
d   = sol(Nua+1:end);

% Configure relevant variables for logging
u_nom = permute(reshape(u_nom,[1 nControls nAgents]),[1 3 2]);
u     = permute(reshape(u,[1 nControls nAgents]),[1 3 2]);


end




%------------- END OF CODE --------------
