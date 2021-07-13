function [u,data] = queue_control(t,x,settings)
%clf_cbf_qp - Controller based on CLF-CBF-QP Formulation
%This controller solves a Quadratic Program with a Control Barrier
%Function (CBF)-based safety constraint and a Control Lyapunov Function
%(CLF)-based performance constraint. 
%
% Syntax:  [u,data] = queue_control(t,x,settings)
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
%    [u(k+1,:,:),data] = queue_control(t,x,settings);
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% Author: Mitchell Black
% Email: mblackjr@umich.edu
% Website: http://www.blackmitchell.com
% July 2021; Last revision: 13-July-2021
%------------- BEGIN CODE --------------

% Deconstruct Settings
dynamics = settings.dynamics;
xGoal    = settings.xGoal;
uLast    = settings.uLast;
uMode    = settings.uMode;
tSlots   = settings.tSlots;
cost     = @min_diff_nominal;
options  = optimoptions('quadprog','Display','off');

% Import remaining control parameters
run(strcat(uMode,'_params.m'))

% Organize parameters
Na  = size(x,1);
Nua = Nu * Na;
Nsa = Ns * Na;
idx = [1:1:Na];


u     = zeros(Na,Nu);
u_nom = zeros(Na,Nua);
d     = zeros(Na,Ns);
wHat  = zeros(Na,Na,Nu);
mincbf = zeros(Na);

tSlots = assign_tslots(t,x);    




for aa = 1:Na
    [drift,f,g] = dynamics('single_integrator',t,x(aa,:),[0]); %#ok<*ASGLU>
    
    % Compute nominal control input -- (they use LQR, we use FXT-CLF-QP)
    newidx      = aa*Nu+(-1:0);
    settings0   = struct('dyn',{f,g},'xg',xGoal(aa,:),'cost',cost,'uLast',uLast(aa,aa*Nu+(-1:0)));
    v_0         = zeros(Nua,1);
    v_0(newidx) = hl_fxt_clf_qp(t,x(aa,:),settings0);
    
    % % If the true dynamics were double integrator, this needs to be uncommented
    [drift,f,g] = dynamics('kinematic_bicycle',t,x,[0]); %#ok<*ASGLU>
    
    T   = 0.5;
    v_a = reshape([x(:,4).*cos(x(:,3)); x(:,4).*sin(x(:,3))],size(v_0));
    acc = (v_0 - v_a) / T;
    
    M1  = [cos(x(1,3)) -x(1,4)*sin(x(1,3)); sin(x(1,3)) x(1,4)*cos(x(1,3))];
    M2  = [cos(x(2,3)) -x(2,4)*sin(x(2,3)); sin(x(2,3)) x(2,4)*cos(x(2,3))];
    M3  = [cos(x(3,3)) -x(3,4)*sin(x(3,3)); sin(x(3,3)) x(3,4)*cos(x(3,3))];
    M   = [M1 zeros(size(M1)) zeros(size(M1));
           zeros(size(M1)) M2 zeros(size(M1));
           zeros(size(M1)) zeros(size(M1)) M3];
    if rank(M) == size(M,1)
        u_0 = M \ acc;
    else
        u_0 = lsqminnorm(M,acc);
    end

    % Load Optimization Cost Fcn
    q = ones(Nua+Ns,1);
    [Q,p] = cost(u_0,q,Nua,0,Ns);

    % Control Constraints
    Ac  = kron(eye(Nua),[1; -1]);
    Ac  = [Ac zeros(2*Nua,Ns)];
%     bc  = u_max*ones(2*Nua,1);
    bc  = [tau_max; tau_max; acc_max; acc_max];
    bc  = [bc; bc; bc];

    % Class K Functions -- alpha(B) = a*B
    Ak  = [zeros(Ns,Nu) -eye(Ns)];
    bk  = ones(Ns,1);
    

    % Constraint Matrix
    % A = [Ac; Ak; As];
    % b = [bc; bk; bs];
%     A = [Ac; As];
%     b = [bc; bs];
    A = [Ac];
    b = [bc];
    % A = [];
    % b = [];

    % Solve Optimization problem
    % 1/2*x^T*Q*x + p*x subject to Ax <= b
    sol = quadprog(Q,p,A,b,[],[],[],[],[],options);
    if isempty(sol)
        A
        b
    end
    
    u(aa,:)     = sol(aa*Nu+(-1:0));
    uLast(aa,:) = sol(1:Nua);
    mincbf(aa)  = 0;%min(cbf);
%     d(aa,:) = sol(Nua+1:end);
%     u_nom(aa,:) = u_0;
end

% Configure relevant variables for logging
% u_nom = permute(reshape(u_nom,[1 Na Nua]),[1 2 3]);
u     = permute(reshape(u,[1 Na Nu]),[1 2 3]);
cbf   = min(mincbf);
data = struct('uLast',uLast,'cbf',cbf);


end




%------------- END OF CODE --------------
