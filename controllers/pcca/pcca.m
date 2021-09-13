function [data] = pcca(t,x,settings)
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
% Import control parameters
uMode    = settings.uMode;
run('control_params.m')

% Deconstruct Settings
dynamics = settings.dynamics;
% xGoal    = settings.xGoal;
uLast    = settings.uLast;
wHat     = settings.wHat;
t0       = settings.t0;
Tfxt     = settings.Tfxt;
cost     = @min_diff_nominal;
tSlots   = settings.tSlots;
options  = optimoptions('quadprog','Display','off');
% options  = optimoptions('quadprog','Display','final');

% Organize parameters
Na  = size(x,1);
Nua = Nu * Na;
Nsa = Ns * Na;
idx = [1:1:Na];

u     = zeros(Na,Nu);
uNom  = zeros(Na,Nu);
d     = zeros(Na,Ns);
% wHat  = zeros(Na,Na,Nu);
mincbf = zeros(Na);
sols   = zeros(Na,Nu*Na);

tSlots = assign_tslots(t,x,tSlots);

for aa = 1:Na
    % PCCA Control Variables
    sat_vec  = [umax(1); umax(2)];
    ctrl_idx = (-1:0)+aa*Nu;
    u00      = zeros(Nu*Na,1);
        
    % Configure path settings for nominal controller
    r           = settings.r;
    rdot        = settings.rdot;
    r2dot       = settings.rddot;

    % Augment state with beta angle
    beta = atan(Lr/(Lr+Lf)*uLast(aa,1));
    state = x(aa,:);
    
    % Generate nominal control input from trajectory tracking controller
    u0  = ailon2020_kb_tracking_fxts(t,state,r(aa,:),rdot(aa,:),r2dot(aa,:),t0(aa),aa);
    u0  = min(sat_vec,max(-sat_vec,u0));
    u00(ctrl_idx) = u0;

    % Control Constraints
    Ac  = kron(eye(Nu*Na),[1; -1]); %Ac  = Ac(3:4,:);
    bc  = repmat([umax(1); umax(1); umax(2); umax(2)],Na,1);%[umax(2); umax(2)];

    % Class K Functions -- alpha(B) = a*B
    Ak  = [];
    bk  = [];
    
    % Safety-Compensating PCCA Control
    pcca_settings = struct('AAA',    aa,     ...
                           'tSlots', tSlots, ...
                           'wHat',   wHat,   ...
                           'uLast',  uLast(:,1));
    [As,bs] = get_pcca_safety_constraints_dynamic(t,x,pcca_settings);
    
    % Load Optimization Cost Fcn
    [Q,p] = cost(u00,repmat(q,Na,1),Nu*Na,0,Ns);
%     uLast(aa,:) = sol1(1:Nu);    

    % Constraint Matrix
    A = [Ac; Ak; As];
    b = [bc; bk; bs];

    % Solve Optimization problem
    % 1/2*x^T*Q*x + p*x subject to Ax <= b
    sol = quadprog(Q,p,A,b,[],[],[],[],[],options);
    if isempty(sol)
        
        % Fix recommended by MATLAB for when quadprog incorrectly returns
        % infeasible
        options2 = optimoptions('linprog','Algorithm','dual-simplex');
        sol = linprog(p,A,b,[],[],[],[],options2);
        if isempty(sol)
            disp(aa)
            disp(t)
            disp(sol)
        end
        
    end
    
    u(aa,:)     = sol(ctrl_idx);
    uLast(aa,:) = u(aa,:);
    uNom(aa,:)  = u0;
    mincbf(aa)  = 0;
    sols(aa,:)  = sol;

end

% Determine new values for wHat: wHat_ij = u_jj - u_ij
wHat = repmat(reshape(u',1,Na*Nu),Na,1) - sols;

% Configure relevant variables for logging
u     = permute(reshape(u,[1 Na Nu]),[1 2 3]);
cbf   = min(mincbf);

% Organize data
data = struct('u',      u,      ...
              'uLast',  uLast,  ...
              'cbf',    cbf,    ...
              'tSlots', tSlots, ...
              'wHat',   wHat,   ...
              'uNom',   uNom);


end




%------------- END OF CODE --------------