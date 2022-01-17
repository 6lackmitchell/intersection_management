function [data] = ff_cbf_darf(t,x,settings)
%ff_cbf_darf - Controller based on Predictive Collision Avoidance Algorithm (pcca)
%The foundation of this controller is a distributed CBF-QP control scheme
%in which an ego agent solves for the inputs for the other agents in
%addition to its own input and then implements its own solutionl.
%
% Syntax:  [data] = pcca(t,x,settings)
%
% Inputs:
%    t:        current time in sec -- float
%    x:        current state vector -- ROW vector
%    settings: struct containing the following variables
%              uLast:  control solution from previous timestep
%              wHat:   actual control minus predicted control
%              t0:     initial time for segment of FxTS nominal controller
%              Tfxt:   time horizon for nominal controller
%              r:      x and y position for current path
%              rdot:   xdot and ydot for current path
%              rddot:  x2dot and y2dot for current path
%
% Outputs:
%    data: struct object containing relevant data recording values
%          u:      array of control inputs for agents
%          uLast:  same as u, used to feed back in during future timestep
%          cbf:    array of current CBF values
%          wHat:   actual control minus predicted control
%          uNom:   nominal control solution
%
% Example: 
%    [data] = pcca(t,x,settings);
%
% Other m-files required: control_params.m
% Subfunctions: none
% MAT-files required: none
%
% Author: Mitchell Black
% Email: mblackjr@umich.edu
% Website: http://www.blackmitchell.com
% Jul 2021; Last revision: 13-Sep-2021
%------------- BEGIN CODE --------------
% Import control parameters
run('control_params.m')

% Deconstruct Settings
uLast    = settings.uLast;
wHat     = settings.wHat;
t0       = settings.t0;
Tfxt     = settings.Tfxt;
tSlots   = settings.tSlots;
% cost     = @min_diff_nominal;
cost     = @safe_pcca;

% Organize parameters
Na  = size(x,1);    % Number of agents
Nn  = 1;            % Number of noncommunicating agents
Nd  = Na*(Nu + Nn); % Number of decision variables

% Initialize variables
u      = zeros(Na,Nu);
uNom   = zeros(Na,Nu);
z      = zeros(Na,Nd);
mincbf = zeros(Na,1);
sols   = zeros(Na,Nd);
gamma_sols = zeros(Na,4);
avalue = zeros(Na,1);
bvalue = zeros(Na,1);

% Assign tslots
tSlots = assign_tslots(t,x,tSlots);

for aa = 1:Na
    % PCCA Control Variables
    sat_vec  = [umax(1); umax(2)];
    ctrl_idx = (-1:0)+aa*Nu;
    u00      = zeros(Nd,1);
        
    % Configure path settings for nominal controller
    r           = settings.r;
    rdot        = settings.rdot;
    r2dot       = settings.rddot;
    
    % Generate nominal control input from trajectory tracking controller
    u0  = ailon2020_kb_tracking_fxts(t,x(aa,:),r(aa,:),rdot(aa,:),r2dot(aa,:),t0(aa),aa);
    u0  = min(sat_vec,max(-sat_vec,u0)); % Saturate nominal control
    u00(ctrl_idx) = u0;

    % Generate nominal reciprocal values -- measure of momentum
    vels   = abs(x(:,4));
    gammas = vels ./ (vels + vels(Na)); % This works for one Nn agent, not more
    gammas = 1 - vels ./ (vels + vels(Na)); % This works for one Nn agent, not more
    u00(Na*Nu+1:end) = gammas;

    % Control Constraints
    LB  = [-repmat([umax(1); umax(2)],Na,1); repmat([-Inf],size(gammas,1),1)];
    UB  = [ repmat([umax(1); umax(2)],Na,1); repmat([Inf],size(gammas,1),1)];
    
    % Safety-Compensating Decentralized Adaptive Reciprocal Control
    darc_settings = struct('Na',     Na,     ...
                           'Nn',     Nn,     ...
                           'AAA',    aa,     ...
                           'vEst',   uLast,  ...
                           'uNom',   u0,     ...
                           'gammas', gammas, ...
                           'tSlots', tSlots);
    [As,bs,params] = get_ffcbf_safety_constraints_dynamic_darf(t,x,darc_settings);
    
    % Load Optimization Cost Fcn
    cost_settings = struct('Nu',  Nd,          ...
                           'q',   [repmat(qu,Na,1); repmat(qg,Na,1)], ...
                           'idx', ctrl_idx,       ...
                           'k',   1.0./ params.h00);
%     [Q,p] = min_diff_nominal(u00,repmat(q,Na,1),Nu*Na,0,Ns);
     u00  = u00 + [params.v00; zeros(4,1)];
    [Q,p] = safe_pcca(u00,cost_settings);
%     [Q,p] = safe_feasibility(u00,cost_settings);

    A = As;
    b = bs;

    % Solve Optimization problem
    % 1/2*x^T*Q*x + p*x subject to Ax <= b
    try
        [sol,fval,exitflag] = solve_quadratic_program(Q,p,A,b,[],[],LB,UB); 
    catch ME
        disp(t)
        disp(ME.message)
        rethrow(ME)
    end
    
    if exitflag ~= 2
        disp(t);
        disp(exitflag);
        disp(aa)
        disp('Error');
        return
    end
           
    u(aa,:)     = sol(ctrl_idx);
    uLast(aa,:) = u(aa,:);
    uNom(aa,:)  = u0;
    mincbf(aa)  = min([params.h; 100]);
    sols(aa,:)  = sol;
    gamma_sols(aa,:) = sol(9:end);
    avalue(aa) = params.avalue;
    bvalue(aa) = params.bvalue;


end

% Determine new values for wHat: wHat_ij = u_jj - u_ij
wHat = repmat(reshape(u',1,Na*Nu),Na,1) - sols(1:Na*Nu);

% Configure relevant variables for logging
u     = permute(reshape(u,[1 Na Nu]),[1 2 3]);
cbf   = mincbf;

% Organize data
data = struct('u',      u,      ...
              'uLast',  uLast,  ...
              'sols',   sols, ...
              'gamma_sols',   gamma_sols, ...
              'cbf',    cbf,    ...
              'wHat',   wHat,   ...
              'uNom',   uNom,   ...
              'avalue', avalue, ...
              'bvalue', bvalue, ...
              'tSlots', tSlots);


end
%------------- END OF CODE --------------
