function [data] = ff_cbf(t,x,settings,params)
%lqr_cbf - Controller based on nominal FxTS tracking and safety-compensating 
% CBF for bicycle model
%
% Syntax:  [data] = ff_cbf(t,x,settings)
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

% Deconstruct Settings and Params
uLast    = settings.uLast;
prior    = settings.prior;
Nu       = params.Nu;
umax     = params.umax;

% Organize parameters
Na  = size(x,1);       % Number of agents
Nn  = settings.Nn;     % Number of noncommunicating agents
Ns  = Na;              % Number of slack variables
Nd  = Na*Nu + Ns;      % Number of decision variables

% Initialize variables
u          = zeros(Na,Nu);
uNom       = zeros(Na,Nu);
mincbf     = zeros(Na,1);
sols       = zeros(Na,Nd);
u00        = zeros(Na*Nu,1);
sat_vec    = [umax(1); umax(2)];  
virt_violations = zeros(Na,1);
phys_violations = zeros(Na,1);

% tSlots
tSlots = settings.tSlots;

% Generate Nominal Control Inputs
for aa = 1:Na
    % Loop Variables
    ctrl_idx = (-1:0)+aa*Nu;
            
    % Generate nominal control input from trajectory tracking controller
    u0  = ailon2020_kb_tracking_fxts(t,x(aa,:),aa,settings,params);
    u0  = min(sat_vec,max(-sat_vec,u0)); % Saturate nominal control
    u00(ctrl_idx) = u0;

end

% Generate "Energy"-based Priority Metric
lookahead       = 5.0;
safety_settings = struct('Na',        Na,          ...
                         'Nn',        Nn,          ...
                         'Ns',        Ns,          ...
                         'SL',        settings.SL, ...
                         'AAA',       aa,          ...
                         'vEst',      uLast,       ...
                         'uNom',      u00,         ...
                         'tSlots',    tSlots,      ...
                         'lookahead', lookahead);
% Safety Constraints -- Same for comm. and noncomm.
[As,bs,safety_params] = get_safety_constraints(t,x,safety_settings);

% Check for safety violations
ia_virt_cbf = safety_params.h(end-(factorial(Na-1)-1):end);
ia_phys_cbf = safety_params.h0(end-(factorial(Na-1)-1):end);
virt_violations(aa) = sum(find(ia_virt_cbf < 0));
phys_violations(aa) = sum(find(ia_phys_cbf < 0));
if phys_violations(aa) > 0
    disp('Physical Barrier Violated')
    vio_magnitude = min(ia_phys_cbf);
    data = struct('code',-1,'v_vio', virt_violations,'p_vio', phys_violations, 'vio_mag', vio_magnitude);
    return
end

% Compute values for priority metrics
power    = 2;
xdot     = x(:,4).*(cos(x(:,3)) - sin(x(:,3)).*tan(x(:,5)));
ydot     = x(:,4).*(sin(x(:,3)) + cos(x(:,3)).*tan(x(:,5)));
h_metric = safety_params.h(end-(factorial(Na-1)-1):end);
Lgh      = As(end-(factorial(Na-1)-1):end,1:Na);

% Compute priority
metric = 'None';
% metric = 'FCFS';
% metric = 'FCFS_V';
% metric = 'HighDev';
% metric = 'LowDev';
% metric = 'HighEffort';
% metric = 'LowEffort';
metric_settings = struct('metric',  metric,        ...
                         'power',   power,         ...
                         'xdot',    [xdot ydot],   ...
                         'xdes',    settings.r,    ...
                         'xdesdot', settings.rdot, ...
                         'h',       h_metric,      ...
                         'Lgh',     Lgh,           ...
                         'prior',   prior,         ...
                         'Na',      Na,            ...
                         'dt',      settings.dt);
priority = get_priority_metric(t,x,metric_settings);
prior    = priority;


for aa = 1:Na
    % Loop Variables
    ctrl_idx = aa;%(-1:0)+aa*Nu;
    
    % Safety-Compensating Decentralized Adaptive Reciprocal Control
    uCost         = [u00(2:2:Na*Nu); zeros(Ns,1)]; % Zeros for h slack

    % Priority / Nominal Control -- different for comm. v noncomm.
    if aa >= Na - (Nn - 1)
        uCost(~ismember(find(uCost>-Inf),ctrl_idx)) = 0; % Estimate control to be zero
%         uCost(~ismember(find(uCost>-Inf),ctrl_idx)) = uLast(1:3,2); % Estimate control to be last input

        % Recompute safety w/ model of noncommunicating uCost
        uSafety = u00;
        uSafety(~ismember(find(uCost>-Inf),ctrl_idx)) = 0;
        safety_settings.uNom  = uSafety;
        [As,bs,safety_params] = get_safety_constraints(t,x,safety_settings);

        % Reassign no priority
        priority = ones(Na,1);
    end

    d = 1; % Param for relaxation of speed limit
    q = [repmat(params.qu(2),Na,1); d*ones(Ns,1)];
    cost_settings = struct('Nu',    Na*(Nu-1) + Ns,    ...
                           'Na',    Na,       ...
                           'q',     q,        ...
                           'idx',   ctrl_idx, ...
                           'k',     priority);
    [Q,p] = priority_cost(uCost,cost_settings);
    LB    = [-repmat(umax(2),Na,1); zeros(Ns,1)];
    UB    = [ repmat(umax(2),Na,1); 1*ones(Ns,1)];
%     LB    = [-100*ones(Na,1);  zeros(Ns,1)];
%     UB    = [ 100*ones(Na,1); 1*ones(Ns,1)];

    % Solve Optimization problem
    % 1/2*x^T*Q*x + p*x subject to Ax <= b
    try
%         As = zeros(size(As)); bs = zeros(size(bs));
        [sol,~,exitflag] = solve_quadratic_program(Q,p,As,bs,[],[],LB,UB); 
    catch ME
        disp(t)
        disp(ME.message)
        rethrow(ME)
    end

    if 0%exitflag == 3 && max(safety_params.h0) > 0
        sol = [-umax(2)*ones(4,1); zeros(6,1)];
    elseif exitflag ~= 2
        disp(t);
        disp(exitflag);
        disp(aa)
        disp('Error');
        data = struct('code',exitflag);
        return 
    end
           
    u(aa,:)     = [u00(2*(aa-1)+1) sol(aa)];
    uLast(aa,:) = u(aa,:);
    uNom(aa,:)  = u00((-1:0)+aa*Nu);
    mincbf(aa)  = min([safety_params.h; 100]);
    sols(aa,:)  = reshape([u00(1:2:Na*Nu); sol]',Na*Nu+Ns,1);

end

% Determine new values for wHat: wHat_ij = u_jj - u_ij
wHat = repmat(reshape(u',1,Na*Nu),Na,1) - sols(1:Na*Nu);

% Configure relevant variables for logging
u     = permute(reshape(u,[1 Na Nu]),[1 2 3]);
cbf   = mincbf;

% Organize data
data = struct('code',   1,      ...
              'u',      u,      ...
              'uLast',  uLast,  ...
              'sols',   sols, ...
              'cbf',    cbf,    ...
              'wHat',   wHat,   ...
              'uNom',   uNom,   ...
              'tSlots', tSlots, ...
              'prior',  prior,  ...
              'v_vio', virt_violations, ...
              'p_vio', phys_violations);

end
%------------- END OF CODE --------------
