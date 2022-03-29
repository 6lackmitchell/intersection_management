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
wHat     = settings.wHat;

% Organize parameters
Na  = size(x,1);       % Number of agents
Nn  = settings.Nn;     % Number of noncommunicating agents
Ng  = 0;               % Number of gamma variables
Ns  = Na;              % Number of velocity slack variables
Nd  = Na*Nu + Ns + Ng; % Number of decision variables

% Initialize variables
u          = zeros(Na,Nu);
uNom       = zeros(Na,Nu);
mincbf     = zeros(Na,1);
sols       = zeros(Na,Nd);
u00        = zeros(Na*Nu,1);
gamma      = zeros(factorial(Na-1),1);
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
    [u0]  = lqr_tracking_kb(t,x(aa,:),aa,settings);
    u0  = min(sat_vec,max(-sat_vec,u0)); % Saturate nominal control
    u00(ctrl_idx) = u0;

end



% Compute values for priority metrics
power    = 2;
xdot     = x(:,4).*(cos(x(:,3)) - sin(x(:,3)).*tan(x(:,5)));
ydot     = x(:,4).*(sin(x(:,3)) + cos(x(:,3)).*tan(x(:,5)));
% h_metric = safety_params.h(end-(factorial(Na-1)-1):end);
% Lgh      = As(end-(factorial(Na-1)-1):end,1:Na);

% Compute priority
metric_settings = struct('metric',  settings.pmetric, ...
                         'power',   power,            ...
                         'xdot',    [xdot ydot],      ...
                         'xdes',    settings.r,       ...
                         'xdesdot', settings.rdot,    ...
                         'prior',   prior,            ...
                         'Na',      Na,               ...
                         'dt',      settings.dt);
%                          'h',       h_metric,         ...
%                          'Lgh',     Lgh,              ...
priority = get_priority_metric(t,x,metric_settings);
prior    = priority;


gg = 1;
for g1 = 1:Na
    for g2 = g1+1:Na
        pweight   = priority(g1) / (priority(g1) + priority(g2));
        gamma(gg) = 1 - pweight;
        gg = gg + 1;
    end
end


% Generate "Energy"-based Priority Metric
lookahead       = 5.0;
safety_settings = struct('Na',        Na,              ...
                         'Nn',        Nn,              ...
                         'Ns',        Ns,              ...
                         'Ng',        Ng,              ...
                         'SL',        settings.SL,     ...
                         'AAA',       1,              ...
                         'vEst',      uLast,           ...
                         'wHat',      wHat,            ...
                         'uNom',      u00,             ...
                         'gamma',     gamma,           ...
                         'tSlots',    tSlots,          ...
                         'lookahead', lookahead,       ...
                         'pcca',      settings.pcca,   ...
                         'classk',    settings.classk, ...
                         'backup',    settings.backup, ...
                         'cbf_type',  settings.cbf_type);
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


for aa = 1:Na
    % Loop Variables
    ctrl_idx = aa;%(-1:0)+aa*Nu;

    % Safety-Compensating Decentralized Adaptive Reciprocal Control
    uCost         = [u00(2:2:Na*Nu); zeros(Ns,1)]; % Zeros for h slack

    % Priority / Nominal Control -- different for comm. v noncomm.
    if aa > (Na - Nn)

        % Estimate control of others to be zero
        uCost(~ismember(find(uCost>-Inf),ctrl_idx)) = 0;

        % Recompute safety w/ model of noncommunicating uCost
        uSafety = u00;
        uSafety(~ismember(find(uCost>-Inf),2*ctrl_idx)) = 0;
        uCost   = [uSafety(2:2:Na*Nu); zeros(Ns,1)];
        safety_settings.uNom  = uSafety;
        safety_settings.AAA   = aa;

        % D-CSS
        [As,bs,safety_params] = get_safety_constraints(t,x,safety_settings);

        % Reassign no priority
        priority = ones(Na,1);
    else
        uCost = [u00(2:2:(Na-Nn)*Nu); zeros(Ns+Nn,1)]; % Zeros for h slack
    end

    d = 1; % Param for relaxation of speed limit
    q = [repmat(params.qu(2),Na,1); d*ones(Ns,1)];
    cost_settings = struct('Nu',    Na*(Nu-1) + Ns,    ...
                           'Na',    Na,       ...
                           'q',     q,        ...
                           'idx',   ctrl_idx, ...
                           'k',     priority);
    [Q,p] = priority_cost(uCost,cost_settings);
    if settings.ubounds
        LB    = [-repmat(umax(2),Na,1); zeros(Ns,1)];
        UB    = [ repmat(umax(2),Na,1); 1*ones(Ns,1)];
    else
        LB    = [-1e3*ones(Na,1);  zeros(Ns,1)];
        UB    = [ 1e3*ones(Na,1); 1*ones(Ns,1)];
    end

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

    if exitflag ~= 2
%         if t ~= 0.01
%             beep
%             sol = [-9.81*ones(4,1); zeros(4,1)];
%         else
            disp(t);
            disp(exitflag);
            disp(aa)
            disp('Error');
            data = struct('code',exitflag);
            return
%         end
         
    end
           
    u(aa,:)     = [u00(2*(aa-1)+1) sol(aa)];
    uLast(aa,:) = u(aa,:);
    uNom(aa,:)  = u00((-1:0)+aa*Nu);
    mincbf(aa)  = min([safety_params.h; 100]);
    sols_inter  = [u00(1:2:Na*Nu)'; sol(1:Na)'];
    sols(aa,:)  = [sols_inter(:); sol(Na+1:end)];

end

% Determine new values for wHat: wHat_ij = u_jj - u_ij
wHat = repmat(reshape(u',1,Na*Nu),Na,1) - sols(:,1:Na*Nu);

% Configure relevant variables for logging
u     = permute(reshape(u,[1 Na Nu]),[1 2 3]);
cbf   = mincbf;

% Organize data
data = struct('code',     1,      ...
              'u',        u,      ...
              'uLast',    uLast,  ...
              'sols',     sols,   ...
              'cbf',      cbf,    ...
              'wHat',     wHat,   ...
              'uNom',     uNom,   ...
              'tSlots',   tSlots, ...
              'prior',    prior,  ...
              'v_vio',    virt_violations, ...
              'p_vio',    phys_violations, ...
              'settings', settings);

end
%------------- END OF CODE --------------
