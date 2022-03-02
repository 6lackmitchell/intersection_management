function [data] = ap_cbf(t,x,settings,params)
%ap_cbf - Controller based on nominal FxTS tracking and safety-compensating 
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
dmode    = settings.dmode;
flip     = settings.flip;

% Organize parameters
Na  = size(x,1);       % Number of agents
Nn  = settings.Nn;     % Number of noncommunicating agents
Ns  = Na;              % Number of slack variables
Ng  = factorial(Na-1);
Nd  = Na*Nu + Ns + Ng;      % Number of decision variables

% Initialize variables
u          = zeros(Na,Nu);
uNom       = zeros(Na,Nu);
mincbf     = zeros(Na,1);
sols       = zeros(Na,Nd);
u00        = zeros(Na*Nu,1);
gamma0     = zeros(Ng,1);
gamma      = zeros(Ng,1);
sat_vec    = [umax(1); umax(2)];  
virt_violations = zeros(Na,1);
phys_violations = zeros(Na,1);

% tSlots
tSlots = settings.tSlots;

if strcmp(dmode,'double_integrator')
    Ae = zeros(Na,Nd);
    be = zeros(Na,1);
end

% Generate Nominal Control Inputs
for aa = 1:Na
    % Loop Variables
    ctrl_idx = (-1:0)+aa*Nu;
            
    % Generate nominal control input from trajectory tracking controller
    if strcmp(dmode,'double_integrator')
        u0  = lqr_tracking(t,x(aa,:),aa,settings);
    else
        u0  = ailon2020_kb_tracking_fxts(t,x(aa,:),aa,settings,params);
    end
    u0  = min(sat_vec,max(-sat_vec,u0)); % Saturate nominal control
    u00(ctrl_idx) = u0;

    if strcmp(dmode,'double_integrator')
        % Rail Constraints for double integrator
        if u0(1) == 0 && u0(2) == 0
            if abs(x(aa,3)) > abs(x(aa,4))
                Ae(aa,ctrl_idx) = [0 1];
            elseif abs(x(aa,3)) < abs(x(aa,4))
                Ae(aa,ctrl_idx) = [1 0];
            else
                Ae(aa,ctrl_idx) = [0 0];
            end
        elseif u0(2) == 0
            Ae(aa,ctrl_idx) = [-u0(2)/u0(1) 1];
        else
            Ae(aa,ctrl_idx) = [1 -u0(1)/u0(2)];
        end
    end

end

% Generate "Energy"-based Priority Metric
lookahead       = 5.0;
safety_settings = struct('Na',        Na,          ...
                         'Nn',        Nn,          ...
                         'Ns',        Ns,          ...
                         'Ng',        Ng,          ...
                         'SL',        settings.SL, ...
                         'AAA',       aa,          ...
                         'vEst',      uLast,       ...
                         'uNom',      u00,         ...
                         'tSlots',    tSlots,      ...
                         'flip',      flip,        ...
                         'lookahead', lookahead);

% Safety Constraints -- Same for comm. and noncomm.
if strcmp(dmode,'double_integrator')
    [As,bs,safety_params] = get_di_safety_constraints(t,x,safety_settings);
else
    [As,bs,safety_params] = get_kb_safety_constraints(t,x,safety_settings);
end

% Compute values for priority metrics
power    = 10;%2;
h_metric = safety_params.h(end-(Ng-1):end);
Lgh      = As(end-(2*Ng-1):end,1:Na);
if strcmp(dmode,'double_integrator')
    xdot     = x(:,3);
    ydot     = x(:,4);
else
    xdot     = x(:,4).*(cos(x(:,3)) - sin(x(:,3)).*tan(x(:,5)));
    ydot     = x(:,4).*(sin(x(:,3)) + cos(x(:,3)).*tan(x(:,5)));
end

% Compute priority
% metric = 'None';
% metric = 'FCFS';
% metric = 'HighDev';
% metric = 'LowDev';
metric = 'HighEffort';
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

% Assign nominal gamma weights
gg = 1;
for g1 = 1:Na
    for g2 = g1+1:Na
        % Relatively high priority gets relatively low responsibility
%         if priority(g1) > priority(g2)
%             gamma0(gg) = 0;%flip;
%         else
%             gamma0(gg) = 1;%1-flip;
%         end
        gamma0(gg) = 1 - sqrt(priority(g1) / (priority(g1) + priority(g2)));
        gg = gg + 1;
    end
end


for aa = 1:Na
    % Loop Variables
    ctrl_idx = aa;%(-1:0)+aa*Nu;
    
    % Safety-Compensating Decentralized Adaptive Reciprocal Control
    if strcmp(dmode,'double_integrator')
        uCost         = [u00; zeros(Ns,1)]; % Zeros for h slack
    else
        uCost         = [u00(2:2:Na*Nu); zeros(Ns,1); gamma0]; % Zeros for h slack
    end

    % Priority / Nominal Control -- different for comm. v noncomm.
    if aa >= Na - (Nn - 1)
        % Safety-Compensating Decentralized Adaptive Reciprocal Control
        if strcmp(dmode,'double_integrator')
            uCost = [u00; zeros(Ns,1); 0.5*ones(Ng,1)]; % Zeros for h slack
        else
            uCost = [u00(2:2:Na*Nu); zeros(Ns,1); 0.5*ones(Ng,1)]; % Zeros for h slack
        end
        uCost(~ismember(find(uCost(1:end-Ng)>-Inf),ctrl_idx)) = 0; % Estimate control to be zero

        % Recompute safety w/ model of noncommunicating uCost
        uSafety = u00;
        uSafety(~ismember(find(uSafety>-Inf),(-1:0)+aa*Nu)) = 0;
        safety_settings.uNom  = uSafety;
        if strcmp(dmode,'double_integrator')
            [As,bs,safety_params] = get_di_safety_constraints(t,x,safety_settings);
        else
            [As,bs,safety_params] = get_kb_safety_constraints(t,x,safety_settings);
        end

        % Reassign no priority
        priority = ones(Na,1);
    end

    d = 1; % Param for relaxation of speed limit
    if strcmp(dmode,'double_integrator')
        q = [repmat(params.qu,Na,1); d*ones(Ns,1)];
        LB    = [-repmat(umax,Na,1); zeros(Ns,1); -100*ones(Ng,1)];
        UB    = [ repmat(umax,Na,1); 1*ones(Ns,1); 100*ones(Ng,1)];
        %     LB    = [-100*ones(Nu*Na,1);  zeros(Ns,1); -inf*ones(Ng,1)];
        %     UB    = [ 100*ones(Nu*Na,1); 1*ones(Ns,1); inf*ones(Ng,1)];
    else
        q = [repmat(params.qu(2),Na,1); d*ones(Ns+Ng,1)];
        LB    = [-repmat(umax(2),Na,1); zeros(Ns,1); -10*ones(Ng,1)];
        UB    = [ repmat(umax(2),Na,1); 1*ones(Ns,1); 10*ones(Ng,1)];
%         LB    = [-100*ones(Na,1);  zeros(Ns,1); -inf*ones(Ng,1)];
%         UB    = [ 100*ones(Na,1); 1*ones(Ns,1); inf*ones(Ng,1)];

    end

    % Check for safety violations
    ia_virt_cbf = safety_params.h(end-2*(factorial(Na-1)-1):end);
    ia_phys_cbf = safety_params.h0(end-2*(factorial(Na-1)-1):end);

    virt_violations(aa) = sum(find(ia_virt_cbf < 0));
    phys_violations(aa) = sum(find(ia_phys_cbf < 0));
    if phys_violations(aa) > 0
        disp('Physical Barrier Violated')
        vio_magnitude = min(ia_phys_cbf);
        data = struct('code',-1,'v_vio', virt_violations,'p_vio', phys_violations, 'vio_mag', vio_magnitude);
        return
    end

    cost_settings = struct('Nu',    Na*(Nu-1) + Ns + Ng, ...
                           'Na',    Na,                  ...
                           'q',     q,                   ...
                           'idx',   ctrl_idx,            ...
                           'k',     priority);
    [Q,p] = priority_cost(uCost,cost_settings);

    % Solve Optimization problem
    % 1/2*x^T*Q*x + p*x subject to Ax <= b
    try
%         As = zeros(size(As)); bs = zeros(size(bs));
        [sol,~,exitflag] = solve_quadratic_program(Q,p,As,bs,[],[],LB,UB); 
    catch ME
        disp(t)
        disp(ME.message)
        data = struct('code',12);
        return
%         rethrow(ME)
    end

    if exitflag ~= 2
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
    sols(aa,:)  = reshape([u00(1:2:Na*Nu); sol]',Na*Nu+Ns+Ng,1);

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
