function [data] = issf_ffcbf_rails(t,x,settings,params)
%ff_cbf_cpca - Controller based on Centralized Priority-Cost Allocation (cpca)
%Need to update the remainder:
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

% Deconstruct Settings and Params
uLast    = settings.uLast;
tSlots   = settings.tSlots;
prior    = settings.prior;
Nu       = params.Nu;
umax     = params.umax;

% Organize parameters
Na  = size(x,1);       % Number of agents
Nn  = settings.Nn;     % Number of noncommunicating agents
Ns  = factorial(Na-1); % Number of slack variables
Nd  = Na*Nu + Ns;      % Number of decision variables

% Initialize variables
u          = zeros(Na,Nu);
uNom       = zeros(Na,Nu);
mincbf     = zeros(Na,1);
sols       = zeros(Na,Nd);
% gamma_sols = zeros(Na,4);
% avalue     = zeros(Na,1);
% bvalue     = zeros(Na,1);
u00        = zeros(Na*Nu,1);
sat_vec    = [umax(1); umax(2)];  
% LyapunovFunc = zeros(Na,1);
virt_violations = zeros(Na,1);
phys_violations = zeros(Na,1);

% Assign tslots
tSlots = assign_tslots(t,x,tSlots);

% Get priorities
power = 10;

% Assign priority array
priority = zeros(Na,1);

% Compute values for priority metrics
xdot = x(:,4).*(cos(x(:,3)) - sin(x(:,3)).*tan(x(:,5)));
ydot = x(:,4).*(sin(x(:,3)) + cos(x(:,3)).*tan(x(:,5)));
% LyapunovFunc = 1/2*vecnorm([xdot ydot]').^2; % Speed (First-come first served)
<<<<<<< HEAD
% LyapunovFunc = 1/2*vecnorm(settings.r' - x(:,1:2)').^2 + 1/2*vecnorm(settings.rdot' - [xdot ydot]').^2;
% [~,idxLF] = sort(LyapunovFunc,'ascend');
=======
LyapunovFunc = 1/2*vecnorm(settings.r' - x(:,1:2)').^2 + 1/2*vecnorm(settings.rdot' - [xdot ydot]').^2;
[~,idxLF] = sort(LyapunovFunc,'ascend');
>>>>>>> 6c5c128 (testing more)

% %     LyapunovFunc(aa) = 1/2*norm(settings.r(aa,:) - x(aa,1:2))^2 + 1/2*norm(settings.rdot(aa,:) - [xdot ydot])^2; % Deviation from nominal trajectory
%     LyapunovFunc(aa) = 1/2*norm([xdot ydot])^2; % Speed
% %     LyapunovFunc(aa) = 1/2*norm(x(aa,1:2))^2; % Distance from intersection center


<<<<<<< HEAD
% % FCFS (based on speed) -- Static
=======
% % FCFS (based on speed)
>>>>>>> 6c5c128 (testing more)
% if t == 0.01
%     priority(idxLF(1)) = power^0; % Rich get richer
%     priority(idxLF(2)) = power^1;
%     priority(idxLF(3)) = power^2;
%     priority(idxLF(4)) = power^3;
% else
%     priority(1) = prior(1);
%     priority(2) = prior(2);
%     priority(3) = prior(3);
%     priority(4) = prior(4);
% end

%     priority(1) = power^3; % Static Priority
%     priority(2) = power^2;
%     priority(3) = power^1;
%     priority(4) = power^4;
%     priority(idxLF(1)) = power^3; % Rich get richer
%     priority(idxLF(2)) = power^2;
%     priority(idxLF(3)) = power^1;
%     priority(idxLF(4)) = power^0;
    priority(idxLF(1)) = power^0; % Wealth Redistribution
    priority(idxLF(2)) = power^1;
    priority(idxLF(3)) = power^2;
    priority(idxLF(4)) = power^3;



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
lookahead     = 1.0;
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
[As,bs,safety_params] = get_issf_ffcbf_safety_constraints(t,x,safety_settings);

Lgh = As(end-(Ns-1):end,1:Na);
LyapunovFunc = 1/2*sum(Lgh.^2);
[~,idxLF] = sort(LyapunovFunc,'ascend');
priority(idxLF(1)) = power^3; % Rich get richer
priority(idxLF(2)) = power^2;
priority(idxLF(3)) = power^1;
priority(idxLF(4)) = power^0;
prior = priority;




for aa = 1:Na
    % Loop Variables
    ctrl_idx = aa;
    
    % Safety-Compensating Decentralized Adaptive Reciprocal Control
    uCost         = [u00(2:2:Na*Nu); zeros(Ns,1)]; % Zeros for h slack
%     lookahead     = 0.0;
%     lookahead     = 0.1;
%     lookahead     = 0.25;
%     lookahead     = 0.35;
%     lookahead     = 0.5;
%     lookahead     = 0.6;
%     lookahead     = 0.7;
%     lookahead     = 0.8;
%     lookahead     = 0.9;
%     lookahead     = 1.0;
% %     lookahead     = 1.25;
% %     lookahead     = 1.5;
%     safety_settings = struct('Na',        Na,          ...
%                              'Nn',        Nn,          ...
%                              'Ns',        Ns,          ...
%                              'SL',        settings.SL, ...
%                              'AAA',       aa,          ...
%                              'vEst',      uLast,       ...
%                              'uNom',      u00,         ...
%                              'tSlots',    tSlots,      ...
%                              'lookahead', lookahead);

    % Priority / Nominal Control -- different for comm. v noncomm.
    if aa >= Na - (Nn - 1)
        uCost(~ismember(find(uCost>-Inf),ctrl_idx)) = 0; % Estimate control to be zero
        priority = ones(Na,1);
%         uCost(~ismember(find(uCost>-Inf),ctrl_idx)) = uLast(1:3,2); % Estimate control to be last input
    end

%     % Safety Constraints -- Same for comm. and noncomm.
%     [As,bs,safety_params] = get_issf_ffcbf_safety_constraints(t,x,safety_settings);

    

    d = 1e1;%power^(Na+1);
    q = [repmat(params.qu(2),Na,1); repmat(d,Ns,1)];
    cost_settings = struct('Nu',    Na*(Nu-1) + Ns, ...
                           'Na',    Na,               ...
                           'q',     q,                ...
                           'idx',   ctrl_idx,         ...
                           'k',     priority);
    [Q,p] = priority_cost(uCost,cost_settings);
    LB    = [-repmat([umax(2)],Na,1); zeros(Ns,1)];
    UB    = [ repmat([umax(2)],Na,1); 1*ones(Ns,1)];

    % Solve Optimization problem
    % 1/2*x^T*Q*x + p*x subject to Ax <= b
    try
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

    ia_virt_cbf = safety_params.h(end-(Ns-1):end);
    ia_phys_cbf = safety_params.h0(end-(Ns-1):end);

    virt_violations(aa) = sum(find(ia_virt_cbf < 0));
    phys_violations(aa) = sum(find(ia_phys_cbf < 0));
    if phys_violations(aa) > 0
        disp('Physical Barrier Violated')
        data = struct('code',-1,'v_vio', virt_violations,'p_vio', phys_violations);
        return
    end
           
    u(aa,:)     = [u00(2*(aa-1)+1) sol(aa)];
    uLast(aa,:) = u(aa,:);
    uNom(aa,:)  = u0;
    mincbf(aa)  = min([safety_params.h; 100]);
    sols(aa,:)  = reshape([u00(1:2:Na*Nu); sol]',Na*Nu+Ns,1);
%     gamma_sols(aa,:) = sol(9:end);
%     avalue(aa) = safety_params.avalue;
%     bvalue(aa) = safety_params.bvalue;


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
...%               'gamma_sols',   gamma_sols, ...
              'cbf',    cbf,    ...
              'wHat',   wHat,   ...
              'uNom',   uNom,   ...
...%               'avalue', avalue, ...
...%               'bvalue', bvalue, ...
              'tSlots', tSlots, ...
              'prior',  prior,  ...
              'v_vio', virt_violations, ...
              'p_vio', phys_violations);


end
%------------- END OF CODE --------------
