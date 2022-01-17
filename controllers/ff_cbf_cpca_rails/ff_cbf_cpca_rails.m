function [data] = ff_cbf_cpca_rails(t,x,settings,params)
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
Nu       = params.Nu;
umax     = params.umax;

% Organize parameters
Na  = size(x,1); % Number of agents
Nn  = 1;         % Number of noncommunicating agents
Nd  = Na*Nu;     % Number of decision variables

% Initialize variables
u          = zeros(Na,Nu);
uNom       = zeros(Na,Nu);
mincbf     = zeros(Na,1);
sols       = zeros(Na,Nd);
% gamma_sols = zeros(Na,4);
% avalue     = zeros(Na,1);
% bvalue     = zeros(Na,1);
u00        = zeros(Nd,1);
sat_vec    = [umax(1); umax(2)];  
LyapunovFunc = zeros(Na,1);

% Assign tslots
tSlots = assign_tslots(t,x,tSlots);

% Generate Nominal Control Inputs
for aa = 1:Na
    % Loop Variables
    ctrl_idx = (-1:0)+aa*Nu;
            
    % Generate nominal control input from trajectory tracking controller
    u0  = ailon2020_kb_tracking_fxts(t,x(aa,:),aa,settings,params);
    u0  = min(sat_vec,max(-sat_vec,u0)); % Saturate nominal control
    u00(ctrl_idx) = u0;

    % Construct function for prioritization
    xdot = x(aa,4)*(cos(x(aa,3)) - sin(x(aa,3))*tan(x(aa,5)));
    ydot = x(aa,4)*(sin(x(aa,3)) + cos(x(aa,3))*tan(x(aa,5)));
    LyapunovFunc(aa) = 1/2*norm(settings.r(aa,:) - x(aa,1:2))^2 + 1/2*norm(settings.rdot(aa,:) - [xdot ydot])^2; % Deviation from nominal trajectory
%     LyapunovFunc(aa) = 1/2*norm([xdot ydot])^2; % Speed
%     LyapunovFunc(aa) = 1/2*norm(x(aa,1:2))^2; % Distance from intersection center
end

% Get priorities
[~,idxLF] = sort(LyapunovFunc,'ascend');

for aa = 1:Na
    % Loop Variables
    ctrl_idx = aa;
    
    % Safety-Compensating Decentralized Adaptive Reciprocal Control
    uCost         = u00(2:2:Na*Nu);
    lookahead     = 1.0;
    safety_settings = struct('Na',        Na,          ...
                             'Nn',        Nn,          ...
                             'SL',        settings.SL, ...
                             'AAA',       aa,          ...
                             'vEst',      uLast,       ...
                             'uNom',      u00,         ...
                             'tSlots',    tSlots,      ...
                             'lookahead', lookahead);

    % Communicating vs. Noncommunicating Agent Safety Constraints
    if aa < 4
        power = 10;
        [As,bs,safety_params] = get_ffcbf_safety_constraints_dynamic_cpca(t,x,safety_settings);
    else
        power = 1;
        [As,bs,safety_params] = get_ffcbf_safety_constraints_dynamic_pcca(t,x,safety_settings);
%         uCost(~ismember(find(uCost>-Inf),ctrl_idx)) = 0; % Estimate control to be zero
        uCost(~ismember(find(uCost>-Inf),ctrl_idx)) = uLast(1:3,2); % Estimate control to be last input
    end

    % Assign priority array
    priority = zeros(Na,1);
%     priority(1) = power^3; % Static Priority
%     priority(2) = power^2;
%     priority(3) = power^1;
%     priority(4) = power^4;
%     priority(idxLF(1)) = power^4; % Rich get richer
%     priority(idxLF(2)) = power^3;
%     priority(idxLF(3)) = power^2;
%     priority(idxLF(4)) = power^1;
    priority(idxLF(1)) = power^1; % Wealth Redistribution
    priority(idxLF(2)) = power^2;
    priority(idxLF(3)) = power^3;
    priority(idxLF(4)) = power^4;

    cost_settings = struct('Nu',    Na*1,                      ...
                           'q',     repmat(params.qu(2),Na,1), ...
                           'idx',   ctrl_idx,                  ...
                           'k',     priority);
    [Q,p] = priority_cost(uCost,cost_settings);
    LB    = -repmat([umax(2)],Na,1);
    UB    =  repmat([umax(2)],Na,1);

    % Solve Optimization problem
    % 1/2*x^T*Q*x + p*x subject to Ax <= b
    try
        [sol,~,exitflag] = solve_quadratic_program(Q,p,As,bs,[],[],LB,UB); 
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
           
    u(aa,:)     = [u00(2*(aa-1)+1) sol(aa)];
    uLast(aa,:) = u(aa,:);
    uNom(aa,:)  = u0;
    mincbf(aa)  = min([safety_params.h; 100]);
    sols(aa,:)  = reshape([u00(1:2:Na*Nu); sol]',2*size(sol,1),1);
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
data = struct('u',      u,      ...
              'uLast',  uLast,  ...
              'sols',   sols, ...
...%               'gamma_sols',   gamma_sols, ...
              'cbf',    cbf,    ...
              'wHat',   wHat,   ...
              'uNom',   uNom,   ...
...%               'avalue', avalue, ...
...%               'bvalue', bvalue, ...
              'tSlots', tSlots);


end
%------------- END OF CODE --------------
