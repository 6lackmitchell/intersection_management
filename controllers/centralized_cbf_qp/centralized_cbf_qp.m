function [data] = centralized_cbf_qp(t,x,aa,unom,settings,params)
%centralized_cbf_qp - QP-based law for safe, centralized control
%
% Syntax:  [data] = centralized_cbf_qp(t,x,unom)
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
%
% Other m-files required: control_params.m
% Subfunctions: none
% MAT-files required: none
%
% Author: Mitchell Black
% Email: mblackjr@umich.edu
% Website: http://www.blackmitchell.com
%------------- BEGIN CODE --------------
Na = size(x,1);
Nu = settings.Nu;

% Generate Safety Settings
safety_settings.Na        = Na;
safety_settings.uNom      = unom;
safety_settings.SL        = settings.SL;
safety_settings.Lr        = settings.Lr;
safety_settings.pcca      = settings.pcca;
safety_settings.classk    = settings.classk;
safety_settings.backup    = settings.backup;
safety_settings.cbf_type  = settings.cbf_type;
safety_settings.lookahead = settings.lookahead;

% Get all safety constraints
[As,bs,hs,h0] = get_ccs(t,x,safety_settings);

% Check for safety violations
ia_virt_cbf     = hs(end-(factorial(Na-1)-1):end);
ia_phys_cbf     = h0(end-(factorial(Na-1)-1):end);
virt_violations = sum(find(ia_virt_cbf < 0));
phys_violations = sum(find(ia_phys_cbf < 0));

% Report physical violations
if phys_violations > 0
    disp('Physical Barrier Violated')
    vio_magnitude = min(ia_phys_cbf);
    data = struct('code',-1,'v_vio', virt_violations,'p_vio', phys_violations, 'vio_mag', vio_magnitude);
    return
end

% Compute values for priority metrics
xdot     = x(:,4).*(cos(x(:,3)) - sin(x(:,3)).*tan(x(:,5)));
ydot     = x(:,4).*(sin(x(:,3)) + cos(x(:,3)).*tan(x(:,5)));
h_metric = hs(end-(factorial(Na-1)-1):end);
Lgh      = As(end-(factorial(Na-1)-1):end,1:Na);

% Generate Priority Settings
metric_settings.Na     = Na;
metric_settings.xdot   = [xdot ydot];
metric_settings.h      = h_metric;
metric_settings.Lgh    = Lgh;
metric_settings.metric = settings.pmetric;
metric_settings.power  = settings.ppower;
metric_settings.prior  = settings.prior;
metric_settings.dt     = settings.dt;

% Generate Priority Weights
priority = get_priority_metric(t,x,metric_settings);
prior    = priority;

% Generate cost function matrix Q and vector p: 
% J = 1/2*x^T*q*x + p*x
q = [repmat(params.qu(2),Na,1)];
cost_settings = struct('Nu',    Na*(Nu-1), ...
                       'Na',    Na,        ...
                       'q',     q,         ...
                       'k',     priority);
[Q,p] = priority_cost(unom(:,2),cost_settings);

% Impose input bounds (or very high bounds for "no")
if settings.ubounds
    LB    = [-repmat(umax(2),Na,1)];
    UB    = [ repmat(umax(2),Na,1)];
else
    LB    = [-1e3*ones(Na,1)];
    UB    = [ 1e3*ones(Na,1)];
end

% Solve Optimization problem
% min J subject to Ax <= b, LB <= x <= UB, no equality constraints
try
    [sol,~,exitflag] = solve_quadratic_program(Q,p,As,bs,[],[],LB,UB); 
catch ME
    disp(t)
    disp(ME.message)
    rethrow(ME)
end

% Error handling: 2 is success code
if exitflag ~= 2
    disp(t);
    disp(exitflag);
    disp(aa)
    disp('Error');
    data.code = exitflag;
    return 
end

% Set control and cbf
u   = [unom(aa); sol(aa)];
cbf = min([hs; 100]);

% Formulate data structure to return
data.code  = 1;
data.u     = u;
data.sol   = sol;
data.cbf   = cbf;
data.prior = prior;
data.v_vio = virt_violations;
data.p_vio = phys_violations;

end
%------------- END OF CODE --------------
