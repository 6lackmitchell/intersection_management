function [sol,hs,code] = im_intervention(t,x,settings,params)
%im_intervention - Intervention from intersection manager
%
% Syntax:  [data] = im_intervention(t,x,unom,umax,nNon)
%
% Inputs:
%    t:    current time in sec -- float
%    x:    current state vector -- ROW vector
%    unom: nominal control input for agents
%
% Outputs:
%    sol: modified nominal control input for agents
%
% Other m-files required: control_params.m
% Subfunctions: none
% MAT-files required: none
%
% Author: Mitchell Black
% Email: mblackjr@umich.edu
% Website: http://www.blackmitchell.com
%------------- BEGIN CODE --------------
code = 2;
Na   = size(x,1);
Nu   = settings.Nu;
Nn   = settings.Nn;
unom = settings.unom;
umax = settings.umax;

% Generate Safety Settings
safety_settings.Na        = Na;
safety_settings.Nn        = Nn;
safety_settings.uNom      = unom;
safety_settings.uMax      = umax;
safety_settings.Lr        = settings.Lr;

% Get all safety constraints
[As,bs,hs] = get_density_constraints(t,x,safety_settings);

% Compute values for priority metrics
xdot     = x(:,4).*(cos(x(:,3)) - sin(x(:,3)).*tan(x(:,5)));
ydot     = x(:,4).*(sin(x(:,3)) + cos(x(:,3)).*tan(x(:,5)));

% Generate Priority Settings
metric_settings.Na     = Na;
metric_settings.xdot   = [xdot ydot];
metric_settings.metric = settings.pmetric;
metric_settings.power  = settings.ppower;
metric_settings.prior  = settings.prior;
metric_settings.dt     = settings.dt;

% Generate Priority Weights
priority = get_priority_metric(t,x,metric_settings);
prior    = priority;

% Generate cost function matrix Q and vector p: 
% J = 1/2*x^T*q*x + p*x
q = [repmat(params.qu(2),(Na-Nn),1)];
cost_settings = struct('Nu',   (Na-Nn)*(Nu-1), ...
                       'Na',    Na,        ...
                       'q',     q,         ...
                       'k',     priority);
[Q,p] = priority_cost(unom(1:Na-Nn,2),cost_settings);

% Impose input bounds (or very high bounds for "no")
if settings.ubounds
    LB    = -repmat(umax(2),Na-Nn,1);
    UB    =  repmat(umax(2),Na-Nn,1);
else
    LB    = -repmat(10*umax(2),Na-Nn,1);
    UB    =  repmat(10*umax(2),Na-Nn,1);
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
    sol = unom(1:(Na-Nn),2);
    disp('IM-Error');
    code = exitflag;
    return 
end

end
%------------- END OF CODE --------------
