function [data] = decentralized_cbf_qp(t,x,aa,unom,settings,params)
%decentralized_cbf_qp - QP-based law for safe, decentralized control
%
% Syntax:  [data] = decentralized_cbf_qp(t,x,unom)
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
Na   = size(x,1);
Nu   = settings.Nu;
umax = settings.umax;

% Generate Safety Settings
safety_settings.Na         = Na;
safety_settings.uNom       = unom;
safety_settings.aa         = settings.aa;
safety_settings.SL         = settings.SL;
safety_settings.Lr         = settings.Lr;
safety_settings.pcca       = settings.pcca;
safety_settings.classk     = settings.classk;
safety_settings.backup     = settings.backup;
safety_settings.cbf_type   = settings.cbf_type;
safety_settings.lookahead  = settings.lookahead;
safety_settings.hysteresis = settings.hysteresis;

% Get all safety constraints
[As,bs,hs,h0,hysteresis,prop_brake] = get_scs(t,x,safety_settings);

% Check for safety violations
ia_virt_cbf     = hs(end-(Na-1)+1:end);
ia_phys_cbf     = h0(end-(Na-1)+1:end);
virt_violations = sum(find(ia_virt_cbf < 0));
phys_violations = sum(find(ia_phys_cbf < 0));

% Report physical violations
if phys_violations > 0
    disp('Physical Barrier Violated')
    vio_magnitude = min(ia_phys_cbf);
    data = struct('code',-1,'v_vio', virt_violations,'p_vio', phys_violations, 'vio_mag', vio_magnitude);
    return
end

if prop_brake
    gain   = 1;
    ubrake = -x(aa,4) * gain;
    u      = [unom(aa,1); max([ubrake,-umax(2)])];
    sol    = [u(2); 0];
    prior  = settings.prior(aa);
   
else

    % Compute values for priority metrics
    xdot     = x(:,4).*(cos(x(:,3)) - sin(x(:,3)).*tan(x(:,5)));
    ydot     = x(:,4).*(sin(x(:,3)) + cos(x(:,3)).*tan(x(:,5)));
    h_metric = hs(end-(Na-1)-1:end);
    Lgh      = As(end-(Na-1)-1:end,:);
    
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
    qs = 1e2;
    q  = [params.qu(2); qs];
    cost_settings = struct('Nu',    Nu-1, ...
                           'Na',    Na,        ...
                           'q',     q,         ...
                           'k',     priority);
    [Q,p] = priority_cost(unom(settings.aa,2),cost_settings);
    
    % Impose input bounds (or very high bounds for "no")
    if settings.ubounds
        LB    = [-umax(2);   0];
        UB    = [ umax(2); 100];
    else
        LB    = [-10*umax(2);   0];
        UB    = [ 10*umax(2); 100];
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
%         disp(t);
%         disp(exitflag);
%         disp(aa)
%         disp('Error');
%         data.code = exitflag;
%         return 

        gain   = 1;
        ubrake = -x(aa,4) * gain;
        sol    = [max([ubrake,-umax(2)]); 0];
    end

    % Set control
    u   = [unom(aa,1); sol(1)];

end

% Formulate data structure to return
cbf        = min([hs; 100]);
data.code  = 1;
data.u     = u;
data.sol   = sol;
data.cbf   = cbf;
data.prior = prior;
data.v_vio = virt_violations;
data.p_vio = phys_violations;
data.hysteresis = hysteresis;

end
%------------- END OF CODE --------------
