function [data] = rdrive(t,x,settings)
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
wHat  = zeros(Na,Na,Nu);
mincbf = zeros(Na);

tSlots = assign_tslots(t,x,tSlots);

for aa = 1:Na
%     [drift,f,g] = dynamics('single_integrator',t,x(aa,:),[0]); %#ok<*ASGLU>
%     newidx      = aa*Nu+(-1:0);
    
    % Configure path settings for nominal controller
    r           = settings.r;
    rdot        = settings.rdot;
    r2dot       = settings.rddot;
%     settings0   = struct('Gamma', settings.Gamma,...
%                          'e1',    settings.e1,   ...
%                          'e2',    settings.e2,   ...
%                          'T',     Tfxt(aa),      ...
%                          'r',     r(aa,:),       ...
%                          'rdot',  rdot(aa,:),    ...
%                          'rddot', r2dot(aa,:),   ...
%                          'cost',  cost);
    
    % FxT Path Following Controller
%     T    = 0.1;
%     [xdot,ydot] = path_following_fxt_clf_qp(t,x(aa,:),settings0);
%     vv   = [xdot ydot];
%     Bnom = (norm([vv])>1.0)*(atan2(ydot,xdot) - x(aa,3));
%     Vnom = cos(Bnom)*norm(uu);
%     Anom = (Vnom - x(aa,4)) / T;
%     u0   = [2*tan(Bnom); Anom];
    beta = atan(Lr/(Lr+Lf)*uLast(aa,1));
    augmented_state = [x(aa,:) beta];
    u0 = ailon2020_kb_tracking(t,augmented_state,r(aa,:),rdot(aa,:),r2dot(aa,:),t0(aa),aa);
    
%     T    = 0.1;
%     [xdot,ydot] = path_following_fxt_clf_qp(t,x(aa,:),settings0);
%     vv   = [xdot ydot];
%     Bnom = (norm([vv])>1.0)*(atan2(ydot,xdot) - x(aa,3));
%     Vnom = cos(Bnom)*norm(uu);
%     Anom = (Vnom - x(aa,4)) / T;
%     u0   = [2*tan(Bnom); Anom];

    
    

    % Control Constraints
    Ac  = []; bc = [];
    Ac  = kron(eye(Nu),[1; -1]);
    Ac  = Ac(3:4,:);
%     Ac  = [Ac zeros(2*Nu,Ns)];
% %     bc  = u_max*ones(2*Nua,1);
    bc  = [umax(2); umax(2)];

    % Class K Functions -- alpha(B) = a*B
    Ak  = [];
    bk  = [];
    
    % QP1
    % Load Optimization Cost Fcn
    aq = 100.0;
%     aq = 1.0;
    [Q,p] = cost([u0; 0],[q; aq],(Nu+1),0,Ns);

    % Use augmented
    Ac1 = [Ac zeros(size(Ac,1),1)];

    % Safety Constraints
    [As,bs] = get_safety_constraints_1(t,x,aa,uLast(:,1));    

    % Constraint Matrix
    A = [Ac1; Ak; As];
    b = [bc; bk; bs];
    
    sat_vec = [umax(1); umax(2)];

    % Solve Optimization problem
    % 1/2*x^T*Q*x + p*x subject to Ax <= b
%     sol1 = quadprog(Q,p,A,b,[],[],[],[],[],options);
%     if isempty(sol1)
%         
%         % Fix recommended by MATLAB for when quadprog incorrectly returns
%         % infeasible
%         options2 = optimoptions('linprog','Algorithm','dual-simplex');
%         sol1 = linprog(p,A,b,[],[],[],[],options2);
%         
%     end

    sol1 = u0;
    
    % QP2
    % Load Optimization Cost Fcn
    [Q,p] = cost(sol1(1:Nu),q,Nu,0,Ns);
    uLast(aa,:) = sol1(1:Nu);
    
    % Safety Constraints
    [As,bs] = get_safety_constraints_2(t,x,aa,tSlots,uLast(:,1));    

    % Constraint Matrix
    A = [Ac; Ak; As];
    b = [bc; bk; bs];
    
    sat_vec = [umax(1); umax(2)];

    % Solve Optimization problem
    % 1/2*x^T*Q*x + p*x subject to Ax <= b
    sol = quadprog(Q,p,A,b,[],[],[],[],[],options);
    if isempty(sol)
        
        % Fix recommended by MATLAB for when quadprog incorrectly returns
        % infeasible
        options2 = optimoptions('linprog','Algorithm','dual-simplex');
        sol = linprog(p,A,b,[],[],[],[],options2);
        
    end
    
%     % Saturate solution in case of error
%     sol = max(-sat_vec,min(sat_vec,sol));
    
    u(aa,:)     = sol;%(aa*Nu+(-1:0));
    uLast(aa,:) = sol;
    uNom(aa,:)  = u0;%(aa*Nu+(-1:0));
    mincbf(aa)  = 0;%min(cbf);

end

% Configure relevant variables for logging
u     = permute(reshape(u,[1 Na Nu]),[1 2 3]);
cbf   = min(mincbf);

% Organize data
data = struct('u',      u,      ...
              'uLast',  uLast,  ...
              'cbf',    cbf,    ...
              'tSlots', tSlots, ...
              'uNom',   uNom);


end




%------------- END OF CODE --------------
