function [data] = queue_control(t,x,settings)
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
run(strcat(uMode,'_params.m'))

% Deconstruct Settings
dynamics = settings.dynamics;
% xGoal    = settings.xGoal;
% uLast    = settings.uLast;
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
    [drift,f,g] = dynamics('single_integrator',t,x(aa,:),[0]); %#ok<*ASGLU>
    newidx      = aa*Nu+(-1:0);

%     % Nominal FXT-CLF-QP Controller
%     settings0   = struct('dyn',   {f,g},       ...
%                          'xg',    xGoal(aa,:), ...
%                          'cost',  cost,        ...
%                          'turning',(aa==1 && x(1,2) > -3 && x(1,1) > -3),...
%                          'uLast', uLast(aa,aa*Nu+(-1:0)));
%     v_0         = zeros(Nua,1);
%     v_0(newidx) = hl_fxt_clf_qp(t,x(aa,1:2),settings0);
    
    % Nominal Analytical FxT Controller
    r           = settings.r;
    rdot        = settings.rdot;
    rddot       = settings.rddot;
    settings0   = struct('Gamma', settings.Gamma,...
                         'e1',    settings.e1,   ...
                         'e2',    settings.e2,   ...
                         'T',     Tfxt(aa),      ...
                         'r',     r(aa,:),       ...
                         'rdot',  rdot(aa,:),    ...
                         'rddot', rddot(aa,:),   ...
                         'cost',  cost);
%     v_0         = zeros(Nua,1);
%     v_0(newidx) = analytical_fxt(t,x(aa,1:2),settings0);
    acc         = zeros(Nua,1);
    acc(newidx) = path_following_fxt_clf_qp(t,x(aa,:),settings0);
    
%     % True Dynamics
%     [drift,f,g] = dynamics('kinematic_bicycle',t,x,[0]); %#ok<*ASGLU>
    
%     T   = 0.25;
%     v_a = reshape([x(:,4).*cos(x(:,3)) x(:,4).*sin(x(:,3))]',size(v_0));
%     acc = (v_0 - v_a) / T;

    
    
%     dx  = x(aa,1:2) - r(aa,:);
%     dvx = v_a - v_0;
%     acc = zeros(6,1);
%     acc1 = -2 * dvx(newidx)' / T - dx / T^2;
%     acc(newidx) = -2 * dvx(newidx)' / T - dx / T^2;
    
%     M = zeros(Nu,Nu);
    M = [-x(aa,4)*sin(x(aa,3)) cos(x(aa,3)); x(aa,4)*cos(x(aa,3)) sin(x(aa,3))];
    th = round(x(aa,3),2);
    M = [-x(aa,4)*sin(th) cos(th); x(aa,4)*cos(th) sin(th)];
%     M1  = [-x(1,4)*sin(x(1,3)) cos(x(1,3)); x(1,4)*cos(x(1,3)) sin(x(1,3))];
%     M2  = [-x(2,4)*sin(x(2,3)) cos(x(2,3)); x(2,4)*cos(x(2,3)) sin(x(2,3))];
%     M3  = [-x(3,4)*sin(x(3,3)) cos(x(3,3)); x(3,4)*cos(x(3,3)) sin(x(3,3))];
%     M   = [           M1   zeros(size(M2)) zeros(size(M3));
%            zeros(size(M1))            M2   zeros(size(M3));
%            zeros(size(M1)) zeros(size(M2))            M3];
    if rank(M) == size(M,1)
        u_00 = M \ acc(newidx);
    else
        u_00 = lsqminnorm(M,acc(newidx));
    end
    
    u_0 = zeros(size(acc,1),1);
    u_0(newidx) = u_00;

    % Load Optimization Cost Fcn
%     q = ones(Nua+Ns,1);
    q = repmat(q,Na,1);
    [Q,p] = cost(u_0,q,Nua,0,Ns);

    % Control Constraints
    Ac  = kron(eye(Nua),[1; -1]);
    Ac  = [Ac zeros(2*Nua,Ns)];
%     bc  = u_max*ones(2*Nua,1);
    bc  = repmat([tau_max; tau_max; acc_max; acc_max],Na,1);

    % Class K Functions -- alpha(B) = a*B
    Ak  = [];
    bk  = [];
    
    % Safety Constraints
    [As,bs] = get_safety_constraints(t,x,aa,tSlots,1);    

    % Constraint Matrix
    A = [Ac; Ak; As];
    b = [bc; bk; bs];
    
    tau_max = pi / 4;
    acc_max = 2 * 9.81;
    sat_vec = repmat([tau_max; acc_max],Na,1);

    % Solve Optimization problem
    % 1/2*x^T*Q*x + p*x subject to Ax <= b
    sol = quadprog(Q,p,A,b,[],[],[],[],[],options);
    if isempty(sol)
        
        % Fix recommended by MATLAB for when quadprog incorrectly returns
        % infeasible
        options2 = optimoptions('linprog','Algorithm','dual-simplex');
        sol = linprog(p,A,b,[],[],[],[],options2);
        
        kk = 1;
        while isempty(sol)
            kk = kk * 10;
            % Safety Constraints
            [As,bs] = get_safety_constraints(t,x,aa,tSlots,kk);    

            % Constraint Matrix
            A = [Ac; Ak; As];
            b = [bc; bk; bs];
            
            % Solve again
            sol = quadprog(Q,p,A,b,[],[],[],[],[],options);
            
            if kk > 1e6
                break
            end
            
        end
        
    end
    
    sol = max(-sat_vec,min(sat_vec,sol));
    
%     if aa == 3 && x(aa,1) < 3.5
%         r(aa,:)
%         x(aa,1:2)
%         v_0(newidx)
%         u_0(newidx)
%     end
    
    u(aa,:)     = sol(aa*Nu+(-1:0));
    uLast(aa,:) = sol(1:Nua);
    uNom(aa,:)  = u_0(aa*Nu+(-1:0));
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
