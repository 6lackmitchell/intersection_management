function [u,data] = distributed_pcca(t,x,settings)
%clf_cbf_qp - Controller based on CLF-CBF-QP Formulation
%This controller solves a Quadratic Program with a Control Barrier
%Function (CBF)-based safety constraint and a Control Lyapunov Function
%(CLF)-based performance constraint. 
%
% Syntax:  [u] = clf_cbf_qp(t,x)
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
%    [u(k+1,:,:),data] = centralized_pcca(t,x,settings);
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: OTHER_FUNCTION_NAME1,  OTHER_FUNCTION_NAME2
% Author: Mitchell Black
% Email: mblackjr@umich.edu
% Website: http://www.blackmitchell.com
% May 2021; Last revision: 6-July-2021
%------------- BEGIN CODE --------------

% Import and organize parameters
run('control_params.m')
options = optimoptions('quadprog','Display','off');
Na = size(x,1);
Nua = Nu * Na;
Nsa = Ns * Na;
idx = [1:1:Na];

cost     = @min_diff_nominal;
dynamics = settings.dynamics;
xGoal    = settings.xGoal;
uLast    = settings.uLast;

u     = zeros(Na,Nu);
u_nom = zeros(Na,Nua);
d     = zeros(Na,Ns);
wHat  = zeros(Na,Na,Nu);
mincbf = zeros(Na);
for aa = 1:Na
    [drift,f,g] = dynamics('single_integrator',t,x(aa,:),[0]); %#ok<*ASGLU>
    
    % Compute nominal control input -- (they use LQR, we use FXT-CLF-QP)
    newidx      = aa*Nu+(-1:0);
    settings0   = struct('dyn',{f,g},'xg',xGoal(aa,:),'cost',cost,'uLast',uLast(aa,aa*Nu+(-1:0)));
    v_0         = zeros(Nua,1);
    v_0(newidx) = hl_fxt_clf_qp(t,x(aa,:),settings0);
    
    % % If the true dynamics were double integrator, this needs to be uncommented
    [drift,f,g] = dynamics('kinematic_bicycle',t,x,[0]); %#ok<*ASGLU>
    
    T   = 0.5;
    v_a = reshape([x(:,4).*cos(x(:,3)); x(:,4).*sin(x(:,3))],size(v_0));
    acc = (v_0 - v_a) / T;
    
    M1  = [cos(x(1,3)) -x(1,4)*sin(x(1,3)); sin(x(1,3)) x(1,4)*cos(x(1,3))];
    M2  = [cos(x(2,3)) -x(2,4)*sin(x(2,3)); sin(x(2,3)) x(2,4)*cos(x(2,3))];
    M3  = [cos(x(3,3)) -x(3,4)*sin(x(3,3)); sin(x(3,3)) x(3,4)*cos(x(3,3))];
    M   = [M1 zeros(size(M1)) zeros(size(M1));
           zeros(size(M1)) M2 zeros(size(M1));
           zeros(size(M1)) zeros(size(M1)) M3];
    if rank(M) == size(M,1)
        u_0 = M \ acc;
    else
        u_0 = lsqminnorm(M,acc);
    end

    % Load Optimization Cost Fcn
    q = ones(Nua+Ns,1);
    [Q,p] = cost(u_0,q,Nua,0,Ns);

    % Control Constraints
    Ac  = kron(eye(Nua),[1; -1]);
    Ac  = [Ac zeros(2*Nua,Ns)];
%     bc  = u_max*ones(2*Nua,1);
    bc  = [tau_max; tau_max; acc_max; acc_max];
    bc  = [bc; bc; bc];

    % Class K Functions -- alpha(B) = a*B
    Ak  = [zeros(Ns,Nu) -eye(Ns)];
    bk  = ones(Ns,1);
    
    % Update fictitious disturbances
%     wHat(aa,:) = uLast(:)

%     % Quick and dirty safety solution
%     As = zeros(factorial(Ns)+1,Nua+Ns);
%     bs = zeros(factorial(Ns)+1,1);
%     
%     if x(aa,1) < -3 && x(aa,2) < 0
%         % A7
%         B   = -(x(aa,2)^2 + 3*x(aa,2));
%         LfB = -3;
%         LgB = [0 -2*x(aa,2)];
%     elseif x(aa,1) < 0 && x(aa,2) > 3
%         % A5
%         B   = -(x(aa,1)^2 + 3*x(aa,1));
%         LfB = -3;
%         LgB = [0 -2*x(aa,1)];
%     elseif x(aa,1) > 0 && x(aa,2) < -3
%         % A1
%         B   = -x(aa,1)^2 + 3*x(aa,1);
%         LfB = 3;
%         LgB = [0 -2*x(aa,1)];
%     elseif x(aa,1) > 3 && x(aa,2) > 0
%         % A3
%         B   = -x(aa,2)^2 + 3*x(aa,2);
%         LfB = 3;
%         LgB = [0 -2*x(aa,2)];
%     else
%         B   = 10;
%         LfB = 0;
%         LgB = [0 0];
%     end
% 
%     row = 1;
%     As(row,aa*Nu+(-1:0)) = -LgB;
%     bs(row)              = LfB + 10*B;
%     cbf(row)             = B;
%     
%     for bb = 1:Na
%         % Update fictitious disturbance j
%         wHat(aa,bb,:) = uLast(bb,bb*Nu+(-1:0)) - uLast(aa,bb*Nu+(-1:0));
%         
%         for cc = bb+1:Na
%             row = row + 1;
%             
%             % Update fictitious disturbance k
%             wHat(aa,cc,:) = uLast(cc,cc*Nu+(-1:0)) - uLast(aa,cc*Nu+(-1:0));
%             
%             % Update Safety terms
%             LfB = 0;
%             LgB = 2 * (x(bb,:) - x(cc,:));
%             LwB = LgB * squeeze(wHat(aa,bb,:) - wHat(aa,cc,:));
%             
%             % Update safety matrix
%             B = (x(bb,:) - x(cc,:))*(x(bb,:) - x(cc,:))' - R^2;
%             newrow = zeros(Nua+Ns,1);
%             newrow((bb-1)*Nu+(1:Nu)) = -LgB;
%             newrow((cc-1)*Nu+(1:Nu)) =  LgB;
%             newcol = LwB + B^3;
%             As(row,:) = newrow;
%             bs(row)   = newcol;
%             
%             % Update logging var
%             cbf(row) = B;
%         end
%     end
            
        
            


%     % Build Safety Constraints
%     for bb = 1:Na
%         LfB = zeros(Nsa,1);
%         LgB = zeros(Nsa,Nua);
%         FB  = zeros(Nsa,1);
%         for cc = bb+1:Na
%             ii1 = cc*Ns-Ns+1:cc*Ns;
%             ii2 = cc*Nu-Nu+1:cc*Nu;
%             xo  = squeeze(x(idx(idx ~= cc),:));
%             LfB(ii1)     = dBdx(t,x(cc,:),xo) * f(t,x(cc,:));
%             LgB(ii1,ii2) = dBdx(t,x(cc,:),xo) * g(t,x(cc,:));
%             FB(ii1)      = B(t,x(cc,:),xo);
%         end
%         
%         % Add new constraints
%     end
            

    % % Safety Constraints
    % LfB = zeros(Nsa,1);
    % LgB = zeros(Nsa,Nua);
    % FB  = zeros(Nsa,1);
    % for aa = 1:Na
    %     ii1 = aa*Ns-Ns+1:aa*Ns;
    %     ii2 = aa*Nu-Nu+1:aa*Nu;
    %     xo  = squeeze(x(idx(idx ~= aa),:));
    %     LfB(ii1)     = dBdx(t,x(aa,:),xo) * f(t,x(aa,:));
    %     LgB(ii1,ii2) = dBdx(t,x(aa,:),xo) * g(t,x(aa,:));
    %     FB(ii1)      = B(t,x(aa,:),xo);
    % end
    % % Check Safety
    % if any(FB < 0)
    %     A
    %     b
    %     error('Safety Violated')
    % end
    % 
    % As  = [-LgB(:,1:Nua) -eye(length(FB)).*FB];
    % bs  = LfB;

    % Constraint Matrix
    % A = [Ac; Ak; As];
    % b = [bc; bk; bs];
%     A = [Ac; As];
%     b = [bc; bs];
    A = [Ac];
    b = [bc];
    % A = [];
    % b = [];

    % Solve Optimization problem
    % 1/2*x^T*Q*x + p*x subject to Ax <= b
    sol = quadprog(Q,p,A,b,[],[],[],[],[],options);
    if isempty(sol)
        A
        b
    end
    
    u(aa,:)     = sol(aa*Nu+(-1:0));
    uLast(aa,:) = sol(1:Nua);
    mincbf(aa)  = 0;%min(cbf);
%     d(aa,:) = sol(Nua+1:end);
%     u_nom(aa,:) = u_0;
end

% Configure relevant variables for logging
% u_nom = permute(reshape(u_nom,[1 Na Nua]),[1 2 3]);
u     = permute(reshape(u,[1 Na Nu]),[1 2 3]);
cbf   = min(mincbf);
data = struct('uLast',uLast,'cbf',cbf);


end




%------------- END OF CODE --------------
