function [u,d] = clf_cbf_qp(dynamics,mode,t,x,xg)
%clf_cbf_qp - Controller based on CLF-CBF-QP Formulation
%This controller solves a Quadratic Program with a Control Barrier
%Function (CBF)-based safety constraint and a Control Lyapunov Function
%(CLF)-based performance constraint. 
%
% Syntax:  [u] = clf_cbf_qp(t,x)
%
% Inputs:
%    t - current time in sec
%    x - current state vector -- ROW vector
%
% Outputs:
%    u - control input - ROW vector
%
% Example: 
%    [u(k+1,:),d] = clf_cbf_qp(t,x)
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: OTHER_FUNCTION_NAME1,  OTHER_FUNCTION_NAME2
% Author: Mitchell Black
% Email: mblackjr@umich.edu
% Website: http://www.blackmitchell.com
% May 2021; Last revision: 29-May-2021
%------------- BEGIN CODE --------------
%run('dynamics.m')
run('control.m')
options = optimoptions('quadprog','Display','off');

[xdot,f,g] = dynamics(mode,t,x,[0]);

% Performance Condition: CLF Derivatives
LfV = dVdx(t,x-xg) * f(t,x);
LgV = dVdx(t,x-xg) * g(t,x);
FV  = -c1*V(t,x-xg)^e1 - c2*V(t,x-xg)^e2;

% Safety Condition: CBF Derivatives
LfB = dBdx(t,x) * f(t,x);
LgB = dBdx(t,x) * g(t,x);
FB  = B(t,x);

% Minimum Norm Controller: J = ||u||^2 + p0*d0^2 + p1*d1^2
Q = [eye(Nu).*q(1:Nu) zeros(Nu,Np+Ns);
     zeros(Np,Nu)     eye(Np).*q(Nu+1:Nu+Np) zeros(Np,Ns);
     zeros(Ns,Nu+Np)  eye(Ns).*q(Nu+Np+1:end)];
p = zeros(Nu+Np+Ns,1);

% Constraints
A = [        1         0   0   0   0   0   0;
            -1         0   0   0   0   0   0;
             0         1   0   0   0   0   0;
             0        -1   0   0   0   0   0;
             0         0  -1   0   0   0   0;
             0         0   0  -1   0   0   0;
             0         0   0   0  -1   0   0;
             0         0   0   0   0  -1   0;
             0         0   0   0   0   0  -1;
        LgV(1)    LgV(2)  -1   0   0   0   0;
     -LgB(:,1) -LgB(:,2)  zeros(length(LgB),1)  [-FB(1); 0; 0; 0]  [0; -FB(2); 0; 0] [0; 0; -FB(3); 0] [0; 0; 0; -FB(4)]];
b = [u_max; u_max; u_max; u_max; 0; 0; 0; 0; 0; FV - LfV; LfB];
    

% 1/2*x^T*Q*x + p*x subject to Ax <= b
sol = quadprog(Q,p,A,b,[],[],[],[],[],options);
if isempty(sol)
    A
    b
end
u   = sol(1:2);
d   = sol(3:end);

if FB(1) < 0 || FB(2) < 0
    A
    b
    throw()
end

end




%------------- END OF CODE --------------
