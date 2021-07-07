function [Q,p] = min_diff_nominal(u_nom,q,Nu,Np,Ns)

% Tracking Nominal Controller: J = 1/2*||u-u_nom||^2 + sum(pi*ai^2)
Q = [eye(Nu).*q(1:Nu) zeros(Nu,Np+Ns)];
p = [-2*ones(Nu,1).*u_nom];

if Np > 0
    Q = [Q; zeros(Np,Nu) eye(Np).*q(Nu+1:Nu+Np) zeros(Np,Ns)];
    p = [p; zeros(Np,1)];
end

if Ns > 0
    Q = [Q; zeros(Ns,Nu+Np)  eye(Ns).*q(Nu+Np+1:end)];
    p = [p; zeros(Ns,1)];
end

end