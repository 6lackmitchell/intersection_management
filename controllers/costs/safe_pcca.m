function [Q,p] = safe_pcca(u_nom,q,Nu,h,idx_aa)

% h raised to the -1/n
n = 2;

% Tracking Nominal Controller: J = 1/2*||u-u_nom||^2 + sum(pi*ai^2)
Q = [eye(Nu).*q(1:Nu) zeros(Nu,Np+Ns)];
p = [-2*ones(Nu,1).*u_nom.*q(1:Nu)];

for ii = 1:length(p)
    if ~ismember(ii,idx_aa)
        Q(ii,ii) = Q(ii,ii) * h(ii)^(-1/n);
        p(ii)    = p(ii)    * h(ii)^(-1/n);
    end
end

if Np > 0
    Q = [Q; zeros(Np,Nu) eye(Np).*q(Nu+1:Nu+Np) zeros(Np,Ns)];
    p = [p; zeros(Np,1)];
end

if Ns > 0
    Q = [Q; zeros(Ns,Nu+Np)  eye(Ns).*q(Nu+Np+1:end)];
    p = [p; zeros(Ns,1)];
end

end