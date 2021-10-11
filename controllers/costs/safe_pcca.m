function [Q,p] = safe_pcca(u_nom,q,Nu,h,idx_aa)

% h raised to the -1/n
n = 4;
k = 0.25;
n = 2;
k = 0.001;

% Tracking Nominal Controller: J = 1/2*||u-u_nom||^2 + sum(pi*ai^2)
Q = [eye(Nu).*q(1:Nu)];
p = [-2*ones(Nu,1).*u_nom.*q(1:Nu)];

for ii = 1:length(p)
    ii_h     = floor((ii-1)/2)+1;
    if ~ismember(ii,idx_aa) && h(ii_h) ~= inf
        Q(ii,ii) = Q(ii,ii) / (k*h(ii_h)^(1/n));
        p(ii)    = p(ii)    / (k*h(ii_h)^(1/n));
    end
end

end