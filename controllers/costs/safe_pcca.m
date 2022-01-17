function [Q,p] = safe_pcca(u_nom,settings)

% Unpack settings
q      = settings.q;
Nu     = settings.Nu;
idx_aa = settings.idx;
k      = settings.k;


% Tracking Nominal Controller: J = 1/2*||u-u_nom||^2 + sum(pi*ai^2)
Q = [eye(Nu).*q(1:Nu)];
p = [-2*ones(Nu,1).*u_nom.*q(1:Nu)];

% % This worked for rail_control setting
% for ii = 1:length(p)
%     if ~ismember(ii,[1 2 3]) && ~ismember(ii,idx_aa) && ii <= 4
%         Q(ii,ii) = k(ii)*Q(ii,ii);
%         p(ii)    = k(ii)*p(ii);
%     end
% end

for ii = 1:length(p)
    if ~ismember(ii,[1:6]) && ~ismember(ii,idx_aa) && ii <= 8
        Q(ii,ii) = k(ceil(ii/2))*Q(ii,ii);
        p(ii)    = k(ceil(ii/2))*p(ii);
    end
end

end