function [Q,p] = priority_cost(u_nom,settings)

% Unpack settings
q      = settings.q;
Nu     = settings.Nu;
k      = settings.k;

% Cost Fcn: J = (x^T * Q * x) + (p * x) 
Q = [eye(Nu).*q(1:Nu)];
p = [-2*ones(Nu,1).*u_nom.*q(1:Nu)];

for ii = 1:length(p)
    if ii <= 8
%         % Nu = 2
%         Q(ii,ii) = k(ceil(ii/2))*Q(ii,ii);
%         p(ii)    = k(ceil(ii/2))*p(ii);

        % Nu = 1
        Q(ii,ii) = k(ii)*Q(ii,ii);
        p(ii)    = k(ii)*p(ii);
    end
end

end