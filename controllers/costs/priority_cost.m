function [Q,p] = priority_cost(u_nom,settings)

% Unpack settings
q      = settings.q;
Nu     = settings.Nu;
Na     = settings.Na;
k      = settings.k;

% Cost Fcn: J = (x^T * Q * x) + (p * x) 
Q = [eye(length(q)).*q];
p = [-2*ones(Nu,1).*u_nom.*q(1:Nu)];
% p = [p; zeros(size(Q,1) - length(p),1)];

for ii = 1:length(p)
    if ii <= 2*Na
        % Nu = 2
        Q(ii,ii) = k(ceil(ii/2))*Q(ii,ii);
        p(ii)    = k(ceil(ii/2))*p(ii);

%         % Nu = 1
%         Q(ii,ii) = k(ii)*Q(ii,ii);
%         p(ii)    = k(ii)*p(ii);
    end
end

end