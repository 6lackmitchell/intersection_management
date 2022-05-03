function [Am,bm,hm] = get_mass_constraints_pairwise(t,x,settings)
% Hard values
Lr = settings.Lr;
Nu = 1;

% Unpack settings
Na    = settings.Na;
Nn    = settings.Nn;
uNom  = settings.uNom;
uMax  = settings.uMax;

% Steering input
betadot = uNom(:,1);

% Constraint setup
Am = zeros((Na-Nn)*Nn,Nu*Na);
bm = zeros((Na-Nn)*Nn,1);
hm = 100*ones((Na-Nn)*Nn,1);

% Mass params

% % These were fine for global
% K        = 2;
% D        = 4.0;

% Trying these for pairwise
K = 0.5;
K = 1;
D = 6.0;
l0 = 100.0;
l1 = sqrt(4.1*l0);

maxL     = 4;
maxDist  = norm([maxL 1.5]); % Max distance 
Mmax     = 100*ones(Na,1);
% Mmax((Na-Nn)+1:end) = 2*Mmax((Na-Nn)+1:end);
Mcrit    = get_Ml(Mmax(end),maxDist,K,D) + get_Ml(Mmax(1),maxDist,K,D); % No more than 1 NC car in intersection

% Get Mass and Derivatives
[M,Mdot,M2dot_unc,M2dot_con,M2dot_disturb] = getMandDerivatives(x,betadot,uMax,Na,Nn,Lr,Mmax,K,D);

idx = 1;
for aa = 1:Na-Nn
    for ii = (Na-Nn+1):Na
        % CBF Ingredients
        B             = Mcrit - M(aa) - M(ii);
        Bdot          = -(Mdot(aa) + Mdot(ii));
        B2dot_unc     = -(M2dot_unc(aa) + M2dot_unc(ii));
        B2dot_con     = zeros(Na,1);
        B2dot_con(aa) = -M2dot_con(aa);

        % Add robust-adaptive disturbance
        B2dot_unc = B2dot_unc - M2dot_disturb(ii-(Na-Nn))*exp(-3*B);

        % CBF
        H   = B;
        LfH = Bdot*l1 + B2dot_unc;
        LgH = B2dot_con;

        % Inequalities: Ax <= b
        Am(idx,:) = -LgH;
        bm(idx)   =  LfH + l0*H; 
        hm(idx)   =  H;

        % Increment
        idx = idx + 1;

    end

end

% % Summations over all vehicles
% M         = sum(M);
% Mdot      = sum(Mdot);
% M2dot_unc = sum(M2dot_unc);
% 
% % Density CBF
% B         = rho_max - M;
% Bdot      = -Mdot;
% B2dot_unc = -M2dot_unc;
% B2dot_con = -M2dot_con;
% 
% % CBF Conditions
% H   = B;
% LfH = Bdot*l1 + B2dot_unc;
% LgH = B2dot_con;
% 
% % Inequalities: Ax <= b
% Am  = -LgH;
% bm  =  LfH + l0*H; 
% hm  =  H;

end

function [M,Mdot,M2dot_unc,M2dot_con,M2dot_disturbance] = getMandDerivatives(x,betadot,uMax,Na,Nn,Lr,Mmax,K,D)

% X components
X    = vecnorm(x(:,1:2)')';

% Xdot components
xdot = x(:,4).*(cos(x(:,3)) - sin(x(:,3)).*tan(x(:,5)));
ydot = x(:,4).*(sin(x(:,3)) + cos(x(:,3)).*tan(x(:,5)));
Xdot = (x(:,1).*xdot + x(:,2).*ydot) ./ (X);

% X2dot components
betadot   = [betadot(1:(Na-Nn)); zeros(Nn,1)];
x2dot_unc = -x(:,4).^2/Lr.*tan(x(:,5)).*(sin(x(:,3)) + cos(x(:,3)).*tan(x(:,5))) - betadot.*x(:,4).*sin(x(:,3)).*sec(x(:,5)).^2;
y2dot_unc =  x(:,4).^2/Lr.*tan(x(:,5)).*(cos(x(:,3)) - sin(x(:,3)).*tan(x(:,5))) + betadot.*x(:,4).*cos(x(:,3)).*sec(x(:,5)).^2;
x2dot_con = cos(x(:,3))-sin(x(:,3)).*tan(x(:,5));
y2dot_con = sin(x(:,3))+cos(x(:,3)).*tan(x(:,5));
X2dot_unc = (x(:,1).*x2dot_unc + x(:,2).*y2dot_unc + xdot.^2 + ydot.^2)./X - (x(:,1).*xdot + x(:,2).*ydot)./(X.^2).*Xdot;
X2dot_con = (x(:,1).*x2dot_con + x(:,2).*y2dot_con)./X;

% Get masses
arg  = K*(X-D);
M = Mmax / 2 .* (1 - tanh(arg));

% Get mass derivatives
Mdot = -K/2 * Mmax .* sech(arg).^2 .* Xdot;

% Get uncontrolled 2nd derivatives
M2dot_unc = -K/2 * Mmax .* (X2dot_unc.*sech(arg).^2 - 2*K*Xdot.*tanh(arg).*sech(arg).^2);

% % Add worst case action for noncommunicating car as disturbance
% M2dot_disturbance = abs(-K/2*Mmax((Na-Nn)+1:end).*sech(arg((Na-Nn)+1:end)).^2.*X2dot_con((Na-Nn)+1:end)) * uMax(2);
% M2dot_unc((Na-Nn)+1:end) = M2dot_unc((Na-Nn)+1:end) + M2dot_disturbance;

% Add robust-adaptive action for noncommunicating car as disturbance
M2dot_disturbance = abs(-K/2*Mmax((Na-Nn)+1:end).*sech(arg((Na-Nn)+1:end)).^2.*X2dot_con((Na-Nn)+1:end)) * uMax(2);

% Controlled 2nd derivatives
M2dot_con = -K/2 * Mmax .* (X2dot_con.*sech(arg).^2);
M2dot_con((Na-Nn)+1:end) = 0*M2dot_con((Na-Nn)+1:end);
M2dot_con = M2dot_con';

end

function [Ml] = get_Ml(Mmax,l,K,D)
Ml   = Mmax / 2 * (1 - tanh(K*(l - D)));
end
