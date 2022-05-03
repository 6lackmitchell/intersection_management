function [Am,bm,hm] = get_density_constraints(t,x,settings)
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
Am = zeros(2,Nu*Na+1);
bm = zeros(2,1);
hm = 100;

% Density params
% Params
K          = 0.2;
D          = 3.0;
N          = 1; % N cars allowed in intersection at once
maxL       = 20;
maxDist    = norm([maxL 1.5]); % Max distance 
total_area = (2*maxL)^2;
Mmax       = total_area / Na * ones(Na,1);
Mmax((Na-Nn)+1:end) = 2*Mmax((Na-Nn)+1:end);
% rho_max    = N / Na; % At most N cars in intersection
rho_max    = (get_Ml(Mmax(end),K,D) + get_Ml(Mmax(1),K,D)) / total_area; % No more than 1 NC car in intersection

% Get Bounding Box Area and Derivatives
[A,Adot,A2dot_unc,A2dot_con] = getAandDerivatives(x,betadot,N,Na,Nn,Nu,Lr,maxDist,maxL);

if A < total_area
    return
end

% Get Mass and Derivatives
[M,Mdot,M2dot_unc,M2dot_con] = getMandDerivatives(x,betadot,uMax,Na,Nn,Lr,Mmax,K,D);

% Summations over all vehicles
M         = sum(M);
Mdot      = sum(Mdot);
M2dot_unc = sum(M2dot_unc);

% Density CBF
B         = rho_max - M/A;
Bdot      = M*Adot/A^2 - Mdot/A;
B2dot_unc = 2*Mdot*Adot/A^2 + M*A2dot_unc/A^2 - 2*M*Adot^2/A^3 - M2dot_unc/A;
B2dot_con = M*A2dot_con/A^2 - M2dot_con/A;

% Exponential CBF Condition
% [l0,l1] = get_l0l1(B,Bdot,B2dot_unc,B2dot_con);


% CBF Conditions
l0  = 1;
H   = B;
LfH = B2dot_unc + l0*Bdot;
LgH = B2dot_con;

% Inequalities: Am * [u; l0; l1] <= bm
Am(1,:)  = [ -LgH (-l0*B-Bdot)]; % CBF Constraint
Am(2,:)  = [0*LgH -1]; % l0 constraint
bm(1)    = LfH;
bm(2)    = -1e-6;
hm  = H;



% H   = B;
% LfH = Bdot*l1 + B2dot_unc;
% LgH = B2dot_con;
% 
% % Inequalities: Ax <= b
% Am  = -LgH;
% bm  =  LfH + l0*H; 
% hm  =  H;

end

function [z,zdot,z2dot_u,z2dot_c] = enforce_rectangle_max(z,zdot,z2dot_u,z2dot_c)

lowerbound = 3;
if z < lowerbound
    z = lowerbound;
    zdot = 0;
    z2dot_u = 0;
    z2dot_c = zeros(size(z2dot_c));
end

end

function [z,zdot,z2dot_u,z2dot_c] = enforce_rectangle_min(z,zdot,z2dot_u,z2dot_c)

upperbound = -3;
if z > upperbound
    z = upperbound;
    zdot = 0;
    z2dot_u = 0;
    z2dot_c = zeros(size(z2dot_c));
end

end

function [A,Adot,A2dot_unc,A2dot_con] = getAandDerivatives(x,betadot,N,Na,Nn,Nu,Lr,maxDist,maxL)

% This is for fixed area approach
[A,Adot,A2dot_unc,A2dot_con,eligible] = getFixedAandDerivatives(x,N,Na,Nn,maxDist,maxL);
return % Uncomment this line for variable area approach

% From here on out is variable area approach

% Vehicle Bounding Box
[xmax,idx_xmax] = max(x(eligible,1));
[xmin,idx_xmin] = min(x(eligible,1));
[ymax,idx_ymax] = max(x(eligible,2));
[ymin,idx_ymin] = min(x(eligible,2));

% Rectangle Vertex Properties
xMd = x(idx_xmax,:);
xmd = x(idx_xmin,:);
yMd = x(idx_ymax,:);
ymd = x(idx_ymin,:);
xmax_dot = xMd(4)*(cos(xMd(3)) - sin(xMd(3))*tan(xMd(5)));
xmin_dot = xmd(4)*(cos(xmd(3)) - sin(xmd(3))*tan(xmd(5)));
ymax_dot = yMd(4)*(sin(yMd(3)) + cos(yMd(3))*tan(yMd(5)));
ymin_dot = ymd(4)*(sin(ymd(3)) + cos(ymd(3))*tan(ymd(5)));
xmax_2dot_unc = -xMd(4)^2/Lr*tan(xMd(5))*(sin(xMd(3)) + cos(xMd(3))*tan(xMd(5))) - betadot(idx_xmax)*xMd(4)*sin(xMd(3))*sec(xMd(5))^2;
xmin_2dot_unc = -xmd(4)^2/Lr*tan(xmd(5))*(sin(xmd(3)) + cos(xmd(3))*tan(xmd(5))) - betadot(idx_xmin)*xmd(4)*sin(xmd(3))*sec(xmd(5))^2;
ymax_2dot_unc =  yMd(4)^2/Lr*tan(yMd(5))*(cos(yMd(3)) - sin(yMd(3))*tan(yMd(5))) + betadot(idx_ymax)*yMd(4)*cos(yMd(3))*sec(yMd(5))^2;
ymin_2dot_unc =  ymd(4)^2/Lr*tan(ymd(5))*(cos(ymd(3)) - sin(ymd(3))*tan(ymd(5))) + betadot(idx_ymin)*ymd(4)*cos(ymd(3))*sec(ymd(5))^2;
xmax_2dot_con = zeros(1,(Na-Nn)*Nu);
xmin_2dot_con = zeros(1,(Na-Nn)*Nu);
ymax_2dot_con = zeros(1,(Na-Nn)*Nu);
ymin_2dot_con = zeros(1,(Na-Nn)*Nu);
xmax_2dot_con(idx_xmax) = cos(xMd(3))-sin(xMd(3))*tan(xMd(5));
xmin_2dot_con(idx_xmin) = cos(xmd(3))-sin(xmd(3))*tan(xmd(5));
ymax_2dot_con(idx_ymax) = sin(yMd(3))+cos(yMd(3))*tan(yMd(5));
ymin_2dot_con(idx_ymin) = sin(ymd(3))+cos(ymd(3))*tan(ymd(5));
[xmax,xmax_dot,xmax_2dot_unc,xmax_2dot_con] = enforce_rectangle_max(xmax,xmax_dot,xmax_2dot_unc,xmax_2dot_con);
[xmin,xmin_dot,xmin_2dot_unc,xmin_2dot_con] = enforce_rectangle_min(xmin,xmin_dot,xmin_2dot_unc,xmin_2dot_con);
[ymax,ymax_dot,ymax_2dot_unc,ymax_2dot_con] = enforce_rectangle_max(ymax,ymax_dot,ymax_2dot_unc,ymax_2dot_con);
[ymin,ymin_dot,ymin_2dot_unc,ymin_2dot_con] = enforce_rectangle_min(ymin,ymin_dot,ymin_2dot_unc,ymin_2dot_con);

% Bounding Box Area and Derivatives
A         = (xmax-xmin)*(ymax-ymin);
Adot      = (xmax_dot-xmin_dot)*(ymax-ymin) + (xmax-xmin)*(ymax_dot-ymin_dot);
A2dot_unc = 2*(xmax_dot-xmin_dot)*(ymax_dot-ymin_dot) + (xmax_2dot_unc-xmin_2dot_unc)*(ymax-ymin) + (xmax-xmin)*(ymax_2dot_unc-ymin_2dot_unc);
A2dot_con = (xmax_2dot_con-xmin_2dot_con)*(ymax-ymin) + (xmax-xmin)*(ymax_2dot_con-ymin_2dot_con);   

end

function [A,Adot,A2dot_unc,A2dot_con,eligible] = getFixedAandDerivatives(x,N,Na,Nn,maxDist,maxL)

% Initialize
A         = 0;
Adot      = 0;
A2dot_unc = 0;
A2dot_con = zeros(1,Na);

% Find eligible vehicles
eligible      = find(vecnorm(x(1:(Na-Nn),1:2)')<maxDist); % Within minimum distance

if length(eligible) < N
    return
end

% This is for fixed area approach. Uncomment these two lines below for
% variable area approach
A = (2*maxL)^2;

end

function [M,Mdot,M2dot_unc,M2dot_con] = getMandDerivatives(x,betadot,uMax,Na,Nn,Lr,Mmax,K,D)

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

% Add worst case action for noncommunicating car as disturbance
M2dot_disturbance = abs(-K/2*Mmax((Na-Nn)+1:end).*sech(arg((Na-Nn)+1:end)).^2.*X2dot_con((Na-Nn)+1:end)) * uMax(2);
M2dot_unc((Na-Nn)+1:end) = M2dot_unc((Na-Nn)+1:end) - M2dot_disturbance;

% Controlled 2nd derivatives
M2dot_con = -K/2 * Mmax .* (X2dot_con.*sech(arg).^2);
M2dot_con((Na-Nn)+1:end) = 0*M2dot_con((Na-Nn)+1:end);
M2dot_con = M2dot_con';

end

function [Ml] = get_Ml(Mmax,K,D)
Ml   = Mmax / 2 * (1 - tanh(K*(norm([4 1.5]) - D)));
end

function [l0,l1] = get_l0l1(B,Bdot,B2dot_unc,B2dot_con)

a = Bdot*sqrt(6);
b = B;
c = -B2dot_unc - sum(abs(B2dot_con))*9.81;

l0 = (a^2 - sqrt(a^4 - 4*a^2*b*c) + 2*b*c)/(2*b);
if l0 < 0
    l0 = (a^2 + sqrt(a^4 - 4*a^2*b*c) + 2*b*c)/(2*b);
end

if l0 < 0
    l0 = 1;
end
   
l1 = sqrt(6*l0);

end
