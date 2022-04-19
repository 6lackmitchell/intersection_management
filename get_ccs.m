function [A,b,h,h0] = get_ccs(t,x,settings)
%GET_SAFETY_CONSTRAINTS This is where the safety measures are considered

% Constraint parameters
Nu      = 1;
Na      = settings.Na;
nAScons = 2; % Number of agent-specific constraints
nIAcons = factorial(Na-1); % Number of interagent constraints
nRows   = nAScons*Na+nIAcons;
nCols   = Nu*Na;

% Constraint matrix/vector: Ax <= b, and CBF h
A  = zeros(nRows,nCols);
b  = zeros(nRows,1);
h  = zeros(nRows,1);
h0 = zeros(nRows,1);

% Iterate over all agents
for aa = 1:Na
    
    % Speed constraints
    [A1,b1,h1]   = get_speed_constraints(t,x,aa,settings.SL);

    % Specify row and column index for agent
    row_idx = (-(nAScons-1):0)+aa*nAScons;
    col_idx = aa;

    % Set constraints for agent
    A(row_idx,col_idx) = A1;
    b(row_idx)         = b1;
    h(row_idx)         = h1;
    h0(row_idx)        = h1;

end

[A4,b4,h4,h04] = get_collision_avoidance_constraints(t,x,settings);

row_idx = nAScons*aa + (1:nIAcons);

A(row_idx,:) = A4;
b(row_idx)   = b4;
h(row_idx)   = h4;
h0(row_idx)  = h04;

end

function [A,b,h] = get_speed_constraints(t,x,aa,SL)
kf   = 10; % Class K gain
hf   = SL - x(aa,4);
Lfhf = 0;
Lghf = -1;

kr   = kf;
Lfhr = 0;

backup = 1;
if backup
    % Reverse allowed
    hr   = 100;
    Lghr = 0;
else
    % Reverse not allowed
    hr   = x(aa,4);
    Lghr = 1;
end

A = [-Lghf; -Lghr];
b = [Lfhf + kf*hf; Lfhr + kr*hr];

A = round(A,12);
b = round(b,12);
h = [hf; hr];

end

function [A,b,Ht,H0] = get_collision_avoidance_constraints(t,x,settings)
% Hard values
Lr = settings.Lr;
sw = 1.0;
Nu = 1;

% Unpack settings
Na    = settings.Na;
uNom  = settings.uNom;
tmax  = settings.lookahead;

% Steering input
betadot = uNom(:,1);

% Constraint setup
nCons = factorial(Na-1);
A     = zeros(nCons,Nu*Na);
b     = zeros(nCons,1);
Ht    = zeros(nCons,1);
H0    = zeros(nCons,1);
cc    = 1;
ss    = 1;

% Loop through every agent for complete constraint set
for aa = 1:Na
    
    nc  = Na-aa;
    Aw  = zeros(nc,Nu*Na);
    bw  = zeros(nc,1);
    hw  = zeros(nc,1);
    hw0 = zeros(nc,1);
    dd  = 1;
    
    % Loop through all other agents for interagent completeness
    for ii = aa+1:Na
        
        xa = x(aa,:);
        xi = x(ii,:);
        
        idx_aa = aa;
        idx_ii = ii;
              
        % dx and dy
        dx  = xa(1) - xi(1);
        dy  = xa(2) - xi(2);
        
        % dvx and dvy
        vax = xa(4)*(cos(xa(3)) - sin(xa(3))*tan(xa(5)));
        vay = xa(4)*(sin(xa(3)) + cos(xa(3))*tan(xa(5)));
        vix = xi(4)*(cos(xi(3)) - sin(xi(3))*tan(xi(5)));
        viy = xi(4)*(sin(xi(3)) + cos(xi(3))*tan(xi(5)));
        dvx = vax - vix;
        dvy = vay - viy;
        
        % Solve for minimizer of h
        kh       = 1000;
        eps      = 1e-3;
        tau_star = -(dx*dvx + dy*dvy)/(dvx^2 + dvy^2 + eps);
        Heavy1   = heavyside(tau_star,kh,0);
        Heavy2   = heavyside(tau_star,kh,tmax);
        tau      = tau_star*Heavy1 - (tau_star - tmax)*Heavy2;

        % accelerations -- controlled (con) and uncontrolled (unc)
        axa_unc = -xa(4)^2/Lr*tan(xa(5))*(sin(xa(3)) + cos(xa(3))*tan(xa(5))) - betadot(aa)*xa(4)*sin(xa(3))*sec(xa(5))^2;
        axi_unc = -xi(4)^2/Lr*tan(xi(5))*(sin(xi(3)) + cos(xi(3))*tan(xi(5))) - betadot(ii)*xi(4)*sin(xi(3))*sec(xi(5))^2;
        aya_unc =  xa(4)^2/Lr*tan(xa(5))*(cos(xa(3)) - sin(xa(3))*tan(xa(5))) + betadot(aa)*xa(4)*cos(xa(3))*sec(xa(5))^2;
        ayi_unc =  xi(4)^2/Lr*tan(xi(5))*(cos(xi(3)) - sin(xi(3))*tan(xi(5))) + betadot(ii)*xi(4)*cos(xi(3))*sec(xi(5))^2;
        axa_con = zeros(1,Na*Nu);
        axi_con = zeros(1,Na*Nu);
        aya_con = zeros(1,Na*Nu);
        ayi_con = zeros(1,Na*Nu);
        axa_con(idx_aa) = cos(xa(3))-sin(xa(3))*tan(xa(5));
        axi_con(idx_ii) = cos(xi(3))-sin(xi(3))*tan(xi(5));
        aya_con(idx_aa) = sin(xa(3))+cos(xa(3))*tan(xa(5));
        ayi_con(idx_ii) = sin(xi(3))+cos(xi(3))*tan(xi(5)); 
        
        % dax and day
        dax_unc = axa_unc - axi_unc;
        day_unc = aya_unc - ayi_unc;
        dax_con = axa_con - axi_con;
        day_con = aya_con - ayi_con;
        
        % taudot
        tau_star_dot_unc = -(dax_unc*(2*dvx*tau_star + dx) + day_unc*(2*dvy*tau_star + dy) + (dvx^2 + dvy^2)) / (dvx^2 + dvy^2 + eps);
        tau_star_dot_con = -(dax_con*(2*dvx*tau_star + dx) + day_con*(2*dvy*tau_star + dy)) / (dvx^2 + dvy^2 + eps);

        Heavy_dot1_unc   = dheavyside(tau_star,kh,0)*tau_star_dot_unc;
        Heavy_dot2_unc   = dheavyside(tau_star,kh,tmax)*tau_star_dot_unc;
        Heavy_dot1_con   = dheavyside(tau_star,kh,0)*tau_star_dot_con;
        Heavy_dot2_con   = dheavyside(tau_star,kh,tmax)*tau_star_dot_con;
        tau_dot_unc      = tau_star_dot_unc*(Heavy1 - Heavy2) + tau_star*(Heavy_dot1_unc - Heavy_dot2_unc);
        tau_dot_con      = tau_star_dot_con*(Heavy1 - Heavy2) + tau_star*(Heavy_dot1_con - Heavy_dot2_con);

        % Class K Function(s)
        l0   = settings.classk;
        l1   = sqrt(6*l0);
        
        % h and hdot (= Lfh + Lgh*u)
        h0   = dx^2 + dy^2 - (2*sw)^2;
        h    = dx^2 + dy^2 + tau^2*(dvx^2 + dvy^2) + 2*tau*(dx*dvx + dy*dvy) - (2*sw)^2;
        Lfh0 = 2*dx*dvx + 2*dy*dvy;
        Lfh  = 2*(dx*dvx + dy*dvy) + 2*tau*(dvx^2 + dvy^2 + dx*dax_unc + dy*day_unc) + 2*tau_dot_unc*(dx*dvx + dy*dvy + tau*(dvx^2 + dvy^2)) + 2*tau^2*(dvx*dax_unc + dvy*day_unc);        
        Lgh  = 2*tau*tau_dot_con*(dvx^2 + dvy^2) + 2*tau^2*(dvx*dax_con + dvy*day_con) + 2*tau_dot_con*(dx*dvx + dy*dvy) + 2*tau*(dx*dax_con + dy*day_con);
       
        if strcmp(settings.cbf_type,'nominal_cbf')
            % Nominal CBF (Rel-Deg 2)
            H   = h0;
            LfH = l1*Lfh0 + 2*(dvx^2 + dvy^2) + 2*(dx*dax_unc + dy*day_unc);
            LgH = 2*(dx*dax_con + dy*day_con);

        elseif strcmp(settings.cbf_type,'ff_cbf')
            % Future Focused CBF
            l0  = h0;
            H   = h;
            LfH = Lfh;
            LgH = Lgh;

        elseif strcmp(settings.cbf_type,'rv_cbf')
            % Robust-Virtual CBF
            a1    = 0.1;
            kh0   = 1;
            H     = h   + a1*max([tau-1,eps])*h0^(1/kh0);
            LfH   = Lfh + a1*(max([tau-1,eps])*(1/kh0)*h0^(1/kh0-1)*Lfh0 + tau_dot_unc*h0^(1/kh0));
            LgH   = Lgh + a1*tau_dot_con*h0^(1/kh0);
        end

        % PCCA Contribution
        if settings.pcca
            LfH = LfH + Lgh*wHat(AAA,1:4)';
        end
    
        % Inequalities: Ax <= b
        Aw(dd,1:Na*Nu)  = -LgH;
        bw(dd)          = LfH + l0*H; 
        hw(dd)          = H;
        hw0(dd)         = h0;

        dd = dd + 1;
        ss = ss + 1;

    end
    
    A(cc:cc+(nc-1),:) = Aw;
    b(cc:cc+(nc-1))   = bw;
    Ht(cc:cc+(nc-1))  = hw;
    H0(cc:cc+(nc-1))  = hw0;

    cc = cc + nc;
    
end

end
