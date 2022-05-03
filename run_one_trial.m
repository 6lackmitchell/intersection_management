function trial_data = run_one_trial(dynamics,nominal_u,controller,trial_setup)
t_params    = trial_setup.t_params;
u_params    = trial_setup.u_params;
misc_params = trial_setup.settings;

% Unpack time parameters
nTimesteps = t_params.nTimesteps;
dt         = t_params.dt;

% Unpack misc params
x0_og      = misc_params.x0;
x0         = misc_params.x0i;
umax       = misc_params.umax;
nNon       = misc_params.nNon;
nControls  = misc_params.nControls;
im_used    = misc_params.im_used;
Lr         = misc_params.Lr;
SL         = misc_params.SL;
pcca       = misc_params.pcca;
pmetric    = misc_params.pmetric;
cbf_type   = misc_params.cbf_type;
classk     = misc_params.classK_l0;
ubounds    = misc_params.u_bounds;
backup     = misc_params.backup;
ppower     = misc_params.ppower;
nAgents    = size(x0,1);
nStates    = size(x0,2);

x          = zeros(nTimesteps,nAgents,nStates);
u          = zeros(nTimesteps,nAgents,nControls);
u0         = zeros(nTimesteps,nAgents,nControls);
% sols       = zeros(nTimesteps,nAgents,2);
sols       = zeros(nTimesteps,nAgents,nAgents);
priority   = zeros(nTimesteps,nAgents,nAgents);
violations = zeros(nTimesteps,nAgents,2);
vio_mag    = zeros(nTimesteps,1);
% barrier    = zeros(nTimesteps,1);
barrier    = zeros(nTimesteps,nAgents);
alpha      = zeros(nTimesteps,nAgents);
hysteresis = zeros(nAgents,1);
exited     = zeros(nAgents,1);
x(1,:,:)   = x0;

% More Settings
uLast     = zeros(nAgents,nAgents,nControls);
wHat      = zeros(nAgents,nAgents*(nControls-1));
thru_time = Inf*ones(nAgents,1);

ii = 1;
unom = zeros(nAgents,nControls);

% Define settings structure for controllers
settings.lookahead = 4.0;
settings.umax      = umax;
settings.ubounds   = ubounds;
settings.backup    = backup;
settings.wHat      = wHat;
settings.SL        = SL;
settings.dt        = dt;
settings.Nu        = nControls;
settings.Nn        = nNon;
settings.Lr        = Lr;

% Intervention settings
intervention_settings.Nu      = nControls;
intervention_settings.Lr      = Lr;
intervention_settings.dt      = dt;
intervention_settings.ubounds = ubounds;
intervention_settings.umax    = umax;

% Iterate timesteps
while ii <= nTimesteps
    continueon = false;
    breakout   = false;

    % Update time and state
    t   = ii *  dt;
    xx  = squeeze(x(ii,:,:));    

    % Get nominal inputs
    for aa = 1:nAgents
        [unom(aa,:),reached] = nominal_u{aa}(t,xx(aa,:),aa,umax);
        exited(aa) = exited(aa) + reached*(exited(aa)==0);
    end

    % Intersection Manager Intervention (if necessary)
    if false % Now putting inside low-level controller
        intervention_settings.unom    = unom;
        intervention_settings.Nn      = nNon;
        intervention_settings.pmetric = pmetric{1};
        intervention_settings.ppower  = ppower{1};
        intervention_settings.prior   = priority(ii,:);
        [unom(1:(nAgents-nNon),2),B,code] = im_intervention(t,xx,intervention_settings,u_params{1});
    else
        B = 100;
    end

    % Check whether all vehicles have reached intersection exit
    if sum(exited) == nAgents
        break
    end

    % Get control inputs
    for aa = 1:nAgents

        % Configure required settings
        settings.aa         = aa;
        settings.wHat       = wHat(aa,:);
        settings.cbf_type   = cbf_type{aa};
        settings.pcca       = pcca{aa};
        settings.pmetric    = pmetric{aa};
        settings.classk     = classk{aa};
        settings.ppower     = ppower{aa};
        settings.prior      = priority(ii,:);
        settings.hysteresis = hysteresis(aa);

        % Noncommunicating -- every other control assumed to be zero
        if aa > nAgents - nNon
            ctrl_idx = (-1:0)+aa*nControls;
            ucost = unom';
            ucost(~ismember(find(ucost>-Inf),ctrl_idx)) = 0;
            ucost = ucost';
            settings.im_intervene = 0;
        
        % Communicating -- every noncomm. control assumed to be zero
        else
            ctrl_idx = 1:nControls*(nAgents-nNon);
            ucost = unom';
            ucost(~ismember(find(ucost>-Inf),ctrl_idx)) = 0;
            ucost = ucost';
            settings.im_intervene = 1;
        end

        % Compute control input
        data = controller{aa}(t,xx,aa,ucost,settings,u_params{aa});

        % Organize data
        if data.code == 1
            u(ii,aa,:)          = data.u;
            sols(ii,aa,:)       = data.sol;
            uLast(aa,:,:)       = data.uSol;
            u0(ii,aa,:)         = unom(aa,:);
%             barrier(ii)         = B;
            barrier(ii,aa)      = data.cbf(1);
            alpha(ii,aa)        = data.alpha;
            priority(ii,aa,:)   = data.prior;
            violations(ii,aa,:) = [data.v_vio; data.p_vio]';
            hysteresis(aa)      = data.hysteresis;
        elseif t == dt
            [x0_new]  = randomize_ic(x0_og,func2str(dynamics));
            x(ii,:,:) = x0_new;
            continueon = true;
        elseif data.code == -1
            violations(ii,aa,:) = [data.v_vio; data.p_vio]';
            vio_mag(ii)         = data.vio_mag;
            breakout = true;
        elseif data.code == 3 || data.code == 4
            data.code = 0;
            breakout = true;
        else
            data.code = 0;
            breakout = true;
        end

        if continueon || breakout
            break
        end
    end

    % Handle continue/break statements inside inner loop
    if continueon
        continue;
    elseif breakout
        break;
    end

    % Update Dynamics
    xdot          = dynamics(t,xx,squeeze(u(ii,:,:)),settings);
    x(ii + 1,:,:) = x(ii,:,:) + dt * reshape(xdot,[1 size(xdot)]);

    % Update PCCA Matrix
    wHat = repmat(squeeze(u(ii,:,2)),nAgents,1) - uLast(:,:,2);

    % Restrict Angles to between -pi and pi
    if ~strcmp(func2str(dynamics),'double_integrator')
        x(ii + 1,:,3) = wrapToPi(x(ii + 1,:,3));
    end

    % Iterate
    ii = ii + 1;

end

trial_data = struct('success', sum(exited)==nAgents, ...
                    'code',    data.code,             ...
                    'TTI',     thru_time,             ...
                    'x',       x,                     ...  
                    'u',       u,                     ...
                    'u0',      u0,                    ...
                    't',       t,                     ...
                    'sols',    sols,                  ...
                    'vios',    violations,            ...,
                    'barrier', barrier,               ...,
                    'alpha',   alpha,                 ...,
                    'vmags',   vio_mag);

end

function [x0_rand] = randomize_ic(x0,dyn_mode)
    x0_rand       = x0;
    d_variability = 5.0;
    s_variability = 3.0;

    for aa = 1:size(x0,1)
        % Random distance from intersection -- uniform dist.
        rand_dist  = (2*d_variability)*(rand(1) - 0.5);
        rand_speed = (2*s_variability)*(rand(1) - 0.5);

        % Randomize Distance from intersection and adjust time
        if abs(x0(aa,2)) == 1.5
            x0_rand(aa,1) = x0(aa,1) + rand_dist;
        else
            x0_rand(aa,2) = x0(aa,2) + rand_dist;
        end
    
        % Random speed
        if ~strcmp(dyn_mode,'double_integrator')
            x0_rand(aa,4) = x0(aa,4) + rand_speed;
        else
            if abs(x0(aa,2)) == 1.5
                x0_rand(aa,3) = x0(aa,3) + rand_speed;
            else
                x0_rand(aa,4) = x0(aa,4) + rand_speed;
            end
        end

    end
end