function trial_data = run_one_trial(trial_setup)
dynamics    = trial_setup.dynamics;
controller  = trial_setup.controller;
t_params    = trial_setup.t_params;
u_params    = trial_setup.u_params;
misc_params = trial_setup.settings;

nTimesteps = t_params.nTimesteps;
dt         = t_params.dt;
nAgents    = misc_params.nAgents;
nStates    = misc_params.nStates;
nControls  = misc_params.nControls;
nNon       = misc_params.nNon;
x0_og      = misc_params.x0;
x0         = misc_params.x0i;
Tpath      = misc_params.Tpath;
Rpath      = misc_params.Rpath;
path       = misc_params.path;
xGoal      = misc_params.xGoal;
im_used    = misc_params.im_used;
Lr         = misc_params.Lr;
SL         = misc_params.SL;
pcca       = misc_params.pcca;
pmetric    = misc_params.pmetric;
cbf_type   = misc_params.cbf_type;
classk     = misc_params.class_k_l0;
ubounds    = misc_params.input_bounds;
backup     = misc_params.backup;

x          = zeros(nTimesteps,nAgents,nStates);
u          = zeros(nTimesteps,nAgents,nControls);
u0         = zeros(nTimesteps,nAgents,nControls);
sols       = zeros(nTimesteps,nAgents,nAgents*nControls+4);
priority   = zeros(nTimesteps,nAgents);
violations = zeros(nTimesteps,nAgents,2);
vio_mag    = zeros(nTimesteps,1);
x(1,:,:)   = x0;

% More Settings
uLast     = zeros(nAgents,nControls);
tSlots    = inf*ones(nAgents,2);
t0        = zeros(nAgents,1);
xS        = zeros(nAgents,2);
th0       = zeros(nAgents,1);
r         = zeros(nAgents,2);
rdot      = zeros(nAgents,2);
r2dot     = zeros(nAgents,2);
gidx      = ones(nAgents,1);
Tfxt      = ones(nAgents,1);
wHat      = zeros(nAgents,nAgents*nControls);
thru_time = Inf*ones(nAgents,1);
success   = zeros(nAgents,1);
tol       = 0.25;

ii = 1;

while ii <= nTimesteps
    % Update time and state
    t   = ii *  dt;
    xx  = squeeze(x(ii,:,:));

    % Get nominal inputs
    unom = get_nominal_trajectories(t,xx);

    % Solve for control inputs
    for aa = 1:nAgents
        data = controller{aa}(t,xx,settings,u_params);
    end

    % Update Settings
    for aa = 1:nAgents

        % Determine which path segment is active
        reached     = norm(xGoal{aa}(gidx(aa),:) - xx(aa,1:2)) < tol;
        closer      = norm(xGoal{aa}(min(gidx(aa)+1,size(xGoal{aa},1)),:) - xx(aa,1:2)) < norm(xGoal{aa}(gidx(aa),:) - xx(aa,1:2));
        past        = past_goal(xx(aa,1:2),xGoal{aa}(gidx(aa),:),x0(aa,1:2));
        allowed     = 1*(~im_used) + im_used*(t > tSlots(aa,1));
        addidx      = (reached || closer || past) && allowed;
        newidx      = gidx(aa) + addidx;
%         if addidx == 1
%             disp("*** Agent "+num2str(aa)+" has reached Goal "+num2str(newidx-1)+"!!")
%         end

        % Set quit flag to true if final goal met
        if newidx == size(xGoal{aa},1)% - 1
            thru_time(aa) = t;
            success(aa) = 1;
        end

        if sum(success) == nAgents
            break
        end

        % Specify path segment
        old_gidx    = gidx(aa);
        gidx(aa)    = min(newidx,size(xGoal{aa},1));

        % Assign path segment data
        if old_gidx ~= gidx(aa) || ii == 1
            t0(aa)      = t;
            xS(aa,:)    = xx(aa,1:2); % Old way

            if strcmp(func2str(dynamics),'double_integrator')
                th0(aa) = atan2(xx(aa,4),xx(aa,3));
            else
                th0(aa) = xx(aa,3);
            end

        end

        % Consolidate path segment data and get trajectory info
        Tfxt(aa) = Tpath{aa}(gidx(aa));

%         % Adjust Tfxt according to priority
%         Tfxt(aa) = adjust_fxts_time(priority);

        settings = struct('T',    Tfxt(aa),             ...
                          't0',   t0(aa),               ...
                          'xS',   xS(aa,:),             ...
                          'xF',   xGoal{aa}(gidx(aa),:),...
                          'th0',  th0(aa),              ...
                          'R',    Rpath{aa}(gidx(aa)),  ...
                          'path', path{aa}{gidx(aa)});
        [r(aa,:),rdot(aa,:),r2dot(aa,:)] = trajectories(t,xx(aa,:),settings);

    end

    settings          = struct('dynamics', @dynamics, ...
                               'uLast',    uLast,     ...
                               'tSlots',   tSlots ,   ...
                               'wHat',     wHat,      ...
                               'Tfxt',     Tfxt,      ...
                               'r',        r,         ...
                               'rdot',     rdot,      ...
                               'r2dot',    r2dot,     ...
                               'Lr',       Lr,        ...
                               'SL',       SL,        ...
                               't0',       t0,        ...
                               'prior',    priority(max(ii-1,1),:), ...
                               'pcca',     pcca,      ...
                               'pmetric',  pmetric,   ...
                               'cbf_type', cbf_type,  ...
                               'classk',   classk,    ...
                               'ubounds',  ubounds,   ...
                               'backup',   backup,    ...
                               'Nn',       nNon,      ...
                               'dt',       dt);
    try
        % Compute control input
        data         = controller{aa}(t,xx,settings,u_params);

        if data.code == 1

            % Organize data
            u(ii,:,:)       = data.u;
            sols(ii,:,:)    = data.sols;
            uLast           = data.uLast;
    %         safety(ii,:)    = data.cbf;
            tSlots          = data.tSlots;
            u0(ii,:,:)      = data.uNom;
            wHat            = data.wHat;
            priority(ii,:)  = data.prior;
            violations(ii,:) = [data.v_vio; data.p_vio]';
        elseif t == dt
            [x0_new,Tpath_new]  = randomize_ic(x0_og,Tpath,func2str(dynamics));
            x(ii,:,:) = x0_new;
            Tpath = Tpath_new;
            continue
        elseif data.code == -1
            violations(ii,:) = [data.v_vio; data.p_vio]';
            vio_mag(ii)      = data.vio_mag;
            break
        elseif data.code == 3 || data.code == 4
            data.code = 0;
            break
        else
            data.code = 0;
            break
        end

%         deadlock1 = 0;%(sum(xx(:,4).^2) < 0.01);
%         deadlock2 = 0;
%         if t > 3
%             dx = squeeze(x(ii,:,1:2) - x(ii-floor(3/dt),:,1:2));
%             deadlock2 = any(sum(dx'.^2)<0.1);
%         end
% 
%         if deadlock1 || deadlock2
%             data.code = -2;
%             break
%         end

    catch ME
        break
    end

    % idx of vehicle outside 20m from intersection center
    outside_idx = find(sqrt(sum(squeeze(x(ii,:,1:2))'.^2)) > 20);
    if sum(success) == nAgents || any(success(outside_idx) == 0)
        break
    end

    % Update Dynamics
    xdot          = dynamics(t,squeeze(x(ii,:,:)),squeeze(u(ii,:,:)),settings);
    x(ii + 1,:,:) = x(ii,:,:) + dt * reshape(xdot,[1 size(xdot)]);

    % Restrict Angles to between -pi and pi
    if ~strcmp(func2str(dynamics),'double_integrator')
        x(ii + 1,:,3) = wrapToPi(x(ii + 1,:,3));
    end

    % Iterate
    ii = ii + 1;

end

trial_data = struct('success', sum(success)==nAgents, ...
                    'code',    data.code,             ...
                    'TTI',     thru_time,             ...
                    'x',       x,                     ...  
                    'u',       u,                     ...
                    'u0',      u0,                    ...
                    't',       t,                     ...
                    'sols',    sols,                  ...
                    'vios',    violations,            ...,
                    'vmags',   vio_mag);

end