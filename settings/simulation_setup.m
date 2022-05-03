%% Script is called by simulate_monte_carlo
% Not intended for standalone use

run('timing.m')

% Dynamics and Controller modes
all_identical  = 1;
nAgents        = 4;
campaign       = "testing";
dyn_mode       = "dynamic_bicycle_rdrive_1u";
cost_mode      = "costs";
input_bounds   = false;
backup         = true;
im_used        = 1;

if all_identical
    con_nom    = 'lqr_tracking';           
    con_mode   = 'centralized_cbf_qp';
    cbf_type   = 'rv_cbf';
    pmetric    = 'high_proximity';
    pcca       = false;
    classK_l0  = 10;
    ppower     = 0.1;

    con_nom    = repmat({con_nom},   1, nAgents);
    con_mode   = repmat({con_mode},  1, nAgents);
    cbf_type   = repmat({cbf_type},  1, nAgents);
    pmetric    = repmat({pmetric},   1, nAgents);
    pcca       = repmat({pcca},      1, nAgents);
    classK_l0  = repmat({classK_l0}, 1, nAgents);
    ppower     = repmat({ppower}, 1, nAgents);

else
    con_nom    = {'lqr_tracking','lqr_tracking','lqr_tracking','lqr_tracking'};
    con_mode   = {'centralized_cbf_qp','centralized_cbf_qp','centralized_cbf_qp','nominal_only'};
    cbf_type   = {'nominal_cbf','nominal_cbf','nominal_cbf','nominal_only'};
    pmetric    = {'no_priority','no_priority','no_priority','nominal_only'};
    pcca       = {false,false,false,false};
    classK_l0  = {10,10,10,5};
    ppower     = {2,2,2,5};
end

% Adds locations to path
root = '/Users/mblack/Documents/git/intersection_management/';
for aa = 1:nAgents
    set_path(root,dyn_mode,con_mode{aa},cost_mode);
    controller{aa} = str2func(con_mode{aa});
    nominal_u{aa}  = str2func(con_nom{aa});
end

% Initialize dynamics
dynamics   = str2func(dyn_mode);
run(strcat('dynamics/',dyn_mode,'/initial_conditions.m'))

% Load control params
for aa = 1:nAgents
    u_params{aa} = load(strcat('./controllers/',con_mode{aa},'/control_params.mat'));
end

% Structure for simulation settings -- retrievable during analysis
sim_settings = struct();
sim_settings.all_identical = all_identical;
sim_settings.dyn_mode = dyn_mode;
sim_settings.cost_mode = cost_mode;
sim_settings.input_bounds = input_bounds;
sim_settings.backup = backup;
sim_settings.im_used = im_used;
sim_settings.con_nom = con_nom;
sim_settings.con_mode = con_mode;
sim_settings.cbf_type = cbf_type;
sim_settings.pmetric = pmetric;
sim_settings.pcca = pcca;
sim_settings.classK_l0 = classK_l0;
sim_settings.ppower = ppower;
