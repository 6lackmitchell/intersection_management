% Define Controller
controller         = @clf_cbf_qp;
controller         = @centralized_cbf_qp;
controller         = @distributed_cbf_qp;
nominal_controller = @hl_fxt_clf_qp;

% Define Cost Functional
cost               = @min_diff_nominal;

% Decentralized Controller
    if con_mode == "distributed"
        
        for aa = 1:nAgents

            % Configure agent variables
            xx          = squeeze(x(ii,aa,:))';
            xo          = squeeze(x(ii,idx(idx ~= aa),:));
            xg          = xGoal(aa,:);
            
            % Get single integrator dynamics
            [drift,f,g] = dynamics(sint_dyn,t,xx,[0]);
            s_dyn         = struct('fg',{f,g});
            [drift,f,g] = dynamics(dint_dyn,t,xx,[0]);
            d_dyn         = struct('fg',{f,g});

            % Compute Nominal and Safe Control Inputs
            u_nom(ii,aa,:)          = nominal_controller(t,xx,xo,xg,cost,s_dyn);        
            [u(ii,aa,:),v(ii,aa,:)] = controller(t,xx,xo,squeeze(u_nom(ii,aa,:)),cost,d_dyn);

            % Log Safety and Performance
            safety(ii,aa,:)         = B(t,xx,xo);
            performance(ii,aa,:)    = V(t,xx-xg);

        end
    
    % Centralized Controller
    elseif con_mode == "centralized"
        % Configure arguments to controller functions
        xx          = squeeze(x(ii,:,:));
        xx_flat     = reshape(squeeze(x(ii,:,:)).',1,[]);
        xg          = reshape(xGoal.',1,[]);
        [drift,f,g] = dynamics(dyn_mode,t,xx,[0]);
        dyn         = struct('fg',{f,g});
        
        % Compute nominal and safe control inputs
        u_nom_flat            = nominal_controller(t,xx_flat,xg,cost,dyn);
        u_nom(ii,:,:)         = permute(reshape(u_nom_flat,[1 nControls nAgents]),[1 3 2]);
        [u_flat,v_flat]       = controller(t,xx,u_nom_flat,cost,dyn);
        u(ii,:,:)             = permute(reshape(u_flat,[1 nControls nAgents]),[1 3 2]);
        v(ii,:,:)             = permute(reshape(v_flat,[1 nCBFs nAgents]),[1 3 2]);
        
        % Log Performance and Safety Constraints
        performance(ii,1,:)   = V(t,xx_flat-xg);

   
    end