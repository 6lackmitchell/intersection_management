function [sol,fval,exitflag] = solve_quadratic_program(Q,p,A,b,Aeq,beq,LB,UB)
%UNTITLED Summary of this function goes here
% Enforce strict inequalities
strict_tol = 1e-4;
b          = b  - strict_tol;
LB         = LB + strict_tol;
UB         = UB - strict_tol;

%   Detailed explanation goes here
model.modelsense = 'min';
model.sense      = [repmat('<',length(b),1); repmat('=',length(beq),1)];   % Specifies that linear constraint is Ax < b
model.obj        = p;
model.Q          = sparse(Q);
model.A          = sparse([A; Aeq]);
model.rhs        = [b; beq];
model.lb         = LB;
model.ub         = UB;

params.OutputFlag     =  0;
params.BarHomogeneous = -1;
params.NumericFocus   =  0;

% Solve optimization problem
result = gurobi(model, params);

% Extract solution info
sol    = result.x;
fval   = result.objval;
status = result.status;

if strcmp(status, 'OPTIMAL')
    exitflag = 2;
else
    gurobi_write(model,'datastore/diagnostic.lp');
    fprintf('Optimization returned status: %s\n', result.status);
    if strcmp(status, 'LOADED')
        exitflag = 1;
    elseif strcmp(status, 'INFEASIBLE')
        exitflag = 3;
    elseif strcmp(status, 'INF_OR_UNBD')
        exitflag = 4;
    elseif strcmp(status, 'UNBOUNDED')
        exitflag = 5;
    elseif strcmp(status, 'CUTOFF')
        exitflag = 6;
    elseif strcmp(status, 'ITERATION_LIMIT')
        exitflag = 7;
    elseif strcmp(status, 'NODE_LIMIT')
        exitflag = 8;
    elseif strcmp(status, 'TIME_LIMIT')
        exitflag = 9;
    elseif strcmp(status, 'SOLUTION_LIMIT')
        exitflag = 10;
    elseif strcmp(status, 'INTERRUPTED')
        exitflag = 11;
    elseif strcmp(status, 'NUMERIC')
        exitflag = 12;
    elseif strcmp(status, 'SUBOPTIMAL')
        exitflag = 13;
    elseif strcmp(status, 'INPROGRESS')
        exitflag = 14;
    elseif strcmp(status, 'USER_OBJ_LIMIT')
        exitflag = 15;
    else
        exitflag = -999;
    end
end    

end

