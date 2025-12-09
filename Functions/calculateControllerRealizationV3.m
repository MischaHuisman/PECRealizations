function [allSolutions, bestSol, Foptimal] = calculateControllerRealizationV3(sys, sys_1, sys_2, settingsSolver, settings)

% Define system dimensions
n_rho   = sys.nrho;     % Dimension of controller state
n_y     = sys.ny;       % Dimension of output y
n_zeta  = sys.nzeta;    % Dimension of zeta
n_omega = sys.nw;       % Dimension of omega
n_v     = sys.ny;       % Dimension of v
% n_e     = sys.nx;       % Dimension of error e
% n_r     = sys.ny;       % Dimension of residual r

% subsystems
n_e1    = sys_1.nx;
n_r1    = sys_1.nr;
n_e2    = sys_2.nx;
n_r2    = sys_2.nr;
Pe1     = sys_1.Pe_Att;
Pi1     = sys_1.Pi;
Pe2     = sys_2.Pe_Att;
Pi2     = sys_2.Pi;


% System Matrices
GammaInv    = sys.GammaInv;
H           = sys.H;

% Sets / bounds
W_omega     = sys.bounds.w;
W_v         = sys.bounds.v;

% Pre-allocate best solution
bestObj = inf;
bestSol = [];
allSolutions = [];

for alpha_zeta = sys.alpha_zeta_vals

    % Define decision variables
    Y_zeta     = sdpvar(n_zeta, n_zeta, 'symmetric');
    F          = sdpvar(n_rho, n_y, 'full');
    beta_omega = sdpvar(1);
    beta_v     = sdpvar(1);
    beta_e1    = sdpvar(1);
    beta_e2    = sdpvar(1);
    beta_r1    = sdpvar(1);
    beta_r2    = sdpvar(1);

    % Closed-loop system matrices
    Acl = [sys.A + sys.B * sys.Dc, sys.B * sys.Cc;
           sys.Bc,                 sys.Ac];

    Gcl = [sys.G;
           zeros(n_rho, n_omega)];

    Hcl = [sys.B * (sys.Dc - sys.Cc * F) * sys.H;
           (sys.Bc - sys.Ac * F + F * sys.A) * sys.H];

    Tcl = [sys.B * (sys.Dc - sys.Cc * F) * sys.Gamma;
           (sys.Bc - sys.Ac * F + F * sys.A) * sys.Gamma]*GammaInv;
    
    Tcl_1 = Tcl(:,1:2);
    Tcl_2 = Tcl(:,3:4);

    % Block matrix construction for the LMI
    M11 = Y_zeta * Acl' + Acl * Y_zeta;
    M12 = Gcl;
    M13 = Hcl - Tcl * H;
    M14 = -Tcl_1; % part affecting e1
    M15 = -Tcl_2; % part affecting e2
    M16 =  Tcl_1; % part affecting r1
    M17 =  Tcl_2; % part affecting r2
    M18 = zeros(n_zeta, 1);

    % Full symmetric LMI matrix
    M = [M11, M12, M13, M14, M15, M16, M17, M18;
         M12', zeros(n_omega), zeros(n_omega, n_v + n_e1 + n_e2 + n_r1 + n_r2), zeros(n_omega,1);
         M13', zeros(n_v, n_omega), zeros(n_v), zeros(n_v, n_e1 + n_e2 + n_r1 + n_r2), zeros(n_v,1);
         M14', zeros(n_e1, n_omega + n_v), zeros(n_e1), zeros(n_e1, n_e2 + n_r1 + n_r2), zeros(n_e1,1);
         M15', zeros(n_e2, n_omega + n_v + n_e1), zeros(n_e2), zeros(n_e2, n_r1 + n_r2), zeros(n_e2,1);
         M16', zeros(n_r1, n_omega + n_v + n_e1 + n_e2), zeros(n_r1), zeros(n_r1, n_r2), zeros(n_r1,1);
         M17', zeros(n_r2, n_omega + n_v + n_e1 + n_e2 + n_r1), zeros(n_r2), zeros(n_r2,1);
         M18', zeros(1, n_omega + n_v + n_e1 + n_e2 + n_r1 + n_r2), 0];

    % N matrix
    N = blkdiag(Y_zeta, zeros(n_omega + n_v + n_e1 + n_e2 + n_r1 + n_r2), -1);

    % S-procedure matrices
    S_omega = blkdiag(zeros(n_zeta), -W_omega, zeros(n_v + n_e1 + n_e2 + n_r1 + n_r2), 1);
    S_v     = blkdiag(zeros(n_zeta + n_omega), -W_v, zeros(n_e1 + n_e2 + n_r1 + n_r2), 1);
    S_e1    = blkdiag(zeros(n_zeta + n_omega + n_v), -Pe1, zeros(n_e2 + n_r1 + n_r2), 1);
    S_e2    = blkdiag(zeros(n_zeta + n_omega + n_v + n_e1), -Pe2, zeros(n_r1 + n_r2), 1);
    S_r1    = blkdiag(zeros(n_zeta + n_omega + n_v + n_e1 + n_e2), -Pi1, zeros(n_r2), 1);
    S_r2    = blkdiag(zeros(n_zeta + n_omega + n_v + n_e1 + n_e2 + n_r1), -Pi2, 1);

    % LMI
    LMI_zeta = -M ...
        - alpha_zeta * N ...
        - beta_omega * S_omega ...
        - beta_v * S_v ...
        - beta_e1 * S_e1 ...
        - beta_e2 * S_e2 ...
        - beta_r1 * S_r1 ...
        - beta_r2 * S_r2;

    % Optimization constraints
    Constraints = [Y_zeta >= settings.epsilon_P * eye(n_zeta),...
        beta_omega >= 0,...
        beta_v >= 0,...
        beta_e1 >= 0,...
        beta_e2 >= 0,...
        beta_r1 >= 0,...
        beta_r2 >= 0,...
        LMI_zeta >= settings.epsilon_SP * eye(size(LMI_zeta,1))];

    % Objective
    obj = trace(Y_zeta);

    % Solve
    diagnostics = optimize(Constraints, obj, settingsSolver);

    % Store current solution
    currentSol                  = struct();
    currentSol.alpha_zeta       = alpha_zeta;
    currentSol.Y_zeta           = value(Y_zeta);
    currentSol.P_zeta           = value(inv(value(Y_zeta)));
    currentSol.beta_zeta_omega  = value(beta_omega);
    currentSol.beta_zeta_v      = value(beta_v);
    currentSol.beta_zeta_e1     = value(beta_e1);
    currentSol.beta_zeta_e2     = value(beta_e2);
    currentSol.beta_zeta_r1     = value(beta_r1);
    currentSol.beta_zeta_r2     = value(beta_r2);
    currentSol.LMI_zeta         = value(LMI_zeta);
    currentSol.settingsSolver   = settingsSolver;
    currentSol.obj              = trace(value(Y_zeta));
    currentSol.logDetObj        = -logdet(currentSol.P_zeta);
    currentSol.problem          = diagnostics.problem;
    currentSol.SolverInfo       = diagnostics.info;
    currentSol.Foptimal         = value(F);

    % Evaluate constraints numerically
    LMI_zeta_val                = value(LMI_zeta);
    Y_zeta_val                  = value(Y_zeta);

    currentSol.isLMI_zeta_OK    = all(eig(LMI_zeta_val)+eps >= 0);
    currentSol.isY_OK          = all(eig(Y_zeta_val)  > 0);

    currentSol.ConstraintsSatisfied = ((currentSol.problem == 4 || currentSol.problem == 0 ) && currentSol.isLMI_zeta_OK && currentSol.isY_OK) ;


    currentSol.min_eig_LMI_zeta = min(eig(LMI_zeta_val));
    currentSol.cond_LMI_zeta    = cond(LMI_zeta_val);
    currentSol.min_eig_Yzeta    = min(eig(Y_zeta_val));
    currentSol.cond_Yzeta       = cond(Y_zeta_val);


    %% Find minimum with Lambda>0
    if extractBefore(currentSol.SolverInfo, " (") == settings.checkMessageSucces
        currentSol.succes = 1;

    elseif extractBefore(currentSol.SolverInfo, " (") == settings.checkMessageProblem
        if currentSol.isLMI_zeta_OK && currentSol.isY_OK
            currentSol.succes = 1;

        else
            currentSol.succes = 0;
        end

        % Always set succes = 0 if above conditions are not met
    else
        currentSol.succes = 0;
    end

    % Add to all solutions array
    allSolutions = [allSolutions; currentSol];

    % if isLMI_eAtt_OK isP_eAtt_OK
    if currentSol.succes
        % Update best
        if currentSol.obj < bestObj
            bestObj = currentSol.obj;
            bestSol = currentSol;
        end
    end
end

% Display best solution
if isempty(bestSol)
    disp('No feasible solution found within $\alpha_\zeta$ range.');
    Foptimal = [];
else
    disp('Best feasible solution found:');
    disp(bestSol);
    Foptimal = bestSol.Foptimal;
end

end