function [allSolutions, bestSol] = calculateAttDetectorSetV2(sys, settingsSolver, settings)

% state information
G       = sys.G;
H       = sys.H;
Ae_Att  = sys.Ae_att;
L       = sys.L;
Gamma   = sys.Gamma;
GammaInv = sys.GammaInv;

% Dimensions
n_x = sys.nx;
n_w = sys.nw;
n_y = sys.ny;
n_r = sys.nr;

% Bounds
W_omega = sys.bounds.w;
W_v     = sys.bounds.v;
Pi      = sys.Pi;

% Pre-allocate best solution
bestObj = inf;
bestSol = [];
allSolutions = [];

for alpha_eAtt = sys.alpha_eAtt_vals

    % Define decision variables
    P_eAtt = sdpvar(n_x, n_x, 'symmetric');
    beta_eAtt_omega = sdpvar(1);
    beta_eAtt_v = sdpvar(1);
    beta_eAtt_r = sdpvar(1);


    % === LMI CONSTRUCTION ===

    % Matrix blocks for \{M}_e
    M11 = Ae_Att.' * P_eAtt + P_eAtt * Ae_Att;
    M12 = G.' * P_eAtt;
    M13 = -(L * (eye(size(n_y)) - Gamma * GammaInv) * H).' * P_eAtt;
    M14 = -(L * Gamma * GammaInv).' * P_eAtt;
    M15 = zeros(1, n_x); % scalar part of LMI

    % Symmetric block matrix
    Me = [ M11, M12.', M13.', M14.', M15';
        M12,    zeros(n_w),                 zeros(n_w,n_y+n_r+1);
        M13,    zeros(n_y,n_w),             zeros(n_y),             zeros(n_y,n_r+1);
        M14,    zeros(n_r,n_w+n_y),         zeros(n_r),             zeros(n_r,1);
        M15,    zeros(1,n_w+n_y+n_r),                               0 ];

    % \{N}_e block diagonal
    Ne = blkdiag(P_eAtt, zeros(n_w+n_y+n_r), -1);

    % S matrices
    S_omega = blkdiag(zeros(n_x), -W_omega, zeros(n_y+n_r), 1);
    S_v     = blkdiag(zeros(n_x+n_w), -W_v, zeros(n_r), 1);
    S_r     = blkdiag(zeros(n_x+n_w+n_y), -Pi, 1);

    % Final LMI constraint
    LMI_eAtt = -Me - alpha_eAtt * Ne - beta_eAtt_omega * S_omega - beta_eAtt_v * S_v - beta_eAtt_r * S_r;

    % Check zero rows and colums --> Feasibility
    % Given LMI matrix M (symmetric)
    threshold = settings.LMIreduced_epsilon;  % numerical zero tolerance
    zero_rows = all(abs(value(Me)) <= threshold, 2);   % logical vector: true if row is (almost) zero
    zero_cols = all(abs(value(Me)) <= threshold, 1).';  % same for columns

    % Only keep indices where both row and column are (near-)zero
    zero_diag = zero_rows & zero_cols;
    zero_diag(end) = 0;     % the last entry of M is always zero --> corresponding to the 1
    
    % Optional: indices to keep
    keep_idx = find(~zero_diag);

    % Reduced LMI
    % LMI_eAtt_reduced = LMI_eAtt(keep_idx, keep_idx);
    LMI_eAtt_reduced = LMI_eAtt;

    % === OPTIMIZATION PROBLEM ===
    Constraints = [P_eAtt >= settings.epsilon_P*eye(n_x), ...
        beta_eAtt_omega >= 0, beta_eAtt_v >= 0, beta_eAtt_r >= 0, ...
        LMI_eAtt_reduced >= settings.epsilon_SP*eye(size(LMI_eAtt_reduced,1))];

    obj = -logdet(P_eAtt);  % minimize volume
    % obj = trace(P_eAtt);  % minimize volume

    % Solve
    diagnostics = optimize(Constraints, obj, settingsSolver);

    % Store current solution
    currentSol                  = struct();
    currentSol.alpha_eAtt       = alpha_eAtt;
    currentSol.P_e              = value(P_eAtt);
    currentSol.beta_eAtt_omega  = value(beta_eAtt_omega);
    currentSol.beta_eAtt_v      = value(beta_eAtt_v);
    currentSol.beta_eAtt_r      = value(beta_eAtt_r);
    currentSol.LMI_eAtt         = value(LMI_eAtt);
    currentSol.LMI_eAtt_reduced = value(LMI_eAtt_reduced);
    currentSol.settingsSolver   = settingsSolver;
    currentSol.obj              = -logdet(value(P_eAtt));
    currentSol.problem          = diagnostics.problem;
    currentSol.SolverInfo       = diagnostics.info;

    % Evaluate constraints numerically
    LMI_eAtt_val                = value(LMI_eAtt);
    P_eAtt_val                  = value(P_eAtt);

    currentSol.isLMI_eAtt_OK    = all(eig(LMI_eAtt_val)+eps >= 0);
    currentSol.isPe_OK          = all(eig(P_eAtt_val)  > 0);

    currentSol.ConstraintsSatisfied = ((currentSol.problem == 4 || currentSol.problem == 0 ) && currentSol.isLMI_eAtt_OK && currentSol.isPe_OK) ;


    currentSol.min_eig_LMI_eAtt = min(eig(LMI_eAtt_val));
    currentSol.cond_LMI_eAtt    = cond(LMI_eAtt_val);
    currentSol.min_eig_PeAtt    = min(eig(P_eAtt_val));
    currentSol.cond_PeAtt       = cond(P_eAtt_val);


    %% Find minimum with Lambda>0
    if extractBefore(currentSol.SolverInfo, " (") == settings.checkMessageSucces
        currentSol.succes = 1;

    elseif extractBefore(currentSol.SolverInfo, " (") == settings.checkMessageProblem
        if currentSol.isLMI_eAtt_OK && currentSol.isPe_OK
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
    disp('No feasible solution found within alpha_e range.');
else
    disp('Best feasible solution found:');
    disp(bestSol);
end

end