function [allSolutions, bestSol] = calculateResidualSet(sys, settingsSolver, settings)
% state information
C = sys.C;
G = sys.G;
H = sys.H;
A_e = sys.Ae;
L = sys.L;

% Dimensions
n_x = sys.nx;
n_w = sys.nw;
n_y = sys.ny;

% Bounds
W_omega = sys.bounds.w;
W_v = sys.bounds.v;

% Pre-allocate best solution
bestObj = inf;
bestSol = [];
allSolutions = [];

for alpha_e = sys.alpha_e_vals
    for alpha_r  = sys.alpha_r_vals
        % Define decision variables
        P_e = sdpvar(n_x, n_x, 'symmetric');
        Pi = sdpvar(n_y, n_y, 'symmetric');
        beta_e_omega = sdpvar(1);
        beta_e_v = sdpvar(1);
        beta_r_v = sdpvar(1);
        beta_r_w = sdpvar(1);

        % Construct matrices
        Me = [A_e.'*P_e + P_e*A_e,       P_e*G,          -P_e*L*H,           zeros(n_x,1);
            G.'*P_e,                     zeros(n_w),     zeros(n_w,n_y),     zeros(n_w,1);
            -(L*H).'*P_e,                zeros(n_y,n_w), zeros(n_y),         zeros(n_y,1);
            zeros(1,n_x + n_w + n_y),                                        0];

        Mr = [C.'*Pi*C,      zeros(n_x,n_w),     C.'*Pi*H,    zeros(n_y,1);
            zeros(n_w, n_x + n_w + n_y + 1);
            H.'*Pi*C,    zeros(n_y, n_w),        H.'*Pi*H,    zeros(n_y,1);
            zeros(1, n_x + n_w + n_y),                        -1];

        % Mr = [C'*Pi*C,                 C'*Pi*H,        zeros(n_y,1);
        %     H'*Pi*C,                H'*Pi*H,        zeros(n_y,1);
        %     zeros(1, n_x + n_y),                      -1];

        % matrices for error set
        Ne = blkdiag(P_e, zeros(n_w), zeros(n_y), -1);
        Sw = blkdiag(zeros(n_x), -W_omega, zeros(n_y), 1);
        Sv_e = blkdiag(zeros(n_x), zeros(n_w), -W_v, 1);
        LMI_e = -Me - alpha_e*Ne - beta_e_omega*Sw - beta_e_v*Sv_e;

        % matrices for residual set
        % Ne_r = blkdiag(P_e, zeros(n_w+n_y), -1);
        % Sv_r = blkdiag(zeros(n_x+n_w), -W_v, 1);
        LMI_r = -Mr + alpha_r*Ne - beta_r_v*Sv_e - beta_r_w*Sw;

        % Define constraints
        constraints = [P_e >= settings.epsilon_P*eye(n_x), Pi >= settings.epsilon_P*eye(n_y), ...
            beta_e_omega >= 0, beta_e_v >= 0, beta_r_v >= 0, beta_r_w >= 0, ...
            LMI_e >= settings.epsilon_SP*eye(size(LMI_e,1)), ...
            LMI_r >= settings.epsilon_SP*eye(size(LMI_r,1))];

        % Objective: maximize log det of volumes (minimize -logdet)
        obj = -logdet(P_e) - logdet(Pi);

        % Solve
        diagnostics = optimize(constraints, obj, settingsSolver);

        % Store current solution
        currentSol              = struct();
        currentSol.alpha_e      = alpha_e;
        currentSol.alpha_r      = alpha_r;
        currentSol.P_e          = value(P_e);
        currentSol.Pi           = value(Pi);
        currentSol.beta_e_omega = value(beta_e_omega);
        currentSol.beta_e_v     = value(beta_e_v);
        currentSol.beta_r_v     = value(beta_r_v);
        currentSol.LMI_e        = value(LMI_e);
        currentSol.LMI_r        = value(LMI_r);
        currentSol.settingsSolver = settingsSolver;
        currentSol.obj          = -logdet(value(P_e))-logdet(value(Pi));
        currentSol.problem      = diagnostics.problem;
        currentSol.SolverInfo   = diagnostics.info;

        % Evaluate constraints numerically
        LMI_e_val                = value(LMI_e);
        LMI_r_val                = value(LMI_r);
        P_e_val                 = value(P_e);
        Pi_val                  = value(Pi);

        currentSol.isLMI_e_OK    = all(eig(LMI_e_val)+eps >= 0);
        currentSol.isLMI_r_OK    = all(eig(LMI_r_val)+eps >= 0);
        currentSol.isPe_OK      = all(eig(P_e_val)  > 0);
        currentSol.isPi_OK      = all(eig(Pi_val)   > 0);

        currentSol.ConstraintsSatisfied = ((currentSol.problem == 4 || currentSol.problem == 0 ) && currentSol.isLMI_e_OK && currentSol.isPe_OK && currentSol.isLMI_r_OK && currentSol.isPi_OK) ;


        currentSol.min_eig_LMI_e     = min(eig(LMI_e_val));
        currentSol.cond_LMI_e        = cond(LMI_e_val);
        currentSol.min_eig_LMI_r     = min(eig(LMI_r_val));
        currentSol.cond_LMI_r        = cond(LMI_r_val);
        currentSol.min_eig_Pe       = min(eig(P_e_val));
        currentSol.cond_Pe          = cond(P_e_val);
        currentSol.min_eig_Pi       = min(eig(Pi_val));
        currentSol.cond_Pi          = cond(Pi_val);

        %% Find minimum with Lambda>0
        if extractBefore(currentSol.SolverInfo, " (") == settings.checkMessageSucces
            currentSol.succes = 1;

        elseif extractBefore(currentSol.SolverInfo, " (") == settings.checkMessageProblem
            if currentSol.isLMI_e_OK && currentSol.isLMI_r_OK && currentSol.isPe_OK && currentSol.isPi_OK
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

        % if isLMI_e_OK && isLMI_r_OK && isPe_OK && isLMI_r_OK &&
        if currentSol.ConstraintsSatisfied
            % if (currentSol.problem == 4 && currentSol.isLMI_e_OK && currentSol.isPe_OK) || currentSol.problem == 0
            % Update best
            if currentSol.obj < bestObj
                bestObj = currentSol.obj;
                bestSol = currentSol;
            end
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