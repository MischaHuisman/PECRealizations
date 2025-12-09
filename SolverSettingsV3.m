%% Solver settings
if selectSolver == 1 % SEDUMI
    settingsSolver = sdpsettings('solver', 'sedumi', 'sedumi.eps', 1e-8, ...
        'sedumi.cg.qprec', 1, 'sedumi.cg.maxiter', 49, ...
        'sedumi.stepdif', 2, 'verbose', 0);
    settings.checkMessageSucces = "Successfully solved";
    settings.checkMessageProblem = "Numerical problems";
    settings.epsilon_P   = 1e-8;    % small epsilon to enforce positive definite bound (due to only >= being allowed in constraint)
    settings.epsilon_SP  = 1e-10;   % small epsilon for semi-positive definite bound (prevent numerical errors)

elseif selectSolver == 2 % SDPT3
    settingsSolver = sdpsettings('solver', 'sdpt3', 'sdpt3.gaptol', 1e-8, 'verbose', 0, ...
        'sdpt3.inftol', 1e-8, 'sdpt3.steptol', 1e-8);
    settings.checkMessageSucces = "Successfully solved";
    settings.checkMessageProblem = "Numerical problems";
    settings.epsilon_P   = 1e-8;    % small epsilon to enforce positive definite bound (due to only >= being allowed in constraint)
    settings.epsilon_SP  = 1e-10;   % small epsilon for semi-positive definite bound (prevent numerical errors)

elseif selectSolver == 3 % MOSEK
    settings.checkMessageSucces = "Successfully solved";
    settings.checkMessageProblem = "Numerical problems";
    settingsSolver = sdpsettings('solver', 'mosek', 'verbose', 0); %, ...
        % 'mosek.MSK_DPAR_INTPNT_TOL_REL_GAP', 1e-8, ...
        % 'mosek.MSK_DPAR_INTPNT_TOL_PFEAS', 1e-7, ...
        % 'mosek.MSK_DPAR_INTPNT_TOL_DFEAS', 1e-7);
    settings.epsilon_P   = 1e-6;    % small epsilon to enforce positive definite bound (due to only >= being allowed in constraint)
    settings.epsilon_SP  = 1e-8;   % small epsilon for semi-positive definite bound (prevent numerical errors)
    settings.LMIreduced_epsilon = 1e-8;

end
