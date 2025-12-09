clear all; close all; clc
FigureSettings
close all;

addpath('Functions\', 'RESULTS\')

%% System formulation
sys.selectSystem = 3; % 1 = original system, 2 = same tanks for analysis
[A,B,C,D,G,H,sys] = Watertank_JohanssonV2(sys);

% Transformation to bar coordinates
sys.T   = C;
sys.A   = sys.T*A*inv(sys.T);       % sys.A = Abar;
sys.B   = sys.T*B;                  % sys.B = Bbar;
sys.C   = C*inv(sys.T);             % sys.C = Cbar;
sys.D   = D;                        % sys.D = Dbar;
sys.G   = sys.T*G;                  % sys.G = Gbar;
sys.H   = H;                        % sys.H = Hbar;

% Detector Scheme
sys.eigALC  = [-2, -2.1, -2, -2.1];             % Randomly chosen

sys.L  = place(sys.A, sys.C, sys.eigALC);    % Observer gain matrix L
sys.Ae = sys.A - sys.L*sys.C;

% Define system dimensions
sys.nx = size(sys.A, 1); % State dimension
sys.nw = size(sys.G, 2); % Disturbance dimension, chosen same as input
sys.ny = size(sys.C, 1); % Sensor dimension
sys.nr = sys.ny;
sys.nrho = size(sys.Ac,1);
sys.nzeta = sys.nx+sys.nrho;

% Define bounds
sys.bounds.w = 1e5*eye(sys.nw);
sys.bounds.v = 400*eye(sys.ny); %% From paper, variance approximately 0.1/2=0.05 --> Wv = 1/0.05^2
clear A B C D G H

%% Settings
selectSolver = 3;   % 1 = SEDUMI, 2 = SDPT3, 3 = MOSEK
% SolverSettingsV2 % SMOOTH RESULTS !!
SolverSettingsV3
settings.plotEllipse = 0;

%% Residual Set
if sys.selectSystem == 1 || sys.selectSystem == 2
    % Line search error
    sys.alpha_e_lower = 0.5;
    sys.alpha_e_upper = 2.5;
    sys.alpha_e_vals = linspace(sys.alpha_e_lower, sys.alpha_e_upper, 30);

    % Line search residual
    sys.alpha_r_lower = 0.2;
    sys.alpha_r_upper = 0.9;
    sys.alpha_r_vals = linspace(sys.alpha_r_lower, sys.alpha_r_upper, 30);

elseif sys.selectSystem == 3
    % Line search error
    sys.alpha_e_lower = 1.5;
    sys.alpha_e_upper = 2.5;
    sys.alpha_e_vals = linspace(sys.alpha_e_lower, sys.alpha_e_upper, 30);

    % Line search residual
    sys.alpha_r_lower = 0.3;
    sys.alpha_r_upper = 0.7;
    sys.alpha_r_vals = linspace(sys.alpha_r_lower, sys.alpha_r_upper, 30);
end

%% Split system into 2 subsystems

% subsystem 1 (tank 1 and 3)
sys_1.A             = sys.A(1:2,1:2);
sys_1.Ae            = sys.Ae(1:2,1:2);
sys_1.C             = sys.C(1:2,1:2);
sys_1.L             = sys.L(1:2,1:2);
sys_1.H             = sys.H(1:2,1:2);
sys_1.G             = sys.G(1:2,1:2);
sys_1.nx            = size(sys_1.A,1);
sys_1.nw            = size(sys_1.C,1);
sys_1.ny            = size(sys_1.C,1);
sys_1.nr            = sys_1.ny;
sys_1.bounds.w      = sys.bounds.w;
sys_1.bounds.v      = sys.bounds.v(1:2,1:2);
sys_1.alpha_r_vals  = sys.alpha_r_vals;
sys_1.alpha_e_vals  = sys.alpha_e_vals;

% subsystem 2 (Tank 2 and 4)
sys_2.A             = sys.A(3:4,3:4);
sys_2.Ae            = sys.Ae(3:4,3:4);
sys_2.C             = sys.C(3:4,3:4);
sys_2.L             = sys.L(3:4,3:4);
sys_2.H             = sys.H(3:4,3:4);
sys_2.G             = sys.G(3:4,1:2);
sys_2.nx            = size(sys_2.A,1);
sys_2.nw            = size(sys_2.C,1);
sys_2.ny            = size(sys_2.C,1);
sys_2.nr            = sys_2.ny;
sys_2.bounds.w      = sys.bounds.w;
sys_2.bounds.v      = sys.bounds.v(3:4,3:4);
sys_2.alpha_r_vals  = sys.alpha_r_vals;
sys_2.alpha_e_vals  = sys.alpha_e_vals;

%% Solve Residual set
% Solve LMI
[allSolutionsResidual, bestSolResidual]     = calculateResidualSet(sys, settingsSolver, settings);
[allSolutionsResidual_1, bestSolResidual_1] = calculateResidualSet(sys_1, settingsSolver, settings);
[allSolutionsResidual_2, bestSolResidual_2] = calculateResidualSet(sys_2, settingsSolver, settings);

% Process data & figures
sys_1.Pi    = bestSolResidual_1.Pi;
sys_2.Pi    = bestSolResidual_2.Pi;
sys.Pi      = bestSolResidual.Pi;

% Figures
procesData_ResidualV2(sys_1, allSolutionsResidual_1, bestSolResidual_1, settings, 'subsystem 1')
procesData_ResidualV2(sys_2, allSolutionsResidual_2, bestSolResidual_2, settings, 'subsystem 2')

%% Sensor attack selection
% Select Attack
attRange = [1]; % 1 = y1, 2 = y4, 3 = {y1, y4}, 4 = {y2,y4}, 5 = {y1,y2,y3,y4}, 6 = {y3, y4}, 7 = {y3} 8 = {y1, y2}
for attSelected = attRange % 1 = y1, 2 = y4, 3 = {y1,y4}, 4 = {y2,y4}, 5 = {y1,y2,y3,y4}, 6 = {y3, y4}, 7 = {y3} 8 = {y1, y2}
    
    %%% select attack data %%%
    attackSelection_V2


    %%% Solve Attacked Detector Set
    % Detector System matrix attacked error dynamics
    sys.Ae_att      = sys.A - sys.L*(eye(sys.nx) - sys.Gamma*sys.GammaInv)*sys.C;
    sys_1.Ae_att    = sys_1.A - sys_1.L*(eye(sys_1.nx) - sys_1.Gamma*sys_1.GammaInv)*sys_1.C;
    sys_2.Ae_att    = sys_2.A - sys_2.L*(eye(sys_2.nx) - sys_2.Gamma*sys_2.GammaInv)*sys_2.C;

    % solve LMI
    [allSolutionsAttErr_1, bestSolAttErr_1] = calculateAttDetectorSetV2(sys_1, settingsSolver, settings);
    [allSolutionsAttErr_2, bestSolAttErr_2] = calculateAttDetectorSetV2(sys_2, settingsSolver, settings);
    % [allSolutionsAttErr, bestSolAttErr]     = calculateAttDetectorSetV2(sys, settingsSolver, settings);

    % process data & figures
    procesData_AttDetector_subsystem(allSolutionsAttErr_1, bestSolAttErr_1, settings, ['Subsystem 1, att: ', num2str(attSelected)])
    procesData_AttDetector_subsystem(allSolutionsAttErr_2, bestSolAttErr_2, settings, ['Subsystem 2, att: ', num2str(attSelected)])
    % procesData_AttDetector(allSolutionsAttErr, bestSolAttErr, settings)
    % sys.Pe_Att = bestSolAttErr.P_e;
    sys_1.Pe_Att = bestSolAttErr_1.P_e;
    sys_2.Pe_Att = bestSolAttErr_2.P_e;

    %%% Solve Controller Realization
    [allSolutionsZeta_opt, bestSolZeta_opt, Foptimal] = calculateControllerRealizationV3(sys, sys_1, sys_2, settingsSolver, settings);
    [bestSolZeta_opt_trace, bestSolZeta_opt_logdet] = processData_closedLoop(allSolutionsZeta_opt, bestSolZeta_opt, settings, ['Optimal Controller (optimization result). Att sel:', num2str(attSelected)]);

    %%% Solve reachable set for optimal controlcler realization %%%
    [allSolutionsZeta_new, bestSolZeta_new] = calculateReachableSet_CLV3(sys, sys_1, sys_2, Foptimal, settingsSolver, settings);
    [bestSolZeta_new_trace, bestSolZeta_new_logdet] = processData_closedLoop(allSolutionsZeta_new, bestSolZeta_new, settings, ['Optimal Controller (post-proccessed). Att sel:', num2str(attSelected)]);

    %%% Solve reachable set for base controlle %%%
    Fbase = zeros(2,4);
    [allSolutionsZeta_old, bestSolZeta_old] = calculateReachableSet_CLV3(sys, sys_1, sys_2, Fbase, settingsSolver, settings);
    [bestSolZeta_old_trace, bestSolZeta_old_logdet] = processData_closedLoop(allSolutionsZeta_old, bestSolZeta_old, settings, ['Base Controller (post-proccessed). Att sel:', num2str(attSelected)]);

    %%
    disp(['Attack Selected: ', num2str(attSelected)])
    disp(['Volume residual set Pi: ', num2str(-logdet(bestSolResidual.Pi))])
    disp(['Volume residual set Pi_1 + Pi_2: ', num2str(-logdet(bestSolResidual_1.Pi)-logdet(bestSolResidual_2.Pi))])
    % disp(['Volume residual set Att Pe: ', num2str(-logdet(bestSolAttErr.P_e))])
    disp(['Volume residual set Att Pe_1 + Att Pe_2: ', num2str(-logdet(bestSolAttErr_1.P_e)-logdet(bestSolAttErr_2.P_e))])
    disp(['Old trace: ', num2str(bestSolZeta_old_trace.traceY)])
    disp(['Old volume: ', num2str(bestSolZeta_old_logdet.logdetP)])
    disp(['New trace: ', num2str(bestSolZeta_new_trace.traceY)])
    disp(['New volume: ', num2str(bestSolZeta_new_logdet.logdetP)])
    disp('F optimal: ');disp(Foptimal)
    disp('sensor attack \Gamma: ');disp(sys.Gamma)
     
    
    % residual results
    results{attSelected}.attSelected     = attSelected;
    % results{attSelected}.logdetPi        = -logdet(bestSolResidual.Pi);
    results{attSelected}.logdetPi_1      = -logdet(bestSolResidual_1.Pi);
    results{attSelected}.logdetPi_2      = -logdet(bestSolResidual_2.Pi);

    % detector attacked resuls
    % results{attSelected}.logdetPeAtt     = -logdet(bestSolAttErr.P_e);
    results{attSelected}.logdetPeAtt_1   = -logdet(bestSolAttErr_1.P_e);
    results{attSelected}.logdetPeAtt_2   = -logdet(bestSolAttErr_2.P_e);
      
    % important results
    results{attSelected}.baseLogdetPzeta = bestSolZeta_old_logdet.logdetP;
    results{attSelected}.baseTraceYzeta  = bestSolZeta_old_trace.traceY;
    results{attSelected}.optLogdetPzeta  = bestSolZeta_new_logdet.logdetP;
    results{attSelected}.optTraceYzeta   = bestSolZeta_new_trace.traceY;
    results{attSelected}.Foptimal        = Foptimal;

    % all solutions post process
    results{attSelected}.allSolZeta_new  = allSolutionsZeta_new;
    results{attSelected}.bestSolZeta_new = bestSolZeta_new;
    results{attSelected}.allSolZeta_old  = allSolutionsZeta_old;
    results{attSelected}.bestSolZeta_old = bestSolZeta_old;

   
    % save systems 
    results{attSelected}.sys             = sys;
    results{attSelected}.sys_1           = sys_1;
    results{attSelected}.sys_2           = sys_2;


end