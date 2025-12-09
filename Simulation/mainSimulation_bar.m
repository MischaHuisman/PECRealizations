clear all; close all; clc
FigureSettings
close all;

addpath('Functions\', 'RESULTS\', 'Simulation\')

%% System formulation
sys = [];
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


%% Base Controller Realization
sys.F = [-3.8390, -5.9290, -1.0982, 0.1157;
         -6.3202, -11.3658, -1.6005, -2.7251];
sys.Ac_F = sys.Ac + sys.F*sys.B*sys.Cc;
sys.Bc_F = sys.Bc - sys.Ac*sys.F + sys.F*sys.A + sys.F*sys.B*sys.Dc - sys.F*sys.B*sys.Cc*sys.F;
sys.Cc_F = sys.Cc;
sys.Dc_F = sys.Dc - sys.Cc*sys.F;


%% Simulation

% init plant
simu.u0 = [0;0];     % u = vi - v0
simu.x0 = [-0.1; 0; 0; 0];%+0.1*rand(4,1); % x = hi - h0!
simu.x0_bar = sys.C*simu.x0;

% init controller
simu.rho_0 = [0;0];
simu.rhoF_0 = simu.rho_0 + sys.F*simu.x0_bar; % The eference is not defined here.. should be (x0 - r)?

% Observer
simu.x0_obs = simu.x0; % + [sys.h1_0; sys.h3_0; sys.h2_0; sys.h4_0];

% simulation settings
simu.T_end = 200;
simu.dt = 0.01;
simu.startAtt = 125;
simu.noisePower_input = 0.003^2*simu.dt; 
simu.noisePower_output = 0.001^2*simu.dt;%0.005^2*simu.dt; %0.01^2*simu.dt; 
simu.attAmp = 0.1;
simu.attFreq = 0.25; 
simu.processNoiseOn = 0;
simu.sensorNoiseOn = 1;
simu.attOn = 1;

simout = sim('QuadruppelTank_BarDyn.slx', [0:simu.dt:simu.T_end]);

simu.u_base = simout.u_base_n + [sys.v1_0, sys.v2_0];
simu.y_base = simout.y_base_n + [sys.h1_0, sys.h3_0, sys.h2_0, sys.h4_0];
simu.u_real = simout.u_real_n + [sys.v1_0, sys.v2_0];
simu.y_real = simout.y_real_n + [sys.h1_0, sys.h3_0, sys.h2_0, sys.h4_0];

%% Figures
colorOutput= [{colorblind(6,:)}, {colorblind(8,:)}, {colorblind(1,:)}, {colorblind(2,:)}];
deviationFigure = .125;
automaticaFigures


% figure
% subplot(2,1,1); hold on; grid on
% plot(simout.tout, simu.y_base(:,1), colorOutput(1)); 
% plot(simout.tout, simu.y_base(:,3), colorOutput(2)); 
% xlabel('Time [s]'); ylabel('Output [V]');
% yline(sys.h1_0, 'k--', 'linewidth', 1.5); yline(sys.h2_0, 'k--', 'linewidth', 1.5)
% legend('$y_1$','$y_2$', 'reference', '' )
% 
% subplot(2,1,2); hold on; grid on
% plot(simout.tout, simu.y_base(:,2), colorOutput(3)); 
% plot(simout.tout, simu.y_base(:,4), colorOutput(4)); 
% yline(sys.h3_0, 'k--', 'linewidth', 1.5); yline(sys.h4_0, 'k--', 'linewidth', 1.5)
% xlabel('Time [s]'); ylabel('Output [V]');
% 
% legend('$y_3$', '$y_4$', 'reference', '' )
% sgtitle('Tank Waterlevels Base Controller')
% 
% figure
% subplot(2,1,1); hold on; grid on
% plot(simout.tout, simu.y_real(:,1), colorOutput(1)); 
% plot(simout.tout, simu.y_real(:,3), colorOutput(2)); 
% xlabel('Time [s]'); ylabel('Output [V]');
% yline(sys.h1_0, 'k--', 'linewidth', 1.5); yline(sys.h2_0, 'k--', 'linewidth', 1.5)
% legend('$y_1$','$y_2$', 'reference', '' )
% 
% subplot(2,1,2); hold on; grid on
% plot(simout.tout, simu.y_real(:,2), colorOutput(3)); 
% plot(simout.tout, simu.y_real(:,4), colorOutput(4)); 
% yline(sys.h3_0, 'k--', 'linewidth', 1.5); yline(sys.h4_0, 'k--', 'linewidth', 1.5)
% xlabel('Time [s]'); ylabel('Output [V]');
% 
% legend('$y_3$', '$y_4$', 'reference', '' )
% sgtitle('Tank Waterlevels Controller Realization')

% %%% Comparing the base vs realization output %%%
% figure; hold on; grid on
% plot(simout.tout, simu.u_base(:,1), colorOutput(1)); 
% plot(simout.tout, simu.u_base(:,2), colorOutput(2)); 
% plot(simout.tout, simu.u_real(:,1), colorOutput(3)); 
% plot(simout.tout, simu.u_real(:,2), colorOutput(4)); 
% xlabel('Time [s]'); ylabel('Input Voltage $u_i$ [V]');
% yline(sys.v1_0, 'k--', 'linewidth', 1.5); yline(sys.v2_0, 'k--', 'linewidth', 1.5)
% legend('$u_1$ base','$u_2$ base','$u_1$ realization','$u_2$ realization', 'reference', '' )

% %%% Subplots comparing the base vs realization %%%
% deviationFigure = .55;
% figure
% subplot(2,2,3); hold on; grid on
% plot(simout.tout, simu.y_base(:,1), "Color", colorOutput{1}); plot(simout.tout, simu.y_real(:,1), "Color", colorOutput{2}, 'linestyle', '--'); yline(sys.h1_0, 'k--', 'linewidth', 1.5);
% xlabel('Time [s]'); ylabel('Waterlevel $h_1$ [cm]'); ylim([sys.h1_0-deviationFigure, sys.h1_0+deviationFigure])
% subplot(2,2,1); hold on; grid on
% plot(simout.tout, simu.y_base(:,2), "Color", colorOutput{1}); plot(simout.tout, simu.y_real(:,2), "Color", colorOutput{2}, 'linestyle', '--'); yline(sys.h3_0, 'k--', 'linewidth', 1.5);
% xlabel('Time [s]'); ylabel('Waterlevel $h_3$ [cm]'); ylim([sys.h3_0-deviationFigure, sys.h3_0+deviationFigure])
% subplot(2,2,4); hold on; grid on
% plot(simout.tout, simu.y_base(:,3), "Color", colorOutput{1}); plot(simout.tout, simu.y_real(:,3), "Color", colorOutput{2}, 'linestyle', '--'); yline(sys.h2_0, 'k--', 'linewidth', 1.5);
% xlabel('Time [s]'); ylabel('Waterlevel $h_2$ [cm]'); ylim([sys.h2_0-deviationFigure, sys.h2_0+deviationFigure])
% subplot(2,2,2); hold on; grid on
% plot(simout.tout, simu.y_base(:,4), "Color", colorOutput{1}); plot(simout.tout, simu.y_real(:,4), "Color", colorOutput{2}, 'linestyle', '--'); yline(sys.h4_0, 'k--', 'linewidth', 1.5);
% xlabel('Time [s]'); ylabel('Waterlevel $h_4$ [cm]'); ylim([sys.h4_0-deviationFigure, sys.h4_0+deviationFigure])
% legend('Base Controller', 'Controller Realization', 'Reference')




