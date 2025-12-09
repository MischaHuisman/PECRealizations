%%% Subplots comparing the system output %%%

figure
%% Subplot 1 %%
subplot(2,2,1); hold on; grid on
plot(simout.tout, simu.y_base(:,2), "Color", colorOutput{2}); plot(simout.tout, simu.y_real(:,2), "Color", colorOutput{1}, 'linestyle', ':'); yline(sys.h3_0, 'k--', 'linewidth', 1.5);
xlabel('time [s]'); ylabel('waterlevel $h_3$ [cm]'); ylim([sys.h3_0-deviationFigure, sys.h3_0+deviationFigure])

% Define zoom region
x_zoom = [160 200];      % region you want to zoom in
y_zoom = [1.79 1.81];     % y-limits for zoomed area

% Highlight region on main plot
hold on;
rect = rectangle('Position',[x_zoom(1), y_zoom(1), diff(x_zoom), diff(y_zoom)], 'LineWidth',1.2);

% Create inset axes
inset = axes('Position',[0.2 0.815 0.15 0.1]); % [left bottom width height] in normalized units
box on; % box around inset

% Plot same data on inset
plot(simout.tout, simu.y_base(:,2), "Color", colorOutput{2}); hold on; 
plot(simout.tout, simu.y_real(:,2), "Color", colorOutput{1}, 'linestyle', ':'); yline(sys.h3_0, 'k--', 'linewidth', 1.5);
xlim(x_zoom); ylim(y_zoom); grid on;


%% Subplot 2 %%
subplot(2,2,2); hold on; grid on
plot(simout.tout, simu.y_base(:,4), "Color", colorOutput{2}); plot(simout.tout, simu.y_real(:,4), "Color", colorOutput{1}, 'linestyle', ':'); yline(sys.h4_0, 'k--', 'linewidth', 1.5);
xlabel('time [s]'); ylabel('waterlevel $h_4$ [cm]'); ylim([sys.h4_0-deviationFigure, sys.h4_0+deviationFigure])
legend('Base Controller', 'Controller Realization', 'Reference', 'location', 'southwest', 'fontsize', FontSizeLegend)

% Define zoom region
x_zoom = [160 200];      % region you want to zoom in
y_zoom = [1.36 1.44];     % y-limits for zoomed area

% Highlight region on main plot
hold on;
rect = rectangle('Position',[x_zoom(1), y_zoom(1), diff(x_zoom), diff(y_zoom)], 'LineWidth',1.2);

% Create inset axes
inset = axes('Position',[0.64 0.815 0.15 0.1]); % [left bottom width height] in normalized units
box on; % box around inset

% Plot same data on inset
plot(simout.tout, simu.y_base(:,4), "Color", colorOutput{2}); hold on; 
plot(simout.tout, simu.y_real(:,4), "Color", colorOutput{1}, 'linestyle', ':'); yline(sys.h4_0, 'k--', 'linewidth', 1.5);
xlim(x_zoom); ylim(y_zoom); grid on;

%% Subplot 3 %%
subplot(2,2,3); hold on; grid on
plot(simout.tout, simu.y_base(:,1), "Color", colorOutput{2}); plot(simout.tout, simu.y_real(:,1), "Color", colorOutput{1}, 'linestyle', ':'); yline(sys.h1_0, 'k--', 'linewidth', 1.5);
xlabel('time [s]'); ylabel('waterlevel $h_1$ [cm]'); ylim([sys.h1_0-deviationFigure, sys.h1_0+deviationFigure])
% Define zoom region
x_zoom = [160 200];      % region you want to zoom in
y_zoom = [12.34 12.46];     % y-limits for zoomed area

% Highlight region on main plot
hold on;
rect = rectangle('Position',[x_zoom(1), y_zoom(1), diff(x_zoom), diff(y_zoom)], 'LineWidth',1.2);

% Create inset axes
inset = axes('Position',[0.2 0.34 0.15 0.1]); % [left bottom width height] in normalized units
box on; % box around inset

% Plot same data on inset
plot(simout.tout, simu.y_base(:,1), "Color", colorOutput{2}); hold on; 
plot(simout.tout, simu.y_real(:,1), "Color", colorOutput{1}, 'linestyle', ':'); yline(sys.h1_0, 'k--', 'linewidth', 1.5);
xlim(x_zoom); ylim(y_zoom); grid on;

%% Subplot 4 %%
subplot(2,2,4); hold on; grid on
plot(simout.tout, simu.y_base(:,3), "Color", colorOutput{2}); plot(simout.tout, simu.y_real(:,3), "Color", colorOutput{1}, 'linestyle', ':'); yline(sys.h2_0, 'k--', 'linewidth', 1.5);
xlabel('time [s]'); ylabel('waterlevel $h_2$ [cm]'); ylim([sys.h2_0-deviationFigure, sys.h2_0+deviationFigure])

% Define zoom region
x_zoom = [160 200];      % region you want to zoom in
y_zoom = [12.69 12.71];     % y-limits for zoomed area

% Highlight region on main plot
hold on;
rect = rectangle('Position',[x_zoom(1), y_zoom(1), diff(x_zoom), diff(y_zoom)], 'LineWidth',1.2);

% Create inset axes
inset = axes('Position',[0.64 0.34 0.15 0.1]); % [left bottom width height] in normalized units
box on; % box around inset

% Plot same data on inset
plot(simout.tout, simu.y_base(:,3), "Color", colorOutput{2}); hold on; 
plot(simout.tout, simu.y_real(:,3), "Color", colorOutput{1}, 'linestyle', ':', 'linestyle', ':'); 
yline(sys.h2_0, 'k--', 'linewidth', 1.5);
xlim(x_zoom); ylim(y_zoom); grid on;


%% Comparing the base vs realization output %%
figure; hold on; grid on
plot(simout.tout, simu.u_base(:,1),"Color", colorOutput{4}); 
plot(simout.tout, simu.u_base(:,2),"Color", colorOutput{2}); 
plot(simout.tout, simu.u_real(:,1),"Color", colorOutput{3}); 
plot(simout.tout, simu.u_real(:,2),"Color", colorOutput{1}); 
xlabel('time [s]', 'fontsize', FontSizeAxes); ylabel('input $u_j$ [V]', 'fontsize', FontSizeAxes);
yline(sys.v1_0, 'k--', 'linewidth', 1.5); yline(sys.v2_0, 'k--', 'linewidth', 1.5)
legend('$u_1 + v_1^0$ (base)','$u_2+v_2^0$ (base)','$u^*_1+ v_1^0$ (optimal)','$u^*_2+ v_2^0$ (optimal)', '$v_1^0,v_2^0$', '' , 'fontsize', FontSizeLegend, 'location', 'southwest')

% Define zoom region
x_zoom = [160 200];      % region you want to zoom in
y_zoom = [2.9 3.1];     % y-limits for zoomed area

% Highlight region on main plot
hold on;
rect = rectangle('Position',[x_zoom(1), y_zoom(1), diff(x_zoom), diff(y_zoom)], 'LineWidth',1.2);

% Create inset axes
inset = axes('Position',[0.25 0.55 0.3 0.3]); % [left bottom width height] in normalized units
box on; % box around inset

% Plot same data on inset
plot(simout.tout, simu.u_base(:,2), "Color", colorOutput{2}); hold on;
plot(simout.tout, simu.u_real(:,2), "Color", colorOutput{1}, 'linestyle', ':'); 
yline(sys.v1_0, 'k--', 'linewidth', 1.5); yline(sys.v2_0, 'k--', 'linewidth', 1.5)
xlim(x_zoom); ylim(y_zoom); grid on;


%% 
open('RESULTS/Figures/Simulation_atty1_controller.fig')
print(gcf, '-depsc', '-painters', '-r600', 'RESULTS/Figures/simulationAtty1_controller.eps');

open('RESULTS/Figures/Simulation_atty1_tanks.fig')
print(gcf, '-depsc', '-painters', '-r600', 'RESULTS/Figures/simulationAtty1_tanks.eps');