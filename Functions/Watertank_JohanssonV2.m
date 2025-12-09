function [A, B, C, D, G, H, sys] = Watertank_JohanssonV2(sys)


% ORIGINAL FROM PAPTER
% Paramters
A1 = 28;  a1 = 0.071;       % [cm2]     cross-section of tank A, and cross section of outlet hole a
A2 = 32;  a2 = 0.057;       % [cm2]     cross-section of tank A, and cross section of outlet hole a
A3 = 28;  a3 = 0.071;       % [cm2]     cross-section of tank A, and cross section of outlet hole a
A4 = 32;  a4 = 0.057;       % [cm2]     cross-section of tank A, and cross section of outlet hole a
kc = 0.50;                  % [cm/s^2]
g = 918;                    % [cm/s^2]

% Equilibrium points
h1_0 = 12.4;        % [cm]  Water level
h2_0 = 12.7;        % [cm]  Water level
h3_0 = 1.8;         % [cm]  Water level
h4_0 = 1.4;         % [cm]  Water level
v1_0 = 3;           % [V]   pump voltage
v2_0 = 3;           % [V]   pump voltage
k1 = 3.33;          % [cm^3/Vs]
k2 = 3.35;          % [cm^3/Vs]
gamma1 = 0.7;       % [-]
gamma2 = 0.6;       % [-]

sys.h1_0 = 12.4;        % [cm]  Water level
sys.h2_0 = 12.7;        % [cm]  Water level
sys.h3_0 = 1.8;         % [cm]  Water level
sys.h4_0 = 1.4;         % [cm]  Water level
sys.v1_0 = 3;           % [V]   pump voltage
sys.v2_0 = 3;           % [V]   pump voltage

%% PI Controller Original
% yr = 0; % NOTICE !!!
sys.K1 = 3;
sys.K2 = 2.7;
sys.TI1 = 30;
sys.TI2 = 40;
sys.Ac = zeros(2);

sys.Bc = -[1, 0, 0, 0; ...
           0, 0, 1, 0];
sys.Cc = diag([sys.K1/sys.TI1, sys.K2/sys.TI2]);
sys.Dc = -[sys.K1, 0, 0, 0; ...
           0, 0, sys.K2, 0];

%% Time constants
T1 = A1/a1 * sqrt(2*h1_0/g);    % [s]
T2 = A2/a2 * sqrt(2*h2_0/g);    % [s]
T3 = A3/a3 * sqrt(2*h3_0/g);    % [s]
T4 = A4/a4 * sqrt(2*h4_0/g);    % [s]

%% Dynamics
% x = hi - hi_0, u = vi - vi_0
%%% System dynamics are switched x = [x1 x3 x2 x4]
A = [-1/T1,    A3/(A1*T3),     0                0;
0,             -1/T3,          0,               0;
0,             0,              -1/T2,           A4/(A2*T4);
0,             0,              0,               -1/T4];

B = [gamma1*k1/A1,          0;
0,                     (1-gamma2)*k2/A3;
0,                     gamma2*k2/A2;
(1-gamma1)*k1/A4,      0];

% T = [1, 0, 0, 0;
%      0, 0, 1, 0;
%      0, 1, 0, 0;
%      0, 0, 0, 1]; % xtilde = [x1; x3, x2; x4] = Tx

% A = T*A*inv(T);
% B = T*B;

C = diag([kc,kc,kc,kc]); % x

D = zeros(4,2);

% rank(obsv(A,C));

G = B;
H = eye(size(C,1));






end