if attSelected == 1 %% y1
    % Attack selection matrices
    sys.Gamma       = eye(4,1);
    sys.GammaInv    = pinv(sys.Gamma);
   
    % allocate data subsystems
    sys_1.Gamma = sys.Gamma(1:2,:);
    sys_1.GammaInv = pinv(sys_1.Gamma);

    sys_2.Gamma = sys.Gamma(3:4,:);
    sys_2.GammaInv = pinv(sys_2.Gamma);

    % Line search attacked dector error
    alpha_eAtt_lower    = 0.01;
    alpha_eAtt_upper    = 0.05;
    sys.alpha_eAtt_vals = linspace(alpha_eAtt_lower, alpha_eAtt_upper, 300);

    % subsystems
    sys_1.alpha_eAtt_vals = linspace(0.01, 0.1, 150);
    sys_2.alpha_eAtt_vals = linspace(0.5, 2, 150);

    % Line search controller realization
    alpha_zeta_lower    = 0.01;
    alpha_zeta_upper    = 0.05;
    sys.alpha_zeta_vals = linspace(alpha_zeta_lower, alpha_zeta_upper, 200);


elseif attSelected == 2 %% y4
    % Attack selection matrices
    sys.Gamma       = zeros(4,1); sys.Gamma(4,1) = 1;
    sys.GammaInv    = pinv(sys.Gamma);
    
    % allocate data subsystems
    sys_1.Gamma = sys.Gamma(1:2,:);
    sys_1.GammaInv = pinv(sys_1.Gamma);

    sys_2.Gamma = sys.Gamma(3:4,:);
    sys_2.GammaInv = pinv(sys_2.Gamma);

    % Line search attacked dector error
    alpha_eAtt_lower    = 0.01;
    alpha_eAtt_upper    = 0.05;
    sys.alpha_eAtt_vals = linspace(alpha_eAtt_lower, alpha_eAtt_upper, 300);

    % subsystems
    sys_1.alpha_eAtt_vals = linspace(1.2, 1.6, 150);
    sys_2.alpha_eAtt_vals = linspace(0.01,0.1, 150);

    % Line search controller realization
    alpha_zeta_lower    = 0.015;
    alpha_zeta_upper    = 0.04;
    sys.alpha_zeta_vals = linspace(alpha_zeta_lower, alpha_zeta_upper, 200);

elseif attSelected == 3 %% y1 & y4
    % Attack selection matrices
    sys.Gamma       = zeros(4,2); sys.Gamma(1,1) = 1; sys.Gamma(4,2) = 1;
    sys.GammaInv    = pinv(sys.Gamma);

    % allocate data subsystems
    sys_1.Gamma = sys.Gamma(1:2,:);
    sys_1.GammaInv = pinv(sys_1.Gamma);

    sys_2.Gamma = sys.Gamma(3:4,:);
    sys_2.GammaInv = pinv(sys_2.Gamma);

    % Line search attacked dector error
    alpha_eAtt_lower    = 0.01;
    alpha_eAtt_upper    = 0.05;
    sys.alpha_eAtt_vals = linspace(alpha_eAtt_lower, alpha_eAtt_upper, 300);

    % subsystems
    sys_1.alpha_eAtt_vals = linspace(0.01, 0.1, 150);
    sys_2.alpha_eAtt_vals = linspace(0.01, 0.1, 150);

    % Line search controller realization
    alpha_zeta_lower    = 0.01;
    alpha_zeta_upper    = 0.05;
    sys.alpha_zeta_vals = linspace(alpha_zeta_lower, alpha_zeta_upper, 200);

elseif attSelected == 4 %% y2, y4
    % Attack selection matrices
    sys.Gamma       = zeros(4,2); sys.Gamma(3,1) = 1; sys.Gamma(4,2) = 1;
    sys.GammaInv    = pinv(sys.Gamma);

    % allocate data subsystems
    sys_1.Gamma = sys.Gamma(1:2,:);
    sys_1.GammaInv = pinv(sys_1.Gamma);

    sys_2.Gamma = sys.Gamma(3:4,:);
    sys_2.GammaInv = pinv(sys_2.Gamma);

    % Line search attacked dector error
    alpha_eAtt_lower    = 0.01;
    alpha_eAtt_upper    = 0.05;
    sys.alpha_eAtt_vals = linspace(alpha_eAtt_lower, alpha_eAtt_upper, 300);

    % subsystems
    sys_1.alpha_eAtt_vals = linspace(1.2, 1.6, 150);
    sys_2.alpha_eAtt_vals = linspace(0.01, 0.1, 150);

    % Line search controller realization
    alpha_zeta_lower    = 0.01;
    alpha_zeta_upper    = 0.05;
    sys.alpha_zeta_vals = linspace(alpha_zeta_lower, alpha_zeta_upper, 200);

elseif attSelected == 5 %% y1, y2, y3, & y4
    % Attack selection matrices
    sys.Gamma       = eye(4);
    sys.GammaInv    = pinv(sys.Gamma);

    % allocate data subsystems
    sys_1.Gamma = sys.Gamma(1:2,:);
    sys_1.GammaInv = pinv(sys_1.Gamma);

    sys_2.Gamma = sys.Gamma(3:4,:);
    sys_2.GammaInv = pinv(sys_2.Gamma);

    % Line search attacked dector error
    alpha_eAtt_lower    = 0.01;
    alpha_eAtt_upper    = 0.05;
    sys.alpha_eAtt_vals = linspace(alpha_eAtt_lower, alpha_eAtt_upper, 300);

    % subsystems
    sys_1.alpha_eAtt_vals = linspace(0.01, 0.1, 150);
    sys_2.alpha_eAtt_vals = linspace(0.01, 0.1, 150);

    % Line search controller realization
    alpha_zeta_lower    = 0.01;
    alpha_zeta_upper    = 0.05;
    sys.alpha_zeta_vals = linspace(alpha_zeta_lower, alpha_zeta_upper, 200);

elseif attSelected == 6 %% y3 & y4
    % Attack selection matrices
    sys.Gamma       = zeros(4,2); sys.Gamma(2,1) = 1; sys.Gamma(4,2) = 1;
    sys.GammaInv    = pinv(sys.Gamma);

    % allocate data subsystems
    sys_1.Gamma = sys.Gamma(1:2,:);
    sys_1.GammaInv = pinv(sys_1.Gamma);

    sys_2.Gamma = sys.Gamma(3:4,:);
    sys_2.GammaInv = pinv(sys_2.Gamma);

    % Line search attacked dector error
    alpha_eAtt_lower    = 0.01;
    alpha_eAtt_upper    = 0.05;
    sys.alpha_eAtt_vals = linspace(alpha_eAtt_lower, alpha_eAtt_upper, 300);

    % subsystems
    sys_1.alpha_eAtt_vals = linspace(0.01, 0.1, 150);
    sys_2.alpha_eAtt_vals = linspace(0.01, 0.1, 150);

    % Line search controller realization
    alpha_zeta_lower    = 0.01;
    alpha_zeta_upper    = 0.05;
    sys.alpha_zeta_vals = linspace(alpha_zeta_lower, alpha_zeta_upper, 200);
   

elseif attSelected == 7 %% y3
    % Attack selection matrices
    sys.Gamma       = zeros(4,1); sys.Gamma(2,1) = 1;
    sys.GammaInv    = pinv(sys.Gamma);

    % allocate data subsystems
    sys_1.Gamma = sys.Gamma(1:2,:);
    sys_1.GammaInv = pinv(sys_1.Gamma);

    sys_2.Gamma = sys.Gamma(3:4,:);
    sys_2.GammaInv = pinv(sys_2.Gamma);

    % Line search attacked dector error
    alpha_eAtt_lower    = 0.01;
    alpha_eAtt_upper    = 0.05;
    sys.alpha_eAtt_vals = linspace(alpha_eAtt_lower, alpha_eAtt_upper, 300);

    % subsystems
    sys_1.alpha_eAtt_vals = linspace(0.01, 0.1, 150);
    sys_2.alpha_eAtt_vals = linspace(1.2, 1.6, 150);

    % Line search controller realization
    alpha_zeta_lower    = 0.01;
    alpha_zeta_upper    = 0.05;
    sys.alpha_zeta_vals = linspace(alpha_zeta_lower, alpha_zeta_upper, 200);

elseif attSelected == 8 %% y1 & y2
    % Attack selection matrices
    sys.Gamma       = zeros(4,2); sys.Gamma(1,1) = 1; sys.Gamma(3,2) = 1;
    sys.GammaInv    = pinv(sys.Gamma);

    % allocate data subsystems
    sys_1.Gamma = sys.Gamma(1:2,:);
    sys_1.GammaInv = pinv(sys_1.Gamma);

    sys_2.Gamma = sys.Gamma(3:4,:);
    sys_2.GammaInv = pinv(sys_2.Gamma);

    % Line search attacked dector error
    alpha_eAtt_lower    = 0.01;
    alpha_eAtt_upper    = 0.05;
    sys.alpha_eAtt_vals = linspace(alpha_eAtt_lower, alpha_eAtt_upper, 300);

    % subsystems
    sys_1.alpha_eAtt_vals = linspace(0.01, 0.1, 150);
    sys_2.alpha_eAtt_vals = linspace(0.01, 0.1, 150);

    % Line search controller realization
    alpha_zeta_lower    = 0.01;
    alpha_zeta_upper    = 0.05;
    sys.alpha_zeta_vals = linspace(alpha_zeta_lower, alpha_zeta_upper, 200);

else
    % Attack selection matrices
    sys.Gamma       = 0;
    sys.GammaInv    = pinv(sys.Gamma);

    % allocate data subsystems
    sys_1.Gamma = sys.Gamma(1:2,:);
    sys_1.GammaInv = pinv(sys_1.Gamma);

    sys_2.Gamma = sys.Gamma(3:4,:);
    sys_2.GammaInv = pinv(sys_2.Gamma);

    % Line search attacked dector error
    alpha_eAtt_lower    = 0;
    alpha_eAtt_upper    = 0;
    sys.alpha_eAtt_vals = linspace(alpha_eAtt_lower, alpha_eAtt_upper, 300);

    % subsystems
    sys_1.alpha_eAtt_vals = linspace(0, 0, 150);
    sys_2.alpha_eAtt_vals = linspace(0, 0, 150);

    % Line search controller realization
    alpha_zeta_lower    = 0;
    alpha_zeta_upper    = 0;
    sys.alpha_zeta_vals = linspace(alpha_zeta_lower, alpha_zeta_upper, 200);
end
