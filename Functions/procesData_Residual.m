function [] = procesData_Residual(allSolutionsResidual, bestSolResidual, settings)

for i = 1:length(allSolutionsResidual)
    alpha_e_sweep(i)    = allSolutionsResidual(i).alpha_e;
    alpha_r_sweep(i)    = allSolutionsResidual(i).alpha_r;
    obj_all(i)          = allSolutionsResidual(i).obj;
    feasible(i)         = allSolutionsResidual(i).ConstraintsSatisfied;
    solverSucces(i)     = allSolutionsResidual(i).succes;
end

% Feasible solutions
obj_feasible = obj_all; obj_feasible(~feasible) = NaN;
obj_succes = obj_all;   obj_succes(~solverSucces) = NaN;

% Create grid
obj_reshaped = reshape(obj_feasible, [length(sys.alpha_r_vals), length(sys.alpha_e_vals)]);
[Alpha1, Alpha2] = meshgrid(sys.alpha_e_vals, sys.alpha_r_vals);
obj_succes_Grid = griddata(alpha_e_sweep, alpha_r_sweep, obj_succes, Alpha1, Alpha2, 'linear');


% Plot
figure;
surf(Alpha1, Alpha2, obj_reshaped); hold on
scatter3(Alpha1, Alpha2, obj_succes_Grid); hold on
xlabel('$\alpha_e$');
ylabel('$\alpha_r$'); %ylim([0 2])
zlabel('Objective value');
% title('Objective surface vs \alpha_1 and \alpha_2');
shading interp;     % Optional: smooths color
colorbar;

%% Ellipse

if settings.plotEllipse == 1
    h =  findobj('type','figure'); n = length(h);
    for Ti = 1:4
        % Determine the required transformation matrix
        if Ti == 1 % projection e1-e2
            T = eye(4);
            c = [1 0 0];
            LS = '-';

        elseif Ti == 2 % projection e2-e3
            T = [0, 1, 0, 0;
                0, 0, 1, 0;
                1, 0, 0, 0;
                0, 0, 0, 1];
            c= [0 1 0];
            LS = '--';

        elseif Ti == 3 % projection e3-e4
            T = [0, 0, 1, 0;
                0, 0, 0, 1;
                1, 0, 0, 0;
                0, 1, 0, 0];
            c = [0 0 1];
            LS = '-.';
        elseif Ti == 4 % projection e1-e4
            T = [1, 0, 0, 0;
                0, 0, 0, 1;
                0, 1, 0, 0;
                0, 0, 1, 0];
            c = [0,0,0];
            LS = ':';
        end


        % Projection
        P_projected_Pe    = getProjection(bestSolResidual.P_e, T);
        P_projected_Pi    = getProjection(bestSolResidual.Pi, T);

        % Call function to plot the projected ellipse
        figure(n+1)
        plot_ellipse(P_projected_Pe, c, LS); hold on
        title('Projection stealthy error set $\mathcal{E}_e$')

        figure(n+2)
        plot_ellipse(P_projected_Pi, c, LS); hold on
        title('Projection stealthy residual set $\mathcal{E}_r$')

    end

    % Call function to plot the projected ellipse
    figure(n+1)
    legend('$e_1$ - $e_2$','$e_2$ - $e_3$','$e_3$ - $e_4$','$e_1$ - $e_4$', Location='best')

    figure(n+2)
    legend('$r_1$ - $r_2$','$r_2$ - $r_3$','$r_3$ - $r_4$','$r_1$ - $r_4$', Location='best')

end

