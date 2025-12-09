function [] = procesData_ResidualV2(sys, allSolutionsResidual, bestSolResidual, settings, titleInput)

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
sgtitle(['Linesearch to obtain ${\mathcal{E}}_e$: ', titleInput])
shading interp;     % Optional: smooths color
colorbar;

%% Ellipse

if settings.plotEllipse == 1
    h =  findobj('type','figure'); n = length(h);
    for Ti = 1
        % Determine the required transformation matrix
        if Ti == 1 % projection e1-e2
            T = eye(2);
            c = [1 0 0];
            LS = '-';
        end


        % Projection
        P_projected_Pe    = getProjection(bestSolResidual.P_e, T);
        P_projected_Pi    = getProjection(bestSolResidual.Pi, T);

        % Call function to plot the projected ellipse
        figure(n+1)
        plot_ellipse(P_projected_Pe, c, LS); hold on
        sgtitle(['Projection healthy error set ${\mathcal{E}}_e$: ', titleInput])

        figure(n+2)
        plot_ellipse(P_projected_Pi, c, LS); hold on
        sgtitle(['Projection residual set ${\mathcal{E}}_r$: ', titleInput])

    end

    % Call function to plot the projected ellipse
    figure(n+1)
    legend('$e_1$ - $e_2$', Location='best')

    figure(n+2)
    legend('$r_1$ - $r_2$', Location='best')

end

