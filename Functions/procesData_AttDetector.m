function [] = procesData_AttDetector(allSolutionsAttErr, bestSolAttErr, settings)

for i = 1:length(allSolutionsAttErr)
    alpha_eAtt_sweep(i)     = allSolutionsAttErr(i).alpha_eAtt;
    obj_eAtt_all(i)         = allSolutionsAttErr(i).obj;
    feasible_eAtt(i)        = allSolutionsAttErr(i).ConstraintsSatisfied;
    solverSucces_eAtt(i)    = allSolutionsAttErr(i).succes;
end

% Feasible solutions
obj_feasible_eAtt = obj_eAtt_all; obj_feasible_eAtt(~feasible_eAtt) = NaN;
obj_succes_eAtt = obj_eAtt_all;   obj_succes_eAtt(~solverSucces_eAtt) = NaN;


% Plot
figure;
plot(alpha_eAtt_sweep, obj_feasible_eAtt,'o', LineWidth=1.5); hold on
plot(alpha_eAtt_sweep, obj_succes_eAtt, LineWidth=1.5);
xlabel('$ \bar \alpha_e$');
ylabel('objective');
shading interp;     % Optional: smooths color

if settings.plotEllipse == 1
    %% Ellipse
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
        P_projected    = getProjection(bestSolAttErr.P_e, T);

        % Call function to plot the projected ellipse
        figure(n+1)
        plot_ellipse(P_projected, c, LS); hold on
        title('Projection attacked error set $\bar{\mathcal{E}}_e$')
        legend('Projection $e_1 - e_2$', 'Projection $e_2 - e_3$', 'Projection $e_3 - e_4$', 'Projection $e_1 - e_4$')
    end
end
end