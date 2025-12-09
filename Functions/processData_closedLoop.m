function [bestSolZeta_trace, bestSolZeta_logdet] = processData_closedLoop(allSolutionsZeta, bestSolZeta, settings, titleInput)


for i = 1:length(allSolutionsZeta)
    alpha_zeta_sweep(i)     = allSolutionsZeta(i).alpha_zeta;
    logdet_Pzeta_all(i)     = -logdet(allSolutionsZeta(i).P_zeta);
    trace_Yzeta_all(i)      = trace(allSolutionsZeta(i).Y_zeta);
    feasible_zeta(i)        = allSolutionsZeta(i).ConstraintsSatisfied;
    solverSucces_zeta(i)    = allSolutionsZeta(i).succes;
end

% Feasible solutions
trace_feasible_zeta = trace_Yzeta_all; trace_feasible_zeta(~feasible_zeta) = NaN;
trace_succes_zeta = trace_Yzeta_all;   trace_succes_zeta(~solverSucces_zeta) = NaN;
logdet_feasible_zeta = logdet_Pzeta_all; logdet_feasible_zeta(~feasible_zeta) = NaN;
logdet_succes_zeta = logdet_Pzeta_all;   logdet_succes_zeta(~solverSucces_zeta) = NaN;

% Find minimum volume (logdet) en minimum trace solution
[~, indTrace] = min(trace_feasible_zeta);
[~, indLogdet] = min(logdet_feasible_zeta);
bestSolZeta_trace = allSolutionsZeta(indTrace);
bestSolZeta_logdet = allSolutionsZeta(indLogdet);


% Plot
figure;
subplot(2,1,1)
plot(alpha_zeta_sweep, trace_feasible_zeta, 'o', LineWidth=1.5); hold on
plot(alpha_zeta_sweep, trace_succes_zeta, LineWidth=1.5);
xlabel('$ \bar \alpha_\zeta $');
ylabel('trace$(Y_ \zeta )$');
subplot(2,1,2)
plot(alpha_zeta_sweep, logdet_feasible_zeta, 'o', LineWidth=1.5); hold on
plot(alpha_zeta_sweep, logdet_succes_zeta, LineWidth=1.5);
xlabel('$ \bar \alpha_\zeta $');
ylabel('$ -\log \det (P_ \zeta) $');
shading interp;     % Optional: smooths color
legend('Constraints satisfied', 'Solver implies succes', 'Location','best')
sgtitle(['Linesearch to obtain ${\mathcal{E}}_\zeta$: ', titleInput])

%% Ellipse
if settings.plotEllipse == 1
    if ~isempty(bestSolZeta)>0
        h =  findobj('type','figure'); n = length(h);
        for Ti = 1:5

            % Determine the required transformation matrix
            if Ti == 1 % projection x1-x2
                T = eye(6);
                c = [1 0 0];
                LS = '-';

            elseif Ti == 2 % projection x2-x3
                T = [0, 1, 0, 0, 0, 0;
                    0, 0, 1, 0, 0, 0;
                    1, 0, 0, 0, 0, 0;
                    0, 0, 0, 1, 0, 0;
                    0, 0, 0, 0, 1, 0;
                    0, 0, 0, 0, 0, 1];
                c= [0 1 0];
                LS = '--';

            elseif Ti == 3 % projection x3-x4
                T = [0, 0, 1, 0, 0, 0;
                    0, 0, 0, 1, 0, 0;
                    1, 0, 0, 0, 0, 0;
                    0, 1, 0, 0, 0, 0;
                    0, 0, 0, 0, 1, 0;
                    0, 0, 0, 0, 0, 1];
                c = [0 0 1];
                LS = '-.';
            elseif Ti == 4 % projection x1-x4
                T = [1, 0, 0, 0, 0, 0;
                    0, 0, 0, 1, 0, 0;
                    0, 1, 0, 0, 0, 0;
                    0, 0, 1, 0, 0, 0;
                    0, 0, 0, 0, 1, 0;
                    0, 0, 0, 0, 0, 1];
                c = [0,0,0];
                LS = ':';
            elseif Ti == 5 % projection rho1-rho2
                T = [0, 0, 0, 0, 1, 0;
                    0, 0, 0, 0, 0, 1;
                    1, 0, 0, 0, 0, 0;
                    0, 1, 0, 0, 0, 0;
                    0, 0, 1, 0, 0, 0;
                    0, 0, 0, 1, 0, 0];
                c = [0,0,0];
                LS = ':';
            end

            % Projection
            P_projected    = getProjection(bestSolZeta.P_zeta, T);

            if Ti <= 4
                % Call function to plot the projected ellipse
                figure(n+1)
                plot_ellipse(P_projected, c, LS); hold on
                sgtitle(['Projection closed-loop ${\mathcal{E}}_\zeta$: ', titleInput])
            else
                h =  findobj('type','figure'); n = length(h);
                figure(n+1)
                plot_ellipse(P_projected, c, LS); hold on
                sgtitle(['Projection closed-loop ${\mathcal{E}}_\zeta$: ', titleInput])

            end
        end
        figure(n)
        xline(0, 'linewidth', 1.5);  yline(0, 'linewidth', 1.5)
        legend('Projection $x_1 - x_2$','Projection $x_2 - x_3$','Projection $x_3 - x_4$','Projection $x_1 - x_4$', "", "")
        figure(n+1)
        xline(0, 'linewidth', 1.5);  yline(0, 'linewidth', 1.5)
        legend('Projection $\rho_1 - \rho_2$', "", "")
    end
end
end