function ellipse = plot_ellipse(E, c, LS)
% plots an ellipse of the form xEx = 1
R = chol(E);
t = linspace(0, 2*pi, 100); % or any high number to make curve smooth
z = [cos(t); sin(t)];
ellipse = inv(R) * z;

plot(ellipse(1,:), ellipse(2,:),'Color',c,'LineWidth',2, 'LineStyle', LS)

end