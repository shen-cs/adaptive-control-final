function drawModel(x)
%DRAWMODEL Summary of this function goes here
%   Detailed explanation goes here
%   xi: distance from mass to equilibrium point
xi = x(1);
radius = .2;
spring_length = 2;
plot([-spring_length-2, xi-radius/2], [0, 0], 'k', 'LineWidth', 2);
hold on;
rectangle('Position', [xi-radius/2, -radius/2, radius, radius], 'Curvature', 1, 'FaceColor', [.1 0.1 1]);
xlim([-spring_length, 4]);
ylim([-2, 2]);
set(gcf,'Position',[100 150 600 400])
drawnow
hold off;
end

