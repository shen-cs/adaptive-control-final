clear;
close all;
clc;

m = 1;
M = 5;
L = 2;
g = -10;
d = 0;
s = 1; % pendulum up (s=1)

A = [0 1 0 0;
    0 -d/M -m*g/M 0;
    0 0 0 1;
    0 -s*d/(M*L) -s*(m+M)*g/(M*L) 0];

B = [0; 1/M; 0; s*1/(M*L)];

sys = ss(A, B, eye(4), zeros(4, 1));
sysd = c2d(sys, .001);
sys_ss = ss(sysd);

Q = [1 0 0 0;
    0 1 0 0;
    0 0 10 0;
    0 0 0 100];
R = .0001;

sim_time = 10;
dt = .001;
interval = 0.1 / dt;
tspan = 0:dt:sim_time;
t_cost = 0:interval*dt:sim_time;
costs = zeros(length(t_cost), 1);
[P, E, G] = dare(sys_ss.A, sys_ss.B, Q, R); % E = eig(A-B*G)
%G = lqrd(A, B, Q, R, dt);

y0 = [0; 0; pi-0.1; 0];
y = y0;
for i=0:length(tspan)
    u = -G*(y-[1; 0; pi; 0]);
    dy = cartpend(y, m, M, L, g, d, u);
    if mod(i, interval) == 0
      drawcartpend(y, m, M, L);
      %cost = y'*P*y;
      cost = y'*Q*y + u'*R*u;
      costs(i ./ interval + 1) = cost;
    end
    y = y + dy*dt;
end

close all;

plot(t_cost, costs, '-');
% [t, y] = ode45(@(t, y)cartpend(y, m, M, L, g, d, 0), tspan, y0);
% for i=1:100:length(t)
%     drawcartpend(y(i, :), m, M, L);
% end
