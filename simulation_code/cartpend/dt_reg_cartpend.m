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
values = zeros(length(t_cost), 1);
P_history = zeros(length(t_cost), 1);
[P, E, G] = dare(sys_ss.A, sys_ss.B, Q, R); % E = eig(A-B*G)    
%G = lqrd(A, B, Q, R, dt);

x0 = [0; 0; pi; 0];
x_target = [0; 0; pi; 0];
x = x0;
A = sys_ss.A;
B = sys_ss.B;
S = [A'*P*A+Q A'*P*B; B'*P*A B'*P*B+R];

controller_rl = Controller_dtrl(S, Q, R);

for i=0:length(tspan)
    % u = controller_dtlqr(G, x, x_target);
    x_error = x - x_target;
    u = controller_rl.policy_improvement(x_error);
    %Suu = S(5:end, 5:end);
    %Sux = S(5:end, 1:4);
    %u = -inv(Suu)*Sux*(x-x_target);
    dx = cartpend(x, m, M, L, g, d, u);
    if mod(i, interval) == 0
      drawcartpend(x, m, M, L);
      %cost = x'*P*x;
      cost = (x_error)'*Q*(x_error) + u'*R*u;
      value = (x_error)'*P*(x_error);
      values(i ./ interval + 1) = value;
      costs(i ./ interval + 1) = cost;
      S_history = controller_rl.S;
      [V, D] = eig(S_history);
      P_history(i ./ interval + 1) = D(1, 1);
    end
    x_error_prev = x_error;
    x = x + dx*dt;
    x_error = x - x_target;
    controller_rl.policy_evaluation([x_error_prev; u], x_error);
end

%close all;
figure
ax1 = subplot(3, 1, 1);
plot(ax1, t_cost, costs, '-');
title('cost');
ax2 = subplot(3, 1, 2);
plot(ax2, t_cost, values);
title('value');
ax3 = subplot(3, 1, 3);
plot(ax3, t_cost, P_history);
title('P11');
%hold on;
%plot(ax3, t_cost, ones(length(t_cost), 1)*P(1, 1));
% [t, y] = ode45(@(t, x)cartpend(y, m, M, L, g, d, 0), tspan, y0);
% for i=1:100:length(t)
%     drawcartpend(x(i, :), m, M, L);
% end
