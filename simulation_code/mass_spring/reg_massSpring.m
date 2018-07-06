clear;
close all;
clc;

m = 10;
k = 9;
b = 4;
A = [0 1; -k/m -b/m];
B = [0; 1/m];

sys = ss(A, B, eye(2), zeros(2, 1));
sysd = c2d(sys, .001);
sys_ss = ss(sysd);
A = sys_ss.A;
B = sys_ss.B;
Q = [10 0; 0 1];
R = 0.001;

[P, E, G] = dare(sys_ss.A, sys_ss.B, Q, R);
sim_time = 30;
dt = .001;
interval = 0.1 / dt;
tspan = 0:dt:sim_time;

x0 = [3; -4];
x = x0;
x_target = [0; 0];
t_cost = 0:interval*dt:sim_time;
x_history = zeros(length(t_cost), 1);
p_history = zeros(2, 2, length(t_cost));
%S_true = [A'*P*A+Q A'*P*B; B'*P*A B'*P*B+R];
S = ones(3) + rand(3, 3);
controller_rl = Controller_dtrl(S, Q, R);
x_int = 0;
for i=0:length(tspan)
    x_error = x - x_target;
    if mod(i, interval) == 0
        drawModel(x);
        x_history(i ./ interval+1) = x(1);
        Sxx_history = controller_rl.S(1:2, 1:2);
        [V, D] = eig(Sxx_history-Q);
        p_history(:, :, i ./ interval + 1) = D;
    end
    u = controller_rl.policy_improvement(x_error, x_int);
    % u = -[G 20]*[x_error; x_int];
    % u = -[80 50 20]*[x_error; x_int]; % PID control
%     Suu = S(3:end, 3:end);
%     Sux = S(3:end, 1:2);
%     K = Suu\Sux;
%     u = -K*(x-x_target);
    dx = massSpring_ct(x, m, k, b, u) + rand(2, 1)*.1;
    x_error_prev = x_error;
    x = x + dx * dt;
    x_error = x - x_target;
    x_int = x_int + x_error(1)*dt;
    controller_rl.policy_evaluation([x_error_prev; u], x_error);
end
disp(x);
%disp(S(1, 1)); % optimal S(1, 1) = 5.33e03
disp(P);
disp(p_history(:, :, end));
figure;
ax1 = subplot(2, 1, 1);
plot(ax1, t_cost, x_history);
title('x');
ax2 = subplot(2, 1, 2);
plot(ax2, t_cost, squeeze(p_history(1, 1, :)));
hold on;
plot(ax2, t_cost, squeeze(p_history(2, 2, :)));
legend('P11', 'P22');
title('P');