clear;
close all;
clc;

m = 10;
k = 9;
b = 1;
A = [0 1; -k/m -b/m];
B = [0; 1/m];

sys = ss(A, B, eye(2), zeros(2, 1));
sysd = c2d(sys, .001);
sys_ss = ss(sysd);

Q = [10 0; 0 1];
R = 0.001;

[P, E, G] = dare(sys_ss.A, sys_ss.B, Q, R);
sim_time = 60;
dt = .001;
interval = 0.1 / dt;
tspan = 0:dt:sim_time;

x0 = [2; 0];
x = x0;
x_target = [1; 0];
S = [A'*P*A+Q A'*P*B; B'*P*A B'*P*B+R];
controller_rl = Controller_dtrl(S, Q, R);

for i=0:length(tspan)
    x_error = x - x_target;
    if mod(i, interval) == 0
        drawModel(x);
    end
    % u = controller_rl.policy_improvement(x_error);
    u = -30*(x_error(1));
    % Suu = S(3:end, 3:end);
    % Sux = S(3:end, 1:2);
    % u = -inv(Suu)*Sux*(x-x_target);
    dx = massSpring_ct(x, m, k, b, u);
    x = x + dx * dt;
end
disp(x);