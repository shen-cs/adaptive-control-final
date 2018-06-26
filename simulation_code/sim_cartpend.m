clear all, close all, clc

m = 1;
M = 5;
L = 2;
g = -10;
d = 0;

tspan = 0:.1:20;
y0 = [0; 0; pi; 0];
[t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,0),tspan,y0);

for k=1:length(t)
    drawcartpend(y(k,:),m,M,L);
end

% function dy = pendcart(y,m,M,L,g,d,u)