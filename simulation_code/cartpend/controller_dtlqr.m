function u = dtlqr_controller(G, x, x_target)
%DTLQR_CONTROLLER Summary of this function goes here
%   Detailed explanation goes here
u = -G*(x-x_target);
end

