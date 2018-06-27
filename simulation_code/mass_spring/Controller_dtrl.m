classdef Controller_dtrl < handle
    %CONTROLLER_DTRL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        S
        Q
        R
    end
    
    methods
        function obj = Controller_dtrl(S, Q, R)
            %CONTROLLER_DTRL Construct an instance of this class
            %   Detailed explanation goes here
            %   n is the dimension of state vector 
            obj.S = S;
            obj.Q = Q;
            obj.R = R;
            
        end
        
        function set.S(obj, value)
            obj.S = value;
        end
        
        function S = policy_evaluation(obj, z, x_next)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            n = size(obj.Q, 1);
            alpha = 0.1;
            gamma = 1;
            x = z(1:n);
            u = z(n+1:end);
            % u* = argmin Q(x_next, u);
            % Q_min = Q(x_next, u*)
            u_min = obj.policy_improvement(x_next);
            z_min = [x_next; u_min];
            q_min = z_min'*obj.S*z_min;
            % delta = x'*obj.Q*x + u'*obj.R*u + gamma*q_min - z'*obj.S*z;
            z_next = [x_next; u_min];
            delta = x'*obj.Q*x + u'*obj.R*u + gamma*z_next'*obj.S*z_next - z'*obj.S*z;
            disp(delta);
            dS = z * z';
            obj.S = obj.S - alpha * delta * dS;
            S = obj.S;
        end
        
        function u = policy_improvement(obj, x)
            % suppose x is n-vector, u is m-vector, n + m = l
            n = size(obj.Q, 1);
            Suu = obj.S(n+1:end, n+1:end); % m * m
            Sux = obj.S(n+1:end, 1:n); % m * n
            u = -inv(Suu)*Sux*x;
        end
        
        function u = learn(obj, z, z_next)
           n = size(obj.Q, 1);
           x_next = z_next(1: n);
           obj.S = obj.q_policy_evaluation(z, z_next);
           u = obj.policy_improvement(x_next);
        end
            
    end
end

