classdef Controller_dtrl < handle
    %CONTROLLER_DTRL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        S
        S_temp
        Q
        R
        x_int
    end
    
    methods
        function obj = Controller_dtrl(S, Q, R)
            %CONTROLLER_DTRL Construct an instance of this class
            %   Detailed explanation goes here
            %   n is the dimension of state vector 
            obj.S = S;
            obj.S_temp = S;
            obj.Q = Q;
            obj.R = R;
            obj.x_int;
        end
        
        function set.S(obj, value)
            obj.S = value;
        end
        
        function S = policy_evaluation(obj, z, x_next)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            n = size(obj.Q, 1);
            alpha = 0.5;
            gamma = 0.99;
            x = z(1:n);
            u = z(n+1:end);
            % u* = argmin Q(x_next, u);
            % Q_min = Q(x_next, u*)
            u_min = obj.policy_improvement(x_next, obj.x_int);
            z_min = [x_next; u_min];
            q_min = z_min'*obj.S*z_min;
            % delta = x'*obj.Q*x + u'*obj.R*u + gamma*q_min - z'*obj.S*z; % q-learning
            z_next = [x_next; u_min];
            delta = x'*obj.Q*x + u'*obj.R*u + gamma*z_next'*obj.S*z_next - z'*obj.S*z;% sarsa
            %disp(delta);
            dS = z * z';
            S_prev = obj.S_temp;
            obj.S_temp = obj.S_temp + alpha * delta * dS;
            if abs(det(obj.S_temp-S_prev)) < 0.0001
                obj.S = obj.S_temp;
            end
            S = obj.S_temp;
        end
        
        function u = policy_improvement(obj, x, x_int)
            % suppose x is n-vector, u is m-vector, n + m = l
            n = size(obj.Q, 1);
            Suu = obj.S(n+1:end, n+1:end); % m * m
            Sux = obj.S(n+1:end, 1:n); % m * n
            K = Suu\Sux;
            % u = -K*x;
            u = -[K 0.8]*[x;x_int] + rand(1, 1) * 5;
            obj.x_int = x_int;
        end
        
        function u = learn(obj, z, z_next)
           n = size(obj.Q, 1);
           x_next = z_next(1: n);
           obj.S = obj.q_policy_evaluation(z, z_next);
           u = obj.policy_improvement(x_next);
        end
            
    end
end

