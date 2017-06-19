classdef kalman < matlab.System & ...
    matlab.system.mixin.Nondirect
    % Untitled Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.

    % Public, tunable properties
    properties

    end

    properties(DiscreteState)
        R;
        Q;
        
        P;
        xe;
    end

    % Pre-computed constants
    properties(Access = private)

    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants

        end

        function updateImpl(obj,u,z)
            % predict
            dt = 0.1
            theta = obj.xe(3);
            sl = u(1);
            sr = u(2);
            
            w = 1;
            a = 0.5*(sl+sr);
            b = (sr-sl)/(2*w);
            F = [1 0 -a*sin(theta+b);
                 0 1  a*cos(theta+b);
                 0 0  1];
            
            obj.xe = obj.xe + [ a*cos(theta + b)*dt;
                                a*sin(theta + b)*dt;
                                2*b*dt];
            obj.P = F * obj.P * transpose(F) + obj.Q;
            
            % update
            H = ones(3);
            
            y = z - obj.xe;
            S = H * obj.P * transpose(H) + obj.R;
            K = (obj.P * transpose(H))/S;
            %obj.xe = obj.xe + K * y;
            
            obj.P = (eye(3) - K * H) * obj.P;
          
        end
        
        function [xe,P] = outputImpl(obj,u,z)   
            xe = obj.xe;
            P = obj.P(3,3);
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        
            obj.Q = [0.01 0   0;
                     0   0.01 0;
                     0   0   0.01];
             
            obj.R = 10;
            
            obj.P = ones(3);
         
            obj.xe = [0;0;0];
        end
    end
end
