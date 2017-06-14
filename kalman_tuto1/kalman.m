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
        A;
        B;
        C;
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
            obj.xe = obj.A * obj.xe + obj.B * u;
            obj.P = obj.A * obj.P * transpose(obj.A) + obj.Q;
            
            % update
            y = z - obj.C * obj.xe;
            S = obj.R + obj.C * obj.P * transpose(obj.C);
            K = (obj.P * transpose(obj.C))/S;
            obj.xe = obj.xe + K * y;
            
            obj.P = (eye(3) - K * obj.C) * obj.P;
          
        end
        
        function [xe,P] = outputImpl(obj,u,z)   
            xe = obj.xe;
            P = obj.P(3,3);
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            obj.A=[0  0  0;
                   0.1 1  0;
                   0  0.1 1];
            obj.B = [1;
                     0;
                     0];
            obj.C = [0 0 1];
        
            obj.Q = [0.01 0   0;
                     0   0.01 0;
                     0   0   0.01];
             
            obj.R = 10;
            
            obj.P = zeros(3);
         
            obj.xe = [0;0;0];
        end
    end
end
