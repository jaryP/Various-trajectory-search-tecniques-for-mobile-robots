classdef unicycle
    properties 
        currentConfig
        q
        v
        omega
    end
    methods 
        function obj = unicycle(initialConfig)
            obj.q = initialConfig;
            obj.v = 1;
            obj.omega = pi/4;
        end
        function [xR,yR,qR] = getRandomConfig(x,y)
               xR = rand * (x(1) -x(2)) + x(2);
               yR = rand * (y(1) -y(2)) + y(2);
               qR = rand * (pi + pi) - pi;
        end
        function dirKin = kinematics(theta)
               dirKin = [ obj.v*cos(theta); obj.v*sin(theta); obj.omega ];
        end
    end  
end