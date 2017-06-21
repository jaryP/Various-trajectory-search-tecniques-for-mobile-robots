classdef unicycle
    properties 
        %currentConfig
        %q
        v
        omega
    end
    methods 
        function obj = unicycle()
            %obj.q = initialConfig;
            obj.v = 0.2;
            obj.omega = pi/4;
        end
        
        function randomConfig = getRandomConfig(obj,x,y)
               x = rand * (x(1) -x(2)) + x(2);
               y = rand * (y(1) -y(2)) + y(2);
               q = rand * (2*pi) - pi;
               randomConfig = [x, y, q];
        end
        
        function dirKin = directKinematics(obj, currentConfig)
            
               dirKin = [];
               dirKin = [obj.v*cos(currentConfig(3)-obj.omega) obj.v*sin( currentConfig(3)-obj.omega) -obj.omega ];
               dirKin = [dirKin; obj.v*cos(currentConfig(3)+obj.omega) obj.v*sin( currentConfig(3)+obj.omega) +obj.omega ];
               dirKin = [dirKin; obj.v*cos(currentConfig(3)) obj.v*sin(currentConfig(3)) 0]';
               
               dirKin = dirKin + currentConfig;
                
        end
        
        function obj = setV(obj,v)
            obj.v = v;
        end
        
         function obj = setOmega(obj,omega)
            obj.omega = omega;
         end
         
         function obj = move(obj,primitiva)
             obj.currentConfig=obj.currentConfig+primitiva;
         end
    end  
end
