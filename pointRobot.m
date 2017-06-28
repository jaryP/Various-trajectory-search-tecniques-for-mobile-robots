classdef pointRobot  
    properties 
        v
    end
    methods 
        function obj = pointRobot()
            obj.v = 0.2;
        end
        
        function randomConfig = getRandomConfig(~,x,y)
               x = rand * (x(1) -x(2)) + x(2);
               y = rand * (y(1) -y(2)) + y(2);
               randomConfig = [x, y];
        end
    end  
end
