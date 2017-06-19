classdef polygon
    properties 
        x
        y
    end
    methods
        
        function obj = polygon(x,y)
                obj.x = x;
                obj.y = y;
        end
        
        function [xi, yi] = intersect(obj,x,y)
                        
            [xp, yp] = polyxpoly(x,y,obj.x,obj.y, 'unique');
            In = [[]; xp, yp];
            ins=obj.inside(x,y);
            if(numel(x(ins)) > 0 || numel(y(ins)) > 0)
                In = [In; x(ins), y(ins)];
            end
            xi = In(:,1);
            yi = In(:,2);
            
        end
        
        function in = inside(obj,x,y)
            in = inpolygon(x,y,obj.x,obj.y);
        end
    end
end