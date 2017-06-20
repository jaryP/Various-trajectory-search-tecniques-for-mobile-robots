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
            
            %Matrix with colum [x y]'
                        
            [xp, yp] = polyxpoly(x,y,obj.x,obj.y, 'unique');
            
            xi = [[] xp];
            yi = [[] yp];
            ins=inpolygon(x,y,obj.x,obj.y);
            xi = [xi x(ins)]';
            yi = [yi y(ins)]';

        end
        
    end
end