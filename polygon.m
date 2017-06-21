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
            xi = xp;
            yi = yp;
            ins= inpolygon(x,y,obj.x,obj.y);
            xin = x(ins)';
            yin = y(ins)';
            xi = [xi; xin]';
            yi = [yi; yin]';

        end
        
    end
end