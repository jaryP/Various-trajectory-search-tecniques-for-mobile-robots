classdef costArea < polygon
   properties 
       costMul = 1
   end
    methods
        function obj = costArea(x,y, costMul)
            obj@polygon(x,y);
            obj.costMul = costMul;
        end
    end
end