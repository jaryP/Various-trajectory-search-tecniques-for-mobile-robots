classdef rrtB < generalRrt
    
    methods
        %costruttore. Parametri: configurazioni iniziale e finale del
        %robot, limiti del configuration space, array di ostacoli, robot
        function obj = rrtB(q_i,q_f,x_min,x_max,y_min,y_max,obstacles,costAreas, goalBias)
            obj@generalRrt(q_i,q_f,x_min,x_max,y_min,y_max,obstacles,costAreas, goalBias);   
        end
        
        function core(obj,maxIteration)

            iteration = 0;
            while(iteration < maxIteration)
   
                    x_rand = obj.sampleFree();
                    iteration = iteration + 1

                    if(~isempty(x_rand))
                        
                        
                        x_nearest = obj.getNearestConf(x_rand);
                        x_new = obj.steer(x_rand',x_nearest');
                        [x,y] = obj.getMovement(x_nearest,x_new);
                          
                        if all(x_nearest == x_new')
                                continue
                        end
                        
                          
                        if obj.obstacleFree(x,y)
                            obj.addNewNode(x_new,x_nearest');
                        end

                    end
            end
        end

    end
end