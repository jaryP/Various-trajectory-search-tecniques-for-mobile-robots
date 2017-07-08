close all

rng('default');
rng(1);

init_conf = [2.5,-1];
final_conf = [4.5,4.5];
x_min = -5;
y_min = -5;
x_max = 5;
y_max = 5;

obst1_x = [-2.5 5 5 -2.5 -2.5];
obst1_y = [1 1 3 3 1];

obstacle = [polygon(obst1_x,obst1_y)];

axis([x_min x_max y_min y_max]);

goalBias = 0.6;
k = 3;

% max_nodes_rrt = 1000;
% rrt = RRT(init_conf,final_conf,x_min,x_max,y_min,y_max,obstacle,max_nodes_rrt,goalBias);
% rrt.run();
% rrt.plot();
% rng(1);

max_nodes_anytime = 2000;

% if(strcmp(rrt.status,'reached')==1)
 
%     [path, upper_bound] = shortestpath(rrt.graph,1, size(rrt.nodes,1));
    upper_bound = 21;
    
    arrt = AnytimeRRT(init_conf,final_conf,x_min,x_max,y_min,y_max,obstacle,goalBias,k,upper_bound,max_nodes_anytime);
    actual_cost = arrt.growRRT();
    
    costi = upper_bound;
    flag = false;
    
    for i=1:10
        
        costi = [costi;actual_cost]
        if(actual_cost~=-1)
            arrt.plot(i);
            arrt.nodes = [arrt.init_node];
            arrt.upper_bound = actual_cost;
            G = graph;
            G = addnode(G,1);
            arrt.graph = G;
            actual_cost = arrt.growRRT();
            if(size(costi,1)>=1)&& (actual_cost>costi(size(costi,1)))
                if(i-1>0)
                    arrt.finalPlot(i-1,[],costi(size(costi,1)));
                    break
                else
                    arrt.finalPlot(i,[],costi(size(costi,1)));
                   %arrt.plotIncompletePath();
                    break;
                end
            end
        else
            if(i-1>0)
                arrt.finalPlot(i-1,[],costi(size(costi,1)));
                break
            else
                arrt.plotIncompletePath();
                break;
            end
        end
        
        arrt.upper_bound = (1-0.1)*actual_cost;
        arrt.distance_bias = arrt.distance_bias - 0.1;
        if(arrt.distance_bias<0)
            arrt.distance_bias = 0;
        end
        arrt.cost_bias = arrt.cost_bias + 0.1;
        if(arrt.cost_bias>1)
            arrt.cost_bias = 1;
        end
        
    end
    if(flag == 0)
        arrt.plotIncompletePath();
    end
% end