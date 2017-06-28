close
init_conf = [0,0];
final_conf = [4.5,0];
x_min = -5;
y_min = -5;
x_max = 5;
y_max = 5;
robot = pointRobot;

obst1_x = [-2.5 -1 -1 -2.5 -2.5];
obst1_y = [1 2 3 3 1];

obst2_x = [1 2 2 1 1];
obst2_y = [1 0 4 1 1];

obst3_x = [3 4 4 3 3 ];
obst3_y = [-4 -4  1 1 -4];

axis([x_min x_max y_min y_max]);

obstacle = [polygon(obst1_x,obst1_y),polygon(obst2_x,obst2_y), polygon(obst3_x,obst3_y)];
goalBias = 0.6;
k = 3;

rrt = RRT(init_conf,final_conf,x_min,x_max,y_min,y_max,obstacle,robot,5000,goalBias);
rrt.run();


if(strcmp(rrt.status,'reached')==1)
    [~, path, ~] = graphshortestpath(adjacency(rrt.graph), 1, size(rrt.nodes,1));
    upper_bound = size(path,2)
    
    arrt = AnytimeRRT(init_conf,final_conf,x_min,x_max,y_min,y_max,obstacle,robot,goalBias,k,upper_bound);
    actual_cost = arrt.growRRT()
    

        if(actual_cost~= -1)
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

    arrt.plot(rrt);
    
    %     arrt.nodes = [arrt.init_node];
    %     G = graph;
    %     G = addnode(G,1);
    %         arrt.graph = G;
else
    error('RRT did not find a path to the goal');
end
