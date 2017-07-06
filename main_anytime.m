close
    
rng('default');
rng(1);
init_conf = [0,0];
final_conf = [4.5,0];
x_min = -5;
y_min = -5;
x_max = 5;
y_max = 5;

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

max_nodes_rrt = 2000;
rrt = RRT(init_conf,final_conf,x_min,x_max,y_min,y_max,obstacle,max_nodes_rrt,goalBias);
rrt.run();

rng(1);
max_nodes_anytime = 3000;
step = 0.2;
if(strcmp(rrt.status,'reached')==1)

[path, upper_bound] = shortestpath(rrt.graph,1, size(rrt.nodes,1));
arrt = AnytimeRRT(init_conf,final_conf,x_min,x_max,y_min,y_max,obstacle,goalBias,k,upper_bound,max_nodes_anytime);
actual_cost = arrt.growRRT();
costi = [];
flag = false;
for i=1:5
    arrt.upper_bound = (1-0.1)*actual_cost;
    arrt.distance_bias = arrt.distance_bias - 0.1;
    if(arrt.distance_bias<0)
        arrt.distance_bias = 0;
    end
    arrt.cost_bias = arrt.cost_bias + 0.1;
    if(arrt.cost_bias>1)
        arrt.cost_bias = 1;
    end
    costi = [costi;actual_cost];
    if(actual_cost~=-1)
        arrt.plot(i);
        arrt.nodes = [arrt.init_node];
        arrt.upper_bound = actual_cost;
        G = graph;
        G = addnode(G,1);
        arrt.graph = G;
        actual_cost = arrt.growRRT();
    else
        flag = true;
        arrt.finalPlot(i-1,rrt,costi(size(costi,1)-1));
    break
    end
end
if(flag ==0)
  arrt.finalPlot(i-1,rrt,costi(size(costi,1)-1));  
end
end
