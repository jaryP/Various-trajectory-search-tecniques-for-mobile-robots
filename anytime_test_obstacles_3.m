init_conf = [0,0];
final_conf = [4,0];
x_min = -5;
y_min = -5;
x_max = 5;
y_max = 5;

obst1_x = [-2.5 3 3 -2.5 -2.5];
obst1_y = [1 1 2 2 1];

obst3_x = [-2.5 3 3 -2.5 -2.5];
obst3_y = [-1 -1 -2 -2 -1];

obst2_x = [2 3 3 2 2];
obst2_y = [2 2 -2 -2 2];

obst4_x = [2 3 3 2 2 ];
obst4_y = [0 0  2 2 0];

obst5_x = [-5 -2 -2 -5 -5 ];
obst5_y = [-2 -2  -1 -1 -2];


obstacle = [polygon(obst1_x,obst1_y), polygon(obst3_x,obst3_y), polygon(obst2_x,obst2_y)];

rng(28);

rrt = rrtB(init_conf,final_conf,x_min, x_max,y_min,y_max,obstacle,[],0.7);
rrt.core(4000);
[dist, path] = rrt.getEndPath;

upper_bound = dist;
max_nodes_anytime = 2000;
k=3;
arrt = AnytimeRRT(init_conf,final_conf,x_min,x_max,y_min,y_max,obstacle,0.7,k,upper_bound,max_nodes_anytime);
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
            arrt.finalPlot(i-1,rrt,costi(size(costi,1)));
            break
        else
            %arrt.finalPlot(i,rrt,costi(size(costi,1)));
            arrt.plotIncompletePath();
            break;
        end
    end
else
    if(i-1>0)
        arrt.finalPlot(i-1,rrt,costi(size(costi,1)-1));
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