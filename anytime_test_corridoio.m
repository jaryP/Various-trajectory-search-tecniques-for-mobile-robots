close all
clear all 

rng('default');
rng(9);

x_min = -2.5;
y_min = -5;
x_max = 2.5;
y_max = 5;
init_conf = [0,-4];
final_conf = [0,4.5];

area = [];

for i=1:15  
    xr = rand*(x_max - x_min) +x_min;
    yr = rand*(y_max - y_min) +y_min;
    if rand <= 0.5
        c = 0.5;
    else
        c = 2.0;
    end
    obj1_x = [.5 0 0 .5 .5]+xr;
    obj1_y = [0 0 1 1 0 ]+yr;
    c = costArea(obj1_x,obj1_y,c);
    area = [area, c];
        
end

rng(42,'simdTwister');

rrt = rrtB(init_conf,final_conf,x_min, x_max,y_min,y_max,[],area,0.7);
rrt.core(500);
[dist, path] = rrt.getEndPath;

upper_bound = dist;
max_nodes_anytime = 1000;
k=3;
arrt = AnytimeRRT(init_conf,final_conf,x_min,x_max,y_min,y_max,[],area,0.7,k,upper_bound,max_nodes_anytime);

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
    if(size(costi,1)>=1)&&(actual_cost>costi(size(costi,1)))
        if(i-1>0)
            arrt.finalPlot(i-1,rrt,costi(size(costi,1)-1));
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