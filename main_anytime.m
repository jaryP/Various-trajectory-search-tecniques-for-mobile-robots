close
rng('default');
rng(1);
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

rng(1);

if(strcmp(rrt.status,'reached')==1)

[~, path, ~] = graphshortestpath(adjacency(rrt.graph), 1, size(rrt.nodes,1));
upper_bound = size(path,2);

arrt = AnytimeRRT(init_conf,final_conf,x_min,x_max,y_min,y_max,obstacle,robot,goalBias,k,upper_bound);
actual_cost = arrt.growRRT();

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
    if(strcmp(arrt.status,'reached')==1)
        arrt.plot(i);
        arrt.nodes = [arrt.init_node];
        arrt.upper_bound = actual_cost;
        G = graph;
        G = addnode(G,1);
        arrt.graph = G;
        actual_cost = arrt.growRRT();
    else
        last_index = i-1;
        filename = strcat('Images\Anytime_',num2str(last_index));
        openfig(filename);

        th = 0:pi/50:2*pi;
        xunit = 0.4 * cos(th) + arrt.final_node(1);
        yunit = 0.4 * sin(th) + arrt.final_node(2);
        plot(xunit, yunit);


        for o=arrt.obstacles
            fill(o.x,o.y,[0.64,0.77,1])
        end

        plot(arrt.init_node(1,1),arrt.init_node(1,2),'b^');
        plot(arrt.final_node(1,1),arrt.final_node(1,2),'b^');

        [~, pathR, ~] = graphshortestpath(adjacency(rrt.graph), 1, size(rrt.nodes,1));
        X = [];
        Y = [];
        for m=2:size(pathR,2)
            qpred = rrt.nodes(pathR(m-1),:)';
            qactual = rrt.nodes(path(m),:)';
            [x,y] = rrt.getMovement(qpred,qactual);
            X = [X;x'];
            Y = [Y;y'];
        end
      
        plot(X,Y,'Color',[0.02,0.65,0.8],'LineWidth',0.75);
        h = findobj(gca,'LineWidth',0.75,'-or','LineWidth',0.9);

        legend([h(1),h(2)],{'RRT','AnytimeRRT'});
        print(strcat(filename,'_final'),'-djpeg');
        break
    end

end
end