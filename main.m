close all
clear all 
init_conf = [2.5,-1,pi];
final_conf = [4.5,4.5,0];
x_min = -5;
y_min = -5;
x_max = 5;
y_max = 5;
% robot = unicycle;

obst1_x = [-2.5 2 2 -2.5 -2.5];
obst1_y = [1 1 3 3 1];

obst2_x = [1 2 2 1 1];
obst2_y = [0 1 1 0 0];

obst3_x = [3 4 4 3 3 ];
obst3_y = [-2 -2  2 2 -2];

obst4_x = [2 3 3 2 2 ];
obst4_y = [0 0  2 2 0];

obst5_x = [-5 -2 -2 -5 -5 ];
obst5_y = [-2 -2  -1 -1 -2];

% axis([x_min x_max y_min y_max]);
rng('default');
numNode = 10000;
step = 1000;
skip = numNode/step;
% obstacle = [];
% area = [];
obstacle = [polygon(obst1_x,obst1_y)];
area = [costArea(obst3_x,obst3_y,0.1),costArea(obst5_x,obst5_y,0.1),costArea(obst4_x,obst4_y,100)];

% obstacle = [];

rrt = rrtStar(init_conf,final_conf,x_min, x_max,y_min,y_max,obstacle,area,1000,0.80);
sum = 0;
rng(7);

for i=1:skip
    f = figure;
    sum = sum + step
    rrt.core(step);
    [d1,f] = rrt.plot(f,true);
    d1
    f.Name = strcat('RRTS: ',num2str(sum),' dist:',num2str(d1));
    return
    saveas(f,[pwd strcat('/img/RRTS/alpha_RRTS_',num2str(sum),'_dist_',num2str(d1),'.png')]);
    close all
end

rrt = Copy_of_rrt(init_conf,final_conf,x_min, x_max,y_min,y_max, obstacle,area,1000,0.7);
rng(1);
sum = 0;

for i=1:skip
    f = figure;
    set(f,'visible','off');
    sum = sum + step
    rrt.core(step);
    [d1,f] = rrt.plot(f,true);
    d1
    f.Name = strcat('RRT: ',num2str(sum),' dist:',num2str(d1));
    saveas(f,[pwd strcat('/img/RRT/RRT_',num2str(sum),'_dist_',num2str(d1),'.png')]);
    close all
end
return


