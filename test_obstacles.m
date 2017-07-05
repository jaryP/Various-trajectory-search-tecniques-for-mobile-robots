close all
clear all 
init_conf = [2.5,-1];
final_conf = [4.5,4.5];
x_min = -5;
y_min = -5;
x_max = 5;
y_max = 5;

obst1_x = [-2.5 3 3 -2.5 -2.5];
obst1_y = [1 1 3 3 1];

obst2_x = [3.2 5 5 3.2 3.2];
obst2_y = [1 1 3 3 1];

rng('default');
numNode = 10000;
step = 1000;
skip = numNode/step;
% obstacle = [];

obstacle = [polygon(obst1_x,obst1_y), polygon(obst2_x,obst2_y)];
% area = [costArea(obst3_x,obst3_y,0.1),costArea(obst5_x,obst5_y,0.1),costArea(obst4_x,obst4_y,100)];
% obstacle = [];

rng(28);

rrt = rrtStar(init_conf,final_conf,x_min, x_max,y_min,y_max,obstacle,[],0.7);
rrt.plot(figure,true);
return
rrt.core(10000);
f = figure;
rrt.plot(f,true)
[d1,f] = rrt.plot(f,true);
f.Name = strcat('RRTS:  dist:',num2str(d1));
saveas(f,[pwd strcat('/img/RRTS/OBJ_NEW_RRTS_dist_',num2str(d1),'.png')]);
rng(28);

rrt = rrtB(init_conf,final_conf,x_min, x_max,y_min,y_max,obstacle,[],0.7);

rrt.core(5000);
f = figure;
rrt.plot(f,true)
[d2,f] = rrt.plot(f,true);
f.Name = strcat('RRTS: dist:',num2str(d2));
saveas(f,[pwd strcat('/img/RRT/OBJ_NEW_RRTS_dist_',num2str(d2),'.png')]);
    



