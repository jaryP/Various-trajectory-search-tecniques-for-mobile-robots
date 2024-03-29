close all
clear all 
init_conf = [-1,-2];
final_conf = [4.5,4.5];
x_min = -5;
y_min = -5;
x_max = 5;
y_max = 5;

obst1_x = [-2.5 3 3 -2.5 -2.5];
obst1_y = [1 1 3 3 1];

obst2_x = [2 3 3 2 2];
obst2_y = [-1 -1 3 3 -1];

obst3_x = [3 4 4 3 3];
obst3_y = [-1 -1 0 0 -1];

obst4_x = [-3 -4 -4 -3 -3];
obst4_y = [-2 -2 0 0 -2];

rng('default');

obstacle = [polygon(obst1_x,obst1_y), polygon(obst2_x,obst2_y), polygon(obst3_x,obst3_y)...
    , polygon(obst4_x,obst4_y)];


rng(28);
rrt = rrtStar(init_conf,final_conf,x_min, x_max,y_min,y_max,obstacle,[],0.7);
rrt.core(500);
f = figure;
[d1,f] = rrt.plot(f,true);
f.Name = strcat('RRTS:  dist:',num2str(d1));
saveas(f,[pwd strcat('/img/RRTS/OBJ_1_RRTS_500_dist_',num2str(d1),'.png')]);
rrt.core(500);
f = figure;
[d1,f] = rrt.plot(f,true);
f.Name = strcat('RRTS:  dist:',num2str(d1));
saveas(f,[pwd strcat('/img/RRTS/OBJ_1_RRTS_1000_dist_',num2str(d1),'.png')]);
rrt.core(1000);
f = figure;
[d1,f] = rrt.plot(f,true);
f.Name = strcat('RRTS:  dist:',num2str(d1));
saveas(f,[pwd strcat('/img/RRTS/OBJ_1_RRTS_2000_dist_',num2str(d1),'.png')]);
rrt.core(1000);
f = figure;
[d1,f] = rrt.plot(f,true);
f.Name = strcat('RRTS:  dist:',num2str(d1));
saveas(f,[pwd strcat('/img/RRTS/OBJ_1_RRTS_3000_dist_',num2str(d1),'.png')]);
rrt.core(1000);
f = figure;
[d1,f] = rrt.plot(f,true);
f.Name = strcat('RRTS:  dist:',num2str(d1));
saveas(f,[pwd strcat('/img/RRTS/OBJ_1_RRTS_4000_dist_',num2str(d1),'.png')]);
rrt.core(1000);
f = figure;
[d1,f] = rrt.plot(f,true);
f.Name = strcat('RRTS:  dist:',num2str(d1));
saveas(f,[pwd strcat('/img/RRTS/OBJ_1_RRTS_5000_dist_',num2str(d1),'.png')]);
rrt.core(1000);
f = figure;
[d1,f] = rrt.plot(f,true);
f.Name = strcat('RRTS:  dist:',num2str(d1));
saveas(f,[pwd strcat('/img/RRTS/OBJ_1_RRTS_6000_dist_',num2str(d1),'.png')]);

rng(28);

rrt = rrtB(init_conf,final_conf,x_min, x_max,y_min,y_max,obstacle,[],0.7);
rrt.core(500);
f = figure;
[d1,f] = rrt.plot(f,true);
f.Name = strcat('RRT:  dist:',num2str(d1));
saveas(f,[pwd strcat('/img/RRT/OBJ_1_RRT_500_dist_',num2str(d1),'.png')]);
rrt.core(500);
f = figure;
[d1,f] = rrt.plot(f,true);
f.Name = strcat('RRT:  dist:',num2str(d1));
saveas(f,[pwd strcat('/img/RRT/OBJ_1_RRT_1000_dist_',num2str(d1),'.png')]);
rrt.core(1000);
f = figure;
[d1,f] = rrt.plot(f,true);
f.Name = strcat('RRT:  dist:',num2str(d1));
saveas(f,[pwd strcat('/img/RRT/OBJ_1_RRT_2000_dist_',num2str(d1),'.png')]);
rrt.core(1000);
f = figure;
[d1,f] = rrt.plot(f,true);
f.Name = strcat('RRT:  dist:',num2str(d1));
saveas(f,[pwd strcat('/img/RRT/OBJ_1_RRT_3000_dist_',num2str(d1),'.png')]);
rrt.core(1000);
f = figure;
[d1,f] = rrt.plot(f,true);
f.Name = strcat('RRT:  dist:',num2str(d1));
saveas(f,[pwd strcat('/img/RRT/OBJ_1_RRT_4000_dist_',num2str(d1),'.png')]);
rrt.core(1000);
f = figure;
[d1,f] = rrt.plot(f,true);
f.Name = strcat('RRT:  dist:',num2str(d1));
saveas(f,[pwd strcat('/img/RRT/OBJ_1_RRT_5000_dist_',num2str(d1),'.png')]);
rrt.core(1000);
f = figure;
[d1,f] = rrt.plot(f,true);
f.Name = strcat('RRT:  dist:',num2str(d1));
saveas(f,[pwd strcat('/img/RRT/OBJ_1_RRT_6000_dist_',num2str(d1),'.png')]);




