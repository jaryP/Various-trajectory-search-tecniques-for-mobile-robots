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


obstacle = [polygon(obst1_x,obst1_y), polygon(obst2_x,obst2_y)];

rng(28);


rng(28);
rrt = rrtStar(init_conf,final_conf,x_min, x_max,y_min,y_max,obstacle,[],0.7);
rrt.core(500);
f = figure;
[d1,f] = rrt.plot(f,true);
f.Name = strcat('RRTS:  dist:',num2str(d1));
saveas(f,[pwd strcat('/img/RRTS/OBJ_RRTS_500_dist_',num2str(d1),'.png')]);
rrt.core(500);
f = figure;
[d1,f] = rrt.plot(f,true);
f.Name = strcat('RRTS:  dist:',num2str(d1));
saveas(f,[pwd strcat('/img/RRTS/OBJ_RRTS_1000_dist_',num2str(d1),'.png')]);
rrt.core(1000);
f = figure;
[d1,f] = rrt.plot(f,true);
f.Name = strcat('RRTS:  dist:',num2str(d1));
saveas(f,[pwd strcat('/img/RRTS/OBJ_RRTS_2000_dist_',num2str(d1),'.png')]);
rrt.core(1000);
f = figure;
[d1,f] = rrt.plot(f,true);
f.Name = strcat('RRTS:  dist:',num2str(d1));
saveas(f,[pwd strcat('/img/RRTS/OBJ_RRTS_3000_dist_',num2str(d1),'.png')]);
rrt.core(1000);
f = figure;
[d1,f] = rrt.plot(f,true);
f.Name = strcat('RRTS:  dist:',num2str(d1));
saveas(f,[pwd strcat('/img/RRTS/OBJ_RRTS_4000_dist_',num2str(d1),'.png')]);
rrt.core(1000);
f = figure;
[d1,f] = rrt.plot(f,true);
f.Name = strcat('RRTS:  dist:',num2str(d1));
saveas(f,[pwd strcat('/img/RRTS/OBJ_RRTS_5000_dist_',num2str(d1),'.png')]);
rrt.core(1000);
f = figure;
[d1,f] = rrt.plot(f,true);
f.Name = strcat('RRTS:  dist:',num2str(d1));
saveas(f,[pwd strcat('/img/RRTS/OBJ_RRTS_6000_dist_',num2str(d1),'.png')]);

rng(28);

rrt = rrtB(init_conf,final_conf,x_min, x_max,y_min,y_max,obstacle,[],0.7);
rrt.core(500);
f = figure;
[d1,f] = rrt.plot(f,true);
f.Name = strcat('RRT:  dist:',num2str(d1));
saveas(f,[pwd strcat('/img/RRT/OBJ_RRT_500_dist_',num2str(d1),'.png')]);
rrt.core(500);
f = figure;
[d1,f] = rrt.plot(f,true);
f.Name = strcat('RRT:  dist:',num2str(d1));
saveas(f,[pwd strcat('/img/RRT/OBJ_RRT_1000_dist_',num2str(d1),'.png')]);
rrt.core(1000);
f = figure;
[d1,f] = rrt.plot(f,true);
f.Name = strcat('RRT:  dist:',num2str(d1));
saveas(f,[pwd strcat('/img/RRT/OBJ_RRT_2000_dist_',num2str(d1),'.png')]);
rrt.core(1000);
f = figure;
[d1,f] = rrt.plot(f,true);
f.Name = strcat('RRT:  dist:',num2str(d1));
saveas(f,[pwd strcat('/img/RRT/OBJ_RRT_3000_dist_',num2str(d1),'.png')]);
rrt.core(1000);
f = figure;
[d1,f] = rrt.plot(f,true);
f.Name = strcat('RRT:  dist:',num2str(d1));
saveas(f,[pwd strcat('/img/RRT/OBJ_RRT_4000_dist_',num2str(d1),'.png')]);
rrt.core(1000);
f = figure;
[d1,f] = rrt.plot(f,true);
f.Name = strcat('RRT:  dist:',num2str(d1));
saveas(f,[pwd strcat('/img/RRT/OBJ_RRT_5000_dist_',num2str(d1),'.png')]);
rrt.core(1000);
f = figure;
[d1,f] = rrt.plot(f,true);
f.Name = strcat('RRT:  dist:',num2str(d1));
saveas(f,[pwd strcat('/img/RRT/OBJ_RRT_6000_dist_',num2str(d1),'.png')]);




