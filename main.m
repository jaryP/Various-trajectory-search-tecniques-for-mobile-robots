close

init_conf = [0,0,0];
final_conf = [4.5,0,0];
x_min = -5;
y_min = -5;
x_max = 5;
y_max = 5;
robot = unicycle;

obst1_x = [-2.5 -1 -1 -2.5 -2.5];
obst1_y = [1 2 3 3 1];

obst2_x = [1 2 2 1 1];
obst2_y = [1 0 4 1 1];

obst3_x = [3 4 4 3 3 ];
obst3_y = [-4 -4  1 1 -4];

axis([x_min x_max y_min y_max]);

obstacle = [polygon(obst1_x,obst1_y),polygon(obst2_x,obst2_y), polygon(obst3_x,obst3_y)];

rrt = RRT(init_conf,final_conf,x_min, x_max,y_min,y_max,obstacle,robot,1000,0.6);
rrt.run();
rrt.plot();
