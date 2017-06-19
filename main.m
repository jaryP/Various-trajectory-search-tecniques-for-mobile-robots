init_conf = [0,0,0];
final_conf = [10,10,0];
x_min = -5;
y_min = -5;
x_max = 15;
y_max = 15;
robot = unicycle;
obst1_x = [8 12 8 12];
obst1_y = [3 3 7 7];
obstacle = polygon(obst1_x,obst1_y);

rrt = RRT(init_conf,final_conf,x_min, x_max,y_min,y_max,obstacle,robot);
rrt.launch();