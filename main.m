close
init_conf = [-3,4,0];
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

hold on
axis([x_min x_max y_min y_max]);

plot(obst1_x,obst1_y);
plot(obst2_x,obst2_y);
plot(obst3_x,obst3_y);

obstacle = [polygon(obst1_x,obst1_y),polygon(obst2_x,obst2_y), polygon(obst3_x,obst3_y)];

rrt = RRT(init_conf,final_conf,x_min, x_max,y_min,y_max,obstacle,robot,1000,0.6);
rrt.run();

x = [0 0 0.1 0];
y = [-0.05 +0.05 0 -0.05 ];
R = [cos(0) -sin(0); sin(0), cos(0)];
rot = [x' y']*R';
rot = rot + [init_conf(1) init_conf(2)  ];
fill(rot(:,1),rot(:,2),'r');

plot(init_conf(1),init_conf(2),'o');
plot(final_conf(1),final_conf(2),'+');

clear all