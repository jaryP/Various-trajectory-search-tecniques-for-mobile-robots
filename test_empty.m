close all 

init_conf = [0,0];
final_conf = [2.5,2.5];

rng('default');
rng(42);

x_min = -3;
y_min = -3;
x_max = 3;
y_max = 3;

'RRT Star'
rrt = rrtStar(init_conf,final_conf,x_min, x_max,y_min,y_max,[],[],1.1);
'250'
rrt.core(250);
f = figure;
[d,f]=rrt.plot(f,true);
saveas(f,[pwd strcat('/img/RRTS/empty_250_dist_',num2str(d),'.png')]);close all;

'500'
rrt.core(250);
f = figure;
[d,f]=rrt.plot(f,true);
saveas(f,[pwd strcat('/img/RRTS/empty_500_dist_',num2str(d),'.png')]);close all;

'2500'
rrt.core(2000);
f = figure;
[d,f]=rrt.plot(f,true);
saveas(f,[pwd strcat('/img/RRTS/empty_2500_dist_',num2str(d),'.png')]);close all;

'5000'
rrt.core(2500);
f = figure;
[d,f]=rrt.plot(f,true);
saveas(f,[pwd strcat('/img/RRTS/empty_5000_dist_',num2str(d),'.png')]);close all;


'10000'
rrt.core(5000);
f = figure;
[d,f]=rrt.plot(f,true);
saveas(f,[pwd strcat('/img/RRTS/empty_10000_dist_',num2str(d),'.png')]);close all;

% '20000'
% rrt.core(10000);
% f = figure;
% [d,f]=rrt.plot(f,true);
% saveas(f,[pwd strcat('/img/RRTS/empty_20000_dist_',num2str(d),'.png')]);close all;


'RRT'
rng(42);

rrt = rrtB(init_conf,final_conf,x_min, x_max,y_min,y_max,[],[],1.1);
'250'
rrt.core(250);
f = figure;
[d,f]=rrt.plot(f,true);
saveas(f,[pwd strcat('/img/RRT/empty_250_dist_',num2str(d),'.png')]);close all;

'500'
rrt.core(250);
f = figure;
[d,f]=rrt.plot(f,true);
saveas(f,[pwd strcat('/img/RRT/empty_500_dist_',num2str(d),'.png')]);close all;

'2500'
rrt.core(2000);
f = figure;
[d,f]=rrt.plot(f,true);
saveas(f,[pwd strcat('/img/RRT/empty_2500_dist_',num2str(d),'.png')]);close all;

'5000'
rrt.core(2500);
f = figure;
[d,f]=rrt.plot(f,true);
saveas(f,[pwd strcat('/img/RRT/empty_5000_dist_',num2str(d),'.png')]);close all;


'10000'
rrt.core(5000);
f = figure;
[d,f]=rrt.plot(f,true);
saveas(f,[pwd strcat('/img/RRT/empty_10000_dist_',num2str(d),'.png')]);close all;

% '20000'
% rrt.core(10000);
% f = figure;
% [d,f]=rrt.plot(f,true);
% saveas(f,[pwd strcat('/img/RRT/empty_20000_dist_',num2str(d),'.png')]);close all;


