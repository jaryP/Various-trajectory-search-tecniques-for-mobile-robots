close all 

init_conf = [0,0];
final_conf = [4.5,4.5];

rng('default');
rng(42,'simdTwister');

x_min = -5;
y_min = -5;
x_max = 5;
y_max = 5;


'RRT Star'
rrt = rrtStar(init_conf,final_conf,x_min, x_max,y_min,y_max,[],[],1.1);
'250'
rrt.core(250);
f = figure;
[d,f]=rrt.plot(f,true);
saveas(f,[pwd strcat('/img/RRTS/empty_RTTS_250_dist_',num2str(d),'.png')]);close all;

'500'
rrt.core(250);
f = figure;
[d,f]=rrt.plot(f,true);
saveas(f,[pwd strcat('/img/RRTS/empty_RTTS_500_dist_',num2str(d),'.png')]);close all;

'2500'
rrt.core(2000);
f = figure;
[d,f]=rrt.plot(f,true);
saveas(f,[pwd strcat('/img/RRTS/empty_RTTS_2500_dist_',num2str(d),'.png')]);close all;

'5000'
rrt.core(2500);
f = figure;
[d,f]=rrt.plot(f,true);
saveas(f,[pwd strcat('/img/RRTS/empty_RTTS_5000_dist_',num2str(d),'.png')]);close all;


'RRT'
rng(42,'simdTwister');

rrt = rrtB(init_conf,final_conf,x_min, x_max,y_min,y_max,[],[],1.1);
'250'
rrt.core(250);
f = figure;
[d,f]=rrt.plot(f,true);
saveas(f,[pwd strcat('/img/RRT/empty_RRT_250_dist_',num2str(d),'.png')]);close all;

'500'
rrt.core(250);
f = figure;
[d,f]=rrt.plot(f,true);
saveas(f,[pwd strcat('/img/RRT/empty_RRT_500_dist_',num2str(d),'.png')]);close all;

'2500'
rrt.core(2000);
f = figure;
[d,f]=rrt.plot(f,true);
saveas(f,[pwd strcat('/img/RRT/empty_RRT_2500_dist_',num2str(d),'.png')]);close all;

'5000'
rrt.core(2500);
f = figure;
[d,f]=rrt.plot(f,true);
saveas(f,[pwd strcat('/img/RRT/empty_RRT_5000_dist_',num2str(d),'.png')]);close all;


