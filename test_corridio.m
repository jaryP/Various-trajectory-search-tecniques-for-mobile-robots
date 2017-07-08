close all
clear all 

rng('default');
rng(9);

x_min = -2.5;
y_min = -5;
x_max = 2.5;
y_max = 5;
init_conf = [0,-4];
final_conf = [0,4.5];

area = [];

for i=1:15  
    xr = rand*(x_max - x_min) +x_min;
    yr = rand*(y_max - y_min) +y_min;
    if rand <= 0.5
        c = 0.5;
    else
        c = 2.0;
    end
    obj1_x = [.5 0 0 .5 .5]+xr;
    obj1_y = [0 0 1 1 0 ]+yr;
    c = costArea(obj1_x,obj1_y,c);
    area = [area, c];
        
end

rng(42,'simdTwister');

'RRT Star'
rrt = rrtStar(init_conf,final_conf,x_min, x_max,y_min,y_max,[],area,0.7);
'250'
rrt.core(250);
f = figure;
[d,f]=rrt.plot(f,true);
saveas(f,[pwd strcat('/img/RRTS/corridoio_RTTS_250_dist_',num2str(d),'.png')]);close all;

'500'
rrt.core(250);
f = figure;
[d,f]=rrt.plot(f,true);
saveas(f,[pwd strcat('/img/RRTS/corridoio_RTTS_500_dist_',num2str(d),'.png')]);close all;

'2500'
rrt.core(2000);
f = figure;
[d,f]=rrt.plot(f,true);
saveas(f,[pwd strcat('/img/RRTS/corridoio_RTTS_2500_dist_',num2str(d),'.png')]);close all;

'5000'
rrt.core(2500);
f = figure;
[d,f]=rrt.plot(f,true);
saveas(f,[pwd strcat('/img/RRTS/corridoio_RTTS_5000_dist_',num2str(d),'.png')]);close all;


'RRT'
rng(42,'simdTwister');

rrt = rrtB(init_conf,final_conf,x_min, x_max,y_min,y_max,[],area,0.7);
'250'
rrt.core(250);
f = figure;
[d,f]=rrt.plot(f,true);
saveas(f,[pwd strcat('/img/RRT/corridoio_RTT_250_dist_',num2str(d),'.png')]);close all;

'500'
rrt.core(250);
f = figure;
[d,f]=rrt.plot(f,true);
saveas(f,[pwd strcat('/img/RRT/corridoio_RTT_500_dist_',num2str(d),'.png')]);close all;

'2500'
rrt.core(2000);
f = figure;
[d,f]=rrt.plot(f,true);
saveas(f,[pwd strcat('/img/RRT/corridoio_RTT_2500_dist_',num2str(d),'.png')]);close all;

'5000'
rrt.core(2500);
f = figure;
[d,f]=rrt.plot(f,true);
saveas(f,[pwd strcat('/img/RRT/corridoio_RTT_5000_dist_',num2str(d),'.png')]);close all;


