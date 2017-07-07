close all
clear all 

obj1_x = [-5 -1 -1 -5 -5 ];
obj1_y = [5 5  -5 -5 5];
obj2_x = [5 1 1 5 5 ];
obj2_y = [5 5  -5 -5 5];
obstacles =[polygon(obj1_x,obj1_y),polygon(obj2_x,obj2_y)];

obj1_x = [-1 0 0 -1 -1];
obj1_y = [0 0 1 1 0 ];
obj2_x = [0.5 0 0 0.5 0.5 ];
obj2_y = [0 0 1 1 0 ];

rng(728)

x_min = -2;
y_min = -5;
x_max = 2;
y_max = 5;

area = [];
for i=1:20
    xr = rand*(x_max - x_min) +x_min;
    yr = rand*(y_max - y_min) +y_min;
    if rand <= 0.5
        c = 0.5;
    else
        c = 2.0;
    end
    obj1_x = [xr-0.5 xr+0.5 xr+0.5 xr-0.5 xr-0.5];
    obj1_y = [yr-0.5 yr-0.5 yr+0.5 yr+0.5 yr-0.5 ];

    area = [area, costArea(obj1_x,obj1_y,c)];
        
end
    
% area = [costArea(obj1_x,obj1_y,0.2),costArea(obj2_x,obj2_y,2)];
init_conf = [0,-4];
final_conf = [0,4.5];
rng('default');

rng(17)


iterations = 15000;

for i=1:5
    'iterazione'
    i
    rrt = rrtStar(init_conf,final_conf,x_min, x_max,y_min,y_max,[],area,0.7);
    rrt.core(100);
    [d,~]= rrt.getEndPath();
    mediaStar500 = mediaStar500 + d;
%     
%     %1000
%     '1000'
%     rrt.core(500);
%     [d,~]= rrt.getEndPath();
%     mediaStar1000 = mediaStar1000 + d;
%     %2000
%     '2000'
%     rrt.core(1000);
%     [d,~]= rrt.getEndPath();
%     mediaStar2000 = mediaStar2000 + d;
%     4000
%     '4000'
%     rrt.core(2000);
%     [d,~]= rrt.getEndPath();
%     mediaStar4000 = mediaStar4000 + d;
%     %6000
%     '6000'
%     rrt.core(2000);
%     [d,~]= rrt.getEndPath();
%     mediaStar6000 = mediaStar6000 + d;
%     %8000
%     '8000'
%     rrt.core(2000);
%     [d,~]= rrt.getEndPath();
%     mediaStar8000 = mediaStar8000 + d;
% %     %10000
%     '10000'
%     rrt.core(2000);
%     [d,~]= rrt.getEndPath();
%     mediaStar10000 = mediaStar10000 + d;
    f = figure;
    [d,f] = rrt.plot(f,true);
    return
    saveas(f,[pwd strcat('/img/RRTS/corridoio_RRTS_',num2str(i),'_dist_',num2str(d),'.png')]);
    close all
end
mediaStar500 = mediaStar500/5
mediaStar1000 = mediaStar1000/5
mediaStar2000 = mediaStar2000/5
mediaStar4000 = mediaStar4000/5
mediaStar6000 = mediaStar6000/5
mediaStar8000 = mediaStar8000/5
mediaStar10000 = mediaStar10000/5


rng(17);


mediaRRT500 = 0;
mediaRRT1000 = 0;
mediaRRT2000 = 0;
mediaRRT4000 = 0;
mediaRRT6000 = 0;
mediaRRT8000 = 0;
mediaRRT10000 = 0;
mediaRRT12000 = 0;
mediaRRT15000 = 0;

for i=1:5

    'iterazione'
    i
    rrt = rrtB(init_conf,final_conf,x_min, x_max,y_min,y_max,obstacles,area,0.7);
    rrt.core(500);
    [d,~]= rrt.getEndPath();
    mediaRRT500 = mediaRRT500 + d;
    %1000
    '1000'
    rrt.core(500);
    [d,~]= rrt.getEndPath();
    mediaRRT1000 = mediaRRT1000 + d;
    %2000
    '2000'
    rrt.core(1000);
    [d,~]= rrt.getEndPath();
    mediaRRT2000 = mediaRRT2000 + d;
    %4000
    '4000'
    rrt.core(2000);
    [d,~]= rrt.getEndPath();
    mediaRRT4000 = mediaRRT4000 + d;
    %6000
    '6000'
    rrt.core(2000);
    [d,~]= rrt.getEndPath();
    mediaRRT6000 = mediaRRT6000 + d;
    %8000
    '8000'
    rrt.core(2000);
    [d,~]= rrt.getEndPath();
    mediaRRT8000 = mediaRRT8000 + d;
    %10000
    '10000'
    rrt.core(2000);
    [d,~]= rrt.getEndPath();
    mediaRRT10000 = mediaRRT10000 + d;
    f = figure;
    [d,f] = rrt.plot(f,true);
    saveas(f,[pwd strcat('/img/RRT/corridoio_RRT_',num2str(i),'_dist_',num2str(d),'.png')]);
    close all
end
mediaRRT500 = mediaRRT500/5
mediaRRT1000 = mediaRRT1000/5
mediaRRT2000 = mediaRRT2000/5
mediaRRT4000 = mediaRRT4000/5
mediaRRT6000 = mediaRRT6000/5
mediaRRT8000 = mediaRRT8000/5
mediaRRT10000 = mediaRRT10000/5

