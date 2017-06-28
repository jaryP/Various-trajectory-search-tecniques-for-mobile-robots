classdef RRT < handle
    properties
        graph
        nodes
        boundaries
        init_node
        final_node
        obstacles
        robot
        status
        maxIteration
        goalBias
        lastNode
    end
    
    methods
        %costruttore. Parametri: configurazioni iniziale e finale del
        %robot, limiti del configuration space, array di ostacoli, robot
        function obj = RRT(q_i,q_f,x_min,x_max,y_min,y_max,obstacles,robot,maxIteration, goalBias)
                        
            x_i = q_i(1);
            y_i = q_i(2);
            teta_i = q_i(3);
            
            x_f = q_f(1);
            y_f = q_f(2);
            teta_f = q_f(3);
            
            obj.maxIteration = maxIteration;
            obj.goalBias = goalBias;
            
            if(x_i>=x_min)&&(y_i>=y_min)&&(x_i<=x_max)&&(y_f<=y_max)
                obj.init_node = [x_i,y_i,teta_i];
                obj.final_node = [x_f,y_f,teta_f];
                
                nodes = [obj.init_node];
                
                obj.lastNode = [];
                obj.nodes = nodes;
                G = digraph;
                G = addnode(G,1);
                obj.graph = G;
                obj.boundaries = [x_min, x_max; y_min, y_max];
                obj.obstacles = obstacles;
                obj.robot = robot;
                
            else
                error('position out of bounds');
            end
        end
        function obj = plot(obj)
            hold on
            
            th = 0:pi/50:2*pi;
            xunit = 0.4 * cos(th) + obj.final_node(1);
            yunit = 0.4 * sin(th) + obj.final_node(2);
            plot(xunit, yunit);
            
            for o=obj.obstacles
                plot(o.x,o.y,'b')
            end
            if strcmp(obj.status,'reached')
                
                [dist, path, pred] = graphshortestpath(adjacency(obj.graph), 1, size(obj.nodes,1));
                
                for k=2:size(path,2)
                    qpred = obj.nodes(path(k-1),:)';
                    qactual = obj.nodes(path(k),:)';
                    [x,y] = getMovement(obj,qpred,qactual);
                    plot(x,y);
                

                    x = [0 0 0.1 0];
                    y = [-0.05 +0.05 0 -0.05 ];
                    R = [cos(qactual(3)) -sin(qactual(3)); sin(qactual(3)), cos(qactual(3))];
                    rot = [x' y']*R';
                    rot = rot + [qactual(1) qactual(2);qactual(1) qactual(2);qactual(1) qactual(2);qactual(1) qactual(2)];
                    fill(rot(:,1),rot(:,2),'r');
                    
                end
            else
                
                [i,j,s] = find(adjacency(obj.graph));
                for k=1:size(i)
                    [x,y] = getMovement(obj,obj.nodes(i(k),:)',obj.nodes(j(k),:)');
                    plot(x,y);
                end
            
            end
            
            x = [0 0 0.1 0];
            y = [-0.05 +0.05 0 -0.05 ];
            R = [cos(obj.init_node(3)) -sin(obj.init_node(3)); sin(obj.init_node(3)), cos(obj.init_node(3))];
            rot = [x' y']*R';
            rot = rot + [obj.init_node(1) obj.init_node(2);obj.init_node(1) obj.init_node(2);obj.init_node(1) obj.init_node(2);obj.init_node(1) obj.init_node(2)];
            fill(rot(:,1),rot(:,2),'r');
            
            x = [0 0 0.1 0];
            y = [-0.05 +0.05 0 -0.05 ];
            R = [cos(obj.final_node(3)) -sin(obj.final_node(3)); sin(obj.final_node(3)), cos(obj.final_node(3))];
            rot = [x' y']*R';
            rot = rot + [obj.final_node(1) obj.final_node(2);obj.final_node(1) obj.final_node(2);obj.final_node(1) obj.final_node(2);obj.final_node(1) obj.final_node(2)];
            fill(rot(:,1),rot(:,2),'r');
            hold off
        end
        
        function [x,y] = getMovement(obj,startConfig, endConfig)
   
            angleDiff = wrapToPi(endConfig(3)-startConfig(3));

            if angleDiff == 0
                x = [startConfig(1)  endConfig(1)];
                y = [startConfig(2)  endConfig(2)];
                return
            end
            
            th = 0:pi/50:pi/2;  
            
            if abs(wrapToPi(startConfig(3))) == 0 
                xC = startConfig(1);
                yC = endConfig(2);
                diff = endConfig-[xC ; yC; 0];
                diff = diff(1:2);
                diff = norm(diff);
                
                if angleDiff >0
                    x = diff*cos(th) + xC;
                    y = diff*-sin(th)+yC ;
                else
                    x = diff*cos(th) + xC;
                    y = diff*sin(th)+yC ;
                    
                end   

            elseif abs(wrapToPi(startConfig(3))) == pi 
                xC = startConfig(1);
                yC = endConfig(2);
                diff = endConfig-[xC ; yC; 0];
                diff = diff(1:2);
                diff = norm(diff);
                
                if angleDiff >0
                    x = diff*-cos(th) + xC;
                    y = diff*sin(th)+yC ;
                else
                    x = diff*-cos(th) + xC;
                    y = diff*-sin(th)+yC ;

                end
            elseif wrapToPi(startConfig(3)) == pi/2
            %%DA FARE DEBUG
                    xC = endConfig(1);
                    yC = startConfig(2);
                    diff = endConfig-[xC ; yC; 0];
                    diff = diff(1:2);
                    diff = norm(diff);
                 if angleDiff > 0
                    x = diff*cos(th) + xC;
                    y = diff*sin(th)+yC ;
                 else
                    x = diff*-cos(th) + xC;
                    y = diff*sin(th)+yC ;  
                 end
                 
            elseif wrapToPi(startConfig(3)) == -pi/2
                    xC = endConfig(1);
                    yC = startConfig(2);
                    diff = endConfig-[xC ; yC; 0];
                    diff = diff(1:2);
                    diff = norm(diff);
                 if angleDiff > 0
                    x = diff*-cos(th) + xC;
                    y = diff*-sin(th)+yC ;
                 else
                    x = diff*cos(th) + xC;
                    y = diff*-sin(th)+yC ; 
                 end
            end
            
          
        end  
        
        % estrae una configurazione casuale, non in collisione con gli
        % ostacoli
        function W = sampleFree(obj)
 
            if rand >= obj.goalBias
                W = obj.final_node;
                return
            end
            
            counter = 0;

            while(counter<1000)
                W = obj.robot.getRandomConfig(obj.boundaries(1,:),obj.boundaries(2,:));
                for o=obj.obstacles
                    if(~isempty(o.intersect(W(1),W(2))))
                        counter = counter +1;
                    else
                        return;
                    end
                end
            end
          obj.status = 'trapped';
          error('can not find free random config');
        end
        
        % restituisce il nodo dell'albero più vicino alla configurazione
        % casuale passata come parametro
        function nearestNode = getNearestConf(obj,X_random)
            min_dist = Inf(1);
            for i=1:size(obj.nodes(),1)
                cart_dist = norm( X_random(1:2)-obj.nodes(i,1:2));
                %ang_dist = min(abs(q_r-obj.nodes(i,3)) , 2*pi - abs(q_r-obj.nodes(i,3)));
                %total_dist = cart_dist + ang_dist;
                total_dist = cart_dist;
                if total_dist < min_dist
                    min_dist = total_dist;
                    nearestNode = obj.nodes(i,:);
                end
            end
        end
        
        function collisionFlag = obstacleFree(obj,x,y)
            collisionFlag = 0;
            for i=obj.obstacles
                [x_coll,y_coll] = i.intersect(x,y);
                if ~isempty([x_coll,y_coll])
                    collisionFlag = 1;
                    break
                end
            end
            collisionFlag  = ~collisionFlag;
        end

        
        function obj = addNewNode(obj, qnew, qnear)

            [tf, index]=ismember(obj.nodes,qnear','rows');
            index = find(index);
            
            [tfN, indexN]=ismember(obj.nodes,qnew','rows');
            
            if sum(indexN) > 0
                return
            end

            obj.nodes =  [obj.nodes; qnew'];
            newIndex = size(obj.nodes,1);
            obj.graph = addnode(obj.graph,newIndex);
            obj.graph = addedge(obj.graph,index,newIndex);
                
            %disp(obj.graph)
        end
            
        function newNode = steer(obj,xRand, xNear)
            
            configurations = obj.robot.directKinematics(xNear);
            
            minDist = Inf(1);
            
            for c = configurations
                dist = norm(c(1:2)-xRand(1:2));
                if dist < minDist
                    minDist = dist;
                    newNode = c;
                end
            end
        end
        
        function [] = run(obj)
            x_start = obj.init_node;

            x_end = obj.final_node;
            
            if(all(x_start == x_end))
                obj.nodes
            else
                for i=1:obj.maxIteration
                    
                    x_rand = obj.sampleFree();
                    if(~isempty(x_rand))
                        
                        x_nearest = obj.getNearestConf(x_rand);
                        x_new = obj.steer(x_rand',x_nearest');
                        [x,y] = obj.getMovement(x_nearest,x_new);
                       
                        if obj.obstacleFree(x,y)
                            obj.addNewNode(x_new,x_nearest');
                        end

                        diff = x_new - obj.final_node';
                        diff = diff(1:2);
                        cart_dist =  norm(diff);
                        %ang_dist = min(abs(obj.final_node(3)-x_new(3)), 2*pi-abs(obj.final_node(3)-x_new(3)));
     
                        if cart_dist <= 0.4
                            obj.status = 'reached';
                            obj.lastNode = x_new;
                            break
                        end
                            
                        
                        %plot(x_new(1),x_new(2),'^');
                        %obj.nodes
   
                    %leaves(obj.graph)
                    end
                obj.status = 'timeRunOut';
                
            end
        end
    end
    end
end