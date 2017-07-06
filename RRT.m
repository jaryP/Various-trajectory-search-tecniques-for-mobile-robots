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
        delta
    end
    
    methods
        %costruttore. Parametri: configurazioni iniziale e finale del
        %robot, limiti del configuration space, array di ostacoli, robot
        function obj = RRT(q_i,q_f,x_min,x_max,y_min,y_max,obstacles,maxIteration, goalBias)
            
            x_i = q_i(1);
            y_i = q_i(2);
            x_f = q_f(1);
            y_f = q_f(2);
            
            obj.maxIteration = maxIteration;
            obj.goalBias = goalBias;
            
            if(x_i>=x_min)&&(y_i>=y_min)&&(x_i<=x_max)&&(y_f<=y_max)
                obj.init_node = [x_i,y_i];
                obj.final_node = [x_f,y_f];
                
                nodes = [obj.init_node];
                
                obj.lastNode = [];
                obj.nodes = nodes;
                G = digraph;
                G = addnode(G,1);
                obj.graph = G;
                obj.boundaries = [x_min, x_max; y_min, y_max];
                obj.obstacles = obstacles;
                obj.delta = 0.2;
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
                
                [~, path, ~] = graphshortestpath(adjacency(obj.graph), 1, size(obj.nodes,1));
                
                for k=2:size(path,2)
                    qpred = obj.nodes(path(k-1),:)';
                    qactual = obj.nodes(path(k),:)';
                    [x,y] = getMovement(obj,qpred,qactual);
                    plot(x,y,'m');                      
                end
            else
                
                [i,j,~] = find(adjacency(obj.graph));
                for k=1:size(i)
                    [x,y] = getMovement(obj,obj.nodes(i(k),:)',obj.nodes(j(k),:)');
                    plot(x,y,'b');
                end
            end
            
        end
        
        function [x,y] = getMovement(~,startConfig, endConfig)
            x = [startConfig(1)  endConfig(1)];
            y = [startConfig(2)  endConfig(2)];
            return
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
               x = rand * (obj.init_node(1,1) - obj.final_node(1,1)) + obj.final_node(1,1);
               y = rand * (obj.boundaries(2,1) - obj.boundaries(2,2)) + obj.boundaries(2,2);
               W = [x, y];
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
                total_dist = norm(X_random(1:2)-obj.nodes(i,1:2));
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
       
            [~, index]=ismember(obj.nodes,qnear','rows');
            index = find(index);
            
            [~, indexN]=ismember(obj.nodes,qnew','rows');
            
            if sum(indexN) > 0
                return
            end
            
            obj.nodes =  [obj.nodes; qnew'];
            newIndex = size(obj.nodes,1);
            obj.graph = addnode(obj.graph,newIndex);
            obj.graph = addedge(obj.graph,index,newIndex);
        end
        
        function inters = solveIntersection(~,x1,x2,y1,y2,cx,cy,r)
            x = [x1 x2];
            y = [y1 y2];
            c = [[1;1] x(:)]\y(:);
            slope = c(2);
            intercept = c(1);
            [x,y] = linecirc(slope,intercept,cx,cy,r);
            inters = [x(1),y(1);x(2),y(2)];
        end
        
        function newNode = steer(obj,qgoal,qact)
            intersections = obj.solveIntersection(qact(1),qgoal(1),qact(2),qgoal(2),qact(1),qact(2),obj.delta);
            point_a = intersections(1,:)';
            point_b = intersections(2,:)';
            
            if(norm(point_a-qgoal(1:2))<norm(point_b-qgoal(1:2)))
                newNode = double(intersections(1,:))';
                return
            else
                newNode = double(intersections(2,:))';
                return
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
                       
                        if cart_dist <= 0.4
                            obj.status = 'reached';
                            obj.lastNode = x_new;
                            break
                        end
                    end
                    obj.status = 'timeRunOut';
                    
                end
            end
        end
    end
end