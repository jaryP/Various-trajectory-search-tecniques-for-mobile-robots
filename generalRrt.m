classdef generalRrt < handle 
    
properties
        graph
        nodes
        boundaries
        init_node
        final_node
        obstacles
        costAreas
        robot
        status
        maxIteration
        goalBias
        lastNode
        nodeDist = [0]
        toleranceRadius = 0.2
        eta = 0.5
        nearToFinalNodes = []
end

methods    
    function obj = generalRrt(q_i,q_f,x_min,x_max,y_min,y_max,obstacles, costAreas, goalBias)

                x_i = q_i(1);
                y_i = q_i(2);

                x_f = q_f(1);
                y_f = q_f(2);

                obj.goalBias = goalBias;

                if(x_i>=x_min)&&(y_i>=y_min)&&(x_i<=x_max)&&(y_f<=y_max)
                    obj.init_node = [x_i,y_i];
                    obj.final_node = [x_f,y_f];
                    obj.lastNode = [];
                    obj.nodes = [obj.init_node];
                    G = digraph;
                    G = addnode(G,1);
                    obj.graph = G;
                    obj.boundaries = [x_min, x_max; y_min, y_max];
                    obj.obstacles = obstacles;
                    obj.costAreas =  costAreas;

                else
                    error('position out of bounds');
                end

    end
    
    function [dist,f] = plot(obj, f, path)
            
            figure(f);

            hold on
            axis([obj.boundaries(1,1) obj.boundaries(1,2) obj.boundaries(2,1) obj.boundaries(2,2)]);

            dist = -1;
            
            for o=obj.obstacles
                fill(o.x,o.y,'b');
            end
            for o=obj.costAreas
                h = fill(o.x,o.y,'m');
                alpha = 1*o.costMul;
                if alpha > 1
                    alpha = 1;
                elseif alpha < 0
                    alpha = 0.1;
                end
                set(h,'facealpha',alpha);
            end

            [i,j,~] = find(adjacency(obj.graph));
            
            for k=1:size(i)
                [x,y] = getMovement(obj,obj.nodes(i(k),:)',obj.nodes(j(k),:)');
                plot(x,y,'k');
            end

            plot(obj.init_node(1),obj.init_node(2),'o');
            
            if path
                [dist, path] =  obj.getEndPath();
                th = 0:pi/50:2*pi;
                xunit = obj.toleranceRadius* cos(th) + obj.final_node(1);
                yunit = obj.toleranceRadius * sin(th) + obj.final_node(2);
                plot(xunit, yunit);
                for k=2:size(path,2)
                    qpred = obj.nodes(path(k-1),:)';
                    qactual = obj.nodes(path(k),:)';
                    [x,y] = getMovement(obj,qpred,qactual);
                    plot(x,y,'r');    
                end
            end
            
            hold off
    end
    
    function [x,y] = getMovement(~,startConfig, endConfig)
            x = [startConfig(1) endConfig(1)];
            y = [startConfig(2)  endConfig(2)];
        end  
        
    function [dist, path] = getEndPath(obj)
            
            minDist = inf(1);
            dist = 0;
            path = [];    
        
            nn = numnodes(obj.graph);
            [s,t] = findedge(obj.graph);
            
            if isempty(s)  || isempty(t)
                return
            end
            
            A = sparse(s,t,obj.graph.Edges.Weight,nn,nn);   
            
            for i=1:size(obj.nearToFinalNodes,1)
                
                nodo = obj.nodes(obj.nearToFinalNodes(i),:);
                [d, p, ~] = graphshortestpath(A, 1, obj.nearToFinalNodes(i));
                d = d  + norm(obj.final_node-nodo);
                
                if d <= minDist
                    minDist = d;
                    dist = d;
                    path = p;
                end
            end
    end
    
    function dist = getDist(obj,goal)
            
            if size(obj.nodeDist,2) >= goal
                dist = obj.nodeDist(goal);
                return
            end
            
            xparent = predecessors(obj.graph,goal);
            obj.nodeDist
            nn = numnodes(obj.graph);
            [s,t] = findedge(obj.graph);
            A = sparse(s,t,obj.graph.Edges.Weight,nn,nn);   
            [dist, ~, ~] = graphshortestpath(A, 1, goal);
            obj.nodeDist = [obj.nodeDist, dist];

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

            index = obj.getNodeIndex(qnear);
            indexN = obj.getNodeIndex(qnew);
            
            if indexN ~= -1
                return
            end
            
            [x,y] = obj.getMovement(qnew,qnear);
            w = obj.getWeightMul(x,y);
            n = norm(qnew-qnear) * w;
            obj.nodes =  [obj.nodes; qnew'];
            newIndex = size(obj.nodes,1);
            obj.graph = addnode(obj.graph,newIndex);
            obj.graph = addedge(obj.graph,index,newIndex,n);
            
            if norm(qnew' - obj.final_node) <= obj.toleranceRadius
                obj.nearToFinalNodes = [obj.nearToFinalNodes, newIndex];
            end
             
        end
            
    function newNode = steer(obj,xRand, xNear)

            diff = xRand - xNear;
            n = norm(diff);
            
            if n >= obj.eta
                diff = diff * (obj.eta/n);
            end
            
            x = xNear(1) +diff(1);
            y = xNear(2) +diff(2);

            newNode = [x;y];

    end
    
    function W = sampleFree(obj)
 
            if rand >= obj.goalBias
                W = obj.final_node;
                return
            end
            counter = 0;

            while(counter<500)
                x = rand * (obj.boundaries(1,1) -obj.boundaries(1,2)) + obj.boundaries(1,2);
                y = rand * (obj.boundaries(2,1) -obj.boundaries(2,2)) + obj.boundaries(2,2);
                W = [x y];
                
                if isempty(obj.obstacles)
                    return
                end
                
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
                
                if cart_dist < min_dist
                    min_dist = cart_dist;
                    nearestNode = obj.nodes(i,:);
                end
            end
   end
   
   function w = getWeightMul(obj,x,y)
            w = 1;
             for i=obj.costAreas
                [x_coll,y_coll] = i.intersect(x,y);
                if ~isempty([x_coll,y_coll])
                    w = w*i.costMul;
                end
            end;
   end
   
   function index = getNodeIndex(obj,node)
    index = -1;
    for i=1:size(obj.nodes(),1)
        if obj.nodes(i,1) == node(1) && obj.nodes(i,2) == node(2)
            index = i;
            return
        end
    end
   end
end
end