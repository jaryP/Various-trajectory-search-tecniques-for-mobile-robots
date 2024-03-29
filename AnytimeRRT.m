classdef AnytimeRRT < handle
    properties
        graph
        nodes
        boundaries
        init_node
        final_node
        obstacles
        robot
        status
        goalBias
        costs
        upper_bound
        k
        distance_bias
        cost_bias
        max_time
    end
    
    methods
        function obj = AnytimeRRT(q_i,q_f,x_min,x_max,y_min,y_max,obstacles,robot,goalBias,k,upper_bound,max_time)
            x_i = q_i(1);
            y_i = q_i(2);
            
            x_f = q_f(1);
            y_f = q_f(2);
            
            obj.goalBias = goalBias;
            obj.k = k;
            obj.upper_bound = upper_bound;
            obj.distance_bias = 1;
            obj.cost_bias = 0;
            obj.costs = zeros(size(obj.nodes));
            obj.max_time = max_time;
            
            if(x_i>=x_min)&&(y_i>=y_min)&&(x_i<=x_max)&&(y_f<=y_max)
                obj.init_node = [x_i,y_i];
                obj.final_node = [x_f,y_f];
                
                nodes = [obj.init_node];
                obj.nodes = nodes;
                G = graph;
                G = addnode(G,1);
                obj.graph = G;
                obj.boundaries = [x_min, x_max; y_min, y_max];
                obj.obstacles = obstacles;
                obj.robot = robot;
                obj.costs = 0;
                
            else
                error('position out of bounds');
            end
        end
        % dato un nodo, restituisce l'indice in cui si trova nell'elenco
        % dei nodi
        function id = findId(obj, node)
            for i=1:size(obj.nodes)
                if all(obj.nodes(i,:)==node)
                    id = i;
                    return
                end
            end
        end
        
        % controlla se la posizione q � all'interno di un ostacolo
        function flag = checkCollision(obj, q)
            for i=1:size(obj.obstacles)
                if(~isempty(obj.obstacles(i).intersect(q(1),q(2))))
                    flag = true;
                    return
                end
            end
            flag = false;
        end
        
        % sceglie una posizione random nel free space che abbia un costo
        % stimato minore dell'upper bound
        function qtarget = chooseTarget(obj)
            if rand >= obj.goalBias
                qtarget = obj.final_node;
                return
            end
            
            counter = 0;
            while(counter < 100)
                W = obj.robot.getRandomConfig([obj.init_node(1,1),obj.final_node(1,1)],obj.boundaries(2,:));
                if(floor(norm(obj.init_node(1,1:2)-W(1:2))+norm(W(1:2)-obj.final_node(1,1:2)))>obj.upper_bound||obj.checkCollision(W))
                    counter = counter + 1;
                else
                    qtarget = W;
                    return;
                end
            end
            qtarget = -1;
        end
        
        function cost = selCost(obj,q,qtarget)
            path = shortestpath(obj.graph,1,obj.findId(q));
            c = size(path,2);
            cost = obj.distance_bias*norm(q(1,1:2)-qtarget(1,1:2))+obj.cost_bias*c;
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
        
        function newNode = generateExtension(obj,qact,qgoal)
            if norm(qgoal(1:2)-qact(1:2))<obj.robot.v
                newNode = qgoal;
                return;
            else
                intersections = obj.solveIntersection(qact(1),qgoal(1),qact(2),qgoal(2),qact(1),qact(2),obj.robot.v);
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
        end
        
        function obj = addNewNode(obj,qnew,qnext)
            [~, index]=ismember(obj.nodes,qnext,'rows');
            index = find(index);
            
            [~, indexN]=ismember(obj.nodes,qnew,'rows');
            
            if sum(indexN) > 0
                return
            end
            
            obj.nodes =  [obj.nodes; qnew];
            newIndex = size(obj.nodes,1);
            obj.graph = addnode(obj.graph,newIndex);
            obj.graph = addedge(obj.graph,index,newIndex,norm(qnew-qnext));
            
            if(newIndex>size(obj.costs))
                n = numnodes(obj.graph);
                [s,t] = findedge(obj.graph);
                if ~isempty(s)&& ~isempty(t)
                    A = sparse(s,t,obj.graph.Edges.Weight,n,n);
                    [dist,~] = graphshortestpath(A,1,newIndex);
                    obj.costs(newIndex) = dist;
                end
            end
        end
        
        %restituisce qnew e il costo per andare da qstart a qnew
        function X = extendToTarget(obj,qtarget)
            ordered = [];
            for i=1:size(obj.nodes)
                distance = norm(obj.nodes(i,1:2)-qtarget(1,1:2));
                
               ordered = [ordered;obj.nodes(i,:),distance];
            end
            ordered = sortrows(ordered,3);
            s = obj.k;
            if(s>size(ordered,1))
                s = size(ordered,1);
            end
            
            Qnew = ordered(1:s,1:2);
            select = [];
            for i = 1:size(Qnew)
                select = [select; Qnew(i,:),obj.selCost(Qnew(i,:),qtarget)];
            end
            
            select = sortrows(select,3);
            
            while(~isempty(select))
                qtree = select(1,1:2);
                select(1,:) = [];
                q_new = obj.generateExtension(qtree',qtarget');
                q_new = q_new';
                index = obj.findId(qtree);
                cost_start_tree = obj.costs(index);
                tempcost = cost_start_tree+(norm(qtree-q_new));
                
                if(floor(tempcost+norm((q_new-obj.final_node))<obj.upper_bound))
                    qnew = q_new;
                    qnear = qtree;
                    X = [qnew,qnear];
                    return
                end
                
            end
            X = -1;
            return
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
        
        function cost = growRRT(obj)
            q_new = obj.init_node';
            q_goal = obj.final_node';
            time = 0;
            
            distance_treshold = 0.4;
            nor = norm(q_new(1:2)-q_goal(1:2));
            while(nor>distance_treshold)
                q_target = obj.chooseTarget();
                if(q_target ~= -1)
                    new = obj.extendToTarget(q_target);
                    if(new~=-1)
                        q_new = new(1,1:2);
                        q_near = new(1,3:4);
                        [x,y] = obj.getMovement(q_near,q_new);
                        if obj.obstacleFree(x,y)
                            obj.addNewNode(q_new,q_near);
                        end
                        nor = norm(q_new(1:2)'-q_goal(1:2));
                    end
                end
                time = time + 1;
                if(time>obj.max_time)
                    obj.status = 'out of time';
                    cost = -1;
                    return              
                end
            end
             n = numnodes(obj.graph);
            [s,t] = findedge(obj.graph);
            if ~isempty(s)&& ~isempty(t)
                A = sparse(s,t,obj.graph.Edges.Weight,n,n);
                
                [~, path, ~] = graphshortestpath(A, 1, obj.findId(obj.nodes(size(obj.nodes,1),:)));
                cost = 0;
                for m=2:size(path,2)
                    cost = cost + obj.graph.Edges.Weight(findedge(obj.graph,path(m-1),path(m)));
                end   
            end
            obj.status = 'reached';
        end
        
        function [x,y] = getMovement(~,startConfig, endConfig)
            x = [startConfig(1)  endConfig(1)];
            y = [startConfig(2)  endConfig(2)];
            return
        end
        
        function [] = run(obj)
            actual_cost = obj.growRRT();
            if(actual_cost~= -1)
                obj.upper_bound = (1-0.1)*actual_cost;
                obj.distance_bias = obj.distance_bias - 0.1;
                if(obj.distance_bias<0)
                    obj.distance_bias = 0;
                end
                obj.cost_bias = obj.cost_bias + 0.1;
                if(obj.cost_bias>1)
                    obj.cost_bias = 1;
                end
            end
        end
        
        function obj = plot(obj,i)
            figure(i);
            hold on
            
            [row,column] = find(adjacency(obj.graph));
            for s = 1:size(row)
                node_a = obj.nodes(row(s),:);
                node_b = obj.nodes(column(s),:);
                X = [node_a(1,1);node_b(1,1)];
                Y = [node_a(1,2);node_b(1,2)];
                line(X,Y,'Color',[0.18,0.2,0.2]);
            end
            
            [~, path, ~] = graphshortestpath(adjacency(obj.graph), 1, size(obj.nodes,1));
            X = [];
            Y = [];
            for m=2:size(path,2)
                qpred = obj.nodes(path(m-1),:)';
                qactual = obj.nodes(path(m),:)';
                [x,y] = obj.getMovement(qpred,qactual);
                X = [X;x'];
                Y = [Y;y'];
            end
            
            plot(X,Y,'Color',[0.8,0.02,0.02],'LineWidth',0.9);
            
            axis([obj.boundaries(1,1),obj.boundaries(1,2),obj.boundaries(2,1),obj.boundaries(2,2)]);
            filename = strcat('Images\Anytime_',num2str(obj.max_time),'__',num2str(i));
            savefig(filename);
        end
        
        function [] = plotIncompletePath(obj)
            [row,column] = find(adjacency(obj.graph));
            for s = 1:size(row)
                node_a = obj.nodes(row(s),:);
                node_b = obj.nodes(column(s),:);
                X = [node_a(1,1);node_b(1,1)];
                Y = [node_a(1,2);node_b(1,2)];
                line(X,Y,'Color',[0.18,0.2,0.2]);
            end
        end
        function [] = finalPlot(obj,last_index,rrt,cost)
            filename = strcat('Images\Anytime_',num2str(obj.max_time),'__',num2str(last_index));
            openfig(filename);
            
            th = 0:pi/50:2*pi;
            xunit = 0.4 * cos(th) + obj.final_node(1);
            yunit = 0.4 * sin(th) + obj.final_node(2);
            plot(xunit, yunit);
            
            
            for o=obj.obstacles
                fill(o.x,o.y,[0.64,0.77,1])
            end
            
            plot(obj.init_node(1,1),obj.init_node(1,2),'b^');
            plot(obj.final_node(1,1),obj.final_node(1,2),'b^');
            
            [~, pathR, ~] = graphshortestpath(adjacency(rrt.graph), 1, size(rrt.nodes,1));
            X = [];
            Y = [];
            for m=2:size(pathR,2)
                qpred = rrt.nodes(pathR(m-1),:)';
                qactual = rrt.nodes(pathR(m),:)';
                [x,y] = rrt.getMovement(qpred,qactual);
                X = [X;x'];
                Y = [Y;y'];
            end
            
            plot(X,Y,'Color',[0.02,0.65,0.8],'LineWidth',0.75);
            h = findobj(gca,'LineWidth',0.75,'-or','LineWidth',0.9);
            
            legend([h(1),h(2)],{'RRT','AnytimeRRT'});
            str_cost = strrep(num2str(cost),'.','_');
            print(strcat(filename,'_final_',str_cost),'-djpeg');
            
            close all
            figure(last_index+2);
            obj.plotIncompletePath();
            hold on
            th = 0:pi/50:2*pi;
            xunit = 0.4 * cos(th) + obj.final_node(1);
            yunit = 0.4 * sin(th) + obj.final_node(2);
            plot(xunit, yunit);
            
            
            for o=obj.obstacles
                fill(o.x,o.y,[0.64,0.77,1])
            end
            
            plot(obj.init_node(1,1),obj.init_node(1,2),'b^');
            plot(obj.final_node(1,1),obj.final_node(1,2),'b^');
            axis([obj.boundaries(1,1),obj.boundaries(1,2),obj.boundaries(2,1),obj.boundaries(2,2)]);
            filename = strcat('Images\Anytime_',num2str(obj.max_time),'__',num2str(last_index+1),'_failed');
            print(filename,'-djpeg');
        end
    end
end