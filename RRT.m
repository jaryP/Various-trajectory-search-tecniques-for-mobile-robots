classdef RRT
    properties
        graph
        nodes
        boundaries
        obstacles
        robot
    end
    
    methods
        %costruttore. Parametri: configurazioni iniziale e finale del
        %robot, limiti del configuration space, array di ostacoli, robot
        function obj = RRT(q_i,q_f,x_min,x_max,y_min,y_max,obstacles,robot)
            x_i = q_i(1);
            y_i = q_i(2);
            teta_i = q_i(3);
            x_f = q_f(1);
            y_f = q_f(2);
            teta_f = q_f(3);
            if(x_i>=x_min)&&(y_i>=y_min)&&(x_i<=x_max)&&(y_f<=y_max)
                init_node = [x_i,y_i,teta_i];
                final_node = [x_f,y_f,teta_f];
                nodes = [init_node;final_node];
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
        
        % estrae una configurazione casuale, non in collisione con gli
        % ostacoli
        function W = sampleFree(obj)
            counter = 0;
            W = obj.robot.getRandomConfig(obj.boundaries(1,:),obj.boundaries(2,:));
        
            while(counter<1000)
                flag = false;
                for i=1:size(obj.obstacles(1))
                    if(~isempty(obj.obstacles(i).intersect(W(1),W(2))))
                        flag = true;
                    end
                end
                if(flag==false)
                   return
                else
                    W = obj.robot.getRandomConfig(obj.boundaries(1:2),obj.boundaries(3:4));
                    counter = counter +1;
                end
            end
            W =[];
        end
        
        % restituisce il nodo dell'albero pi� vicino alla configurazione
        % casuale passata come parametro
        function nearestNode = getNearestConf(obj,X_random)
            x_r = X_random(1);
            y_r = X_random(2);
            q_r = X_random(3);
            min_dist = Inf(1);
            nearest_node_index = 0;
            for i=1:size(obj.nodes(1))
                cart_dist = sqrt((x_r-obj.nodes(i,1))^2+ (y_r-obj.nodes(i,2))^2);
                ang_dist = min(abs(q_r-obj.nodes(i,3)) , 2*pi - abs(q_r-obj.nodes(i,3)));
                total_dist = cart_dist + ang_dist;
                if total_dist < min_dist
                    min_dist = total_dist;
                    nearest_node_index = i;
                end
            end
            
            nearestNode = obj.nodes(nearest_node_index,:);
        end
        
        % restituisce la primitiva pi� vicina a partire dal nearestNode e
        % se non � in collisione aggiunge all'albero la configurazione
        function newNode = steer(obj,nearestNode)
            configurations = obj.robot.directKinematics(nearestNode);
            if(size(configurations(2))==0)
                newNode = -1;
                return
            else
                newNode = obj.getClosestConfig(nearestNode,configurations);
                while(size(configurations(2))>0)
                    collisionCheck = obj.obstacleFree(nearestNode,newNode);
                    if collisionCheck == 0
                        obj.addNewNode(newNode,nearestNode);
                        break;
                    else
                        configurations(:,newConf_index) = [];
                        newNode = obj.getClosestConfig(nearestNode,configurations);
                    end
                end
            end
        end
        
        function conf = getClosestConfig(~,qnear,config)
            newConf_index = 0;
            minimumDistConf = Inf(1);
            for j=1:size(config(2))
                cart_dist = sqrt(sqr(qnear(1)-config(1,j))+sqr(qnear(2)-config(2,j)));
                ang_dist = min(abs(qnear(3)-config(3,j)), 2*pi-abs(qnear(3)-config(3,j)));
                total_dist = cart_dist + ang_dist;
                if total_dist<minimumDistConf
                    minimumDistConf = total_dist;
                    newConf_index = j;
                end
            end
            conf = config(:,newConf_index);
        end
        
        function flag = obstacleFree(obj, qnear, qnew)
            flag = 0;
            for i=1:size(obj.obstacles(1))
                [x_coll,y_coll] = obj.obstacle(i).intersect(qnear,qnew);
                if ~isempty([x_coll,y_coll])
                    flag = 1;
                end
            end
        end
        
        function obj = addNewNode(obj, qnew,qnear)
            new_nodes = [obj.nodes; [qnew(1),qnew(2),qnew(3)]];
            obj.nodes = new_nodes;
            obj.graph = addnode(obj.graph,size(obj.nodes)+1);
            obj.graph = addedge(obj.graph,qnear,qnew);
        end
        
        function [] = launch(obj)
            x_start = obj.nodes(1,:);
           % disp(x_start);
            x_end = obj.nodes(2,:);
            %disp(x_end);
            if(all(x_start == x_end))
                obj.nodes
            else
                x_rand = obj.sampleFree();
                if(~isempty(x_rand))
                    x_nearest = obj.getNearestConf(x_rand);
                    x_nearest = x_nearest';
                    disp(size(x_nearest))
                    x_new = obj.steer(x_nearest)
                    
                end
            end
                
        end
    end
end