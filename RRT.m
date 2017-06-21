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
                                
                obj.nodes = nodes;
                G = graph;
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
 
            if rand >= obj.goalBias
                W = obj.final_node;
                return
            end
            
            counter = 0;

            while(counter<1000)
                W = obj.robot.getRandomConfig(obj.boundaries(1,:),obj.boundaries(2,:));
                for o=obj.obstacles
                    if(~isempty(o.intersect(W(1),W(2))))
                        %plot(W(1),W(2),'.');
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
            x_r = X_random(1);
            y_r = X_random(2);
            q_r = X_random(3);
            min_dist = Inf(1);
            nearest_node_index = 0;
            for i=1:size(obj.nodes(),1)
                cart_dist = norm( X_random(1:2)-obj.nodes(i,1:2));
                %ang_dist = min(abs(q_r-obj.nodes(i,3)) , 2*pi - abs(q_r-obj.nodes(i,3)));
                %total_dist = cart_dist + ang_dist;
                total_dist = cart_dist;
                if total_dist < min_dist
                    min_dist = total_dist;
                    nearestNode = obj.nodes(i,:);
                    nearest_node_index = i;
                end
            end
            %nearestNode = obj.nodes(nearest_node_index,:);
        end
        
        % restituisce la primitiva più vicina a partire dal nearestNode e
        % se non è in collisione aggiunge all'albero la configurazione
        
        function newNode = steer(obj,nearestNode)
            
            configurations = obj.robot.directKinematics(nearestNode);
            %disp(configurations);
            if(size(configurations(2))==0)
                newNode = -1;
                return
            else
                
                newNode = obj.Config(nearestNode,configurations);
                
                while(size(configurations(2))>0)
                    
                    collisionCheck = obj.obstacleFree(nearestNode,newNode);
                    
                    if collisionCheck
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
            for j=c
                
                cart_dist =  norm(qnear-c);
                ang_dist = min(abs(qnear(3)-config(3,j)), 2*pi-abs(qnear(3)-config(3,j)));
                total_dist = cart_dist + ang_dist;
                if total_dist<=minimumDistConf
                    minimumDistConf = total_dist;
                    newConf_index = j;
                end
                
            end
            conf = config(:,newConf_index);
            plot(conf(1),conf(2),'.');
        end
        
        function flag = obstacleFree(obj, qnear, qnew)

            flag = 0;
            for i=obj.obstacles
                punti = [qnear';qnew'];
                [x_coll,y_coll] = i.intersect(punti(:,1),punti(:,2));
                if ~isempty([x_coll,y_coll])
                    flag = 1;
                    break
                end
            end
            flag  = ~flag;
        end
        
        function obj = addNewNode(obj, qnew, qnear)

            [tf, index]=ismember(obj.nodes,qnear','rows');
            index = find(index);
            
            [tfN, indexN]=ismember(obj.nodes,qnew','rows');
            
            if sum(indexN) > 0
                return
            end
            
           %plot(qnew(1),qnew(2),'*');
            punti = [qnear';qnew'];
            
            if(qnew(3) == qnear(3))
             plot(punti(:,1),punti(:,2));
            else
             plot(punti(:,1),punti(:,2),'.-');   
            end

            obj.nodes =  [obj.nodes; qnew'];
            newIndex = size(obj.nodes,1);
            obj.graph = addnode(obj.graph,newIndex');
            obj.graph = addedge(obj.graph,index,newIndex);

            x = [0 0 0.1 0];
            y = [-0.05 +0.05 0 -0.05 ];
            R = [cos(qnew(3)) -sin(qnew(3)); sin(qnew(3)), cos(qnew(3))];
            rot = [x' y']*R';
            rot = rot + [qnew(1) qnew(2);qnew(1) qnew(2);qnew(1) qnew(2);qnew(1) qnew(2)];
            fill(rot(:,1),rot(:,2),'r');

                
            %disp(obj.graph)
        end
            
        function newNode = steerJary(obj,xRand, xNear)
            
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
           % disp(x_start);
            x_end = obj.final_node;
            %disp(x_end);
            
            if(all(x_start == x_end))
                obj.nodes
            else
                for i=1:obj.maxIteration
                    
                    x_rand = obj.sampleFree();
                    
                    %txt1 = strcat('\leftarrow ',int2str(i));
                    %text(x_rand(1),x_rand(2),txt1);
                    
                    %disp(x_rand);
                    if(~isempty(x_rand))
                        x_nearest = obj.getNearestConf(x_rand);
                        %plot(x_rand(1),x_rand(2),'s');
                        %plot(x_nearest(1),x_nearest(2),'*');
                        %text(x_nearest(1),x_nearest(2),txt1);
                        %x_nearest = x_nearest';
                        x_new = obj.steerJary(x_rand',x_nearest');
                        if obj.obstacleFree(x_nearest',x_new)
                            obj.addNewNode(x_new,x_nearest');
                        end
                        cart_dist =  norm(x_new -obj.final_node');
                        ang_dist = min(abs(obj.final_node(3)-x_new(3)), 2*pi-abs(obj.final_node(3)-x_new(3)));
     
                        if cart_dist <= 0.4
                            obj.status = 'reached';
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