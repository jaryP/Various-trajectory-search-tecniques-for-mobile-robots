classdef rrtStar < generalRrt
    properties

        gammaScalar

    end
    
    methods
        %costruttore. Parametri: configurazioni iniziale e finale del
        %robot, limiti del configuration space, array di ostacoli, robot
        function obj = rrtStar(q_i,q_f,x_min,x_max,y_min,y_max,obstacles, costAreas, goalBias)
            obj@generalRrt(q_i,q_f,x_min,x_max,y_min,y_max,obstacles,costAreas, goalBias);   
         
            areaWS = polyarea([x_min x_max x_max x_min x_min],[y_min y_min y_max y_max y_min]);
            areaObs = 0;
            for o=obstacles
                areaObs =areaObs + polyarea(o.x,o.y);
            end
            lebesgueMeasure = areaWS - areaObs;
            obj.gammaScalar = 2*sqrt((1+1/2))*sqrt(lebesgueMeasure/pi );
        end
  
        function obj = addNewNode(obj, qnew)
             
            indexN = obj.getNodeIndex(qnew);
            if indexN ~= -1
                return
            end
            %Aggiungo il nuovo nodo al grafo
            newIndex = size(obj.nodes,1);
            obj.graph = addnode(obj.graph,newIndex);
            obj.nodes =  [obj.nodes; qnew];
        end
         
        function nearNodes = nearRadius(obj,point, radius)
            nearNodes = [];

            for i=1:size(obj.nodes(),1)
                node = obj.nodes(i,1:2);
                if all(point == node)
                    continue;
                end
                dist = norm(node-point);
                if dist <= radius
                    nearNodes =[nearNodes; node];                 
                end
            end

        end

        function core(obj, maxIteration)
            
           iteration = 0;
            
           while iteration < maxIteration
                    
                    x_rand = obj.sampleFree();
                    
                    if(~isempty(x_rand))
                        
                        %calcolo il nodo pi� vicino al punto random ed
                        %applico la funzione steer
                        x_nearest = obj.getNearestConf(x_rand);
                        x_new = obj.steer(x_rand',x_nearest');
                        iteration = iteration + 1

                        if all(x_nearest == x_new')
                            continue
                        end
%                         
                        %Calcolo della retta da near a new e controllo se
                        %tale percorso e libero
                        [x,y] = obj.getMovement(x_nearest,x_new);
                                                
                        if obj.obstacleFree(x,y)
                            
                            %Calcolo del reggio della circonferenza e dei
                            %nodi in essa
                            r = sqrt((log(size(obj.nodes,1))/size(obj.nodes,1)))*obj.gammaScalar;
                            r = min(r,obj.eta);
                            Xnear = obj.nearRadius(x_new',r);
                            
                            %Aggiungo il nuovo nodo alla lista dei nodi ed
                            %ed aumento il contatore
                            obj.addNewNode(x_new');
                            
   
                            %L'indice del nuovo node                        
                            newIndex = size(obj.nodes,1);
                            
                            if norm(x_new' -obj.final_node) <= obj.toleranceRadius
                                obj.nearToFinalNodes = [obj.nearToFinalNodes, newIndex];
                            end
                            
                            %L'indice del nodo piu vicino
                            index = obj.getNodeIndex(x_nearest);

                            %Calcolo del costo minimo per raggiungere
                            %x nearest
                            cmin = obj.getDist(index);
                            
                            n = norm(x_nearest'-x_new);
                            w = obj.getWeightMul(x,y);
                            n = n*w;
                            cmin = cmin + n;
                            nmin = n;
                            xmin = x_nearest;
                            
                            for i=1:size(Xnear,1)
                                xNear = Xnear(i,:);
                                
                                [x,y] = obj.getMovement(xNear,x_new);
                                if obj.obstacleFree(x,y)
                                    %calcolo l'indice del nodo near
                                    indexNear = obj.getNodeIndex(xNear);

                                    %calcolo del percorso per raggiungere
                                    %nodo near
                                    c = obj.nodeDist(indexNear);
                                    n = norm(xNear-x_new');
                                    w = obj.getWeightMul(x,y);
                                    n = n*w;
                                    c = c + n;
                                    
                                    %se il costo � minore allora � pi�
                                    %conveniente
                                    if c < cmin
%                                       'minore'
                                      xmin = xNear;
                                      cmin = c;
                                      nmin = n;
                                    end
                                end
                            end
                            
                            indexMin = obj.getNodeIndex(xmin);

                            obj.graph = addedge(obj.graph,indexMin,newIndex,nmin);
                            obj.nodeDist = [obj.nodeDist, cmin];
                            
                             %Riscrivo l'albero
                             for i=1:size(Xnear,1)
                                xNear = Xnear(i,:);
                                [x,y] = obj.getMovement(x_new,xNear);
                                if obj.obstacleFree(x,y)
                                    
                                    %calcolo l'indice del nodo near
                                    indexNear = obj.getNodeIndex(xNear);
                                    %calcolo del percorso per raggiungere
                                    %nodo near
                                    c = obj.nodeDist(newIndex);
                                    n = norm(x_new-xNear');
                                    w = obj.getWeightMul(x,y);
                                    n = n*w;
                                    c = c + n;
                                    
                                    %se il costo � minore allora � pi�
                                    %conveniente
                                    if c < obj.nodeDist(indexNear)
%                                         'riscrivo'
                                        xparent = predecessors(obj.graph,indexNear);
                                        obj.graph = rmedge(obj.graph,xparent,indexNear);
                                        obj.graph = addedge(obj.graph,newIndex,indexNear,n);
                                        obj.nodeDist(indexNear)=c;
                                    end
                                end
                             end
                        end
                    end
           end
        end
            
    end
end
