classdef GameTreePlanner
    %GAMETREEPLANNER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Configurations
        maxDepth
        deltaT
        cutOffValue_unsafety
        
        % Tree Object
        tree
        
    end
    
    methods
        function obj = GameTreePlanner(maxDepth,deltaT,cutOffValue_unsafety)
            %GAMETREEPLANNER Construct an instance of this class
            %   Detailed explanation goes here
            obj.maxDepth = maxDepth;
            obj.deltaT = deltaT;
            obj.cutOffValue_unsafety = cutOffValue_unsafety;
        end
        function obj = calculateBestDecision(obj,currentState_Ego,currentStates_Other,Maneuvers)           
            %% Tree Generation
            % Start by creating the root node
            count = 1; % Id of the first node
            rootNodes = [];
            UnSafetyValue=0; % Initial state for decision making shouldn't be a collision state
            
            % Create the root node but since it is the only node, it should be first a
            % leaf node so that we expand it.
            leafNodes = Node([],[],count,currentState_Ego,Maneuvers,UnSafetyValue);
            count = count + 1; % Increase the Id for the next nodes
            
            for depth = 1:obj.maxDepth
                
                
                for leafNode = leafNodes
                    newleafNodes = [];
                    % Expand the root node if safe
                    if leafNode.UnsafetyValue < obj.cutOffValue_unsafety
                        
                        for maneuver = Maneuvers
                            % Expand for each maneuever
                            newleafNode = leafNode.expand(count,maneuver,obj.deltaT);
                            count = count + 1; % Increase the Id for the next nodes
                            
                            leafNode.targetNodeID = [leafNode.targetNodeID newleafNode.id];
                            
                            % Calculate unsafety value for each other vehicle
                            % and take the max unsafety as the unsafety value of the state
                            UnSafetyValue = [];
                            
                            for otherVehicle =1:length(currentStates_Other)
                                
                                % Calculate PDF of Other Vehicles
                                pdf_other = Maneuver.calculatePDFofOtherVehicles(currentStates_Other(otherVehicle),obj.deltaT,depth);
                                
                                % UnsafetyValue = Ego vehicle's area under the normal distribution curve of other vehicles
                                if abs(newleafNode.state.d - currentStates_Other(otherVehicle).d) < 0.03 % Tolerance value for "d"
                                    % If on the same lane
                                    UnSafetyValue_new = abs(pdf_other.cdf(newleafNode.state.s+2)-pdf_other.cdf(newleafNode.state.s-2)); % Size +-2 meters from the center
                                    UnSafetyValue = [UnSafetyValue UnSafetyValue_new]; % Add unsafety value for each other vehicle to array
                                else
                                    % If not on the same lane
                                    UnSafetyValue_new = 0;
                                    UnSafetyValue = [UnSafetyValue UnSafetyValue_new];
                                end
                            end
                            
                            % Take the max unsafety
                            newleafNode.UnsafetyValue = max(UnSafetyValue);
                            
                            % Add each newly discovered leaf Node to the leaf nodes array
                            newleafNodes = [newleafNodes newleafNode];
                            
                        end
                    else
                        % Report pruned states because it is over the unsafety value threshold (for debugging purposes)
                        disp(strcat(num2str(leafNode.sourceNodeID),'-', leafNode.sourceEdgeName{1}.name,'-','pruned'));
                    end
                    
                    % Make the leafNode a rootNode and add to the array
                    rootNodes = [rootNodes leafNode];
                    
                    leafNodes(1)=[]; % Remove the root node from the leafNodes array to avoid duplicates
                    
                    % Add all "safe" new leaf nodes to the all leafNodes array
                    leafNodes = [leafNodes newleafNodes];
                    
                end
            end
            
            count = count - 1; % Undo the last increment
            
            % Build the tree
            obj.tree = GameTree(rootNodes,leafNodes,obj.cutOffValue_unsafety);
        end
        
    end
end

