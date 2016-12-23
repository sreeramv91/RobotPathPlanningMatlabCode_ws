    
image_Map=im2bw(imread('BinaryOccupancyGrid.bmp')); % input map read from a bmp file. for new maps write the file name here
Starting_point=[50 50]; %  Y, X format
Destination=[450 450]; % goal position in Y, X format
Nodes=50; % number of points in the PRM
imshow(image_Map);
Total_Nodes=Nodes+2; % Summation of all the nodes and the starting and the ending point

% Checking if the starting point can be safely taken
if ~PointPossible(Starting_point,image_Map)
    error('Starting_point inside obstacle or ourside the map'); 
end
% Checking if the End point can be safely taken
if ~PointPossible(Destination,image_Map)
    error('Destination inside obstacle or outside the map'); 
end

imshow(image_Map);
rectangle('position',[1 1 size(image_Map)-1],'edgecolor','b')

% source and goal taken as additional vertices in the path planning to ease planning of the robot
Valid_NodePoints=[Starting_point;Destination]; 
rectangle('Position',[Valid_NodePoints(1,2)-5,Valid_NodePoints(1,1)-5,10,10],'Curvature',[1,1],'FaceColor','g'); 
rectangle('Position',[Valid_NodePoints(2,2)-5,Valid_NodePoints(2,1)-5,10,10],'Curvature',[1,1],'FaceColor','g'); 
while length(Valid_NodePoints)<Total_Nodes % iteratively add vertices
    x=double(int32(rand(1,2) .* size(image_Map)));
    if PointPossible(x,image_Map)
        Valid_NodePoints=[Valid_NodePoints;x]; % will add x to the bottom of the list vertex
        rectangle('Position',[x(2)-5,x(1)-5,10,10],'Curvature',[1,1],'FaceColor','r'); 
    end
end
disp('click/press any key');
waitforbuttonpress; 
edge_points=cell(Nodes+2,1); % edges to be stored as an adjacency list
for i=1:Nodes+2
    for j=i+1:Nodes+2
        if PathCheck(Valid_NodePoints(i,:),Valid_NodePoints(j,:),image_Map)
            edge_points{i}=[edge_points{i};j];edge_points{j}=[edge_points{j};i];
            line([Valid_NodePoints(i,2);Valid_NodePoints(j,2)],[Valid_NodePoints(i,1);Valid_NodePoints(j,1)]);
        end
    end
end
disp('click/press any key');
waitforbuttonpress; 
    
%structure of a node is taken as index of node in vertex, historic cost, heuristic cost, total cost, parent index in closed list (-1 for source) 
A_Star=[1 0 heuristic(Valid_NodePoints(1,:),Destination) 0+heuristic(Valid_NodePoints(1,:),Destination) -1]; % the processing queue of A* algorihtm, open list
completed_nodes=[]; % the closed list taken as a list
path_detected=false;
i=0;
while size(A_Star,1)>0
     [A, I]=min(A_Star,[],1); % here I gives the index and A will store the minimum value of Q
     n=A_Star(I(4),:); % n will have the smallest cost element to process
     A_Star=[A_Star(1:I(4)-1,:);A_Star(I(4)+1:end,:)]; % delete element under processing
     if n(1)==2 % goal test
         path_detected=true;
         break;
     end
     for mv=1:length(edge_points{n(1),1}) %iterate through all edges from the node
         newVertex=edge_points{n(1),1}(mv);
         if isempty(completed_nodes) || isempty(find(completed_nodes(:,1)==newVertex, 1)) % not already in closed
             g=n(2)+PreviousPathDist(Valid_NodePoints(n(1),:),Valid_NodePoints(newVertex,:));
             h=heuristic(Valid_NodePoints(newVertex,:),Destination);
             totalCost=g+h;
             add=true; % to be added in queue with better cost
             if length(find(A_Star(:,1)==newVertex))>=1
                 I=find(A_Star(:,1)==newVertex);
                 if A_Star(I,4)<totalCost, add=false;
                 else A_Star=[A_Star(1:I-1,:);A_Star(I+1:end,:);];add=true;
                 end
             end
             if add
                 A_Star=[A_Star;newVertex g h totalCost size(completed_nodes,1)+1]; % add new nodes in queue
             end
         end           
     end
     completed_nodes=[completed_nodes;n]; % update closed lists
end
if ~path_detected
    error('no path found')
end
path=[Valid_NodePoints(n(1),:)]; %retrieve path from parent information
prev=n(5);
while prev>0
    path=[Valid_NodePoints(completed_nodes(prev,1),:);path];
    prev=completed_nodes(prev,5);
end

imshow(image_Map);
rectangle('Position',[Valid_NodePoints(1,2)-5,Valid_NodePoints(1,1)-5,10,10],'Curvature',[1,1],'FaceColor','g');
rectangle('Position',[Valid_NodePoints(2,2)-5,Valid_NodePoints(2,1)-5,10,10],'Curvature',[1,1],'FaceColor','g');
rectangle('position',[1 1 size(image_Map)-1],'edgecolor','k')
line(path(:,2),path(:,1),'color','r');