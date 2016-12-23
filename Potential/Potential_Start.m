map=int16(im2bw(imread('BinaryOccupancyGrid.bmp'))); % input map read from a bmp file. for new maps write the file name here
StartingPoint=[50 50]; % Y, X format of starting point
Destination=[450 450]; % Y, X format of end point
rDirection=1; % initial direction from the starting point
rSize=[5 5]; %length and breadth of the robot
rSpeed=1; % spped of the bot
maxRSpeed=1;  

DistanceLimit=5; % a threshold distace. points within this threshold can be taken as same. 
acc=5; % acceleration of the robot
Turn=10*pi/180; % Turn of the robot
Degree_Potential=3; % degree of calculating potential
attractiveScale=300000; % scaling factor for attractive potential
repulsiveScale=300000; % scaling factor for repulsive potential
minAttractivePotential=1.75; % minimum attractive potential at any point



Present_Pos=StartingPoint; % position of the centre of the robot
Present_Dir=rDirection; % direction of orientation of the robot
robot_Dia_Distance=((rSize(1)/2)^2+(rSize(2)/2)^2)^0.5; % used for distance calculations 
pathFound=false; % has goal been reached
pathCost=0;
t=1;
imshow(map==1);
rectangle('position',[1 1 size(map)-1],'edgecolor','k')
pathLength=0; 
if ~plotRobot(Present_Pos,Present_Dir,map,robot_Dia_Distance)
     error('source lies on an obstacle or outside map'); 
end
M(t)=getframe;
t=t+1;

if ~PointPossible(Destination,map)
    error('goal lies on an obstacle or outside map'); 
end

while ~pathFound
    
    % calculate distance from obstacle at front
    Steps=rSize(1)/2+1;
    while true
        Direction_Obstacle=int16(Present_Pos+Steps*[sin(Present_Dir) cos(Present_Dir)]);
        if ~PointPossible(Direction_Obstacle,map)
            break; 
        end
        Steps=Steps+1;
    end
    FrontObstacle_Dist=Steps-rSize(1)/2; % robotSize(1)/2 distance included in i was inside the robot body 
    
    % calculate distance from obstacle at left
    Steps=rSize(2)/2+1;
    while true
        Direction_Obstacle=int16(Present_Pos+Steps*[sin(Present_Dir-pi/2) cos(Present_Dir-pi/2)]);
        if ~PointPossible(Direction_Obstacle,map)
            break; 
        end
        Steps=Steps+1;
    end
    LeftObstacle_Dist=Steps-rSize(2)/2;  
    
    % calculate distance from obstacle at right
    Steps=rSize(2)/2+1;
    while true
        Direction_Obstacle=int16(Present_Pos+Steps*[sin(Present_Dir+pi/2) cos(Present_Dir+pi/2)]);
        if ~PointPossible(Direction_Obstacle,map)
            break; 
        end
        Steps=Steps+1;
    end
    RightObstacle_Dist=Steps-rSize(2)/2;  
    
    % calculate distance from obstacle at front-left diagonal
    Steps=robot_Dia_Distance+1;
    while true
        Direction_Obstacle=int16(Present_Pos+Steps*[sin(Present_Dir-pi/4) cos(Present_Dir-pi/4)]);
        if ~PointPossible(Direction_Obstacle,map) 
            break; 
        end
        Steps=Steps+1;
    end
    FrontLeftDiagonalObstacle_Dist=Steps-robot_Dia_Distance;
    
    % calculate distance from obstacle at front-right diagonal
    Steps=robot_Dia_Distance+1;
    while true
        Direction_Obstacle=int16(Present_Pos+Steps*[sin(Present_Dir+pi/4) cos(Present_Dir+pi/4)]);
        if ~PointPossible(Direction_Obstacle,map) 
            break; 
        end
        Steps=Steps+1;
    end
    FrontRightDiagonalObstacle_Dist=Steps-robot_Dia_Distance;
    
    % calculate angle from goal
     Angle_Goal=atan2(Destination(1)-Present_Pos(1),Destination(2)-Present_Pos(2));
    
     % calculate diatance from goal
     Dist_Goal=( sqrt(sum((Present_Pos-Destination).^2)) );
     if Dist_Goal<DistanceLimit 
         pathFound=true; 
     end
     
     % Getting the repulsive and the attarctive potentials
     repulsivePotential=(1.0/FrontObstacle_Dist)^Degree_Potential*[sin(Present_Dir) cos(Present_Dir)] + ...
     (1.0/LeftObstacle_Dist)^Degree_Potential*[sin(Present_Dir-pi/2) cos(Present_Dir-pi/2)] + ...
     (1.0/RightObstacle_Dist)^Degree_Potential*[sin(Present_Dir+pi/2) cos(Present_Dir+pi/2)] + ...
     (1.0/FrontLeftDiagonalObstacle_Dist)^Degree_Potential*[sin(Present_Dir-pi/4) cos(Present_Dir-pi/4)] + ...
     (1.0/FrontRightDiagonalObstacle_Dist)^Degree_Potential*[sin(Present_Dir+pi/4) cos(Present_Dir+pi/4)];
     
     attractivePotential=max([(1.0/Dist_Goal)^Degree_Potential*attractiveScale minAttractivePotential])*[sin(Angle_Goal) cos(Angle_Goal)];
     totalPotential=attractivePotential-repulsiveScale*repulsivePotential;
     
     % perform steer
     preferredSteer=atan2(rSpeed*sin(Present_Dir)+totalPotential(1),rSpeed*cos(Present_Dir)+totalPotential(2))-Present_Dir;
     while preferredSteer>pi
         preferredSteer=preferredSteer-2*pi; 
     end 
     while preferredSteer<-pi 
         preferredSteer=preferredSteer+2*pi; 
     end 
     preferredSteer=min([Turn preferredSteer]);
     preferredSteer=max([-Turn preferredSteer]);
     Present_Dir=Present_Dir+preferredSteer;
     
     % Setting the speed. it depends on the speed and acc of the robot
     preferredSpeed=sqrt(sum((rSpeed*[sin(Present_Dir) cos(Present_Dir)] + totalPotential).^2));
     preferredSpeed=min([rSpeed+acc preferredSpeed]);
     rSpeed=max([rSpeed-acc preferredSpeed]);
     rSpeed=min([rSpeed maxRSpeed]);
     rSpeed=max([rSpeed 0]);
     
     if rSpeed==0
         error('Robot stopped to avoid collision'); 
     end
     
     % New position of the robot based on speed and steer
     newPosition=Present_Pos+rSpeed*[sin(Present_Dir) cos(Present_Dir)];
     pathCost=pathCost+distanceCost(newPosition,Present_Pos);
     Present_Pos=newPosition;
     if ~PointPossible(int16(Present_Pos),map)
         error('Collided'); 
     end
     
     % Plot the robot steps
     if ~plotRobot(Present_Pos,Present_Dir,map,robot_Dia_Distance)
        error('Collided');
     end
     M(t)=getframe;t=t+1;
end

