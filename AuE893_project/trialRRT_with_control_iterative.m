% Load the occupancy map of the environment
image = imread('playpen_map.pgm');
imageCropped = image();
imshow(imageCropped);
%% 

imageBW = image < 100;
imshow(imageBW);

figure;
tb3map = binaryOccupancyMap(imageBW);
show(tb3map)
%% 


% Define start and goal points on the map
start = [900.0,950.0, 0];
goal = [1100.0, 1100.0, 0];

% Set the start and goal positions of the robot
hold on
plot(start(1), start(2), 'ro')
plot(goal(1), goal(2), 'mo')

hold off

bounds = [tb3map.XWorldLimits; tb3map.YWorldLimits; [-pi pi]];

% Specify the state space of the robot and the min turning radius
ss = stateSpaceDubins(bounds);
ss.MinTurningRadius = 2;

% Vaidate states and discretize motion
stateValidator = validatorOccupancyMap(ss); 
stateValidator.Map = tb3map;
stateValidator.ValidationDistance = 0.05;

% Create a path planner and set parameters
planner = plannerRRT(ss, stateValidator);
planner.MaxConnectionDistance = 2.0;
planner.MaxIterations = 20000;


%planner.GoalReachedFcn = @exampleHelperCheckIfGoal;

% Plan a path
[pthObj, solnInfo] = plan(planner, start, goal);

wayPoints = (pthObj.States(1:2,[1,2]));
%wayPoints = (pthObj.States(:,[1,2]));

%to convert into world files
wayPoints = (wayPoints*0.05);
wayPoints = wayPoints - 50;

wayPoints(end,:) = wayPoints(end, :) + [20, 20];
%len = length(wayPoints);
%wayPoints(len+1) = wayPoints(len);

%wayPoints2 = [wayPoints; length(wayPoints)];
%wayPoints(end+1,:) = wayPoints(end, :);
%wayPoints(end+1,:) = wayPoints(end, :);
%wayPoints(end+1,:) = wayPoints(end, :);

% Display the map
show(tb3map)
hold on

% Search tree
plot(solnInfo.TreeData(:,1), solnInfo.TreeData(:,2), '.-');
%% 

% Interpolate and plot path
interpolate(pthObj, pthObj.NumStates)
plot(pthObj.States(:,1), pthObj.States(:,2), 'r-', 'LineWidth', 2)

% Show the start and goal in the grid map
plot(start(1), start(2), 'ro')
plot(goal(1), goal(2), 'mo')
title('RRT Path')
hold off

%open_system('GazeboDifferentialDriveControl')
%sim('GazeboDifferentialDriveControl');

%% running controller logic
%wayPointTemp = [0,0];
i = 1;
for i=1:3
    % initialize global node
    rosinit("http://localhost:11311")

    % subscribe to odometry data
    odom = rossubscriber('/odom');  
    odomdata = receive(odom,3);

    % publish velocity
    velPub = ros.Publisher([], '/cmd_vel', 'geometry_msgs/Twist');

    pose = odomdata.Pose.Pose;

    speed = rosmessage(velPub);

    % set goal point
    goal = pose.Position;
    goal.X = wayPoints(i,1);
    goal.Y = wayPoints(i,2);

    %frequency of the loop (rate of execution)
    r = rateControl(6);

    tic;
    while toc<1000

        odomdata = receive(odom,3);
        pose = odomdata.Pose.Pose;

        % get current position of the robot
        x = pose.Position.X;
        y = pose.Position.Y;

        % get current orientation (yaw) of the robot
        quat = pose.Orientation;
        angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
        theta = angles(1);

        % difffernce between the present position and goal position
        dist_x = goal.X - x;
        dist_y = goal.Y - y;

        % angle between the current orientation of the robot (about z-axis) and
        % the goal position from the present position
        angle_to_goal = atan2(dist_y, dist_x);


        % the robot will initially estimate the angle and turn towards it at a
        % given angular velocity and then starts moving towards it at a given   
        % linear velocity until it reaches the end position
        if (abs(angle_to_goal - theta) < 0.1)
            speed.Linear.X = 0.6;
            speed.Angular.Z = 0.0;
        elseif (abs(x-goal.X)<0.1 && abs(y-goal.Y)<0.1)
            speed.Linear.X = 0.0;
            speed.Angular.Z = 0.0;
        else
            speed.Linear.X = 0.0;
            speed.Angular.Z = 0.25;
        end

        % send daata to the robot in Gazebo
        send(velPub,speed);

        waitfor(r);

    end
    %i = i+ 1;
end
% shutdown MATLAB rosnode
rosshutdown


% function isReached = exampleHelperCheckIfGoal(planner, goalState, newState)
%     isReached = false;
%     threshold = 0.1;
%     if planner.StateSpace.distance(newState, goalState) < threshold
%         isReached = true;
%     end
% end