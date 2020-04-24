rosshutdown, clear, close all, clc;
% Defines:
Robot_width = 0.8; % SI: m
Map_pixel_m_resolution = 34.36;
margin = 0.10;
% Destinations
A = [1580, 950];
B = [120, 400];
C = [40, 40];
establishConnection('1.164', '1.144');
% Making objects for subscribing and publishing the topics
robot = rospublisher('/mobile_base/commands/velocity') ;
odom = rossubscriber('/odom');
resetOdometry = rospublisher('/mobile_base/commands/reset_odometry');
laserSub = rossubscriber('/scan');
odomSub = rossubscriber('/odom');
[velPub,velMsg] = rospublisher('/mobile_base/commands/velocity','geometry_msgs/Twist');
numUpdates = 60; % antal updateringer til monte carlo, inden start
%%
% Loading occupancy map of Shannon
% Load Color Image
map_color = imread('World_Simple.png');

%figure(1), imshow(map_color)

% Convert to Grayscale
map_bw = rgb2gray(map_color);
%imshow(map_bw)

% Calculating Mean Pixel Value
mean_value = mean(map_bw(:));

% Converting the image to Binary using threshold
im_binary = map_bw>mean_value;
%figure(), imshow(im_binary);
im_binary = rot90(im_binary,3);
%figure(), imshow(im_binary)
im_binary = flip(im_binary);
%figure(), imshow(im_binary)
% Making dilation on walls
SE = ones(round(((Robot_width/2)+margin)*Map_pixel_m_resolution));
im_binary_dilated = imdilate(im_binary,SE);
map = binaryOccupancyMap(im_binary_dilated,Map_pixel_m_resolution);
%% PRM
%Add probabilistic roadmapping
figure
imagesc(im_binary_dilated), colorbar
hold on, plot(A(1), A(2), 'r*'), text(A(1), A(2), 'A')
hold on, plot(B(1), B(2), 'r*'), text(B(1), B(2), 'B')

prm = PRM(im_binary_dilated)
prm.plan('npoints', 300)  % planning
prm.plot() 

path = prm.query(A, B) % waypoints
prm.plot()
path = path/Map_pixel_m_resolution;
%% Monte Carlo
odometryModel = odometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];
rangeFinderModel = likelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0.45 8];
rangeFinderModel.Map = map;
% Query the Transformation Tree (tf tree) in ROS.
tftree = rostf;
waitForTransform(tftree,'/base_link','/camera_depth_frame');
sensorTransform = getTransform(tftree,'/base_link', '/camera_depth_frame');

% Get the euler rotation angles.
laserQuat = [sensorTransform.Transform.Rotation.W sensorTransform.Transform.Rotation.X ...
    sensorTransform.Transform.Rotation.Y sensorTransform.Transform.Rotation.Z];
laserRotation = quat2eul(laserQuat, 'ZYX');

% Setup the |SensorPose|, which includes the translation along base_link's
% +X, +Y direction in meters and rotation angle along base_link's +Z axis
% in radians.
rangeFinderModel.SensorPose = [sensorTransform.Transform.Translation.X sensorTransform.Transform.Translation.Y laserRotation(1)];
amcl = monteCarloLocalization;
amcl.UseLidarScan = true;
amcl.MotionModel = odometryModel;
amcl.SensorModel = rangeFinderModel;
amcl.UpdateThresholds = [0.2,0.2,0.2];
amcl.ResamplingInterval = 1;
amcl.ParticleLimits = [500 5000];
amcl.GlobalLocalization = true;
amcl.InitialPose = ExampleHelperAMCLGazeboTruePose 
amcl.InitialCovariance = eye(3)*0.5;
visualizationHelper = ExampleHelperAMCLVisualization(map);
wanderHelper = ExampleHelperAMCLWanderer(laserSub, sensorTransform, velPub, velMsg);

%% 
% Reset odometry (pure persuit)
reset_msg = rosmessage(resetOdometry);
send(resetOdometry,reset_msg);
pause(2)
numUpdates = 20;
% Creating array for logging the position of the robots movement - from
% its perspective 
logarray = zeros(100,2); %(pure persuit)
index = 1;
%%
i = 0;
while i < numUpdates
    % Receive laser scan and odometry message.
    scanMsg = receive(laserSub);
    odompose = odomSub.LatestMessage;
    
    % Create lidarScan object to pass to the AMCL object.
    scan = lidarScan(scanMsg);
    
    % For sensors that are mounted upside down, you need to reverse the
    % order of scan angle readings using 'flip' function.
    
    % Compute robot's pose [x,y,yaw] from odometry message.
    odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
    odomRotation = quat2eul(odomQuat);
    pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y odomRotation(1)];
    
    % Update estimated robot's pose and covariance using new odometry and
    % sensor readings.
    [isUpdated,estimatedPose, estimatedCovariance] = amcl(pose, scan);
    
    % Drive robot to next pose.
    %rotateDegree(6, 0.5, 0);
    %wander(wanderHelper);
    disp (pose(1))
    %pause(1)
    
    % Plot the robot's estimated pose, particles and laser scans on the map.
    if isUpdated
        i = i + 1;
        plotStep(visualizationHelper, amcl, estimatedPose, scan, i)
        disp('update')
    end
    
end

%% Pure persuit
controller = controllerPurePursuit('DesiredLinearVelocity',0.5,'LookaheadDistance',0.4,'MaxAngularVelocity',2,'Waypoints',path);
amcl.InitialPose = ExampleHelperAMCLGazeboTruePose 
while(1)
     % Position update
        pose = getPose(odom);
        % Convert orientation
        quat = pose.Orientation;
        angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
        % Calculation distance to goal
        distanceToGoal1 = norm([pose.Position.X;pose.Position.Y]-B);
        % Asking controller 1 for linear velocity and angular velocity
        [vel,ang_vel] = controller([pose.Position.X + amcl.InitialPose(1), pose.Position.Y + amcl.InitialPose(2), angles(1) + amcl.InitialPose(3)]);
        % Tells the robot to move
        %disp(angles)
        move(vel,ang_vel,robot);
        % Logging data and incrementing index
        logarray(index,1) = pose.Position.X + amcl.InitialPose(1);
        logarray(index,2) = pose.Position.Y + amcl.InitialPose(2);
        index = index + 1
end

