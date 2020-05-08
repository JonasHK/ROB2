rosshutdown, clear, close all, clc;
% Defines:
Robot_width = 0.8; % SI: m
Map_pixel_m_resolution = (2445-8)/53.5;
margin = 0.10;
% Destinations
A = [46, 27.5];
B = [2, 13];
C = [40, 40];
establishConnection('1.164', '1.145');
% Making objects for subscribing and publishing the topics
robot = rospublisher('/mobile_base/commands/velocity') ;
odom = rossubscriber('/odom');
resetOdometry = rospublisher('/mobile_base/commands/reset_odometry');
laserSub = rossubscriber('/scan');
odomSub = rossubscriber('/odom');
[velPub,velMsg] = rospublisher('/mobile_base/commands/velocity','geometry_msgs/Twist');
numUpdates = 60; % antal updateringer til monte carlo, inden start
startup_rvc
%%
% Loading occupancy map of Shannon
% Load Color Image
%map_color = imread('World_Simple.png');
%figure(1), imshow(map_color)

% Convert to Grayscale
%map_bw = rgb2gray(map_color);
map_bw = imread('shannonMap.tif');
figure(1), imshow(map_bw)

% Calculating Mean Pixel Value
mean_value = mean(map_bw(:));

% Converting the image to Binary using threshold
im_binary = map_bw<mean_value;
%figure(), imshow(im_binary);
im_binary = rot90(im_binary,3);
%figure(), imshow(im_binary)
im_binary = flip(im_binary);
im_binary = fliplr(im_binary);
figure(), imshow(im_binary)
% Making dilation on walls
SE = ones(round(((Robot_width/2)+margin)*Map_pixel_m_resolution));
im_binary_dilated = imdilate(im_binary,SE);
figure(), imshow(im_binary_dilated)
% Invert binary

map = binaryOccupancyMap(im_binary_dilated,Map_pixel_m_resolution);
map_without_dilate = binaryOccupancyMap(im_binary,Map_pixel_m_resolution);
figure(), show(map)
%% Matlab PRM - better than Corke
close all
prmComplex = mobileRobotPRM(map,400);
show(prmComplex)
path = findpath(prmComplex,A,B);
show(prmComplex)
%% PRM - Corke
%Add probabilistic roadmapping
% figure
% imagesc(im_binary_dilated), colorbar
% hold on, plot(A(1), A(2), 'r*'), text(A(1), A(2), 'A')
% hold on, plot(B(1), B(2), 'r*'), text(B(1), B(2), 'B')
% 
% prm = PRM(im_binary_dilated)
% prm.plan('npoints', 300)  % planning
% prm.plot() 
% 
% path = prm.query(A, B) % waypoints
% prm.plot()
% path = path/Map_pixel_m_resolution;

%%
% Reset odometry (pure persuit)
reset_msg = rosmessage(resetOdometry);
send(resetOdometry,reset_msg);
pause(2)

%amcl.release()
numUpdates = 20;
% Creating array for logging the position of the robots movement - from
% its perspective 
logarray = zeros(100,3); %(pure persuit)
index = 1;
% Monte Carlo
odometryModel = odometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];
rangeFinderModel = likelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0.45 8];
rangeFinderModel.Map = map_without_dilate;
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
amcl.GlobalLocalization = false;
amcl.InitialPose = ExampleHelperAMCLGazeboTruePose 
amcl.InitialCovariance = eye(3)*0.5;
visualizationHelper = ExampleHelperAMCLVisualization(map_without_dilate); % Start a figure for amcl to print points in. PlotStep() later.
%wanderHelper = ExampleHelperAMCLWanderer(laserSub, sensorTransform, velPub, velMsg);

%% Pure persuit
controller = controllerPurePursuit('DesiredLinearVelocity',0.5,'LookaheadDistance',0.4,'MaxAngularVelocity',2,'Waypoints',path);
amcl.InitialPose = ExampleHelperAMCLGazeboTruePose;
i = 0;
% Instantiate vector field histograms

while(1)
    %tic
    % Receive laser scan and odometry message.
    scanMsg = receive(laserSub);
    odompose = odomSub.LatestMessage;
    % Create lidarScan object to pass to the AMCL object.
    scan = lidarScan(scanMsg);
    
    
    % Compute robot's pose [x,y,yaw] from odometry message.
    odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
    odomRotation = quat2eul(odomQuat);
  

    
    pose = [odompose.Pose.Pose.Position.X + amcl.InitialPose(1), odompose.Pose.Pose.Position.Y + amcl.InitialPose(2), odomRotation(1)];
    % Update estimated robot's pose and covariance using new odometry and
    % sensor readings.
    [isUpdated,estimatedPose, estimatedCovariance] = amcl(pose, scan);

    % Calculation distance to goal
    %distanceToGoal1 = norm([pose.Position.X;pose.Position.Y]-B);
    % Asking controller 1 for linear velocity and angular velocity
    [vel,ang_vel] = controller([estimatedPose(1), estimatedPose(2), estimatedPose(3)]);
    % Tells the robot to move
    %disp(angles)
    move(vel,ang_vel,robot);
    % Logging data and incrementing index
    logarray(index,1) = estimatedPose(1)-odompose.Pose.Pose.Position.X + amcl.InitialPose(1);
    logarray(index,2) = estimatedPose(2)-odompose.Pose.Pose.Position.Y + amcl.InitialPose(2);
    logarray(index,3) = estimatedPose(3)-odomRotation(1) + amcl.InitialPose(3);
    index = index + 1;
    
    %if(objekt deteced)
    %    gemmer orientering
    %    map loades
    %    bug 2 på den
    %    kører efter bug'en
    %    ExampleHelperAMCLGazeboTruePose
    
    ObDis = mean(scan.Ranges(300:340));
    if(~isnan(ObDis)  && ObDis<0.5)
        pause(2);
        while(mean(scan.Ranges(300:340))<0.5 || mean(scan.Ranges(90:100))<0.5 || mean(scan.Ranges(515:525))<0.5)
            if (~isnan(ObDis) && ObDis<0.5)
                ObDisLeft = mean(scan.Ranges(620:640)); 
                ObDisRight = mean(scan.Ranges(1:21));
                % If NaNs are detected mean returns NaN.
                % If NaNs to the right and to the left, prefer going left
                ObDisLeft(isnan(ObDisLeft)) = 15;
                ObDisRight(isnan(ObDisRight)) = 14;
                if(ObDisLeft>ObDisRight)
                    turningFactor = -1;
                else
                    turningFactor = 1;
                end
            end
            rotateDegree2(90*turningFactor,0.3,false,odomSub,velPub);
             for i = 1:9
                move(0.2,0,velPub);
                pause(0.5);
            end
            rotateDegree2(-90*turningFactor,0.3,false,odomSub,velPub);
            % Takes a new scan
            scanMsg = receive(laserSub); scan = lidarScan(scanMsg);
        end
        for i = 1:9
            move(0.2,0,velPub);
            pause(0.5);
        end
            rotateDegree2(-90*turningFactor,0.3,false,odomSub,velPub);
            
   
    end
%     
%     ObDis = mean(scan.Ranges(300:340))
%     if(isnan(ObDis) == 0  && ObDis<0.5)
%          Rotation_old = estimatedPose(3);
%          obsMap = zeros(round(2*Map_pixel_m_resolution));
%          obsMap(round(0.5*Map_pixel_m_resolution):round(1.5*Map_pixel_m_resolution),round(0.5*Map_pixel_m_resolution):round(1.5*Map_pixel_m_resolution)) = 1;
%          imagesc(obsMap)
%          obsStart = [round(1*Map_pixel_m_resolution),round(0.45*Map_pixel_m_resolution)];
%          obsGoal = [round(1*Map_pixel_m_resolution)+1,round(1.55*Map_pixel_m_resolution)];
%          bug = Bug2(obsMap);
%          bug.goal=obsGoal;
%          bug.query(obsStart,obsGoal,'animate')
%     end
    
     %disp (pose(1))
    %pause(1)
    
    % Plot the robot's estimated pose, particles and laser scans on the map.
    if isUpdated
        i = i + 1;
        plotStep(visualizationHelper, amcl, estimatedPose, scan, i)
    end
    %toc
end

% %% Pure persuit
% controller = controllerPurePursuit('DesiredLinearVelocity',0.5,'LookaheadDistance',0.4,'MaxAngularVelocity',2,'Waypoints',path);
% amcl.InitialPose = ExampleHelperAMCLGazeboTruePose 
% while(1)
%      % Position update
%         pose = getPose(odom);
%         % Convert orientation
%         quat = pose.Orientation;
%         angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
%         % Calculation distance to goal
%         distanceToGoal1 = norm([pose.Position.X;pose.Position.Y]-B);
%         % Asking controller 1 for linear velocity and angular velocity
%         [vel,ang_vel] = controller([pose.Position.X + amcl.InitialPose(1), pose.Position.Y + amcl.InitialPose(2), angles(1) + amcl.InitialPose(3)]);
%         % Tells the robot to move
%         %disp(angles)
%         move(vel,ang_vel,robot);
%         % Logging data and incrementing index
%         logarray(index,1) = pose.Position.X + amcl.InitialPose(1);
%         logarray(index,2) = pose.Position.Y + amcl.InitialPose(2);
%         index = index + 1
% end
%%
stillInFrontFlag = false;
offsetcounter = 0;
midRange = 0.7;
sideRange = 0.8;
while(1)
    move(0.2,0,velPub);
    % Takes a new scan
    scanMsg = receive(laserSub); scan = lidarScan(scanMsg);
    figure(1);
    plot(scan);
    ObDis = mean(scan.Ranges(318:322));
    if((~isnan(ObDis)  && ObDis<midRange) || mean(scan.Ranges(90:100))<sideRange|| mean(scan.Ranges(515:525))<sideRange)
        stillInFrontFlag = true;
        disp('In If');
        pause(2);
        % Takes a new scan
        scanMsg = receive(laserSub); scan = lidarScan(scanMsg);
        ObDis = mean(scan.Ranges(318:322));
        while(offsetcounter ~= 0 || stillInFrontFlag == true)
            while(mean(scan.Ranges(318:322))<midRange || mean(scan.Ranges(90:100))<sideRange || mean(scan.Ranges(515:525))<sideRange)
                disp('In while');
                if (~isnan(ObDis) && ObDis<midRange)
                    disp('In if2');
                    ObDisLeft = mean(scan.Ranges(620:640)); 
                    ObDisRight = mean(scan.Ranges(1:21));
                    figure(2); plot(scan);
                    % If NaNs are detected mean returns NaN.
                    % If NaNs to the right and to the left, prefer going left
                    ObDisLeft(isnan(ObDisLeft)) = 15;
                    ObDisRight(isnan(ObDisRight)) = 14;
                    if(ObDisLeft>ObDisRight)
                        turningFactor = 1;
                        offsetcounter = offsetcounter -1;
                        disp(turningFactor);
                    else
                        turningFactor = -1;
                        offsetcounter = offsetcounter +1;
                        disp(turningFactor);
                    end
                end
                rotateDegree2(90*turningFactor,0.3*turningFactor,false,odomSub,velPub);
                 for i = 1:2
                    move(0.2,0,velPub);
                    pause(0.5);
                end
                rotateDegree2(-90*turningFactor,-0.3*turningFactor,false,odomSub,velPub);
                % Takes a new scan
                scanMsg = receive(laserSub); scan = lidarScan(scanMsg);
            end
            stillInFrontFlag = false;
            
            
            % Now, maybe parallel with the object
            while(offsetcounter ~= 0 && stillInFrontFlag == false)
                disp('In while offsetCounter~=0');
                disp(offsetcounter);
                % Takes a new scan
                scanMsg = receive(laserSub); scan = lidarScan(scanMsg);
                if(~(mean(scan.Ranges(318:322))<midRange || mean(scan.Ranges(90:100))<sideRange || mean(scan.Ranges(515:525))<sideRange))
                    disp('In move parallel');
                    stillInFrontFlag = false;
                    % Drives forward
                    for i = 1:5
                            move(0.2,0,velPub);
                            pause(0.5);
                    end
                    rotateDegree2(-90*turningFactor,-0.3*turningFactor,false,odomSub,velPub);
                    % Takes a new scan
                     scanMsg = receive(laserSub); scan = lidarScan(scanMsg);
                     if(mean(scan.Ranges(318:322))<midRange || mean(scan.Ranges(90:100))<sideRange || mean(scan.Ranges(515:525))<sideRange)
                         disp('Not behind');
                         rotateDegree2(90*turningFactor,0.3*turningFactor,false,odomSub,velPub);
                     else
                         disp('maybe behind');
                         while(offsetcounter ~= 0 && ~(mean(scan.Ranges(318:322))<midRange || mean(scan.Ranges(90:100))<sideRange || mean(scan.Ranges(515:525))<sideRange))
                             for i = 1:2
                                move(0.2,0,velPub);
                                pause(0.5);
                             end
                             % Takes a new scan
                            scanMsg = receive(laserSub); scan = lidarScan(scanMsg);
                            offsetcounter = offsetcounter + turningFactor;
                            disp(offsetcounter);
                         end
                        rotateDegree2(90*turningFactor,0.3*turningFactor,false,odomSub,velPub);
                     end
                else
                    stillInFrontFlag = true;
                end
            end
        end
    end
end
