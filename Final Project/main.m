clear, close all;
% Defines:
Robot_width = 0.4; % SI: m
Map_pixel_m_resolution = 34.36;
margin = 0.10;
% Destinations
A = [100, 580];
B = [600, 2050];
C = [40, 40];
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
%imshow(im_binary);

% Making dilation on walls
SE = ones(round(((Robot_width/2)+margin)*Map_pixel_m_resolution));
im_binary_dilated = imdilate(im_binary,SE);
%imshow(im_binary_dilated);
%%
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
%% Pure persuit
controller = controllerPurePursuit('DesiredLinearVelocity',0.3,'LookaheadDistance',0.3,'MaxAngularVelocity',4,'Waypoints',path);

while(1)
     % Position update
        pose = getPose(odom);
        % Convert orientation
        quat = pose.Orientation;
        angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
        % Calculation distance to goal
        distanceToGoal1 = norm([pose.Position.X;pose.Position.Y]-goal1);
        % Asking controller 1 for linear velocity and angular velocity
        [vel,ang_vel] = controller1([pose.Position.X, pose.Position.Y, angles(1)]);
        % Tells the robot to move
        move(vel,ang_vel,robot);
        % Logging data and incrementing index
        logarray(index,1) = pose.Position.X;
        logarray(index,2) = pose.Position.Y;
        index = index + 1
end

