function [] = allignToWall(scansub, odomsub, velpub) 

% Calculate parallel distance to move
% Receive linescan data and convert them to cartesian coordinates.
scandata = receive(scansub);
cart = readCartesian(scandata);
x = cart(:,2); % x-pos
d = cart(:,1); % depth

% Find the distance to the object in a straight line in front of the robot.
idx = abs(cart(:,2)) == min(abs(cart(:,2)));
dist = cart(idx, 1);


% Select center measurement. Then select 50 datapoint above and below the
% center. 
% This reduce the risk of including a bend in the wall when making the
% linefit.
c_idx = find(idx==1);
xcenter = x((c_idx-50):(c_idx+50));
dcenter = d((c_idx-50):(c_idx+50));

% Make a linear fit of the center data. 
% This can be used to calculate the angle between the robot and the wall.
% After that, trigonometri is used to calculate the perpendicular and
% parallel distance to the green circle.
mdl = fitlm(xcenter,dcenter);
coef = mdl.Coefficients.Estimate;
angleEst = atand(coef(2));

% Perpendicular and parallel distance to wall
perpDist = cosd(angleEst)*dist;
parallel = dist * sind(-angleEst);

% Print information
fprintf('Angle to wall: %6.4f\n', angleEst);
fprintf('Perpendicular distance: %6.4f\n', perpDist);
fprintf('Parallel distance: %6.4f\n', parallel);


% Depending on whether the robot is on the left or right side of the green
% circle, the robot need to turn one or the other direction.
% After that, it moves the calculated distance and rotate towards the green
% circle.
if parallel > 0
	angleBack = 90 -(-angleEst);
	rotateDegree2(-angleBack, 1, 0, odomsub, velpub)
	moveRelative(parallel-0.08, 0.1, velpub, odomsub)
	rotateDegree2(90, 1, 0, odomsub, velpub)
elseif parallel < 0
	angleBack2 = 90 -(angleEst);
	rotateDegree2(angleBack2, 1, 0, odomsub, velpub)
	moveRelative((-parallel)-0.08, 0.1, velpub, odomsub)
	rotateDegree2(-90, 1, 0, odomsub, velpub)
end

% Make a new linescan and measure the distance to wall. 
scandata = receive(scansub);
cart = readCartesian(scandata);
idx = abs(cart(:,2)) == min(abs(cart(:,2)));
dist = cart(idx, 1);

% Drive the robot to the wall
	% In order for the from of the robot to be 40cm from the wall, the
	% sensor need to register 66 cm to the wall.
driveDist = dist - 0.66;
moveRelative(driveDist, 0.1, velpub, odomsub)