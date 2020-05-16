function [aoa] = driveTowardsGreenCircle(scansub, velpub, odomsub) 

% Calculate distance to drive in first step
scandata = receive(scansub);
cart = readCartesian(scandata);
idx = abs(cart(:,2)) == min(abs(cart(:,2)));
dist = cart(idx, 1);
x = cart(:,2); % x-pos
d = cart(:,1); % depth

% Select center measurement
c_idx = find(idx==1);
xcenter = x((c_idx-50):(c_idx+50));
dcenter = d((c_idx-50):(c_idx+50));


% Use linefit to calculate angle between robot and wall.
mdl = fitlm(xcenter,dcenter);
coef = mdl.Coefficients.Estimate;
angleEst = atand(coef(2));

% Perpendicular distance to wall
perpDist = cosd(angleEst)*dist;

% Distance to drive
driveDist = ((perpDist-1)/perpDist)*dist;
moveRelative(driveDist, 0.2, velpub, odomsub)




% The sign of the angle indicate whether the robot is on the left or the
% right side of the green circle.
if angleEst < 0
	aoa = 1;
elseif angleEst > 0
	aoa = 2;
end

