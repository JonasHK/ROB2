function [] = moveRelative(distance, vel, velpub, odomsub)
%Moves the turtlebot a given distance
%	The first 90% of the distance is moved with the input velocity. For the
%	remaining 10%, the volocity is 0.05 m/s.
	

% Set the initial velocity
velmsg = rosmessage(velpub);
velmsg.Linear.X = vel;

% Set start and current position
odomdata = receive(odomsub);
Xstart = odomdata.Pose.Pose.Position.X;
Ystart = odomdata.Pose.Pose.Position.Y;

Xcurrent = Xstart;
Ycurrent = Ystart;

% Calculate moved distance
start = [Xstart, Ystart];
current = [Xcurrent, Ycurrent];
moveVec = norm(current-start);

% Adjust for observed overshoot
% distance = distance - 0.02;


% Drive until the 90% of the desired distance is reached.
while moveVec < (distance*0.9)
	% Move robot and update odometry info
	send(velpub, velmsg);
	odomdata = receive(odomsub);
	
	% Calculate moved distance
	Xcurrent = odomdata.Pose.Pose.Position.X;
	Ycurrent = odomdata.Pose.Pose.Position.Y;
	start = [Xstart, Ystart];
	current = [Xcurrent, Ycurrent];
	moveVec = norm(current-start);
end

% Reduce movement speed the remaining 10% of the distance.
velmsg.Linear.X = 0.05;
while moveVec < distance
	% Move robot and update odometry info
	send(velpub, velmsg);
	odomdata = receive(odomsub);
	
	% Calculate moved distance
	Xcurrent = odomdata.Pose.Pose.Position.X;
	Ycurrent = odomdata.Pose.Pose.Position.Y;
	start = [Xstart, Ystart];
	current = [Xcurrent, Ycurrent];
	moveVec = norm(current-start);
% 	fprintf('x = %6.4f  -  y = %6.4f  -  dist = %6.4f\n', Xcurrent, Ycurrent, moveVec);
end

% Actively stopping the robot's movement.
stopMotion(velpub);

end

