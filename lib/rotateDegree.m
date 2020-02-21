function [] = rotateDegree(degrees,ang_velocity)
%rotateDegree Rotates the turtlebot 'degrees' positive meaning 
%clockwise with angular velocity 'ang_velocity'
%   Detailed explanation goes here
tolerance = 1;
robot = rospublisher('/mobile_base/commands/velocity') ;
odom = rossubscriber('/odom');
odomdata = receive(odom,3);
pose = odomdata.Pose.Pose;
start_degree = conv2Degree(pose.Orientation.Z);
final_degree = start_degree + degrees

velmsg = rosmessage(robot);
velmsg.Angular.Z = -ang_velocity;         % Angular velocity (rad/s)
current_degree = start_degree

while (current_degree > (final_degree+tolerance) || current_degree < (final_degree-tolerance))
send(robot,velmsg);
odomdata = receive(odom,3);
pose = odomdata.Pose.Pose;
current_degree = conv2Degree(pose.Orientation.Z);
[start_degree,final_degree,current_degree]
end

