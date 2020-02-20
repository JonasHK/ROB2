function [] = establishConnection()
%establishConnection Establish connection to the robot, and call rostopic
%list
%   Detailed explanation goes here
rosshutdown
setenv('ROS_MASTER_URI','http://192.168.1.200:11311')
setenv('ROS_IP','192.168.1.100')
rosinit('http://192.168.1.200:11311','NodeHost','192.168.1.100');
rostopic list
end

