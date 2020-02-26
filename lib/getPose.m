function [pose] = getPose()
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
odom = rossubscriber('/odom');
odomdata = receive(odom,3);
pose = odomdata.Pose.Pose;
end

