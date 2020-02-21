function [] = stopMotion(robot)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
velmsg = rosmessage(robot);
velmsg.Angular.X = 0; 
velmsg.Angular.Y = 0;
velmsg.Angular.Z = 0;
velmsg.Linear.X = 0;
velmsg.Linear.Y = 0;
velmsg.Linear.Z = 0;
send(robot,velmsg);
end

