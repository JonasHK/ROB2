function [] = move(vel,ang_vel,robot)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
velmsg = rosmessage(robot);
velmsg.Linear.X = vel;
velmsg.Angular.Z = ang_vel;
send(robot,velmsg);
end

