function [retur] = perpendicular(scan,ang_velo)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%%
%Driving param
velmsg = rosmessage(robot);

tolerance = 0.10;
avrLeft = nanmean(scan.Ranges(1:320));
avrRight = nanmean(scan.Ranges(321:640));

while avrLeft-avrRight > tolerance
   if avrLeft>avrRight
       velmsg.Angular.Z = ang_velo;	% Angular velocity (rad/s)
       send(robot,velmsg);
   end
   if avrLeft<avrRight
       velmsg.Angular.Z = -ang_velo;	% Angular velocity (rad/s)
       send(robot,velmsg);
   end
end

retur = true;
end

