function [] = allignGreenCircle(Tolerance, angVel, imsub, odomsub, velpub)  
% Turn toward the green circle
	% Using the RGB-camera, the robot will rotate toward the green circle until
	% the circle's center coordinate is within the image's horizontal
	% coordinate plus/minus a tolarance.

% Update picture of circle
[imCenter, circleCenter, ~] = findGreenCircle(imsub);


% Turn toward the circle, until it's within the tolerance.
while or(circleCenter >= imCenter + Tolerance, ...
		 circleCenter <= imCenter - Tolerance)
		
	% If the circle is to the right, turn ccw
	if circleCenter > imCenter
	   rotateDegree2(-angVel, -1, 0, odomsub, velpub);
	end

	% If the circle is to the right, turn cw
	if circleCenter < imCenter
	   rotateDegree2(angVel, 1, 0, odomsub, velpub);
	end


	[imCenter, circleCenter, ~] = findGreenCircle(imsub);
end




