function [objectAvoided] = avoidObject(laserSub,odomSub,velPub,scan)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% Defines:
stillInFrontFlag = false;
offsetcounter = 0;
midRange = 0.65;
sideRange = 0.75;
parallelMoveSize = 0.3;
stepSize = 0.18;
objectAvoided = false;
midValues = [295:345];
rightValues = [90:113];
leftValues = [497:520];
	% Takes new scan
	%scanMsg = receive(laserSub); scan = lidarScan(scanMsg);
    %figure(1);
    %plot(scan);
    ObDis = mean(scan.Ranges(midValues),'omitnan');
    if((~isnan(ObDis)  && ObDis<midRange) || mean(scan.Ranges(rightValues),'omitnan')<sideRange|| mean(scan.Ranges(leftValues),'omitnan')<sideRange)
		objectAvoided = true;
        stillInFrontFlag = true;
        %disp('In If');
        pause(2);
        % Takes a new scan
        scanMsg = receive(laserSub); scan = lidarScan(scanMsg);
        ObDis = mean(scan.Ranges(midValues),'omitnan');
        while(offsetcounter ~= 0 || stillInFrontFlag == true)
            while(mean(scan.Ranges(midValues),'omitnan')<midRange || mean(scan.Ranges(rightValues),'omitnan')<sideRange || mean(scan.Ranges(leftValues),'omitnan')<sideRange)
                disp('In while');
                %if (~isnan(ObDis) && ObDis<(midRange+0.1))
                    %disp('In if2');
                    ObDisLeft = mean(scan.Ranges(620:640),'omitnan'); 
                    ObDisRight = mean(scan.Ranges(1:21),'omitnan');
                    %figure(2); plot(scan);
                    % If NaNs are detected mean returns NaN.
                    % If NaNs to the right and to the left, prefer going left
                    ObDisLeft(isnan(ObDisLeft)) = 15;
                    ObDisRight(isnan(ObDisRight)) = 14;
                    if(ObDisLeft>ObDisRight)
                        turningFactor = 1;
                        offsetcounter = offsetcounter -1;
                        disp(turningFactor);
                    else
                        turningFactor = -1;
                        offsetcounter = offsetcounter +1;
                        %disp(turningFactor);
                    end
               % end
                rotateDegree2(90*turningFactor,0.3*turningFactor,false,odomSub,velPub);
                moveRelative(stepSize,0.2,velPub,odomSub);
                rotateDegree2(-90*turningFactor,-0.3*turningFactor,false,odomSub,velPub);
                % Takes a new scan
                scanMsg = receive(laserSub); scan = lidarScan(scanMsg);
            end
            stillInFrontFlag = false;
            
            
            % Now, maybe parallel with the object
            while(offsetcounter ~= 0 && stillInFrontFlag == false)
               % disp('In while offsetCounter~=0');
               % disp(offsetcounter);
                % Takes a new scan
                scanMsg = receive(laserSub); scan = lidarScan(scanMsg);
                if(~(mean(scan.Ranges(midValues),'omitnan')<midRange || mean(scan.Ranges(rightValues),'omitnan')<sideRange || mean(scan.Ranges(leftValues),'omitnan')<sideRange))
                    disp('In move parallel');
                    stillInFrontFlag = false;
                    % Drives forward
                    moveRelative(parallelMoveSize,0.2,velPub,odomSub);
                    rotateDegree2(-90*turningFactor,-0.3*turningFactor,false,odomSub,velPub);
                    % Takes a new scan
                     scanMsg = receive(laserSub); scan = lidarScan(scanMsg);
                     if(mean(scan.Ranges(midValues),'omitnan')<midRange || mean(scan.Ranges(rightValues),'omitnan')<sideRange || mean(scan.Ranges(leftValues),'omitnan')<sideRange)
                         %disp('Not behind');
                         rotateDegree2(90*turningFactor,0.3*turningFactor,false,odomSub,velPub);
                     else
                         %disp('maybe behind');
                         while(offsetcounter ~= 0 && ~(mean(scan.Ranges(midValues),'omitnan')<midRange || mean(scan.Ranges(rightValues),'omitnan')<sideRange || mean(scan.Ranges(leftValues),'omitnan')<sideRange))
                             moveRelative(stepSize,0.2,velPub,odomSub);
                             % Takes a new scan
                            scanMsg = receive(laserSub); scan = lidarScan(scanMsg);
                            offsetcounter = offsetcounter + turningFactor;
                            %disp(offsetcounter);
                         end
                        rotateDegree2(90*turningFactor,0.3*turningFactor,false,odomSub,velPub);
                     end
                else
                    stillInFrontFlag = true;
                end
            end
        end
	end
end

