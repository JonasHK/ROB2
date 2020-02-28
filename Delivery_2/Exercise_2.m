map = binaryOccupancyMap(10,10,10);

x = [1.2; 2.3; 3.4; 4.5; 5.6];
y = [5.0; 4.0; 3.0; 2.0; 1.0];

setOccupancy(map, [x y], ones(5,1))
figure
show(map)

%%
establishConnection()

robot = rospublisher('/mobile_base/commands/velocity') ;
odom = rossubscriber('/odom');
%%
map = zeros(400,400); % Each cell equals 1*1 cm
map(101:1:300,101:1:300) = ones(200,200);
figure

start = [1, 1];
goal = [300, 350];

imagesc(map)
hold on, plot(start(1), start(2), 'r*'), text(start(1), start(2), 'START')
hold on, plot(goal(1), goal(2), 'ro'), text(goal(1), goal(2), 'GOAL')
%%
bug = Bug2(map)
bug.goal = goal;
bug.verbose = 0;
%bug.query(start, goal, 'animate');
wp = bug.query(start, goal);
wp = wp./100;
wp = wp(1:16:size(wp,1),:);

%%hold on, plot(start(1), start(2), 'r*'), text(start(1), start(2), 'START')
%%hold on, plot(goal(1), goal(2), 'ro'), text(goal(1), goal(2), 'GOAL')
%%
controller = controllerPurePursuit('DesiredLinearVelocity',0.15,'LookaheadDistance',0.4,'MaxAngularVelocity',1.5,'Waypoints',wp_rectangle)
pose = getPose(odom)
counter = 1;
logarray = zeros(100,2);
while pose.Position.X ~= goal(1) && pose.Position.Y ~= goal(2)
    [vel,ang_vel] = controller([pose.Position.X,pose.Position.Y,pose.Orientation.Z])
    move(vel,ang_vel,robot);
    pose = getPose(odom);
    logarray(counter,1) = pose.Position.X;
    logarray(counter,2) = pose.Position.Y;
    counter = counter + 1;
end

%%
figure(33)
plot(logarray(:,1),logarray(:,2))

%%
logarrayScaled = logarray*100;
figure(1)
hold on
imagesc(map)
hold on, plot(start(1), start(2), 'r*'), text(start(1), start(2), 'START')
hold on, plot(goal(1), goal(2), 'ro'), text(goal(1), goal(2), 'GOAL')
hold on 
plot(wp(:,1)*100,wp(:,2)*100,'.');
hold on
plot(logarrayScaled(:,1),logarrayScaled(:,2))   