establishConnection()
%%

close all
point = rossubscriber('/camera/depth_registered/points');

pointcloud = receive(point,3)

figure
scatter3(pointcloud);
%%
clear
Vinkelret_1 = load('Vinkelret_1.mat') 
Vinkelret_2 = load('Vinkelret_2.mat') 
Vinkel_1 = load('Vinkel_1.mat') 
Vinkel_2 = load('Vinkel_2.mat') 

Vinkelret_1 = Vinkelret_1.pointcloud
Vinkelret_2 = Vinkelret_2.pointcloud
Vinkel_1 = Vinkel_1.pointcloud
Vinkel_2 = Vinkel_2.pointcloud

%%
% read XYZ format
xyz = Vinkelret_1.readXYZ();
figure
scatter3(xyz(:,1),xyz(:,2), xyz(:,3))

xyzc = xyz(~isnan(xyz(:,3)),:); % remove NaN

%% kmeans for clustering

K = 20;
[idx,C,sumd] = kmeans(xyzc, K, 'Display', 'iter'); 
C = C'

figure(1)
scatter3(xyz(:,1),xyz(:,2), xyz(:,3), '.')
hold on
for i=1:K,
    plot3(C(1,i), C(2,i), C(3,i), 'r.')
end

figure
for i=1:K
    scatter3(xyzc(idx==i,1),xyzc(idx==i,2), xyzc(idx==i,3), '.')
    hold on
end

%% Plane fitting on single cluster
N= zeros(1,K)

for i = 1:K
    
    %M = 6; % cluster id
    %figure
    %scatter3(xyzc(idx==i,1),xyzc(idx==i,2), xyzc(idx==i,3), '.')

    P = xyzc(idx==i,:)';    % Plane fitting..
    N(1,i) = size(P,2); % number points 
end

%%
N_max = 0;
[N_max, index] = max(N) 

figure
scatter3(xyzc(idx==index,1),xyzc(idx==index,2), xyzc(idx==index,3), '.')

%% 

x0=zeros(K,3);
for i = 1:K
    
    P = xyzc(idx==i,:)';    % Plane fitting..
    
    x0(i,:) = mean(P');
    
    % SIMILAR TO Plane_fitting_example_CORKE.m
    P0 = P - repmat(x0(i,:)', 1, N(i));
    w = ones(1,N(i)); % Weights - could be different..
    J = (repmat(w,3,1).*P0)*P0';
    J  % show J

    %[v(1,i),d(1,i)] = eig(J);  % eigen-vec/values
end
%%
  



%%
n = v(:,1)'% eigenvector corresponding to lowest eigenvalue..
figure
scatter3(P(1,:), P(2,:), P(3,:), '.')
hold on
nvec = [x0 ; x0 + 0.5*n]; % normal vector for plotting (from mean to mean+normvec)
line(nvec(:,1), nvec(:,2), nvec(:,3), 'Color', 'r')