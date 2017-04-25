clear
close all
clc

addpath(genpath('..\libs'));
camera_setup = 2;
load(['../application/stereoParams',num2str(camera_setup),'.mat']);

[CL, CR] = getCMatrices(stereoParams);

w = 960;
h = 540;

% 1. select CAD model to sample

%filename = '..\STL\Knuckle_cut_to_size2.stl
%filename = '..\STL\knuckle+Cassete.stl'; 
filename = '..\STL\mockup3.stl'; 

% Initial transform of a model (better to be transformed right in the STL file)
RTmat = [eye(3), [0 0 0]'; 0 0 0 1]; 
[f, v] = load_CAD_model(filename, RTmat);


M = [rodrigues([0 0 0]), [0 0 500]'; 0 0 0 1];
[Zm, Im, ~] = render_CAD_model(f, v, CL, CR, M, h, w, 0, 0, 1/1000, 0);
figure; imshow(Zm, [min(Zm(:))*0.8 max(Zm(:))*1.2]); colormap(gca, pink); title('Current Model ');

% one can re-align STL from MATLAB and re-save 
%[v, f, ~, ~, ~] = stlread('..\STL\mockup2.stl', true);
%f = uint64(f);
%v = single(v);
%%v = v * 1000; % convert meters to mm
%v = v*(rodrigues([0 0 pi/2]))' + repmat([-200 0 0], [size(v, 1) 1]);
%stlwrite('..\STL\mockup3.stl', f, v);


%% 2. Here we select one or more view-points for model sampling 
close all

M1 = rodrigues([-0.1 0 0])*[0.9*eye(3), [700 300 1000]'];
M2 = rodrigues([0.1 0 0])*[0.9*eye(3), [300 300 1000]'];
M3 = rodrigues([0 0.1 0])*[0.9*eye(3), [500 300 1000]'];

[XYZ1m, NV1s, Z1, NV1] = sampleCADmodel(f, (v), M1, h, w, 1, 1);
[XYZ2m, NV2s, Z2, NV2] = sampleCADmodel(f, (v), M2, h, w, 1, 1);
[XYZ3m, NV3s, Z3, NV3] = sampleCADmodel(f, (v), M3, h, w, 1, 1);
figure; imshow((NV1+1)/2); title('Surface normals from veiw 1');
figure; imshow((NV2+1)/2); title('Surface normals from veiw 2');
figure; imshow((NV3+1)/2); title('Surface normals from veiw 3');


%figure; imshow(Z1, [])
%figure; imshow(Z2, [])
%figure; imshow(Z3, [])


%% 3. Preparing well-sampled point-cloud
% Collecting all points in one array
XYZm = [XYZ1m; XYZ2m; XYZ3m]; % points 
NVs = [NV1s; NV2s; NV3s]; % normals 
clear XYZ1m XYZ2m XYZ3m NV1s NV2s NV3s

% filter out too distant points
NVs = NVs(find(XYZm(:,3) < 200), :); 
XYZm = XYZm(find(XYZm(:,3) < 200), :); 

% filter out too close (to each other)
P = pointCloud(XYZm, 'Normal', NVs);
P = pcdownsample(P, 'gridAverage', 5);
XYZm = P.Location;
NVs = P.Normal;
clear P;

%figure; scatter3(XYZm(:,1),XYZm(:,3),XYZm(:,2), 1, (NVs+1)/2); 
%set(gca, 'ZDir', 'reverse');
%%set(gca, 'XDir', 'reverse');
%xlabel('x'); ylabel('z'); zlabel('y')
%axis equal
%title('Model Point Cloud');

%% find boundary points 
tic
IND = knnsearch(XYZm,XYZm,'K', 5);
toc
%

sz = size(IND,1)
prob = zeros([sz 1]);
for i=1:sz
    prob(i) = norm(var(NVs(IND(i,:), :))) > 0;
end
%% boundary points are left with 0.3 probability, others with 0.05 
selected = rand(size(prob)) < (prob*0.2 + 0.02);
XYZ = XYZm(selected, :);
NV = NVs(selected, :);

figure; scatter3(XYZ(:,1),XYZ(:,3),XYZ(:,2), 1, (NV+1)/2); 
set(gca, 'ZDir', 'reverse');
%set(gca, 'XDir', 'reverse');
xlabel('x'); ylabel('z'); zlabel('y')
axis equal
title('Model Point Cloud');

%% Finally randomly select fixed number of points 

P = pointCloud(XYZm);
P = pcdownsample(P, 'random', 3000/P.Count);

figure; 
scatter3(P.Location(:,1), P.Location(:,3), P.Location(:,2), '.', 'b'); 
axis equal
set(gca, 'ZDir', 'reverse');
xlabel('x'); ylabel('z'); zlabel('y')

%save3DPoints('..\STL\mockup2.bin', P.Location)
%save3DPoints('..\STL\knuckle+Cassete_1000.bin', P.Location)



