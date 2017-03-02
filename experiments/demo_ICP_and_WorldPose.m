close all
clear 
clc

addpath(genpath('..\libs'));
camera_setup = 2;

load(['../application/hand2Eye',num2str(camera_setup),'.mat']);
load(['../application/stereoParams',num2str(camera_setup),'.mat']);

experiment = 'calib6-exp';
POSITIONS = [
1212.763 1258.115 1108.814 51.970 130.230 113.008;
1212.763 1258.115 1108.814 51.970 130.230 113.008;
1212.763 1258.115 1108.814 51.970 130.230 113.008;
1212.763 1258.115 1108.814 51.970 130.230 113.008;

1171.111 1296.961 1108.765 53.838 130.234 113.009;
1171.111 1296.961 1108.765 53.838 130.234 113.009;
1171.111 1296.961 1108.765 53.838 130.234 113.009;
1171.111 1296.961 1108.765 53.838 130.234 113.009;

1152.489 1290.787 1092.729 59.372 137.523 116.853;
1152.489 1290.787 1092.729 59.372 137.523 116.853;
1152.489 1290.787 1092.729 59.372 137.523 116.853;
1152.489 1290.787 1092.729 59.372 137.523 116.853;

1134.795 1283.486 1080.693 65.753 143.739 121.799;
1134.795 1283.486 1080.693 65.753 143.739 121.799;
1134.795 1283.486 1080.693 65.753 143.739 121.799;
1134.795 1283.486 1080.693 65.753 143.739 121.799;
];

    
%filename = '..\STL\Knuckle_cut_to_size2.stl'; 
filename = '..\STL\knuckle+Cassete.stl'; 
RTmat = [eye(3), [0 0 0]'; 0 0 0 1];
[f, v] = load_CAD_model(filename, RTmat);   
   
%XYZm = load3dPoints('points2.bin');
XYZm = load3dPoints('pointsK+C.bin');
xyzaer2rotm = @(x) ([eul2rotm(pi * [x(4) x(5) x(6)] / 180, 'ZYZ'), [x(1) x(2) x(3)]'; 0 0 0 1]);
rtmat2xyzear = @(x) [x(1,4), x(2,4), x(3,4), 180*rotm2eul(x(1:3,1:3), 'ZYZ')/pi];



[CL, CR] = getCMatrices(stereoParams);


%% select image

imageID = 1;
Hand_Poisition = POSITIONS(imageID, :);
Pose = xyzaer2rotm(Hand_Poisition);

I1 = imread(['images\', experiment, '\1_', num2str(imageID), '.png']);
I2 = imread(['images\', experiment, '\2_', num2str(imageID), '.png']);
[h, w] = size(I1);


L = single(undistortImage(I1, stereoParams.CameraParameters1));
R = single(undistortImage(I2, stereoParams.CameraParameters2));
L = 255*L./max(L(:));
R = nanmean(L(:))*R./nanmean(R(:));
figure; imshow(uint8(L)); title(['Image #', num2str(imageID)]);
%figure; imshow(uint8(R)); title('Right Image');
%%

minZ = 200;
maxZ = 2000;
layers = 256;
threshG = 5;
threshC = 10;    
robust_sampling = 0.001;
adaptive_thr = 0;
thrPlane = 40;
WorstRejection = 0.03;
tic
[M, XYZl2] = simplified_ICP(L, R, CL, CR, minZ, maxZ, layers, threshG, threshC, robust_sampling, adaptive_thr, thrPlane, XYZm, WorstRejection);
toc
M(4,:) = [0 0 0 1];
Mi = inv(M);

figure;
scatter3(XYZl2(:,1), XYZl2(:,3), XYZl2(:,2), '.', 'r');
hold on
scatter3(XYZm(:,1), XYZm(:,3), XYZm(:,2), '.', 'b');
xlim([-500 500]);
axis equal    
title('After Alignment');
drawnow;    
E = Pose*X*Mi*eye(4); % eye(4) - it's a point of interest and its direction
rtmat2xyzear(Pose*X*Mi*eye(4))

%%

Mini 
