close all
clear 
clc

addpath(genpath('..\libs'));
camera_setup = 2;
format lonG

load(['../application/hand2Eye',num2str(camera_setup),'.mat']);
%load(['../application/hand2Eye',num2str(camera_setup),'.bak']);
load(['../application/stereoParams',num2str(camera_setup),'.mat']);

%experiment = 'calib6-exp';
experiment = '16-Mar-2017-126987';


if strcmp(experiment, 'calib6-exp')
    xyzaer2rotm = @(x) ([eul2rotm(pi * [x(4) x(5) x(6)] / 180, 'ZYZ'), [x(1) x(2) x(3)]'; 0 0 0 1]);

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
else
    load(['images\', experiment, '\XYZABC.mat']);
    xyzaer2rotm = @(x) ([angle2dcm( deg2rad(x(6)), deg2rad(x(5)),deg2rad(x(4)) ,'XYZ')', [x(1) x(2) x(3)]'; 0 0 0 1]);
    POSITIONS = XYZABC;    
end

    
%filename = '..\STL\Knuckle_cut_to_size2.stl'; 
filename = '..\STL\knuckle+Cassete.stl'; 
RTmat = [eye(3), [0 0 0]'; 0 0 0 1];
[f, v] = load_CAD_model(filename, RTmat);   
   
%XYZm = load3dPoints('points2.bin');
XYZm = load3dPoints('..\STL\knuckle+Cassete.bin');
rtmat2xyzear = @(x) [x(1,4), x(2,4), x(3,4), 180*rotm2eul(x(1:3,1:3), 'ZYZ')/pi];

[CL, CR] = getCMatrices(stereoParams);
XYZAERs = {};

%% Single Estimation by single Stereo-Pair
%close all

imageID = 1;
%cellID = 3;
Hand_Poisition = POSITIONS(imageID, :);
Pose = xyzaer2rotm(Hand_Poisition);

I1 = imread(['images\', experiment, '\1_', num2str(imageID), '.png']);
I2 = imread(['images\', experiment, '\2_', num2str(imageID), '.png']);
[h, w] = size(I1);


L = single(undistortImage(I1, stereoParams.CameraParameters1));
R = single(undistortImage(I2, stereoParams.CameraParameters2));
L = 255*L./max(L(:));
R = nanmean(L(:))*R./nanmean(R(:));
figure; imshow(uint8(L)); title(['Image #', num2str(imageID)]);drawnow;



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

    if 1==1
        figure;
        scatter3(XYZl2(:,1), XYZl2(:,3), XYZl2(:,2), '.', 'r');
        hold on
        scatter3(XYZm(:,1), XYZm(:,3), XYZm(:,2), '.', 'b');
        xlim([-500 500]);
        axis equal    
        title('After Alignment');
        drawnow;    
    end
    
    E = Pose*X*Mi*eye(4); % eye(4) - it's a point of interest and its direction
    XYZAER = rtmat2xyzear(Pose*X*Mi*eye(4));    
    disp(['Image #', num2str(imageID)]);
    disp([num2str(XYZAER)]);
    
    
%% Multiple Experiemtnts
images = [1, 4];
XYZAER = [];

a = figure();
for cellID = 1:numel(images)
    imageID = images(cellID);
    Hand_Poisition = POSITIONS(imageID, :);
    Pose = xyzaer2rotm(Hand_Poisition);
    I1 = imread(['images\', experiment, '\1_', num2str(imageID), '.png']);
    I2 = imread(['images\', experiment, '\2_', num2str(imageID), '.png']);
    L = single(undistortImage(I1, stereoParams.CameraParameters1));
    R = single(undistortImage(I2, stereoParams.CameraParameters2));
    L = 255*L./max(L(:));
    R = nanmean(L(:))*R./nanmean(R(:));

    for i=1:20
        disp(num2str(i))
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

        %figure;
        %scatter3(XYZl2(:,1), XYZl2(:,3), XYZl2(:,2), '.', 'r');
        %hold on
        %scatter3(XYZm(:,1), XYZm(:,3), XYZm(:,2), '.', 'b');
        %xlim([-500 500]);
        %axis equal    
        %title('After Alignment');
        %drawnow;    
        E = Pose*X*Mi*eye(4); % eye(4) - it's a point of interest and its direction
        XYZAER(i, :) = rtmat2xyzear(Pose*X*Mi*eye(4));
        disp(['XYZAER = ', num2str(XYZAER(i,1:3))]);        
        figure(a);
        scatter3(XYZAER(:,1), XYZAER(:,2), XYZAER(:,3), '.', 'b');
        axis equal
        title('Single image estimates');
        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        xlim([750 756]);
        ylim([1939 1945]);
        zlim([925 930])
        drawnow;
    end
    XYZAERs{cellID} = XYZAER;

end
figure();
for i=1:numel(XYZAERs)
    XYZAER = XYZAERs{i};
    scatter3(XYZAER(:,1), XYZAER(:,2), XYZAER(:,3), '.');    
    hold on        
end
axis equal
title('Knuckle world coordinates estimated from different viewpoints');
xlabel('X')
ylabel('Y')
zlabel('Z')
xlim([750 756]);
ylim([1939 1945]);
zlim([925 930])


