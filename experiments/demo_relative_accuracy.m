close all
clear 
clc

addpath(genpath('..\libs'));
camera_setup = 2;
format lonG

load(['../application/hand2Eye',num2str(camera_setup),'.mat']);
load(['../application/stereoParams',num2str(camera_setup),'.mat']);

experiment = '16-Mar-2017-126987';


if strcmp(experiment, '16-Mar-2017-126987')
   
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

%% Multiple Experiemtnts
images = [7, 8, 9];
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

    for i=1:10
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
        


        XYZs(i, :) = M(1:3, 4)';
        disp(['XYZ = ', num2str(XYZs(i,1:3))]);        
        figure(a);
        scatter3(XYZs(:,1), XYZs(:,2), XYZs(:,3), '.', 'b');
        axis equal
        title('Single image estimates');
        xlabel('X')
        ylabel('Y')
        zlabel('Z')
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
title('Relative coordinates estimated from different viewpoints');
xlabel('X')
ylabel('Y')
zlabel('Z')
xlim([750 756]);
ylim([1939 1945]);
zlim([925 930])