clear 
close all
clc

addpath(genpath('..\libs'));


minZ = 200;
maxZ = 2000;
layers = 100;    

    
load('..\application\stereoParams2.mat');

I1 = imread(['images\exp1_HL\1_1.png']);
I2 = imread(['images\exp1_HL\2_1.png']);

L = undistortImageMy(I1, stereoParams.CameraParameters1);
R = undistortImageMy(I2, stereoParams.CameraParameters2);

%figure; imshow(I1); title('Original');
%L = single( undistortImage(I1, stereoParams.CameraParameters1));
figure; imshow(uint8(L)); title('Undistorted Left');
%R = single(undistortImage(I2, stereoParams.CameraParameters2));
figure; imshow(uint8(R)); title('Undistorted Right');
R = nanmean(L(:)).*R./nanmean(R(:));
figure; imshow(uint8(R)); title('Undistorted Right + luminance balance');


[CL, CR] = getCMatrices(stereoParams);

%%
    threshG = 6;
    threshC = 10;    
    robust_sampling = 0.00001;
    adaptive_theshold = 0;
   
    tic
    [ZL, ~] = estimate_iter_depth(L, R, CL, CR, minZ, maxZ, layers, threshG, threshC, robust_sampling);
    toc
    %figure; imshow(imrotate(ZL, 90), [minZ maxZ]);
    figure; imshow(ZL, [minZ maxZ]); colormap(gca, pink)
%%    
XYZl = backproject_Z(ZL, CL);
XYZl = pointCloud(XYZl);
%XYZl = pcdenoise(XYZl);
%XYZl = pcdownsample(XYZl, 'gridAverage', 4);
XYZl = pcdownsample(XYZl, 'random', 2000/XYZl.Count);
XYZl = XYZl.Location;
%[a, b, c] = find_major_plane(XYZl(:,1), XYZl(:,2), XYZl(:,3), 1000, 20);
%Err = abs(XYZl(:,1).*a+XYZl(:,2).*b + c - XYZl(:,3));%    
%XYZl = XYZl(find(Err < thrPlane), :);    
%XYZm = Pm.Location;
%%
XYZm = load3dPoints('points2.bin');

%%
figure();
scatter3(XYZm(:,1), XYZm(:,2), XYZm(:,3), '.', 'b');
hold on
scatter3(XYZl(:,1), XYZl(:,2), XYZl(:,3), '.', 'r');
axis equal
%%

tic
[TR, TT, ~, ~] = icp2(XYZm', XYZl', 2000, 'Verbose', true, 'WorstRejection', 0.2, 'Matching', 'kDtree', 'Minimize', 'point');
toc
M = [TR, TT;];

XYZl2 = XYZl;
XYZl2(:,4) = 1;
XYZl2 = XYZl2*M';

%XYZm(:,4) = 1;
%XYZm = XYZm*M';
    
figure;
scatter3(-XYZl2(:,1), XYZl2(:,3), XYZl2(:,2), '.', 'r');
hold on
scatter3(-XYZm(:,1), XYZm(:,3), XYZm(:,2), '.', 'b');
xlim([-500 500]);
axis equal    
drawnow;    
   


