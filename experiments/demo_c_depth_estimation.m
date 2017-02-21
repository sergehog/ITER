clear 
close all
clc

addpath(genpath('..\libs'));

%%
load('..\application\stereoParams.mat');

I1 = imread(['images\exp5\1_17.png']);
I2 = imread(['images\exp5\2_17.png']);

L = undistortImageMy(I1, stereoParams.CameraParameters1);
R = undistortImageMy(I2, stereoParams.CameraParameters2);

%figure; imshow(I1); title('Original');
%L = single( undistortImage(I1, stereoParams.CameraParameters1));
figure; imshow(uint8(L)); title('Undistorted Left');
%R = single(undistortImage(I2, stereoParams.CameraParameters2));
figure; imshow(uint8(R)); title('Undistorted Right');
R = nanmean(L(:)).*R./nanmean(R(:));
figure; imshow(uint8(R)); title('Undistorted Right + luminance balance');
figure; imshow(R, []); title('Undistorted Right + luminance balance');
%%
%L = single(undistortImage(I1, stereoParams.CameraParameters1));
%R = single(undistortImage(I2, stereoParams.CameraParameters2));

%%

    KL = stereoParams.CameraParameters1.IntrinsicMatrix';
    KR = stereoParams.CameraParameters2.IntrinsicMatrix';
    RT = [stereoParams.RotationOfCamera2', stereoParams.TranslationOfCamera2'];
    CL = single(KL*[eye(3), [0 0 0]']);
    CR = single(KR*RT);
    minZ = 200;
    maxZ = 2000;
    layers = 100;    
%%
    threshG = 0;
    threshC = 0;    
    robust_sampling = 0;
    
   
    tic
    [ZL, ~] = estimate_iter_depth(L, R, CL, CR, minZ, maxZ, layers, threshG, threshC, robust_sampling);
    toc
    %figure; imshow(imrotate(ZL, 90), [minZ maxZ]);
    figure; imshow(ZL, [minZ maxZ]);
    