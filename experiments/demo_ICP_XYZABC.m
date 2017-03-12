close all
clear 
clc

addpath(genpath('..\libs'));
camera_setup = 2;

load(['../application/hand2Eye',num2str(camera_setup),'.mat']);
load(['../application/stereoParams',num2str(camera_setup),'.mat']);

experiment = '08-Mar-2017';

load(['images/',experiment,'/XYZABC.mat']);

%filename = '..\STL\Knuckle_cut_to_size2.stl'; 
filename = '..\STL\knuckle+Cassete.stl'; 
RTmat = [eye(3), [0 0 0]'; 0 0 0 1];
[f, v] = load_CAD_model(filename, RTmat);   
   

XYZm = load3dPoints('..\STL\knuckle+Cassete.bin');

[CL, CR] = getCMatrices(stereoParams);


%% select image
%close all

%imageID = 7; 
%325.9592  320.1182   -9.7598   -1.4926   -1.4390    1.1347
%960.0301 -952.9360 -912.5309   -1.3226   -1.0895    2.2911

  
imageID = 6; 
%263.3254  364.8172   17.7959   -1.4243   -1.4258    1.1464

Hand_Poisition = XYZABC(imageID, :);
Pose = xyzabc2rtmat(Hand_Poisition);

I1 = imread(['images\', experiment, '\1_', num2str(imageID), '.png']);
I2 = imread(['images\', experiment, '\2_', num2str(imageID), '.png']);
I1 = imresize(I1, [540 960], 'bilinear');
I2 = imresize(I2, [540 960], 'bilinear');
I1 = I1*4;
I2 = I2*4;
[h, w] = size(I1);


L = single(undistortImageMy(I1, stereoParams.CameraParameters1));
R = single(undistortImageMy(I2, stereoParams.CameraParameters2));
%L = 255*L./max(L(:));
%R = nanmean(L(:))*R./nanmean(R(:));
figure; imshow(uint8(L)); title(['Image #', num2str(imageID)]);
%figure; imshow(uint8(R)); title('Right Image');
%

minZ = 200;
maxZ = 2000;
layers = 256;
threshG = 6;
threshC = 10;    
robust_sampling = 0.01;
adaptive_thr = 0;
thrPlane = 40;
WorstRejection = 0.1;
tic
[M, XYZl2, ZL] = simplified_ICP(L, R, CL, CR, minZ, maxZ, layers, threshG, threshC, robust_sampling, adaptive_thr, thrPlane, XYZm, WorstRejection);
toc
M(4,:) = [0 0 0 1];
Mi = inv(M);

%figure; imshow(ZL, [minZ maxZ]); colormap(gca, 'pink'); title('Estimated Depth')

if 1==2
figure;
scatter3(XYZl2(:,1), XYZl2(:,3), XYZl2(:,2), '.', 'r');
hold on
scatter3(XYZm(:,1), XYZm(:,3), XYZm(:,2), '.', 'b');
%xlim([-500 500]);
axis equal    
title('After Alignment');
drawnow;    
end

[~, Ix, ~] = render_CAD_model(f, v, CL, CR, inv(M), h, w, minZ, maxZ, 1/1000, 0);
Ix = 255*Ix(:,:,1)/max(max(Ix(:,:,1)));
Iv = L*2;
Iv(:,:,2) = (Ix(:,:,1) + L)/2;
Iv(:,:,3) = Ix;
figure; imshow(uint8(Iv)); title('Rendered Aligned Model');

E = Pose*X*Mi*eye(4); % eye(4) - it's a point of interest and its direction
rtmat2xyzabc(Pose*X*Mi*eye(4))
rtmat2xyzabc(Pose*X*M*eye(4))

%%

