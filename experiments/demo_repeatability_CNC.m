close all
clear 
clc

addpath(genpath('..\libs'));
format longG

calibration = '1a_low_res_calibration';
%experiment = '31-Mar-2017-957507'; imgs = 1:9; XX = 1;
experiment = '31-Mar-2017-970593'; imgs = 1:9; XX = 2;
%experiment = '31-Mar-2017-970593'; imgs = 10:18; XX = 3;
%experiment = '31-Mar-2017-970593'; imgs = 19:27; XX = 4;

filename = '..\STL\mockup3.stl'; 

%calibration = '1b_high_res';
%experiment = '31-Mar-2017-743132';


%calibration = '2b_low_res_decim';
%experiment = '31-Mar-2017-957507';

load(['../application/images/',calibration,'/stereoParams.mat']);

%save(['../application/images/',calibration,'/stereoParams.mat'], 'stereoParams');
RTmat = [eye(3), [0 0 0]'; 0 0 0 1]; 
[f, v] = load_CAD_model(filename, RTmat);

[CL, CR, CL2, CR2] = getCMatrices(stereoParams);


minZ = 200;
maxZ = 2000;
layers = 256;
robust_sampling = 0.01;
adaptive_thr = 0;

WorstRejection = 0.02;

%% Single image check
close all
clc

minZ = 500;
maxZ = 1400;
layers = 256;


threshG = 7;
threshC = 10;
WorstRejection = 0.15;
robust_sampling = 0.00001;
thrPosPlane = 100;
thrNegPlane = 20;

i = 25;
I1 = single(imread(['../application/images/',experiment,'/1_', num2str(i),'.png']));
I2 = single(imread(['../application/images/',experiment,'/2_', num2str(i),'.png']));
L = single(undistortImage(I1, stereoParams.CameraParameters1));
R = single(undistortImage(I2, stereoParams.CameraParameters2));
[h, w, ~] = size(L);
L = 255*L./max(L(:));
R = nanmean(L(:))*R./nanmean(R(:));

    
[M, ZL, Zm] = full_ICP(f, v, L, R, CL, CR, minZ, maxZ, layers, threshG, threshC, robust_sampling, adaptive_thr, WorstRejection, thrPosPlane, thrNegPlane);

    %
    %clc
    %close all
    %thrPosPlane = 20;
    %[M, ZL, Zm] = twoD_ICP(f, v, L, R, CL, CR, minZ, maxZ, layers, threshG, threshC, robust_sampling, adaptive_thr, WorstRejection, thrPosPlane, thrNegPlane);
    %
    %XYZl = backproject_Z(ZL, CL);
    %figure;
    %scatter3(XYZl(:,1),XYZl(:,3),XYZl(:,2),'.','r'); axis equal
    %title('Before alignment');

    [~, Im, ~] = render_CAD_model(f, v, CL2, CR2, M, h*2, w*2);
    Im = imresize(Im, [h w]);
    Im = Im(:,:,1)./max(max(Im(:,:,1)));
    I = Im*255;
    I(:,:,2) = (Im*255 + L)/2;
    I(:,:,3) = L;
    figure(); imshow(uint8(I)); title('Aligned Model'); drawnow;
    figure(); imshow(uint8(Im*255)); title('Aligned Model'); drawnow;
    figure(); imshow(uint8(L)); title('Image'); drawnow;
    
%%
close all
clc

threshG = 7;
threshC = 10;
WorstRejection = 0.15;
robust_sampling = 0.0001;
thrPosPlane = 20;
thrNegPlane = 20;


Ms = {};
af = figure();
for j=1:numel(imgs)
    i = imgs(j);
    I1 = single(imread(['../application/images/',experiment,'/1_', num2str(i),'.png']));
    I2 = single(imread(['../application/images/',experiment,'/2_', num2str(i),'.png']));
    L = single(undistortImage(I1, stereoParams.CameraParameters1));
    R = single(undistortImage(I2, stereoParams.CameraParameters2));
    [h, w, ~] = size(L);
    L = 255*L./max(L(:));
    R = nanmean(L(:))*R./nanmean(R(:));
    %figure; imshow(L, []); title('Given Image')
    [M, ZL, Zm] = full_ICP(f, v, L, R, CL, CR, minZ, maxZ, layers, threshG, threshC, robust_sampling, adaptive_thr, WorstRejection, thrPosPlane, thrNegPlane);
    %[M, ZL, Zm] = twoD_ICP(f, v, L, R, CL, CR, minZ, maxZ, layers, threshG, threshC, robust_sampling, adaptive_thr, WorstRejection, thrPosPlane, thrNegPlane);
    %figure; imshow(ZL, []); title('Estimated Depth')
    
    [~, Im, ~] = render_CAD_model(f, v, CL2, CR2, M, h*2, w*2);
    Im = imresize(Im, [h w]);

    I = Im(:,:,1)*255;
    I(:,:,2) = (Im(:,:,1)*255 + L)/2;
    I(:,:,3) = L;
    %figure(af); 
    figure; 
    imshow(uint8(I)); title('Aligned Model'); drawnow;
    Ms{j} = M;
end



Ts = [];
for i=1:9
    M = Ms{i};
    Ts(i,1:3) = M(1:3, 4);
end
YY{XX} = Ts; % 
%Ts = Ts - repmat(Ts(5,:), [9 1]);
Ts = Ts - repmat(mean(Ts), [9 1]);
figure; scatter3(Ts(:,1), Ts(:,2), Ts(:,3), 'o', 'b');
axis equal;

%%
To = round(Ts/100)*100;
R = inv(Ts'*Ts) * Ts' * To ;
Ra = rodrigues(rodrigues(R));
Tsx = Ts*R
figure; scatter(Tsx(:,1), Tsx(:,2)); axis equal;
e = Tsx(:,1:2)-To(:,1:2);
er = mean(sqrt(sum(e.^2,2)));


disp(['Mean error: ', num2str(mean(e)), '; std deviation: ', num2str(std(e))])
%%
figure; 
scatter3(YY{1}(:,1), YY{1}(:,2), YY{1}(:,3), 'b')
hold on
scatter3(YY{2}(:,1), YY{2}(:,2), YY{2}(:,3), 'r')
scatter3(YY{3}(:,1), YY{3}(:,2), YY{3}(:,3), 'g')
%scatter3(YY{4}(:,1), YY{4}(:,2), YY{4}(:,3), 'k')

%% check repeatability and relative accuracy together
close all
clc

threshG = 7;
threshC = 10;
WorstRejection = 0.15;
robust_sampling = 0.0001;
thrPosPlane = 20;
thrNegPlane = 20;


Ms = {};
%af = figure();

N = 2;
Ts = zeros([9 3 N]);
for n=1:N
    for j=1:numel(imgs)
        disp([num2str(j),'/',num2str(n)]);
        i = imgs(j);
        I1 = single(imread(['../application/images/',experiment,'/1_', num2str(i),'.png']));
        I2 = single(imread(['../application/images/',experiment,'/2_', num2str(i),'.png']));
        L = single(undistortImage(I1, stereoParams.CameraParameters1));
        R = single(undistortImage(I2, stereoParams.CameraParameters2));
        [h, w, ~] = size(L);
        L = 255*L./max(L(:));
        R = nanmean(L(:))*R./nanmean(R(:));
        
        [M, ZL, Zm] = full_ICP(f, v, L, R, CL, CR, minZ, maxZ, layers, threshG, threshC, robust_sampling, adaptive_thr, WorstRejection, thrPosPlane, thrNegPlane);
        

        [~, Im, ~] = render_CAD_model(f, v, CL2, CR2, M, h*2, w*2);
        Im = imresize(Im, [h w]);

        I = Im(:,:,1)*255;
        I(:,:,2) = (Im(:,:,1)*255 + L)/2;
        I(:,:,3) = L;
        
        %figure(af);         
        %imshow(uint8(I)); title('Aligned Model'); drawnow;
        Ts(j,:,n) = M(1:3, 4);
    end
    
end

figure; 
scatter3(Ts(:,1), Ts(:,2), Ts(:,3), '.', 'b');

%scatter3(Ts(:,1,1), Ts(:,2,1), Ts(:,3,1), '.', 'b');
%hold on
%scatter3(Ts(:,1,2), Ts(:,2,2), Ts(:,3,2), '.', 'r');
axis equal;
