close all
clear 
clc

addpath(genpath('..\libs'));
format longG

calibration = '1a_low_res_calibration';

%experiment = '31-Mar-2017-957507'; imgs = 1:9; XX = 1;
%experiment = '31-Mar-2017-970593'; imgs = 1:9; XX = 2;
experiment = '31-Mar-2017-970593'; imgs = 10:18; XX = 3;
%experiment = '31-Mar-2017-970593'; imgs = 19:27; XX = 4; % contain outliers :'(

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

relative_error = [0.397244277666932,  0.414589506557475, 0.614310665597862];
figure; bar(relative_error); 

%% Relative accuracy multiple times (to minimize error due to repeatability)
N = 40;

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

filename = ['saves\relative_accuracy_',experiment,'_',num2str(XX), '.mat'];
if exist(filename, 'file')
    load(filename);
else
    
    Ts = zeros([9 3 N]);
        
    for j=1:numel(imgs)
        Ms{j} = zeros([4 4 N]);
        for n=1:N
            disp(['img #',num2str(j),' (9) / try #',num2str(n),' (',num2str(N),')']);
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
            Ms{j}(:,:,n) = M;
            
        end

    end

    save(filename, 'Ms', 'Ts', 'XX', 'experiment', 'N', 'threshG', 'threshC', 'WorstRejection', 'robust_sampling', 'thrPosPlane', 'thrNegPlane');
end

T1=[];
for n=1:N
    T1 = [T1; Ts(:,:,n)];
end

figure(af);
scatter3(T1(:,1), T1(:,2), T1(:,3), '.', 'b');
title('Estimated Mockup Positions');
axis equal;



%% Estimate Relative Error 

Mcntr = mean(T1);
Tz = T1 - Mcntr; % zero-mean points

To = round(Tz/100)*100;
To(:,3) = 1;
% find transform between 2D pattern and 3D points
A = inv(To'*To) * To' * Tz; 
% underlying assumption is that we have 2D template (+/-100 mm grid), 
% and we want to map it onto observed positions

% average distance between expected and observed coordinate
E = sqrt(sum((Tz-To*A).^2,2));
mean(E)

%%
Xe = Tz-To*A;
figure();
Eimg = zeros([3 3 3]);
poses = [1 1; 2 1; 3 1; 3 2; 2 2; 1 2; 1 3; 2 3; 3 3]
for n=1:9       
    Xee = Xe(n:9:end, :);
    Eimg(poses(n,1), poses(n,2), :) = mean(Xee);
    mean(Xee)
    scatter3(Xee(:,1), Xee(:,2), Xee(:,3), '.');
    hold on
end
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
title('All errors from all points');
Aimg = Eimg - min(Eimg(:));
Aimg = imresize(Aimg, 100, 'nearest');
%Aimg = rgb2lab(Aimg);
figure; imshow(Aimg);



%% relative move accuracy

Mcntr = mean(T1);
Tz = T1 - Mcntr; % zero-mean points

Cntr = [];
for n=1:9       
    Cntr = [Cntr;mean(Tz(n:9:end, :))];    
end

To = round(Cntr/100)*100;
To(:,3) = 1;
A = inv(To'*To) * To' * Cntr; 

%E = sqrt(sum((Cntr-To*A).^2,2));
%mean(E)


Errs = [];
AllErr = 0;
AbsErr = 0;
values = 0;
XY = []; 
for i=1:size(Cntr, 1)
    for j=2:size(Cntr, 1)
        if i==j
            continue;
        end
        XY = [XY; To(i, :); To(j,:)];
        Dexp = sqrt(sum((To(i, :)-To(j,:)).^2, 2));
        Dreal = sqrt(sum((Cntr(i, :)-Cntr(j,:)).^2, 2));
        Err = Dreal-Dexp;
        AllErr = AllErr + Err;
        AbsErr = AbsErr + abs(Err);
        values = values + 1;
        Errs = [Errs; [Err Dexp Dreal]];
    end
end
%figure; plot(XY(:,1),XY(:,2), 'LineWidth', 1);
disp(['average error: ' num2str(AllErr/values)]);




figure; 
scatter(Errs(:, 2), Errs(:, 1), 'o'); 
title('experiment 3');
xlim([50 300]);
ylabel('error [mm]');
xlabel('move [mm]')








%% Single image check
if 1==2
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



    [~, Im, ~] = render_CAD_model(f, v, CL2, CR2, M, h*2, w*2);
    Im = imresize(Im, [h w]);
    Im = Im(:,:,1)./max(max(Im(:,:,1)));
    I = Im*255;
    I(:,:,2) = (Im*255 + L)/2;
    I(:,:,3) = L;
    figure(); imshow(uint8(I)); title('Aligned Model'); drawnow;
    figure(); imshow(uint8(Im*255)); title('Aligned Model'); drawnow;
    figure(); imshow(uint8(L)); title('Image'); drawnow;
end    

%%
