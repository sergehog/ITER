close all
clear 
clc

addpath(genpath('..\libs'));
format longG


%experiment = '02-May-2017-667983'; imgs = 1:15;
%experiment = '02-May-2017-597992'; imgs = 1:15;
%experiment = '02-May-2017-216202'; imgs = 1:17;
%experiment = '03-May-2017-434187'; imgs = 1:11;
experiment = '03-May-2017-372126'; imgs = 1:12;


filename = '..\STL\mockup3.stl'; 


load(['../application/images/',experiment,'/stereoParams.mat']);

RTmat = [eye(3), [0 0 0]'; 0 0 0 1]; 
[f, v] = load_CAD_model(filename, RTmat);

[CL, CR, CL2, CR2] = getCMatrices(stereoParams);

%% Single image check
close all
clc

%minZ = 450;
%maxZ = 1500;
%layers = 256;
minZ = 400;
maxZ = 2000;
layers = 100;


threshG = 25*0.2;
threshC = 50*0.2;
%WorstRejection = 0.02;
WorstRejection = 0.1;
thrPosPlane = 100*0.4; %in front of main plane
thrNegPlane = thrPosPlane; % behind main plane
robust_sampling = 0.01;
adaptive_thr = 0; % 

i = 3;
I1 = single(imread(['../application/images/',experiment,'/1_', num2str(i),'.png']));
I2 = single(imread(['../application/images/',experiment,'/2_', num2str(i),'.png']));
L = single(undistortImage(I1, stereoParams.CameraParameters1));
R = single(undistortImage(I2, stereoParams.CameraParameters2));
[h, w, ~] = size(L);
L = 255*L./max(L(:));
R = nanmean(L(:))*R./nanmean(R(:));
figure(); imshow(uint8(L)); title('Given Image'); drawnow;
   
if 1==1
    [M, ZL, Zm, Points] = full_ICP(f, v, L, R, CL, CR, minZ, maxZ, layers, threshG, threshC, robust_sampling, adaptive_thr, WorstRejection, thrPosPlane, thrNegPlane);
else
        
    thrPosPlane = 20;
    [M, ZL, Zm] = twoD_ICP(f, v, L, R, CL, CR, minZ, maxZ, layers, threshG, threshC, robust_sampling, adaptive_thr, WorstRejection, thrPosPlane, thrNegPlane);
    
    XYZl = backproject_Z(ZL, CL);
    figure;
    scatter3(XYZl(:,1),XYZl(:,3),XYZl(:,2),'.','r'); axis equal
    title('Before alignment');
end

    XYZm = Points{1};
    XYZl = Points{2};
    figure;
    scatter3(XYZm(:,1),XYZm(:,3),-XYZm(:,2),'.','b'); 
    hold on
    scatter3(XYZl(:,1),XYZl(:,3),-XYZl(:,2),'.','r'); 
    axis equal
    title('Point Cloud Alignment');


[~, Im, ~] = render_CAD_model(f, v, CL, CR, M, h, w);
Im = imresize(Im, [h w]);
Im = Im(:,:,1)./max(max(Im(:,:,1)));
I = Im*255;
I(:,:,2) = (Im*255 + L)/2;
I(:,:,3) = L;
figure(); imshow(uint8(I)); title('Aligned Model'); drawnow;

%%
N = 40;
vw = VideoWriter(['images/',experiment,'_',num2str(N),'.avi'], 'Motion JPEG AVI');
vW.FrameRate = 10;
vW.Quality = 90;
open(vw);



minZ = 400;
maxZ = 2000;
layers = 100;


threshG = 25*0.2;
threshC = 50*0.2;
%WorstRejection = 0.02;
WorstRejection = 0.1;
thrPosPlane = 100*0.4; %in front of main plane
thrNegPlane = thrPosPlane; % behind main plane
robust_sampling = 0.01;
adaptive_thr = 0; % 

filename = ['saves/repetability_distance_', experiment, '_', num2str(N), '.mat'];
af = figure();
if exist(filename, 'file')
    load(filename)
else
    
    Coords = {};
    for i = imgs
        Ms = zeros([4 4 N]);
        I1 = single(imread(['../application/images/',experiment,'/1_', num2str(i),'.png']));
        I2 = single(imread(['../application/images/',experiment,'/2_', num2str(i),'.png']));
        L = single(undistortImage(I1, stereoParams.CameraParameters1));
        R = single(undistortImage(I2, stereoParams.CameraParameters2));
        [h, w, ~] = size(L);
        L = 255*L./max(L(:));
        R = nanmean(L(:))*R./nanmean(R(:));
        disp(['Image ', num2str(i), ' from ', num2str(max(imgs))]);
        for n=1:N
            disp(['Trial ', num2str(n), ' from ', num2str(N)]);
            [M, ZL, Zm, Points] = full_ICP(f, v, L, R, CL, CR, minZ, maxZ, layers, threshG, threshC, robust_sampling, adaptive_thr, WorstRejection, thrPosPlane, thrNegPlane);
            Ms(:,:,n) = M;
            [~, Im, ~] = render_CAD_model(f, v, CL, CR, M, h, w);
            Im = imresize(Im, [h w]);
            Im = Im(:,:,1)./max(max(Im(:,:,1)));
            I = Im*255;
            I(:,:,2) = (Im*255 + L)/2;
            I(:,:,3) = L;
            %figure(af); imshow(uint8(I)); title('Aligned Model'); drawnow;
            writeVideo(vw, uint8(I));
        end
        Coords{i} = Ms;
    end
    close(vw);
    save(filename, 'Coords', 'imgs', 'N', 'threshG','threshC','WorstRejection','thrPosPlane','thrNegPlane','robust_sampling', 'adaptive_thr');
end

%%
close all
experiment = '02-May-2017-667983'; N = 20;
load(['saves/repetability_distance_', experiment, '_', num2str(N), '.mat'])


X = imgs*0; Y = imgs*0;
for i=imgs
    T = [];
    Ms = Coords{i};
    for n=1:N
        T = [T, Ms(1:3, 4, n)];
    end
    %Tm = median(T, 2);
    Tm = mean(T, 2);
    Err = T-repmat(Tm, [1 N]);
    Err = sqrt(sum(Err.^2, 1));
    X(i) = Tm(3);
    Y(i) = mean(Err);    
    %figure; scatter3(T(1,:),T(2,:),T(3,:),'.','b')
end
figure; 
plot(X, Y, 'LineWidth', 2); 
hold on


experiment = '02-May-2017-597992'; N = 40;
load(['saves/repetability_distance_', experiment, '_', num2str(N), '.mat'])

X = imgs*0; Y = imgs*0;
for i=imgs
    T = [];
    Ms = Coords{i};
    for n=1:N
        T = [T, Ms(1:3, 4, n)];
    end
    %Tm = median(T, 2);
    Tm = mean(T, 2);
    Err = T-repmat(Tm, [1 N]);
    Err = sqrt(sum(Err.^2, 1));
    X(i) = Tm(3);
    Y(i) = mean(Err);    
    %figure; scatter3(T(1,:),T(2,:),T(3,:),'.','b')
end

plot(X, Y, 'LineWidth', 2); 

experiment = '03-May-2017-434187'; N = 40;
load(['saves/repetability_distance_', experiment, '_', num2str(N), '.mat'])
X = imgs*0; Y = imgs*0;
for i=imgs
    T = [];
    Ms = Coords{i};
    for n=1:N
        T = [T, Ms(1:3, 4, n)];
    end
    %Tm = median(T, 2);
    Tm = mean(T, 2);
    Err = T-repmat(Tm, [1 N]);
    Err = sqrt(sum(Err.^2, 1));
    X(i) = Tm(3);
    Y(i) = mean(Err);    
    %figure; scatter3(T(1,:),T(2,:),T(3,:),'.','b')
end


plot(X, Y, 'LineWidth', 2); 

experiment = '03-May-2017-372126'; N = 40; 
load(['saves/repetability_distance_', experiment, '_', num2str(N), '.mat'])
X = imgs*0; Y = imgs*0;
for i=imgs
    T = [];
    Ms = Coords{i};
    for n=1:N
        T = [T, Ms(1:3, 4, n)];
    end
    %Tm = median(T, 2);
    Tm = mean(T, 2);
    Err = T-repmat(Tm, [1 N]);
    Err = sqrt(sum(Err.^2, 1));
    X(i) = Tm(3);
    Y(i) = mean(Err);    
    %figure; scatter3(T(1,:),T(2,:),T(3,:),'.','b')
end
plot(X, Y, 'LineWidth', 2); 

experiment = '02-May-2017-216202'; N = 40;
load(['saves/repetability_distance_', experiment, '_', num2str(N), '.mat'])

X = imgs*0; Y = imgs*0;
for i=imgs
    T = [];
    Ms = Coords{i};
    for n=1:N
        T = [T, Ms(1:3, 4, n)];
    end
    %Tm = median(T, 2);
    Tm = mean(T, 2);
    Err = T-repmat(Tm, [1 N]);
    Err = sqrt(sum(Err.^2, 1));
    X(i) = Tm(3);
    Y(i) = mean(Err);    
    %figure; scatter3(T(1,:),T(2,:),T(3,:),'.','b')
end

plot(X, Y, 'LineWidth', 2); 

ylim([0 1.5]); xlabel('Distance'); 
ylabel('Repetability (mean Eucl deviation) [mm]')
legend({'dataset #1', 'dataset #2', 'dataset #3', 'dataset #4', 'AFTER IMPACT'}, 'Location', 'SouthEast');
title('Repetability Experiments');

%% Now estimate angular stability (repeatability)
close all
figure; 


experiment = '02-May-2017-667983'; N = 20;
load(['saves/repetability_distance_', experiment, '_', num2str(N), '.mat'])


X = imgs*0; Y = imgs*0;
for i=imgs
    T = [];
    Ms = Coords{i};
    Rs = [];    
    for n=1:N        
        Rs = [Rs; rodrigues(Ms(1:3,1:3, n))'];
        T = [T, Ms(1:3, 4, n)];
    end
    Rm = mean(Rs);
    Err = Rs-repmat(Rm, [N 1]);
    Tm = mean(T, 2);    
    X(i) = Tm(3);
    Y(i) = mean(sqrt(sum(Err.^2,2))) * 180/pi;    
end

plot(X, Y, 'LineWidth', 2); 
hold on

experiment = '02-May-2017-597992'; N = 40;
load(['saves/repetability_distance_', experiment, '_', num2str(N), '.mat'])

X = imgs*0; Y = imgs*0;
for i=imgs
    T = [];
    Ms = Coords{i};
    Rs = [];    
    for n=1:N        
        Rs = [Rs; rodrigues(Ms(1:3,1:3, n))'];
        T = [T, Ms(1:3, 4, n)];
    end
    Rm = mean(Rs);
    Err = Rs-repmat(Rm, [N 1]);
    Tm = mean(T, 2);    
    X(i) = Tm(3);
    Y(i) = mean(sqrt(sum(Err.^2,2))) * 180/pi;    
end

plot(X, Y, 'LineWidth', 2); 
hold on

 

experiment = '03-May-2017-434187'; N = 40;
load(['saves/repetability_distance_', experiment, '_', num2str(N), '.mat'])

X = imgs*0; Y = imgs*0;
for i=imgs
    T = [];
    Ms = Coords{i};
    Rs = [];    
    for n=1:N        
        Rs = [Rs; rodrigues(Ms(1:3,1:3, n))'];
        T = [T, Ms(1:3, 4, n)];
    end
    Rm = mean(Rs);
    Err = Rs-repmat(Rm, [N 1]);
    Tm = mean(T, 2);    
    X(i) = Tm(3);
    Y(i) = mean(sqrt(sum(Err.^2,2))) * 180/pi;    
end

plot(X, Y, 'LineWidth', 2); 
hold on

experiment = '03-May-2017-372126'; N = 40;
load(['saves/repetability_distance_', experiment, '_', num2str(N), '.mat'])

X = imgs*0; Y = imgs*0;
for i=imgs
    T = [];
    Ms = Coords{i};
    Rs = [];    
    for n=1:N        
        Rs = [Rs; rodrigues(Ms(1:3,1:3, n))'];
        T = [T, Ms(1:3, 4, n)];
    end
    Rm = mean(Rs);
    Err = Rs-repmat(Rm, [N 1]);
    Tm = mean(T, 2);    
    X(i) = Tm(3);
    Y(i) = mean(sqrt(sum(Err.^2,2))) * 180/pi;    
end

plot(X, Y, 'LineWidth', 2); 
hold on
ylim([0 0.7]); 
xlabel('Distance'); 
ylabel('Stability [degrees]')
legend({'dataset #1', 'dataset #2', 'dataset #3', 'dataset #4'}, 'Location', 'SouthEast');
title('Angular Stability (Repetability) Experiments');
