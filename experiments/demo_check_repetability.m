close all
clear 
clc

addpath(genpath('..\libs'));
camera_setup = 2;
format lonG

%load(['../application/hand2Eye',num2str(camera_setup),'.mat']);
load(['../application/stereoParams',num2str(camera_setup),'.mat']);

experiment = '16-Mar-2017-126987';


if strcmp(experiment, '16-Mar-2017-126987')
   
    load(['images\', experiment, '\XYZABC.mat']);
    
    xyzaer2rotm = @(x) ([angle2dcm( deg2rad(x(6)), deg2rad(x(5)),deg2rad(x(4)) ,'XYZ')', [x(1) x(2) x(3)]'; 0 0 0 1]);
    POSITIONS = XYZABC;    
end



%% Independent Experiments
clear XYZs Ms;
close all

images = [7, 8, 9];
for i=1:numel(images)
    imageID = images(i);

    I1 = imread(['images\', experiment, '\1_', num2str(imageID), '.png']);
    I2 = imread(['images\', experiment, '\2_', num2str(imageID), '.png']);
    L = single(undistortImage(I1, stereoParams.CameraParameters1));
    R = single(undistortImage(I2, stereoParams.CameraParameters2));
    L = 255*L./max(L(:));
    R = nanmean(L(:))*R./nanmean(R(:));

    af = figure();
    cf = figure();
    for i=1:100
        disp(num2str(i))
        XYZm = XYZm(:,1:3);
        tic

        [M, XYZl2, Zx] = simplified_ICP(L, R, CL, CR, minZ, maxZ, layers, threshG, threshC, robust_sampling, adaptive_thr, thrPlane, XYZm, WorstRejection);
        toc
        M(4,:) = [0 0 0 1];
        %figure; imshow(Zx, [minZ maxZ]); colormap(gca, jet)
        XYZs(i, :) = M(1:3, 4)';
        Ms(:,:,i) = M;
        disp(['XYZ = ', num2str(XYZs(i,1:3))]);        
        figure(af);
        scatter3(XYZs(:,1), XYZs(:,2), -XYZs(:,3), '.', 'b');
        axis equal
        title('Independently estimated positions');
        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        drawnow;     

        clf(cf);
        figure(cf);    
        scatter3(XYZl2(:,1), -XYZl2(:,2), -XYZl2(:,3), '.', 'r');
        hold on
        scatter3(XYZm(:,1), -XYZm(:,2), -XYZm(:,3), '.', 'b');
        xlim([-500 500]);
        axis equal    
        title('After Alignment');
        drawnow;    

    end

    saveas(af, ['figures/independent_repeatability_',experiment,'_',num2str(imageID),'.fig'], 'fig');
    save(['saves/repeatability1_',experiment,'_',num2str(imageID),'.mat'], 'XYZs', 'Ms');
end

%% Experiments with prediction
clear XYZ2s;
close all

imageID = 7;

I1 = imread(['images\', experiment, '\1_', num2str(imageID), '.png']);
I2 = imread(['images\', experiment, '\2_', num2str(imageID), '.png']);
L = single(undistortImage(I1, stereoParams.CameraParameters1));
R = single(undistortImage(I2, stereoParams.CameraParameters2));
L = 255*L./max(L(:));
R = nanmean(L(:))*R./nanmean(R(:));

% no initial estimate 
M_previous = eye(4);

XYZm(:,4) = 1;
bf = figure();
cf = figure();
%%
for i=1:100
    disp(num2str(i))
    
    Mi_previous = inv(M_previous);
    XYZ2m = XYZm*Mi_previous';
    XYZ2m = XYZ2m(:,1:3);
    tic
    [M, XYZl2] = simplified_ICP(L, R, CL, CR, minZ, maxZ, layers, threshG, threshC, robust_sampling, adaptive_thr, thrPlane, XYZ2m, WorstRejection);
    toc
    M(4,:) = [0 0 0 1];
    M = M_previous * M;    
    
    
    XYZ2s(i, :) = M(1:3, 4)';
    disp(['XYZ = ', num2str(XYZ2s(i,1:3))]);        
    figure(bf);
    scatter3(XYZ2s(:,1), XYZ2s(:,2), XYZ2s(:,3), '.', 'b');
    axis equal
    title('Positions estimated with prediction');
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    drawnow;   
     
    XYZl3 = XYZl2*(M_previous');
    clf(cf);
    figure(cf);    
    scatter3(XYZl3(:,1), XYZl3(:,2), XYZl3(:,3), '.', 'r');
    hold on
    scatter3(XYZm(:,1), XYZm(:,2), XYZm(:,3), '.', 'b');
    xlim([-500 500]);
    axis equal    
    title('After Alignment');
    drawnow;    
    
    M_previous = M;
end
saveas(bf, ['figures/predicted_repeatability_',experiment,'_',num2str(imageID),'.fig'], 'fig');
save(['saves/repeatability2_',experiment,'_',num2str(imageID),'.mat'], 'XYZ2s');