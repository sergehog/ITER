close all
clear 
clc

addpath(genpath('..\libs'));
camera_setup = 2;

%load(['../application/hand2Eye',num2str(camera_setup),'.mat']);
load(['../application/stereoParams',num2str(camera_setup),'.mat']);

experiment = 'exp5';

Pm = pcread('points.ply'); % points of model
XYZm = Pm.Location;
%figure(); scatter3(XYZm(:,1), XYZm(:,2), XYZm(:,3), '.', 'b'); axis equal; xlabel('x'); ylabel('y'); zlabel('z')


filename = '..\STL\Knuckle_cut_to_size.stl'; 
RTmat = [eye(3), [0 0 0]'; 0 0 0 1];
[f, v] = load_CAD_model(filename, RTmat);   
   
I1 = imread(['images\', experiment, '\1_17.png']);
I2 = imread(['images\', experiment, '\2_17.png']);
[h, w] = size(I1);


%%
L = single(undistortImage(I1, stereoParams.CameraParameters1));
R = single(undistortImage(I2, stereoParams.CameraParameters2));
L = 255*L./max(L(:));
R = nanmean(L(:))*R./nanmean(R(:));
figure; imshow(uint8(L)); title('Left Image');
figure; imshow(uint8(R)); title('Right Image');

%La = fast_average(L, 20);
%figure; imshow(L > (La + 1))

    KL = stereoParams.CameraParameters1.IntrinsicMatrix';
    KR = stereoParams.CameraParameters2.IntrinsicMatrix';
    RT = [stereoParams.RotationOfCamera2', stereoParams.TranslationOfCamera2'];
    CL = single(KL*[eye(3), [0 0 0]']);
    CR = single(KR*RT);
    minZ = 200;
    maxZ = 2000;
    layers = 256;
%%    
    %threshG = 3;
    %threshC = 30;
    threshG = 3;
    threshC = 5;    
    robust_sampling = 0.01;
    adaptive_thr = 0.3;
   
    tic
    [ZL, ~] = estimate_iter_depth(L, R, CL, CR, minZ, maxZ, layers, threshG, threshC, robust_sampling, adaptive_thr);
    toc
    %tic
    %[ZL] = estimate_depth_fast(L, R, CL, CR, minZ, maxZ, layers, threshG, threshC, robust_sampling, adaptive_thr);
    %toc
    
    
    figure; imshow(ZL, [minZ maxZ]); colormap(gca, pink);
%%
thrPlane = 30;
XYZl = backproject_Z(ZL, CL);
[a, b, c] = find_major_plane(XYZl(:,1), XYZl(:,2), XYZl(:,3), 1000, 20);
Err = abs(XYZl(:,1).*a+XYZl(:,2).*b + c - XYZl(:,3));%    
XYZl = XYZl(find(Err < thrPlane), :);    
XYZm = Pm.Location;

figure();
scatter3(XYZm(:,1), XYZm(:,2), XYZm(:,3), '.', 'b');
hold on
scatter3(XYZl(:,1), XYZl(:,2), XYZl(:,3), '.', 'r');
axis equal


%%
P2 = pointCloud(XYZl);
%P2 = pcdenoise(P2);
P2 = pcdownsample(P2, 'gridAverage', 2);
tform = pcregrigid(Pm,P2, 'Metric','pointToPlane','Extrapolate', true);
%XYZ1 = (tform.T(1:3,:)' * P1.Location')';
XYZ2 = P2.Location;
XYZm = Pm.Location;
XYZm(:,4) = 1;
XYZm = (tform.T(:,1:4)' * XYZm')';

figure();
scatter3(XYZm(:,1), XYZm(:,2), XYZm(:,3), '.', 'b');
hold on
scatter3(XYZ2(:,1), XYZ2(:,2), XYZ2(:,3), '.', 'r');
axis equal

%%
P2 = pointCloud(XYZl);
%P2 = pcdenoise(P2);
P2 = pcdownsample(P2, 'gridAverage', 3);
tform = pcregrigid(P2,Pm, 'Metric','pointToPlane','Extrapolate', true);
XYZm = Pm.Location;
XYZ2 = P2.Location;
XYZ2(:,4) = 1;
XYZ2 = (tform.T(:,1:4)' * XYZ2')';

figure();
scatter3(XYZm(:,1), XYZm(:,2), XYZm(:,3), '.', 'b');
hold on
scatter3(XYZ2(:,1), XYZ2(:,2), XYZ2(:,3), '.', 'r');
axis equal

%%

    light = 1000;
    [Zx, Ix, ~] = render_CAD_model(f, v, CL, CR, (tform.T'), h, w, minZ, maxZ, 1/light, 0);
    Zx(Zx > maxZ) = nan;
    Ix = 255*Ix(:,:,1)/max(max(Ix(:,:,1)));
    %Zx(Ix < threshC) = nan;
    Ix(:,:,2) = (Ix(:,:,1) + L)/2;
    Ix(:,:,3) = L*2;
    figure; imshow(uint8(Ix)); title('Rendered Pre-Aligned');
    %figure; imshow(Zx, [minZ maxZ]); title('Rendered Pre-Aligned Depth'); colormap(gca, pink);
%%    
    XYZl = backproject_Z(ZL, CL);
    XYZlm = median(XYZl); 

    [a, b, c] = find_major_plane(XYZl(:,1), XYZl(:,2), XYZl(:,3), 1000, 20);
    thrPlane = 100;
    Err = abs(XYZl(:,1).*a+XYZl(:,2).*b + c - XYZl(:,3));%    
    XYZl = XYZl(Err < thrPlane, :);    
    
    
%    
    %[v, d] = eigs(XYZl'*XYZl);
    %A = v'*XYZm';    
    %figure; scatter(A(1,:), A(2,:), '.')


    Rini = rodrigues([atan(b), -atan(a), 0]);       
    Tini = [1 0 0 XYZlm(1); 0 1 0 XYZlm(2); 0 0 1 XYZlm(3); 0 0 0 1];
    Mnew = Tini*[Rini, [0 0 0]'; 0 0 0 1];

    light = 1000;
    [Zx, Ix, ~] = render_CAD_model(f, v, CL, CR, Mnew, h, w, minZ, maxZ, 1/light);
    Zx(Zx > maxZ) = nan;
    Ix = uint8(Ix(:,:,1)*255);
    Zx(Ix < threshC) = nan;
    %figure; imshow(Ix); title('Rendered Pre-Aligned');
    %figure; imshow(Zx, [minZ maxZ]); title('Rendered Pre-Aligned Depth'); colormap(gca, pink);
    
    

    

    if robust_sampling > 0
        Ix(isnan(Ix)) = 0;
        Ix(Ix > 255) = 255;
        [Lx, Ly] = gradient(single(Ix));
        Ig = sqrt(Lx.^2 + Ly.^2);
        clear Lx Ly;
        Mask = imdilate(Ig>=threshG, [1 1 1; 1 1 1; 1 1 1]);
        
        %figure; imshow(Mask); 
    
        Z2x = Zx;
        Z2x(rand(size(Z2x)) > robust_sampling) = nan;
        Zx(~Mask) = nan;
        Zx(~isnan(Z2x)) = Z2x(~isnan(Z2x));
        clear Z2x Mask;
    
    end
    figure; imshow(Zx, [minZ maxZ]); title('Rendered Pre-Aligned Depth'); colormap(gca, pink);
%%
    XYZm = backproject_Z(Zx, CL);
        
    figure(); 
    scatter3(XYZl(:,1), XYZl(:,3), -XYZl(:,2), '.', 'r');
    hold on
    scatter3(XYZm(:,1), XYZm(:,3), -XYZm(:,2), '.', 'b');
    axis equal
    title('Initial Alignment (before ICP)');      
    drawnow;
    

    XYZm = XYZm(:,1:3);
    XYZl = XYZl(:,1:3);

    tic
        [TR, TT, ~, ~] = icp2(XYZl', XYZm', 100, 'Verbose', true, 'WorstRejection', 0.2, 'Matching', 'kDtree');
    toc
    M = [TR, TT; [0 0 0 1]];
    
        % Visualization
    light = 1000; % lumen (kinda lamp strength)
    [~, Iz, ~] = render_CAD_model(f, v, CL, CR, M*Mnew, h, w, minZ, maxZ, 1/light);
    Iz(isnan(Iz)) = 0;
    Iz(Iz > 1) = 1;
    Iv = L/255;
    Iv(:,:,3) = Iz(:,:,1);
    Iv(:,:,2) = (L/255 + Iz(:,:,1))/2;
    

    
    figure();imshow(Iv)

