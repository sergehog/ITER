function [M, ZL, Zm] = full_ICP(f, v, L, R, CL, CR, minZ, maxZ, layers, threshG, threshC, robust_sampling, adaptive_thr, WorstRejection, thrPosPlane, thrNegPlane)
    random_points = 2000;
    iterations = 1000;
    % Estimate Depth from stereo 
    [h, w, ~] = size(L);
    [ZL, ~] = estimate_iter_depth(L, R, CL, CR, minZ, maxZ, layers, threshG, threshC, robust_sampling);    
    
    XYZl = backproject_Z(ZL, CL);
    
    % find majour plane in the scene in camera coordinate system
    [a, b, c] = find_major_plane(XYZl(:,1), XYZl(:,2), XYZl(:,3), 1000, 20);
    
    
    % filter out too distant points
    Err = (XYZl(:,1).*a+XYZl(:,2).*b + c - XYZl(:,3));%    
    XYZl = XYZl((Err < thrPosPlane & Err > -thrNegPlane), :);    

    XYZlm = median(XYZl); % centroid of a cloud    

    
    % coarse alignment matrix
    Rini = rodrigues([atan(b), -atan(a), 0]);       
    Tini = [1 0 0 XYZlm(1); 0 1 0 XYZlm(2); 0 0 1 XYZlm(3); 0 0 0 1];
    Mnew = Tini*[Rini, [0 0 0]'; 0 0 0 1];

    
    % render pre-aligned model
    % Zm - depth of rendered model, Im - rendered image 
    [Zm, Im, ~] = render_CAD_model(f, v, CL, CR, Mnew, h, w, 1000, 1);
    %Zm(Zm > maxZ) = nan;
    Im = uint8(Im(:,:,1)*255/max(max(Im(:,:,1))));
    %Zm(Im < threshC) = nan;
    %figure; imshow(Im, []); title('Coarsely-aligned Image');        
    
    % filter out planar points
    if robust_sampling > 0
        Im(isnan(Im)) = 0;        
        [Lx, Ly] = gradient(single(Im));
        Ig = sqrt(Lx.^2 + Ly.^2);
        clear Lx Ly;
        Mask = imdilate(Ig>=threshG, [1 1 1; 1 1 1; 1 1 1]);

        Z2x = Zm;
        Z2x(rand(size(Z2x)) > robust_sampling/10) = nan;
        Zm(~Mask) = nan;
        Zm(~isnan(Z2x)) = Z2x(~isnan(Z2x));
        clear Z2x Mask;            
    end
    
    %figure; imshow(ZL, [min(ZL(:))*0.8 max(ZL(:))*1.2]); title('Estimated Depth'); colormap(gca, pink)
    %figure; imshow(Zm, [min(Zm(:))*0.8 max(Zm(:))*1.2]); title('Coarsely Aligned Rendered Depth'); colormap(gca, pink)

    
    % prepare point cloud from depth map    
    XYZm = backproject_Z(Zm, CL);
    
    % filter too distant points
    Err = (XYZm(:,1).*a+XYZm(:,2).*b + c - XYZm(:,3));%    
    XYZm = XYZm((Err < thrPosPlane & Err > -thrNegPlane), :);    
    
    Pm = pointCloud(XYZm);    
    Pm = pcdownsample(Pm, 'gridAverage', 5);
    if Pm.Count > random_points
        Pm = pcdownsample(Pm, 'random', random_points/Pm.Count);
    end
    XYZm = Pm.Location;
    clear Pm;
    
    
    Pl = pointCloud(XYZl);    
    Pl = pcdownsample(Pl, 'gridAverage', 5);
    if Pl.Count > random_points
        Pl = pcdownsample(Pl, 'random', random_points/Pl.Count);
    end
    XYZl = Pl.Location;
    clear Pl;
    
    %figure;
    %scatter3(XYZm(:,1),XYZm(:,3),XYZm(:,2),'.','b');
    %hold on
    %scatter3(XYZl(:,1),XYZl(:,3),XYZl(:,2),'.','r');
    %title('Before alignment');
    

    XYZm = XYZm(:,1:3);
    XYZl = XYZl(:,1:3);

    if 1==2
        [TR, TT, ~, ~] = icp2(XYZm', XYZl', iterations, 'Verbose', true, 'WorstRejection', WorstRejection, 'Matching', 'kDtree');
        M = ([TR, TT; [0 0 0 1]]);    
        M = inv(M)*Mnew;
        XYZl2 = XYZl * TR + repmat(TT', [size(XYZl, 1) 1]);
    
    else
        [TR, TT, ~, ~] = icp2(XYZl',XYZm', iterations, 'Verbose', true, 'WorstRejection', WorstRejection, 'Matching', 'kDtree');
        M = ([TR, TT; [0 0 0 1]]);    
        M = M*Mnew;
        XYZl2 = XYZl * TR' + repmat(-TT'*TR', [size(XYZl, 1) 1]);
    end
    
    %figure;
    %scatter3(XYZm(:,1),XYZm(:,3),XYZm(:,2),'.','b');
    %hold on
    %scatter3(XYZl2(:,1),XYZl2(:,3),XYZl2(:,2),'.','r');
    %title('After alignment');