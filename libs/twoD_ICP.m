function [M, ZL, Zm] = twoD_ICP(f, v, L, R, CL, CR, minZ, maxZ, layers, threshG, threshC, robust_sampling, adaptive_thr, WorstRejection, thrPosPlane, thrNegPlane)
    random_points = 2000;
    iterations = 100;
    % Estimate Depth from stereo 
    [h, w, ~] = size(L);
    [ZL, ~] = estimate_iter_depth(L, R, CL, CR, minZ, maxZ, layers, threshG, threshC, robust_sampling);    
    
    XYZl = backproject_Z(ZL, CL);
    XYZlm = median(XYZl); % centroid of a cloud    
    
    % find majour plane in the scene in camera coordinate system
    [a, b, c] = find_major_plane(XYZl(:,1), XYZl(:,2), XYZl(:,3), 1000, 20);
    
    Rini = rodrigues([atan(b), -atan(a), 0]);       
    Mini = [Rini', - Rini'*[0 0 c]'];
    
    %XYZl = XYZl * Rini - repmat([0 0 c]*Rini, [size(XYZl,1) 1]);
    XYZl(:,4) = 1;
    XYZl = (Mini*XYZl')';
    
    figure;
    scatter3(XYZl(:,1),XYZl(:,3),XYZl(:,2),'.','r');
    axis equal

    XYZl = XYZl(abs(XYZl(:,3)) < thrPosPlane, :);
    

   
    % render pre-aligned model
    % Zm - depth of rendered model, Im - rendered image 
    M2 = [1 0 100 0; 0 1 0 0; 0 0 1 600; 0 0 0 1];
    [Zm, Im, ~] = render_CAD_model(f, v, CL, CR, M2, h, w, 1000, 1);
    %[~, Im, ~] = render_CAD_model(f, v, CL2, CR2, M2, h*2, w*2, 100, 1);
    %Im = imresize(Im, [h w], 'bilinear');
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
    XYZm = backproject_Z(Zm, CL*M2);
    %XYZm = XYZm(abs(XYZm(:,3)-600) < 10, :);
    XYZm = XYZm(abs(XYZm(:,3)) < 10, :);
     
    XYl = XYZl(:,1:2);
    XYm = XYZm(:,1:2);
    
    figure;
    scatter(XYm(:,1),XYm(:,2),'.','b');
    hold on
    scatter(XYl(:,1),XYl(:,2),'.','r');
    title('Before alignment');
    
    [TR,TT,data] = icp(double(XYm)', double(XYl)', iterations);
    data = data';
    
    Rt = [TR', [0 0]', TT; 0 0 1 0; 0 0 0 1];
    Rti = inv(Rt);    
    M = inv([Mini; 0 0 0 1]) * Rti;
    M(1:3, 1:3) = rodrigues(rodrigues(M(1:3, 1:3)));
    
    
    figure;
    scatter(XYm(:,1),XYm(:,2),'.','b');    
    hold on
    scatter(data(:,1),data(:,2),'.','r');
    axis equal
    title('After alignment');