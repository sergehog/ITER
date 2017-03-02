function [M, XYZl2] = simplified_ICP(L, R, CL, CR, minZ, maxZ, layers, threshG, threshC, robust_sampling, adaptive_thr, thrPlane, XYZm, WorstRejection)

   
tic
[ZL, ~] = estimate_iter_depth(L, R, CL, CR, minZ, maxZ, layers, threshG, threshC, robust_sampling, adaptive_thr);
toc

%figure; imshow(ZL, [minZ maxZ]); colormap(gca, jet); title('Estimated Depth');

XYZl = backproject_Z(ZL, CL);
[a, b, c] = find_major_plane(XYZl(:,1), XYZl(:,2), XYZl(:,3), 1000, 20);
Err = abs(XYZl(:,1).*a+XYZl(:,2).*b + c - XYZl(:,3));%    
XYZl = XYZl(find(Err < thrPlane), :);    

% Filtering Observed Points
P2 = pointCloud(XYZl);
P2 = pcdenoise(P2);
P2 = pcdownsample(P2, 'gridAverage', 4);
if P2.Count > size(XYZm,1)
    P2 = pcdownsample(P2, 'random', size(XYZm,1)/P2.Count);
end
XYZl = P2.Location;


%%
tic
[TR, TT, ~, ~] = icp2(XYZm', XYZl', 2000, 'Verbose', true, 'WorstRejection', WorstRejection, 'Matching', 'kDtree');
toc
M = [TR, TT;];

XYZl2 = XYZl;
XYZl2(:,4) = 1;
XYZl2 = XYZl2*M';
