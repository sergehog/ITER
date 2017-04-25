function [M, XYZl2, ZL] = simplified_ICP(L, R, CL, CR, minZ, maxZ, layers, threshG, threshC, robust_sampling, adaptive_thr, thrPosPlane, XYZm, WorstRejection, thrNegPlane)
if nargin < 15
    thrNegPlane = thrPosPlane;
end
   
[ZL, ~] = estimate_iter_depth(L, R, CL, CR, minZ, maxZ, layers, threshG, threshC, robust_sampling, adaptive_thr);

%figure; imshow(ZL, [minZ maxZ]); colormap(gca, jet); title('Estimated Depth');

XYZl = backproject_Z(ZL, CL);
[a, b, c] = find_major_plane(XYZl(:,1), XYZl(:,2), XYZl(:,3), 2000, 20);
disp(num2str([a, b, c]));
Err = (XYZl(:,1).*a+XYZl(:,2).*b + c - XYZl(:,3));%    
XYZl = XYZl(find(Err < thrPosPlane & Err > -thrNegPlane), :);    



% Filtering Observed Points
P2 = pointCloud(XYZl);
P2 = pcdenoise(P2);
P2 = pcdownsample(P2, 'gridAverage', 8);
if P2.Count > size(XYZm,1)
    P2 = pcdownsample(P2, 'random', size(XYZm,1)/P2.Count);
end
XYZl = P2.Location;


%%
METHOD = 1;
if METHOD == 1
    [TR, TT, ~, ~] = icp2(XYZl', XYZm', 1000, 'Verbose', true, 'WorstRejection', WorstRejection, 'Matching', 'kDtree');
    M1 = [TR, TT; 0 0 0 1];
    M1 = inv(M1);

    XYZl2 = XYZl;
    XYZl2(:,4) = 1;
    XYZl = (XYZl2*(M1)');
    XYZl = XYZl(:,1:3);
    [TR, TT, ~, ~] = icp2(XYZm', XYZl', 1000, 'Verbose', true, 'WorstRejection', WorstRejection, 'Matching', 'kDtree');
    M2 = [TR, TT; 0 0 0 1];

    XYZl(:,4) = 1;
    XYZl2 = XYZl*(M2)';

    M = M2*M1;
elseif METHOD == 2
    [TR, TT, ~, ~] = icp2(XYZl', XYZm', 1000, 'Verbose', true, 'WorstRejection', WorstRejection, 'Matching', 'kDtree');
    M = [TR, TT; 0 0 0 1];
    M = inv(M);

    XYZl2 = XYZl;
    XYZl2(:,4) = 1;
    XYZl2 = XYZl2*(M)';
    
else
    [TR, TT, ~, ~] = icp2(XYZm', XYZl', 1000, 'Verbose', true, 'WorstRejection', WorstRejection, 'Matching', 'kDtree');
    M = [TR, TT; 0 0 0 1];


    XYZl2 = XYZl;
    XYZl2(:,4) = 1;
    XYZl2 = XYZl2*(M)';
end