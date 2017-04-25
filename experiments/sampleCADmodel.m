% Sample CAD model using orthographic projection
%
% [XYZs, NVs, Z, NV] = sampleCADmodel(f, v, M, h, w, rand_sampling, zgrad_thr)
% XYZs - sampled points 
% NVs - their normals
% Z - rendered depth map (for visualization)
% NV - rendered normals map (for visualization)

function [XYZs, NVs, Z, NV] = sampleCADmodel(f, v, M, h, w, rand_sampling, zgrad_thr)
[~, nv] = estimate_normals(f, v);

v2 = v;
v2(:,4) = 1;
v2 = v2 * M';
[NV, Z] = triangle_rasterization(f, v2, w, h, single(nv), reshape([0 0 0], [1 1 3]), 1);


Z(isnan(Z)) = 0;
[Zx, Zy] = gradient(Z);
[Zxx, Zxy] = gradient(Zx);
[Zyx, Zyy] = gradient(Zy);

G = (abs(Zxx) + abs(Zxy) + abs(Zyy)) > zgrad_thr; % second-order gradient thresholding
%figure; imshow(G, [])
Z2 = Z;
%Z2(rand(size(Z2)) .* (1+abs(Zx) + abs(Zy)) < 1) = nan;
Z2(rand(size(Z2)) > rand_sampling) = nan;
Z2(G) = Z(G);
%Z2(Ig > 1) = Z(Ig > 1);
Z2(Z == 0) = nan;

[X, Y] = meshgrid(0:(w-1), 0:(h-1));
Minv = inv([M(1:3, 1:3), M(1:3,4); 0 0 0 1]);

XYZs = [X(:), Y(:), Z2(:)];
XYZs(:,4) = 1;
XYZs = (XYZs * Minv');
NVs = reshape(NV, [h*w 3]);
NVs = NVs(find(~isnan(XYZs(:,1))), 1:3);
XYZs = XYZs(find(~isnan(XYZs(:,1))), 1:3);



