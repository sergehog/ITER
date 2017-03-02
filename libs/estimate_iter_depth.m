function [ZL, ZR] = estimate_iter_depth(L, R, CL, CR, minZ, maxZ, layers, threshG, threshC, robust_sampling, adaptive_theshold)
if nargin < 10
    robust_sampling = 0;
end
if nargin < 11
    adaptive_theshold = 0;
end

[Lx, Ly] = gradient(L);
[Rx, Ry] = gradient(R);
L2(:,:,1) = Lx;
L2(:,:,2) = Ly;
L2(:,:,3) = L*0.1;
R2(:,:,1) = Rx;
R2(:,:,2) = Ry;
R2(:,:,3) = R*0.1;
maxZ2 = maxZ*1.5; % increase actual Z-range in order to properly threshould out too distant objects
%clear Lx Ly Rx Ry;
%figure; imshow(uint8(L2*100))
%
%radius = 5;
%tic
%[AL, AR, EL, ER] = patchmatch_unrectified(L2, R2, L, R, CL, CR, radius, minZ, maxZ);
%toc
%ZL = AL(:,:,1).*X + AL(:,:,2).*Y + AL(:,:,3);
%ZR = AR(:,:,1).*X + AR(:,:,2).*Y + AR(:,:,3);

%tic
%sigma = 0.02;
%alpha = 0.77;
%layers = 400;
[CostL] = depth_estimation(minZ, maxZ2, layers, L2, CL, R2, CR);
%CostLf = recursive_bilateral(CostL, L, sigma, alpha);
%CostLf = recursive_gaussian(CostL, alpha);
CostL(CostL > 20) = 20;
CostLf = fast_average(CostL, 5);
clear CostL
DispL = wta_simple(CostLf, 0, 1);
clear CostLf
%figure; imshow(DispL, [0 layers]); colormap(pink); title('Raw Depth');
[CostR] = depth_estimation(minZ, maxZ2, layers, R2, CR, L2, CL);
%CostRf = recursive_bilateral(CostR, R, sigma, alpha);
%CostRf = recursive_gaussian(CostR, alpha);
CostR(CostR > 20) = 20;
CostRf = fast_average(CostR, 5);
clear CostR
DispR = wta_simple(CostRf, 0, 1);
clear CostRf

%toc
clear mex

%figure; imshow(DispL, [0 layers]); colormap(pink)
%figure; imshow(ZL, [minZ maxZ]); colormap(pink); title('Estimated Left Depth (mm)');
%%
%z_thr = 1500;
ZL = 1 ./ ((DispL / layers)*(1/minZ - 1/maxZ2) + 1/maxZ2);
ZR = 1 ./ ((DispR / layers)*(1/minZ - 1/maxZ2) + 1/maxZ2);
%figure; imshow(EL, [0 10]); colormap(jet)

[ValidL, ValidR] = left2right(ZL, CL, ZR, CR, 5, minZ, maxZ2);
%[ValidL, ValidR] = ltr_planar(AL, AR, 10, 10);
ZL(~ValidL) = nan;
ZR(~ValidR) = nan;
ZL(ZL > maxZ) = nan;
ZR(ZR > maxZ) = nan;

%figure; imshow(ZL, [minZ maxZ]); colormap(pink); title('Estimated Depth Map');

if adaptive_theshold>0
    La = fast_average(L, 20);
    ZL(L(:,:,1) < (La + adaptive_theshold)) = nan;
end
Lg = sqrt(Lx.^2+Ly.^2);

if robust_sampling > 0    
    ZL2 = ZL;
    ZL2(rand(size(ZL2)) > robust_sampling) = nan;
    ZL(Lg < threshG) = nan;
    ZL(~isnan(ZL2)) = ZL2(~isnan(ZL2));
    Rg = sqrt(Lx.^2+Ly.^2);
    ZR(Rg < threshG) = nan;
else
    ZL(Lg < threshG) = nan;
end

ZL(L < threshC) = nan;
ZR(R < threshC) = nan;
ZL(L >= 255) = nan;
ZR(R >= 255) = nan;
