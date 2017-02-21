function [ZL] = estimate_depth_fast(L, R, CL, CR, minZ, maxZ, layers, threshG, threshC, robust_sampling, adaptive_theshold)
if nargin < 10
    robust_sampling = 0;
end
if nargin < 11
    adaptive_theshold = 0;
end

La = fast_average(L, 20);

[Lx, Ly] = gradient(L);
[Rx, Ry] = gradient(R);
L2(:,:,1) = Lx;
L2(:,:,2) = Ly;
L2(:,:,3) = L*0.1;
R2(:,:,1) = Rx;
R2(:,:,2) = Ry;
R2(:,:,3) = R*0.1;
maxZ2 = maxZ*1.5;
[CostL] = depth_estimation(minZ, maxZ2, layers, L2, CL, R2, CR);
%CostLf = recursive_bilateral(CostL, L, sigma, alpha);
%CostLf = recursive_gaussian(CostL, alpha);
CostL(CostL > 20) = 20;
CostLf = fast_average(CostL, 6);
clear CostL
DispL = wta_simple(CostLf, 0, 1);
clear CostLf

ZL = 1 ./ ((DispL / layers)*(1/minZ - 1/maxZ2) + 1/maxZ2);
clear DispL;

if adaptive_theshold>0
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

