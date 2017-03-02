function [Zq, Iq, Nq] = render_CAD_model(f, v, CL, CR, M, h, w, minZ, maxZ, aaa, doLeft2right)
if nargin < 10
    aaa = 0.001;
end
if nargin < 11
    doLeft2right = 1;
end



CLm = CL*M;
%Cini = CL * M*Tini*Rini;
%Cini = CL*M*pinv(CL)*Cini;
% New Rendering with normals
v2 = v;
v2(:,4) = 1;
v2 = v2*(CLm)';
v2(:,1) = v2(:,1)./v2(:,3);
v2(:,2) = v2(:,2)./v2(:,3);
[NF, NV] = estimate_normals(f, v);
%L = v./repmat(sqrt(v(:,1).^2 + v(:,2).^2 + v(:,3).^2), [1 3]);
C = zeros(size(NV));

for i=1:size(v, 1)
    a = v2(i,:); % 'a' is location of a light source  
    %a(1) = a(1) + 1000; 
    b = abs(NV(i,:));
    C(i, 1) = ((dot(a(:), b(:))./(norm(a)*norm(b))))/((aaa*norm(a))^2);
    
    %sqrt()/1000;
end
C(:,2) = C(:,1);
C(:,3) = C(:,1);
C = C;

[Iq, Zq] = triangle_rasterization(f, v2, w, h, single(C), reshape([0 0 0], [1 1 3]));
[Nq, ~] = triangle_rasterization(f, v2, w, h, single(NV), reshape([0 0 0], [1 1 3]));
clear mex
%figure; imshow(Zq, [minZ maxZ]); colormap(pink); title('Rendered Depth');
%figure; imshow(abs(Iq));
%Iq(isnan(Iq)) = 0;
%[Ix, Iy] = gradient(Iq);
%Mask = sum(abs(Ix)+abs(Iy), 3) > 0.1;
%Zx = Zq;
%Zx(~Mask) = nan;
%figure; imshow(Mask); 
%% Remove Occluded Parts
%CRini = CR*Tini*Rini;
if  doLeft2right > 0
    CRm = CR*M;
    % New Rendering with normals
    v2 = v;
    v2(:,4) = 1;
    v2 = v2*(CRm)';
    v2(:,1) = v2(:,1)./v2(:,3);
    v2(:,2) = v2(:,2)./v2(:,3);
    [~, ZRq] = triangle_rasterization(f, v2, w, h, v2(:,1:2)*0, reshape([0 0 0], [1 1 3]));
    clear mex
    [ValidLq, ValidRq] = left2right(Zq, CL, ZRq, CR, 5, minZ, maxZ);
    %figure; imshow(ValidLq); 
    %figure; imshow(ZRq, [minZ threshZ]); colormap(pink); title('Right Depth');
    Zq(~ValidLq) = nan;
    %Zx(isnan(ZL)) = nan;
    %figure; imshow(Zx, [minZ maxZ]); colormap(pink); title('Model Depth Invisibles removed');
end