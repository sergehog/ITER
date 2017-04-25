function [Zq, Iq, Nq] = render_CAD_model(f, v, CL, CR, M, h, w, light, doLeft2right)
if nargin < 8
    light = 1000;
end
if nargin < 9
    doLeft2right = 0;
end


CLm = CL*M;


v2 = v;
v2(:,4) = 1;
v2 = v2*(CLm)';
v2(:,1) = v2(:,1)./v2(:,3);
v2(:,2) = v2(:,2)./v2(:,3);

% Rendering with phong reflection - coloring
[~, NV] = estimate_normals(f, v); 


% prepare RGB color attributes for each vertex
C = zeros(size(NV));
% location of a light source (we use ring-light)
light_sourse = [0 0 0]; 

for i=1:size(v, 1)
    normal = -(NV(i,:));
    normal = normal/norm(normal);    
    light_dir = v2(i,:)-light_sourse;     
    light_dir = light_dir/norm(light_dir);
    
    lambertian = max(dot(light_dir,normal), 0.0);
    diffuse = 0.1;
    color = max(diffuse,lambertian);
    color = color^1.2;
    C(i, 1) = min(1,max(0, color));
    
    %b = NV(i,:);
    %C(i, 1) = ((dot(light_dir(:), b(:))./(norm(light_dir)*norm(b))))/(1/light*(norm(light_dir))^2);    
end

%for i=1:size(v, 1)
%    a = v2(i,:) - light_sourse;     
%    b = abs(NV(i,:));
%    %b = NV(i,:);
%    C(i, 1) = ((dot(a(:), b(:))./(norm(a)*norm(b))))/(1/light*(norm(a))^2) + 0.1;    
%end

C(:,2) = C(:,1);
C(:,3) = C(:,1);

[Iq, Zq] = triangle_rasterization(f, v2, w, h, single(C), reshape([0 0 0], [1 1 3]));
[Nq, ~] = triangle_rasterization(f, v2, w, h, single(NV), reshape([0 0 0], [1 1 3]));
clear mex

%% Remove Occluded Parts

if  doLeft2right > 0
    CRm = CR*M;
    
    v2 = v;
    v2(:,4) = 1;
    v2 = v2*(CRm)';
    v2(:,1) = v2(:,1)./v2(:,3);
    v2(:,2) = v2(:,2)./v2(:,3);
    [~, ZRq] = triangle_rasterization(f, v2, w, h, v2(:,1:2)*0, reshape([0 0 0], [1 1 3]));
    clear mex
    minZ = min(Zq(:));
    maxZ = max(Zq(:));
    [ValidLq, ~] = left2right(Zq, CL, ZRq, CR, 5, minZ, maxZ);
    
    Zq(~ValidLq) = nan;    
end