clear
close all
clc

addpath(genpath('..\libs'));
load('..\application\stereoParams2.mat');

KL = stereoParams.CameraParameters1.IntrinsicMatrix';
CL = single([KL*[eye(3), [0 0 0]']; 0 0 0 1]);
w = 960;
h = 540;
%
filename = '..\STL\Knuckle_cut_to_size.stl'; 
%R = rodrigues([pi/2 0 0]);  T = [6705 360 290]; RTmat = [R, -R*T'; 0 0 0 1];
%RTmat = [rodrigues([0 0 1.2]), [237 8 -12]'; 0 0 0 1]*[R, -R*T'; 0 0 0 1];
RTmat = [eye(3), [0 0 0]'; 0 0 0 1];
[f, v] = load_CAD_model(filename, RTmat);
light = 600;
%%
close all
clc
XYZ = [];
%%

minZ = 200;
maxZ = 2000;
M = [1 0 0 0;
     0 1 0 0;
     0 0 1 500;
     0 0 0 1];
CL2 = [CL*M];     
CL2 = CL2(1:3, :);
[Z, I, ~] = render_CAD_model(f, v, CL2, 0, eye(4), h, w, minZ, maxZ, 1/light, 0);
figure; imshow(Z, [400 600]); colormap(gca, pink)

I = I(:,:,1);
I(isnan(I)) = 0;
I = 255 * I / max(I(:));
figure; imshow(uint8(I), [0 255]); 
[Ix Iy] = gradient(I);
Ig = abs(Ix) + abs(Iy);
%figure; imshow(Ig, [0 1])
%
Z(isnan(Z)) = 0;
[Zx, Zy] = gradient(Z);
%[Zxx, Zxy] = gradient(Zx);
%[Zyx, Zyy] = gradient(Zy);
%
G = (abs(Zx) + abs(Zy)) > 3;
%G = (abs(Zxx) + abs(Zxy) + abs(Zyy)) > 3;
%figure; imshow(G, [])
Z2 = Z;
%Z2(rand(size(Z2)) .* (1+abs(Zx) + abs(Zy)) < 1) = nan;
Z2(rand(size(Z2)) > 0.001) = nan;
Z2(G) = Z(G);
Z2(Ig > 1) = Z(Ig > 1);
Z2(Z == 0) = nan;

figure; imshow(Z2, [400 600]); colormap(gca, pink); title('Sampled Depth');
%%
XYZm = backproject_Z(Z2, CL2);
XYZm = XYZm(find(XYZm(:,3) < 5), :);

%
%XYZ = [XYZ; XYZl];
%
figure; scatter3(XYZm(:,1), XYZm(:,2), XYZm(:,3), '.', 'b'); 
set(gca, 'YDir', 'reverse');
xlabel('x'); ylabel('y'); zlabel('z')
axis equal
%%
%savepcd('points.txt', XYZ');
P = pointCloud(XYZm);
P = pcdownsample(P, 'gridAverage', 4);
figure; scatter3(P.Location(:,1), P.Location(:,2), P.Location(:,3), '.', 'b'); axis equal
set(gca, 'YDir', 'reverse');
xlabel('x'); ylabel('y'); zlabel('z')

%pcwrite(P,'points.ply','PLYFormat','binary');
%%
f = fopen('points.bin', 'w');
fwrite(f, P.Count, 'uint32');
for i=1:P.Count
    fwrite(f, P.Location(i,1), 'single');
    fwrite(f, P.Location(i,2), 'single');
    fwrite(f, P.Location(i,3), 'single');
end
fclose(f);
    
%%
P2 = pcread('points2.ply');
figure(); scatter3(P2.Location(:,1), P2.Location(:,2), P2.Location(:,3), '.', 'b'); axis equal

