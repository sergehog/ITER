% Developed by Michalis Zervos - All rights reserved
% http://michal.is/projects/phong-reflection-model-matlab/

% Sample application using the Phong reflection model
% Simulating the light/shade pattern of the moon over a month
 

% Image parameters
imgSize = 256;
circRadius = 90;

% Phong model parameters
ka = 0.2; 
kd = 0.6;
ks = 0.5;
alpha = 20;

% Light source (Lm) and viewer direction (V)
Lm = zeros(1,1,3);
V = zeros(1,1,3);
V(1,1,:) = [ 0, 0, 1];

normals = sphereNormals(imgSize,circRadius);
I = (normals(:,:,3) > 0) * 0.8;

figure;
step = 360/28;
for day = 0:27
	theta = day * step;
	
	% Light source - sun (Lm)
	Lm = computeLm(theta);
	
	% Direction that a perfectly reflected ray of light would take from
	% each point on the surface (Rm)
	Rm = computeRm(normals, Lm);
	
	% Compute the shading of each pixel according to the phong model
	Ip = phong(I, normals, ka, kd, ks, Lm, Rm, V, alpha );

	figure; imshow(Ip); drawnow;
end



