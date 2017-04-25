% Developed by Michalis Zervos - All rights reserved
% http://michal.is/projects/phong-reflection-model-matlab/

function [Lm] = computeLm(theta)

% Sun position
sunX = 2500*sin((30 + (theta))/360*2*pi); 
sunY = 3500*cos(theta/360*2*pi); 
sunZ = 10000*sin((theta-30)/360*2*pi);

% Set the light source at the sun position (normalized)
Lm(1,1,:) = [sunX, sunY ,sunZ]./(sqrt(sunX.^2 +sunY.^2 + sunZ.^2));

end
