% Developed by Michalis Zervos - All rights reserved
% http://michal.is/projects/phong-reflection-model-matlab/

function [Ip, Ia, Id, Is] = phong(I, normals, ka, kd, ks, Lm, Rm, V, alpha)

% Ambient light
Ia = ka * I;

% Diffuse light
Lm = repmat(Lm, size(I,1), size(I,2));
Id = kd * dot(Lm, normals, 3) .* I;

% Specular light
V = repmat(V, size(I,1), size(I,2));
Is = ks * dot(Rm, V, 3).^alpha .* I;

% Delete negative values that might even out the ambient component
Id = (Id > 0).*Id;
Is = (Is > 0).*Is;

Ip = Ia + Id + Is;

end