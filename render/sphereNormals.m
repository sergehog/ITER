% Developed by Michalis Zervos - All rights reserved
% http://michal.is/projects/phong-reflection-model-matlab/

function [normals] = sphereNormals(imgSize,circRadius)

% Center
cx = imgSize / 2;
cy = cx;

normals = zeros(imgSize,imgSize,3);


% For each pixel, translate to get sphere center at (0,0)
for i = 1:imgSize
    x = i - cy;
    for j = 1:imgSize
        y = j - cx;
        % If it's inside the circle (or on circle)
        if (x^2 + y^2 <= circRadius^2)
            % Compute z
            z = sqrt(circRadius^2 - x^2 - y^2);
            % Norm is the normalized vector [(0,0,0), (x,y,z)]
            normals(i, j, :) = [x y z] ./ norm([x y z]);
        end
    end
end


end