% Developed by Michalis Zervos - All rights reserved
% http://michal.is/projects/phong-reflection-model-matlab/

function [Rm] = computeRm(normals, Lm)

Rm = zeros(size(normals));

for i = 1:size(normals,1)
    for j = 1:size(normals,2)
        % u = Normal vector
        % v = Lm vector
        
        u = reshape(normals(i,j,:), 3, 1);
        v = reshape(Lm, 3, 1);
        
        if (norm(u) > 0)
            % Project Lm(vector v) on N(vector u) == dot(u,v)*u
            % (Note: Lm is pointing towards light source)
            % Then extend it (twice the length) and subtract Lm
            % This way we create a parallelogram and the computed vertex
            % (vector since origin at 0,0,0) is vector Rm.
            
            r = (2 * dot(u,v) * u - v);
            r = r / norm(r);
            
            % Discard light falling behind the surface
            % Normal and Rm facing the same direction (towards viewer)
            if (r(3) > 0 && u(3) > 0)
                Rm(i,j,:) = r;
            end
        end
    end
end

