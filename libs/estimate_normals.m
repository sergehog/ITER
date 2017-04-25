% Estimates Normals using triangular mesh representation, where 
% f - faces and v - vertices 
%
% [NF] = estimate_normals(f, v)
% estimates normals for every face
% 
% [NF, NV] = estimate_normals(f, v)
% interpolates normals also for every vertex
%

function [NF, varargout] = estimate_normals(f, v)
% Estimate Face Normals 

NF = zeros(size(f)); % normals per face
for i=1:size(f,1)
    p1 = v(f(i,1), :);
    p2 = v(f(i,2), :);
    p3 = v(f(i,3), :);
    n = cross(p2-p1, p3-p1);
    NF(i,:) = n./norm(n);
end

if nargout > 1
    % Estimate normals per vertex
    NV = zeros(size(v)); % normals per vertex
    nn = zeros([size(v,1) 1]);
    for i=1:size(f,1)
        n = NF(i,:);
        NV(f(i,1),:) = NV(f(i,1),:) + n;
        NV(f(i,2),:) = NV(f(i,2),:) + n;
        NV(f(i,3),:) = NV(f(i,3),:) + n;    
        nn(f(i,1)) = nn(f(i,1)) + 1;
        nn(f(i,2)) = nn(f(i,2)) + 1;
        nn(f(i,3)) = nn(f(i,3)) + 1;
    end

    for i=1:size(v,1)
        NV(i,:) = NV(i,:)./nn(i);
    end
    
    varargout{1} = NV;
end