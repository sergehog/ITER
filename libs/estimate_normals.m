function [NF, NV] = estimate_normals(f, v)
%% Estimate Face Normals 

NF = zeros(size(f)); % normals per face
for i=1:size(f,1)
    p1 = v(f(i,1), :);
    p2 = v(f(i,2), :);
    p3 = v(f(i,3), :);
    n = cross(p2-p1, p3-p1);
    NF(i,:) = n./norm(n);
end

% Face Culling
i2 = 1;
for i=1:size(f,1)
     %p1 = v(f(i,1), :);
     %p1(3) = 0;
     p1 = [0 0 -1];
     n = NF(i,:);
     if dot(p1, n) >= -0.1
         f(i2,:) = f(i,:);
         NF(i2,:) = NF(i,:);
         i2 = i2+1;
     end
end
f = f(1:i2-1,:);
NF = NF(1:i2-1,:);

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