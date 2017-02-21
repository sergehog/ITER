function [f, v] = load_CAD_model(filename, RTmat)
if nargin < 2
    RTmat = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
end
if nargin < 1
    filename = 'STL\CLS Mock-Up kokoonpano.stl';
end

[v, f, ~, ~, ~] = stlread(filename, true);
f = uint64(f);
v = single(v);
%v = v - repmat(mean(v), [size(v,1) 1]);
%v = v*1000; % convert from meters to mm
% %Rmat = rodrigues([0,pi,pi]);
%Rmat = rodrigues([0,0,0]);
%Rmat = [Rmat, [0 0 0]'; 0 0 0 1];
%Tmat = [1 0 0 50; 0 1 0 300; 0 0 1 0; 0 0 0 1];
%Rmat = rotation(xa, ya, za);
%RTmat = Tmat*Rmat;
v(:,4) = 1;
v = v * RTmat';
v = v(:,1:3);
% % Remove too remote vertices
% %v(v(:,3) < -500, :) = nan;

