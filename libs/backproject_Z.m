function [XYZpoints] = backproject_Z(Z, C)
[h, w, ~] = size(Z);
[X, Y] = meshgrid(0:(w-1), 0:(h-1));
%Cinv = [C(1:3, 1:3)', -C(1:3, 1:3)'*C(1:3,4)];
Cinv = inv([C(1:3, 1:3), C(1:3,4); 0 0 0 1]);

XYZpoints = [X(:), Y(:), Z(:)];
XYZpoints(:,1) = XYZpoints(:,1).*XYZpoints(:,3);
XYZpoints(:,2) = XYZpoints(:,2).*XYZpoints(:,3);
XYZpoints(:,4) = 1;
%XYZpoints = XYZpoints * pinv(C)';
XYZpoints = (Cinv * XYZpoints')';
%XYZpoints(:,1) = XYZpoints(:,1)./XYZpoints(:,3);
%XYZpoints(:,2) = XYZpoints(:,2)./XYZpoints(:,3);
XYZpoints = XYZpoints(find(~isnan(XYZpoints(:,1))), 1:3);


