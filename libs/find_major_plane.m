% function [a, b, c] = find_major_plane(Xs, Ys, Zs, N, thr)
% Extract major planes from the scene. 
% Xs, Yz, Zs may contain NaNs
%
% N - number of RANSAC iterations
% thr - minimal distance between pixel and plane hypothesis to consider if
% pixel belongs to the plane

function [aa, bb, cc] = find_major_plane(Xs, Ys, Zs, N, thr)
x = [Xs, Ys];
x(:,3) = 1;
z = Zs;
clear y;

    bestPix = 0;
    bestA = [0,0,0];
    len = numel(z);
    
    for n=1:N
        i1 = randi([1 len]);
        i2 = i1;
        i3 = i2;
        while i2 == i1
            i2 = randi([1 len]);
        end    
        while i3==i1 || i3 == i2
            i3 = randi([1 len]);
        end
        X = [x(i1, :); x(i2, :);  x(i3, :)];
        Z = [z(i1); z(i2); z(i3)];    
        a = X \ Z;    
        E = x*a - z;
        E = abs(E);
        Pix = sum(E<thr);
        if Pix > bestPix
            bestPix = Pix;
            bestA = a;
        end          
    end
    
    %E = abs(x*bestA - z);
    %bestPix = sum(E<thr);
    %Ps(k) = bestPix;
    aa = bestA(1);
    bb = bestA(2);
    cc = bestA(3);
    
   % x = x(find(E>=thr), :);    
   % z = z(find(E>=thr));
   