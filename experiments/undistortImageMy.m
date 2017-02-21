function Iu = undistortImageMy(I1, params)
[h, w, ~] = size(I1);

F = params.RadialDistortion;
K = params.IntrinsicMatrix';

cam1_k1 = F(1);
cam1_k2 = F(2);
cam1_ox = K(1,3);
cam1_oy = K(2,3);
cam1_fx = K(1,1);
cam1_fy = K(2,2);

LT = zeros([h w 2]);

for yi=0:(h-1)
        
    for xi=0:(w-1)
        x1 = (xi + 1 - cam1_ox) / cam1_fx;
		y1 = (yi + 1 - cam1_oy) / cam1_fy;
        
        r1 = (x1*x1 + y1*y1);
		x1 = x1 * (1 + cam1_k1*r1 + cam1_k2*r1*r1);
		y1 = y1 * (1 + cam1_k1*r1 + cam1_k2*r1*r1);
		LT(yi + 1, xi + 1, 1) = (x1*cam1_fx + cam1_ox);
		LT(yi + 1, xi + 1, 2) = (y1*cam1_fy + cam1_oy);
    end
end


Iu = interp2(single(I1), LT(:,:,1), LT(:,:,2));