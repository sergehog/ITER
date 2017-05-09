function xyzaer = rotm2xyzaer(M) 

[a, b, c] = dcm2angle(M(1:3,1:3)');
xyzaer = [M(1:3, 4); rad2deg(c); rad2deg(b); rad2deg(a)];