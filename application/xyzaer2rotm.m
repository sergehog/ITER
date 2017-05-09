function M = xyzaer2rotm(x) 

M = [angle2dcm( deg2rad(x(6)), deg2rad(x(5)),deg2rad(x(4)) ,'XYZ')', [x(1) x(2) x(3)]'; 0 0 0 1];
