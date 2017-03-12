function RT = xyzabc2rtmat(x)
%RT = ([eul2rotm([x(4) x(5) x(6)], 'ZYX'), [x(1) x(2) x(3)]'; 0 0 0 1]);
RT = ([eul2rotm([x(4) x(5) x(6)], 'ZYZ'), [x(1) x(2) x(3)]'; 0 0 0 1]);