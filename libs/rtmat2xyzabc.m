function xyzabc = rtmat2xyzabc(x) 
%xyzabc = ([x(1,4) x(2,4) x(3,4), rotm2eul([x(1:3,1:3)], 'ZYX')]);
xyzabc = ([x(1,4) x(2,4) x(3,4), rotm2eul([x(1:3,1:3)], 'ZYZ')]);