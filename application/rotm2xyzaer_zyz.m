function xyzaer = rotm2xyzaer_zyz(M) 

[abc] = 180*rotm2eul(M(1:3,1:3)', 'ZYZ')/pi;
xyzaer = [M(1:3, 4)', -abc(3), -abc(2) -abc(1)];
