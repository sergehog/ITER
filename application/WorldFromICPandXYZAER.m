function Xworld = WorldFromICPandXYZAER(Micp, XYZAERfeedback, HandEye)
%xyzaer2rotm = @(x) ([angle2dcm( deg2rad(x(6)), deg2rad(x(5)),deg2rad(x(4)) ,'XYZ')', [x(1) x(2) x(3)]'; 0 0 0 1]);

HandPosition = xyzaer2rotm(XYZAERfeedback);

Xworld = HandPosition*HandEye*Micp;
