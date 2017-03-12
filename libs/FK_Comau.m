function [pXYZ, DH] = FK_Comau ( q_vec)
a = [0.4 0.750 0.25 0 0 0];
alpha = (pi/2)*[1 0 1 -1 1 0];
d = [0  0  0  0.8124  0  0 ] ;
NJ = length(q_vec);
A  = zeros(4,4,NJ);
DH = eye(4);
for jt = 1:NJ
    
    q_vecjt     = q_vec(jt);
    alpjt   = alpha (jt);
    ajt     = a(jt);
    djt     = d(jt);
    
    A(:,:,jt) = [cos(q_vecjt)   -sin(q_vecjt)*cos(alpjt)    sin(q_vecjt)*sin(alpjt)     ajt*cos(q_vecjt);...
                sin(q_vecjt)    cos(q_vecjt)*cos(alpjt)     -cos(q_vecjt)*sin(alpjt)    ajt*sin(q_vecjt);...
                0           sin(alpjt)              cos(alpjt)              djt;...
                0           0                       0                       1];
            
   
    DH = DH * A(:,:,jt);
end
%ZYZ angles
%the angle range between (-pi/2,pi/2 )
RzR =atan2(DH(2,3),DH(1,3));   
RyR=atan2(sqrt(DH(1,3)^2+DH(2,3)^2),DH(3,3)); 
RZR=atan2(DH(3,2),-DH(3,1));

pXYZ = [DH(1:3,4);RzR;RyR;RZR];

DH(1:3, 4) = DH(1:3, 4) * 1000;
