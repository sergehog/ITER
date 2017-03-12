close all
clear 
clc

addpath(genpath('..\libs'));


%experiment = 'calib4'; 
%experiment = 'calib3'; 
experiment = 'e1'; 

if strcmp(experiment,'calib3')
    DATA = [1796.259 -988.560 835.209 27.493 111.587 23.788;
    1781.146 -1015.539 835.177 26.629 111.588 23.789;
    1765.625 -1042.294 835.216 25.764 111.588 23.787;
    1723.389 -1110.739 835.203 23.517 111.588 23.787;
    1660.529 -1202.687 835.207 20.403 111.587 23.788;
    1660.537 -1202.677 835.197 20.403 111.587 11.950;
    1660.533 -1202.679 835.235 20.403 111.587 5.372];
elseif strcmp(experiment,'calib4')
    DATA = [1963.370 -1381.642 898.949 19.611 110.392 3.158;
    %2139.669 -1088.839 898.924 27.775 110.392 3.159;
    1999.358 -1422.386 593.909 -20.418 114.959 -12.005;
    1855.277 -1301.006 597.831 -47.447 118.383 7.152;
    1873.134 -1444.215 539.327 -29.207 109.520 32.367;
    1664.698 -1574.885 418.620 -45.182 116.446 27.783;
    1721.682 -1615.142 612.783 -43.820 108.755 6.625;
    1429.939 -1504.572 499.325 -55.175 120.837 43.844;
    1016.596 -1737.278 231.631 -70.274 129.790 40.102;
    1015.149 -1698.499 128.126 -70.823 132.784 46.499;
    ];
elseif strcmp(experiment,'e1')
    DATA = [1066.614 -1659.587 1352.878 -65.212 89.785 78.161;
            1375.169 -1414.504 1352.828 -53.749 89.787 78.161;
            1588.085 -1170.417 1352.878 -44.331 89.785 78.162;
            1853.037  -676.914 1352.828 -28.008 89.787 78.161;
            1972.678    22.355 1352.826  -7.291 89.787 78.161;
            1963.564   -29.662 1414.095  -6.692 89.786 58.859;
            1951.641   218.219 1414.082   0.553 89.786 58.858;
            1872.648   591.367 1414.077  11.699 89.787 58.858;
            1950.149   630.214 1158.198  11.522 98.717 56.463;
            1991.620   650.987  866.646  10.871 108.460 53.704;
            1973.197   835.716  801.300  36.878 114.063 64.053;
            2139.162   126.277  801.340  19.301 114.062 64.052;
            2076.968  -527.375  801.277   1.676 114.064 64.052;
            2045.669  -598.956  825.291 -10.124 112.332 59.390;
    ];

end

%
xyzaer2rotm = @(x) ([eul2rotm(pi * [x(4) x(5) x(6)] / 180, 'ZYZ'), [x(1) x(2) x(3)]'; 0 0 0 1]);        
clear B;
for i=1:size(DATA, 1)
    B(:,:,i) = xyzaer2rotm(DATA(i, :));
    disp(180*rodrigues(B(1:3,1:3,i))'/pi);
end
plotRTs(B, 300); title('Robot Hand Coords');

%%
close all
clear 
clc

experiment = 'bbbb';
load(['images\',experiment,'\XYZABC.mat']);

%%
XYZABC(:,1:3) = XYZABC(:,1:3)/1000;
%%
clear B;
for i=2:size(XYZABC, 1)
    %B(:,:,i-1) = (xyzabc2rtmat(XYZABC(i, :)));
    [xyz, DH] = FK_Comau ( XYZABC(i, 1:6));
    B(:,:,i-1) = DH;
end
plotRTs(B, 300); title('Robot Hand Coords');
