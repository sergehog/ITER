close all
clear 
clc

addpath(genpath('..\libs'));

camera_setup = 2;

%
%if 1==2
%experiment = 'calibration4'; 
%frames = [1, 3:10];
%%frames = [1, 4, 5, 6, 8];
%DATA = [1963.370 -1381.642 898.949 19.611 110.392 3.158;
%%2139.669 -1088.839 898.924 27.775 110.392 3.159;
%1999.358 -1422.386 593.909 -20.418 114.959 -12.005;
%1855.277 -1301.006 597.831 -47.447 118.383 7.152;
%1873.134 -1444.215 539.327 -29.207 109.520 32.367;
%1664.698 -1574.885 418.620 -45.182 116.446 27.783;
%1721.682 -1615.142 612.783 -43.820 108.755 6.625;
%1429.939 -1504.572 499.325 -55.175 120.837 43.844;
%1016.596 -1737.278 231.631 -70.274 129.790 40.102;
%1015.149 -1698.499 128.126 -70.823 132.784 46.499;
%];
%end

experiment = 'calibration5';
frames = 1:11;
DATA = [1972.456 -1017.043 753.244 -19.494 119.656 21.768;
    1615.824 -1521.209 753.248 -35.489 119.656 21.768;
    1694.787 -1504.806 610.338 -33.964 121.080 21.782;
    2060.996  -942.859 610.304 -16.945 121.081 21.782;
    2127.226  -963.678 563.697 -16.966 116.649 21.738;
    1788.997 -1501.080 563.660 -32.593 116.650 21.738;
    1713.000 -1436.405 269.319 -33.232 122.238 49.038;
    1975.687 -1245.997 422.164 -24.856 122.237 11.218;
    1992.391 -1253.924 559.744 -24.877 118.150 11.175;
    2176.567 -959.353 608.821 -16.867 118.152 -0.358;
    2205.528 -992.271 524.503 12.592 119.558 17.289;
];


%% Hand2eye Calibration
clc
imageFileNames1 = {};
imageFileNames2 = {};

for i=1:numel(frames)
    imageFileNames1{i} = ['images\',experiment,'\1\1_', num2str(frames(i)),'.png'];
    imageFileNames2{i} = ['images\',experiment,'\2\2_', num2str(frames(i)),'.png'];    
end

[imagePoints, boardSize, used] = detectCheckerboardPoints(imageFileNames1, imageFileNames2);
disp(['boardSize = ', num2str(boardSize)]);   

squareSize = 25;  % in units of 'mm'
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera
[stereoParams, pairsUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'mm');

%figure;
%showExtrinsics(stereoParams, 'CameraCentric');

figure;
showExtrinsics(stereoParams.CameraParameters2, 'PatternCentric');

save(['stereoParams',num2str(camera_setup),'.mat'], 'stereoParams')
%%
%close all           
clc

R = stereoParams.CameraParameters1.RotationMatrices;
T = stereoParams.CameraParameters1.TranslationVectors;                        
xyzaer2rotm = @(x) ([eul2rotm(pi * [x(4) x(5) x(6)] / 180, 'ZYZ'), [x(1) x(2) x(3)]'; 0 0 0 1]);
clear A Ai B;

for i=1:size(R, 3)
    A(:,:,i) = inv([R(:,:,i)', T(i,:)'; 0 0 0 1]);
    Ai(:,:,i) = ([R(:,:,i)', T(i,:)'; 0 0 0 1]);
    B(:,:,i) = xyzaer2rotm(DATA(i, :));
end

%plotRTs(A); title('Camera coords');
%
%plotRTs(B); title('Robot Hand Coords');

X = hand2Eye(B, A);
[X2, ~] = TSAIleastSquareCalibration(B, Ai);
[X3, ~] = hand_eye_dual_quaternion(B, Ai);

disp('Implementation 1 & 2 & 3');
disp(num2str([X(1:3, 4)' , 180/pi*rodrigues(X(1:3, 1:3))']));
disp(num2str([X2(1:3, 4)' , 180/pi*rodrigues(X2(1:3, 1:3))']));
disp(num2str([X3(1:3, 4)' , 180/pi*rodrigues(X3(1:3, 1:3))']));


CC = [0 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 0];
CC2 = CC;
for i=1:size(R,3)-1
    A1 = A(:,:,i);
    A2 = A(:,:,i+1);

    B1 = B(:,:,i);
    B2 = B(:,:,i+1);

    BB = inv(B2)*B1*X;
    AA = X*inv(A2)*A1;
    
    CC = CC + (BB-AA).^2;
    %disp(num2str([BB-AA]));
    
    BB2 = inv(B2)*B1*X2;
    AA2 = X2*inv(A2)*A1;
    
    CC2 = CC2 + (BB2-AA2).^2;
end
CC = sqrt(CC)./(size(R,3)-1);
CC2 = sqrt(CC2)./(size(R,3)-1);
disp('Average Error 1 & 2');
disp(num2str(CC(1:3, 4)' ));
disp(num2str(CC2(1:3, 4)'));

save(['hand2Eye',num2str(camera_setup),'.mat'], 'X', 'X2', 'X3');

%%
experiment = 'calibration3';
clc
DATA = [1796.259 -988.560 835.209 27.493 111.587 23.788;
1781.146 -1015.539 835.177 26.629 111.588 23.789;
1765.625 -1042.294 835.216 25.764 111.588 23.787;
1723.389 -1110.739 835.203 23.517 111.588 23.787;
1660.529 -1202.687 835.207 20.403 111.587 23.788;
1660.537 -1202.677 835.197 20.403 111.587 11.950;
1660.533 -1202.679 835.235 20.403 111.587 5.372];
xyzaer2rotm = @(x) ([eul2rotm(pi * [x(4) x(5) x(6)] / 180, 'ZYZ'), [x(1) x(2) x(3)]'; 0 0 0 1]);        
clear B;
for i=1:7
    B(:,:,i) = xyzaer2rotm(DATA(i, :));
    disp(180*rodrigues(B(1:3,1:3,i))'/pi);
end
plotRTs(B); title('Robot Hand Coords');
