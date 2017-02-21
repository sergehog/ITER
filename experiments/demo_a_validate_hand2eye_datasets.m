close all
clear 
clc

addpath(genpath('..\libs'));


experiment = 'calibration2'; 
frames = 9;

D =[[26.502 100.336 29.239], [1572 -1666 962];
    [24.754 100.336 29.248], [1521 -1713 962];
    [25.627 101.002 33.964], [1514 -1704 886];
    [25.627 101.002 27.230], [1523 -1708 898];
    [15.808 103.945 25.106], [1497 -1787 889];
    [24.982 119.880 18.091], [1763 -1515 822];
    [27.846 120.818 21.477], [1794 -1480 720];
    [29.637 119.409 22.845], [1798 -1466 722.5];
    [29.221 119.208 21.996], [1809.2 -1477.1 705.3]];

%% Hand2eye Calibration

imageFileNames1 = {};
imageFileNames2 = {};

for i=1:frames
    imageFileNames1{i} = ['images\',experiment,'\1\', num2str(i),'.png'];
    imageFileNames2{i} = ['images\',experiment,'\2\', num2str(i),'.png'];    
end

[imagePoints, boardSize, used] = detectCheckerboardPoints(imageFileNames1, imageFileNames2);
disp(['boardSize = ', num2str(boardSize)]);   
disp(['used = ', num2str(used')]);   
squareSize = 30;  % in units of 'mm'
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera
[stereoParams, pairsUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'mm');

%figure;
%showExtrinsics(stereoParams, 'CameraCentric');

figure;
showExtrinsics(stereoParams.CameraParameters1, 'PatternCentric');


%%
clc
R = stereoParams.CameraParameters2.RotationMatrices;
T = stereoParams.CameraParameters2.TranslationVectors;                        
           
xyzaer2rotm = @(x) ([eul2rotm(pi * [x(1) x(2) x(3)] / 180, 'ZYZ'), [x(4) x(5) x(6)]'; 0 0 0 1]);
clear A Ai B;

for i=1:size(R, 3)
    A(:,:,i) = inv([R(:,:,i)', T(i,:)'; 0 0 0 1]);
    Ai(:,:,i) = ([R(:,:,i)', T(i,:)'; 0 0 0 1]);
    B(:,:,i) = xyzaer2rotm(D(i, :));
end

X = hand2Eye(B, A);
[X2, ~] = TSAIleastSquareCalibration(B, Ai);

disp('Implementation 1 & 2');
disp(num2str([X(1:3, 4)' , 180/pi*rodrigues(X(1:3, 1:3))']));
disp(num2str([X2(1:3, 4)' , 180/pi*rodrigues(X2(1:3, 1:3))']));


CC = [0 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 0];
CC2 = CC;
for i=1:size(R,3)-1
    A1 = A(:,:,i);
    A2 = A(:,:,i+1);

    B1 = B(:,:,i);
    B2 = B(:,:,i+1);

    BB = inv(B2)*B1*X;
    AA = X*inv(A2)*A1;
    
    CC = CC + abs(BB-AA);
    %disp(num2str(BB-AA));
    
    BB2 = inv(B2)*B1*X2;
    AA2 = X2*inv(A2)*A1;
    
    CC2 = CC2 + abs(BB2-AA2);
end
CC = CC./(size(R,3)-1);
CC2 = CC2./(size(R,3)-1);
disp('Average Error 1 & 2');
disp(num2str([CC(1:3, 4)' , 180/pi*rodrigues(CC(1:3, 1:3))']));
disp(num2str([CC2(1:3, 4)' , 180/pi*rodrigues(CC2(1:3, 1:3))']));
%disp(num2str([CC , CC2]));


%%
close all
plotRTs(bHg); title('Robot Hand Coords');
plotRTs(wHc); title('Camera coords');

%%
AA = [];
for i=1:9
    AA(:,:,i) = inv(wHc(:,:,i))*inv(X);
end
plotRTs(AA); title('Robot Gripper in CW coords');

