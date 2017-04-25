close all
clear 
clc

addpath(genpath('..\libs'));

camera_setup = 2;
save_results = 0;

%experiment = 'calibration4'; 
%experiment = 'calibration5';
%experiment = 'calib6';

%experiment = '16-Mar-2017-957507';
%experiment = '16-Mar-2017-970593';
experiment = '17-Mar-2017-126987';

xyzaer2rotm = @(x) ([eul2rotm(pi * [x(4) x(5) x(6)] / 180, 'ZYZ'), [x(1) x(2) x(3)]'; 0 0 0 1]);
prefix = 'images\calibration\';

if strcmp(experiment,'calibration4') 
    frames = [1, 3:10];
    %frames = [1, 4, 5, 6, 8];
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

elseif strcmp(experiment,'calibration5') 

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
elseif strcmp(experiment,'calib6') 
    frames = 1:20;
    DATA = [
    1207.477 -1535.624 1368.316 -52.142 119.147 22.651;
    1277.269 -1624.296 887.693 -52.232 136.127 22.504;
    1705.629 -1173.526 843.326 -42.973 136.741 5.117;
    1705.620 -1173.536 843.272 -42.977 136.743 -13.322;
    1705.977 -1173.761 664.509 -44.177 142.784 -14.888;
    1503.778 -1423.469 664.293 -34.191 142.878 23.945;
    1386.842 -1537.657 602.681 -39.198 143.312 23.758;
    1411.964 -1509.620 609.180 -19.716 139.884 60.883;
    1431.485 -1531.617 606.866 -21.755 136.403 58.148;
    1294.754 -1648.827 606.842 -26.678 136.403 58.149;
    1229.017 -1590.687 545.553 -33.028 143.501 43.972;
    1143.128 -1516.826 464.565 -46.889 144.797 44.402;
    1057.186 -1578.805 465.184 -49.448 144.449 49.485;
    0981.091 -1627.175 465.196 -52.170 144.448 49.483;
    1042.540 -1614.479 626.141 -51.291 138.391 48.347;
    1219.551 -1504.386 640.287 -52.566 131.826 42.707;
    1107.319 -1407.522 618.910 -63.222 127.380 28.420;
    1108.616 -1397.362 608.796 -60.708 131.686 58.227;
    1130.112 -1253.362 535.608 -60.843 118.000 43.132; 
    1173.636 -1283.687 584.092 -57.393 123.998 45.974;
];
elseif strcmp(experiment,'16-Mar-2017-97540') 
    load(['images\calibration\', experiment, '\XYZABC.mat']);
    frames = [1:11];
    DATA = XYZABC(frames, :); 
    DATA2 = DATA; 
    DATA2(1,4:6) = [-43.841 137.098 21.807];
    DATA2(2,4:6) = [-53.343 137.716 29.209];
    DATA2(3,4:6) = [-50.156 140.110 26.803];
    DATA2(4,4:6) = [-37.554 139.546 20.243];
    DATA2(5,4:6) = [-27.381 138.899 8.068];
    DATA2(6,4:6) = [-25.844 143.831 9.165];
    DATA2(7,4:6) = [-24.735 146.886 5.525];
    DATA2(8,4:6) = [-24.733 142.954 5.528];
    DATA2(9,4:6) = [-32.579 142.955 8.918];    
    DATA2(10,4:6) = [-14.630 139.370 -4.222];
    DATA2(11,4:6) = [-20.154 139.368 -4.222];    
    
elseif strcmp(experiment,'17-Mar-2017-126987') 
    frames = [1 10 11 12 16 17 18 19 2 20 23 24 25 3 34 4 45 46 48 49 5 55 56];
    xyzaer2rotm = @(x) ([angle2dcm( deg2rad(x(6)), deg2rad(x(5)),deg2rad(x(4)) ,'XYZ')', [x(1) x(2) x(3)]'; 0 0 0 1]);
    prefix = '..\application\images\';
    load([prefix, experiment, '\XYZABC.mat']);
    DATA = XYZABC(frames, :); 
else    
    xyzaer2rotm = @(x) ([angle2dcm( deg2rad(x(6)), deg2rad(x(5)),deg2rad(x(4)) ,'XYZ')', [x(1) x(2) x(3)]'; 0 0 0 1]);
    
    prefix = '..\application\images\';
    load([prefix, experiment, '\XYZABC.mat']);
    
    %frames = 1:(size(XYZABC, 1));
    frames = randperm(size(XYZABC,1));
    frames = frames(1:40);
    DATA = XYZABC(frames, :); 
end

%% Hand2eye Calibration
clc
imageFileNames1 = {};
imageFileNames2 = {};

for i=1:numel(frames)
    imageFileNames1{i} = [prefix, experiment,'\1_', num2str(frames(i)),'.png'];
    imageFileNames2{i} = [prefix, experiment,'\2_', num2str(frames(i)),'.png'];    
end

[imagePoints, boardSize, used] = detectCheckerboardPoints(imageFileNames1, imageFileNames2);
disp(['boardSize = ', num2str(boardSize)]);   
DATA = DATA(used, :);
%DATA2 = DATA2(used, :);
squareSize = 25;  % in units of 'mm'
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera
[stereoParams, pairsUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'mm');
DATA = DATA(pairsUsed, :);
%figure;
%showExtrinsics(stereoParams, 'CameraCentric');

figure;
showExtrinsics(stereoParams.CameraParameters2, 'PatternCentric');

if save_results > 0
    save(['..\application\stereoParams',num2str(camera_setup),'.mat'], 'stereoParams')
    filename = ['..\application\prosilica_cameras', num2str(camera_setup) ,'.txt'];
    saveStereoParameters(stereoParams, filename)
end
%%

R = stereoParams.CameraParameters1.RotationMatrices;
T = stereoParams.CameraParameters1.TranslationVectors;                        


if strcmp(experiment,'calibration7_March_10') 
        
    clear A Ai B;

    for i=1:(size(R, 3)-1)
        A(:,:,i) = inv([R(:,:,i)', T(i,:)'; 0 0 0 1]);
        Ai(:,:,i) = ([R(:,:,i)', T(i,:)'; 0 0 0 1]);
        [~,Bj] = FK_Comau(XYZABC(i, 1:6));
        B(:,:,i) = inv(Bj);
    end
else

    clc
    
    clear A Ai B;
    
    for i=1:size(R, 3)
        A(:,:,i) = inv([R(:,:,i)', T(i,:)'; 0 0 0 1]);
        Ai(:,:,i) = ([R(:,:,i)', T(i,:)'; 0 0 0 1]);
        B(:,:,i) = xyzaer2rotm(DATA(i, :));
    end
end
%plotRTs(A, 100); title('Camera coords');
%
%plotRTs(B, 100); title('Robot Hand Coords');

X = hand2Eye(B, A);
[X2, ~] = TSAIleastSquareCalibration(B, Ai);
[X3, ~] = hand_eye_dual_quaternion(B, Ai);

rtmat2xyzaer = @(x) [x(1:3, 4)' , 180/pi*rodrigues(x(1:3, 1:3))'];
disp(experiment);
disp('Implementation 1 & 2 & 3');
disp(num2str(rtmat2xyzaer(X)));
disp(num2str(rtmat2xyzaer(X2)));
disp(num2str(rtmat2xyzaer(X3)));
if save_results > 0
    save(['..\application\hand2Eye',num2str(camera_setup),'.mat'], 'X', 'X2', 'X3');
end
% Measure error of each approach

E1 = [0 0 0 0 0 0];
E2 = [0 0 0 0 0 0];
E3 = [0 0 0 0 0 0];
e1 = 0;e2 = 0; e3 = 0;

for i=1:size(R,3)
    A1 = A(:,:,i);
    B1 = B(:,:,i);
    
    for j=1:size(R,3)
        if i==j
            continue;
        end
        A2 = A(:,:,j);    
        B2 = B(:,:,j);

        BB1 = inv(B2)*B1*X;
        AA1 = X*inv(A2)*A1;
        

        E = (rtmat2xyzaer(BB1) - rtmat2xyzaer(AA1)).^2;
        e1 = e1 + sqrt(sum(E(1:3)));
        E1 = E1 + E;
                   
        BB2 = inv(B2)*B1*X2;
        AA2 = X2*inv(A2)*A1;
        E = (rtmat2xyzaer(BB2) - rtmat2xyzaer(AA2)).^2;
        e2 = e2 + sqrt(sum(E(1:3)));
        E2 = E2 + E;
    
    
        BB3 = inv(B2)*B1*X3;
        AA3 = X3*inv(A2)*A1;    
        E = (rtmat2xyzaer(BB3) - rtmat2xyzaer(AA3)).^2;
        e3 = e3 + sqrt(sum(E(1:3)));
        E3 = E3 + E;
    end
    
    
    
end
e1 = e1/(size(R,3)*(size(R,3) - 1));
e2 = e2/(size(R,3)*(size(R,3) - 1));
e3 = e3/(size(R,3)*(size(R,3) - 1));

disp(['reprojection errors (mm): ', num2str(e1), ', ', num2str(e2), ', ', num2str(e3)]);

%E1 = sqrt(E1)./(size(R,3)*(size(R,3) - 1));
%E2 = sqrt(E2)./(size(R,3)*(size(R,3) - 1));
%E3 = sqrt(E3)./(size(R,3)*(size(R,3) - 1));
% 
%disp('Average Error 1 & 2 & 3');
%disp(num2str(E1));
%disp(num2str(E2));
%disp(num2str(E3));



