close all
clear 
clc

addpath(genpath('..\libs'));

camera_setup = 2;
save_results = 0;

%experiment = '17-Mar-2017-126987';
experiment = '05-May-2017-970593';
%experiment = '05-May-2017-915736';

%xyzaer2rotm = @(x) ([eul2rotm(pi * [x(4) x(5) x(6)] / 180, 'ZYZ'), [x(1) x(2) x(3)]'; 0 0 0 1]);
%prefix = 'images\calibration\';

xyzaer2rotm = @(x) ([angle2dcm( deg2rad(x(6)), deg2rad(x(5)),deg2rad(x(4)) ,'XYZ')', [x(1) x(2) x(3)]'; 0 0 0 1]);
prefix = '..\application\images\';
load([prefix, experiment, '\XYZABC.mat']);
frames = 1:size(XYZABC,1);
%frames = randperm(size(XYZABC,1));
%frames = frames(1:40);
DATA = XYZABC(frames, :); 

withRotation = sum(abs(DATA(1:(end-1),4:6)-DATA(2:(end),4:6)),2) > 1e-05;
frames = frames(withRotation);
DATA = DATA(withRotation, :);

% 123.6651      89.30392      54.55433      59.52874      33.13968      51.52555
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
frames = frames(used);
%DATA2 = DATA2(used, :);
squareSize = 25;  % in units of 'mm'
worldPoints = generateCheckerboardPoints(boardSize, squareSize);
%%
% Calibrate the camera
[stereoParams, pairsUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'mm');
DATA = DATA(pairsUsed, :);
frames = frames(pairsUsed);
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
end

    
clear A Ai B;
    
for i=1:size(R, 3)
    A(:,:,i) = inv([R(:,:,i)', T(i,:)'; 0 0 0 1]);
    Ai(:,:,i) = ([R(:,:,i)', T(i,:)'; 0 0 0 1]);
    B(:,:,i) = xyzaer2rotm(DATA(i, :));
end

%plotRTs(A, 100); title('Camera coords');
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



