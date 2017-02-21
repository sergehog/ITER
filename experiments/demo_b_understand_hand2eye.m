clear
close all
clc

addpath(genpath('..\libs'));

%% Generate some fake datapoints and noise in measurements
X = ([rodrigues([0.1 0.1 0.1]), [12 13 -14]'; 0 0 0 1]);
CW = [rodrigues([-0.1 0.1 1.1]), [500 6 7]'; 0 0 0 1];

B = eye(4);
B(:,:,2) = [rodrigues([1 0 0]), [11 1 0]'; 0 0 0 1];
B(:,:,3) = [rodrigues([0 1 0]), [2 13 1]'; 0 0 0 1];
B(:,:,4) = [rodrigues([0 0 1]), [0 1 11]'; 0 0 0 1];
B(:,:,5) = [rodrigues([1 0 1]), [1 4 11]'; 0 0 0 1];
B(:,:,6) = [rodrigues([0 1 1]), [2 3 -1]'; 0 0 0 1];
B(:,:,7) = [rodrigues([1 2 1]), [3 1 11]'; 0 0 0 1];

sigma = 0;
clear A Ai;
for i=1:size(B, 3)
    A(:,:,i) = (CW * B(:,:,i) * X)  + [zeros(3), randn([3 1])*sigma; 0 0 0 0];
    Ai(:,:,i) = inv(A(:,:,i));
end

%gHc = hand2Eye(B, A);
%[gHc, ~] = TSAIleastSquareCalibration(B, Ai);
[gHc, ~] = hand_eye_dual_quaternion(B, Ai);


disp(' == Reconstructed X Difference == ')
E = X-gHc;
disp([180*rodrigues(E(1:3, 1:3))/pi, E(1:3, 4)]);

% vlidation that AX=XB without knowlegde of CW
CC = [0 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 0];
for i=1:size(A,3)-1
    A1 = A(:,:,i);
    A2 = A(:,:,i+1);

    B1 = B(:,:,i);
    B2 = B(:,:,i+1);

    BB = inv(B2)*B1*X;
    AA = X*inv(A2)*A1;
    
    CC = CC + abs(BB-AA);
end
CC = CC./(size(A,3)-1);
disp(' == Average Error == ');
disp(num2str(CC));

%%
BB = inv(G2)*G1*X
AA = X*inv(C2)*C1 % don't forget that Ci are defined slightly diffeerent (as inverses)

disp('Error')
disp(AA-BB)
%%

close all
plotRTs(B, 5); title('Robot Hand Coords');
plotRTs(A, 5); title('Camera coords');



