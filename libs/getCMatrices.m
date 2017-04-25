function [CL, CR, CL2, CR2] = getCMatrices(stereoParams)

KL = stereoParams.CameraParameters1.IntrinsicMatrix';
KR = stereoParams.CameraParameters2.IntrinsicMatrix';
RT = [stereoParams.RotationOfCamera2', stereoParams.TranslationOfCamera2'];
CL = single(KL*[eye(3), [0 0 0]']);
CR = single(KR*RT);

KL2 = KL*2;
KL2(3,3) = 1;
KR2 = KR*2;
KR2(3,3) = 1;
CL2 = single(KL2*[eye(3), [0 0 0]']);
CR2 = single(KR2*RT);
