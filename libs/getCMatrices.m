function [CL, CR] = getCMatrices(stereoParams)

KL = stereoParams.CameraParameters1.IntrinsicMatrix';
KR = stereoParams.CameraParameters2.IntrinsicMatrix';
RT = [stereoParams.RotationOfCamera2', stereoParams.TranslationOfCamera2'];
CL = single(KL*[eye(3), [0 0 0]']);
CR = single(KR*RT);
