close all
clc
clear 


camera_setup = 2;

load(['../application/hand2Eye',num2str(camera_setup),'.mat']);
load(['../application/stereoParams',num2str(camera_setup),'.mat']);


experiment = '16-Mar-2017-126987';

load(['images\', experiment, '\XYZABC.mat']);

imageID = 7;
load(['saves/repeatability1_',experiment,'_',num2str(imageID),'.mat'])


XYZ7 = XYZs;
imageID = 8;
load(['saves/repeatability1_',experiment,'_',num2str(imageID),'.mat'])
XYZ8 = XYZs;
imageID = 9;
load(['saves/repeatability1_',experiment,'_',num2str(imageID),'.mat'])
XYZ9 = XYZs;

%%

 % eye(4) - it's a point of interest and its direction
    XYZAER = rtmat2xyzear(Pose*X*Mi*eye(4));    