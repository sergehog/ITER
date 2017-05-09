close all
clear 
clc

addpath(genpath('..\libs'));
camera_setup = 2;


experiment = '05-May-2017-762596';

load(['../application/images/',experiment,'/hand2Eye',num2str(camera_setup),'.mat']);
load(['../application/images/',experiment,'/stereoParams',num2str(camera_setup),'.mat']);
load(['../application/images/',experiment,'/XYZABC.mat']);
%%

i=1;
