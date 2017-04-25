close all
clc
clear 

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

d1 = sqrt(sum((XYZABC(7,1:3)-XYZABC(8,1:3)).^2));
d2 = sqrt(sum((XYZABC(7,1:3)-XYZABC(9,1:3)).^2));
d3 = sqrt(sum((XYZABC(9,1:3)-XYZABC(8,1:3)).^2));



%%

m7 = mean(XYZ7(:,1:3));
m8 = mean(XYZ8(:,1:3));
m9 = mean(XYZ9(:,1:3));
M = [m7; m8; m9; m7];
d1e = round(sqrt(sum((m7-m8).^2)) * 100)/100;
d2e = round(sqrt(sum((m7-m9).^2)) * 100)/100;
d3e = round(sqrt(sum((m9-m8).^2)) * 100)/100;


figure;
scatter3(XYZ7(:,1),XYZ7(:,2),XYZ7(:,3), '.', 'b')
hold on
scatter3(XYZ8(:,1),XYZ8(:,2),XYZ8(:,3), '.', 'r')
scatter3(XYZ9(:,1),XYZ9(:,2),XYZ9(:,3), '.', 'g')
%hold on
%scatter3(m7(1),m7(2),m7(3), 'x', 'b')
%hold on
%scatter3(m8(1),m8(2),m8(3), 'x', 'r')
%hold on
%scatter3(m9(1),m9(2),m9(3), 'x', 'g')
plot3(M(:,1), M(:,2), M(:,3))
m78 = (m7+m8)/2;
text(m78(1), m78(2), m78(3),[num2str(d1e),'/', num2str(d1)],'fontsize',9);
m79 = (m7+m9)/2;
text(m79(1), m79(2), m79(3),[num2str(d2e),'/', num2str(d2)],'fontsize',9);
m89 = (m8+m9)/2;
text(m89(1), m89(2), m89(3),[num2str(d3e),'/', num2str(d3)],'fontsize',9);
axis equal
title ('Relative distance between centroids');

%%


m7 = median(XYZ7(:,1:3))
m8 = median(XYZ8(:,1:3))
m9 = median(XYZ9(:,1:3))
M = [m7; m8; m9; m7];
d1e = round(sqrt(sum((m7-m8).^2)) * 100)/100;
d2e = round(sqrt(sum((m7-m9).^2)) * 100)/100;
d3e = round(sqrt(sum((m9-m8).^2)) * 100)/100;


figure;
scatter3(XYZ7(:,1),XYZ7(:,2),XYZ7(:,3), '.', 'b')
hold on
scatter3(XYZ8(:,1),XYZ8(:,2),XYZ8(:,3), '.', 'r')
hold on
scatter3(XYZ9(:,1),XYZ9(:,2),XYZ9(:,3), '.', 'g')
hold on
scatter3(m7(1),m7(2),m7(3), 'x', 'b')
hold on
scatter3(m8(1),m8(2),m8(3), 'x', 'r')
hold on
scatter3(m9(1),m9(2),m9(3), 'x', 'g')
axis equal
plot3(M(:,1), M(:,2), M(:,3))
m78 = (m7+m8)/2;
text(m78(1), m78(2), m78(3),[num2str(d1e),'/', num2str(d1)],'fontsize',9);
m79 = (m7+m9)/2;
text(m79(1), m79(2), m79(3),[num2str(d2e),'/', num2str(d2)],'fontsize',9);
m89 = (m8+m9)/2;
text(m89(1), m89(2), m89(3),[num2str(d3e),'/', num2str(d3)],'fontsize',9);
axis equal
title ('Relative distance between median-centroids');

