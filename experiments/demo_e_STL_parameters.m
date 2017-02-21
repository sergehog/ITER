clear
close all
clc


addpath(genpath('..\libs'));
filename = 'STL\Knuckle_paaty_printattavaksi.STL'; 
%%
[v, f, ~, ~, ~] = stlread(filename, true);
f = uint64(f);
v = single(v);
%%

% center is on the point of interest
R = rodrigues([pi/2 0 0]);  T = [6705 360 290]; RTmat = [R, -R*T'; 0 0 0 1];
RTmat = [rodrigues([0 0 1.2]), [237 8 -12]'; 0 0 0 1]*[R, -R*T'; 0 0 0 1];
[f, v] = load_CAD_model(filename, RTmat);
