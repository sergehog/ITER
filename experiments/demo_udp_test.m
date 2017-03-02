close all
clear
clc

hudpr = dsp.UDPReceiver('LocalIPPort',35001);
%setup(hudpr);

%%
clc

%for i=1:1000
    dataReceived = hudpr();
    dataReceived    
%end
%%
release(hudpr);