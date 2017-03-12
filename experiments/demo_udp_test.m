close all
clear
clc
%
hudpr = dsp.UDPReceiver('LocalIPPort',35001,'ReceiveBufferSize', 100);
setup(hudpr);

clc

for i=1:10000
    
    dataReceived = step(hudpr);

    XYZAER = typecast(dataReceived, 'double')';    
    disp([num2str(i), ' ', num2str(XYZAER)]);
    pause(1.0)
end
%0.3414    -0.26473    -0.21985      2.2208      1.9868      -1.716

release(hudpr);
%%
