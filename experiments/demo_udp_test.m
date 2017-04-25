close all
clear
clc
%
hudpr = dsp.UDPReceiver('LocalIPPort',51002,'ReceiveBufferSize', 200);
setup(hudpr);

clc

for i=1:10000
    
    dataReceived = step(hudpr);
    char(dataReceived')
    disp(str2num(char(dataReceived')))

    %XYZAER = typecast(dataReceived, 'double')';    
    %disp([num2str(i), ' ', num2str(XYZAER)]);
    pause(1.0)
end
%0.3414    -0.26473    -0.21985      2.2208      1.9868      -1.716
xyzaer2rotm = @(x) ([eul2rotm(pi * [x(4) x(5) x(6)] / 180, 'ZYZ'), [x(1) x(2) x(3)]'; 0 0 0 1]);
a1 = [1511.449  -986.744  722.061  -138.014  35.195  152.584];
a2 = [1495.819  -976.542  979.953  -131.961  42.678  142.988];

b = xyzaer2rotm(a1)
b(:,:,2) = xyzaer2rotm(a2);
plotRTs(b, 10)

release(hudpr);
%%

