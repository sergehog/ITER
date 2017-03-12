function plotRTs(RTs, len)
if nargin < 2
    len = 20;
end
    n = size(RTs, 3);
    f = figure;
    
    
    Xs = [];Ys = [];Zs = [];Ts = [];
    
    for i=1:n
        %disp(num2str(i))
        RT = RTs(:,:,i);
        R = RT(1:3, 1:3);
        T = RT(1:3, 4);                
        scatter3(T(1), T(2), T(3));
        hold on
        text(T(1), T(2), T(3),num2str(i),'fontsize',9);
        hold on
        TX = R*[len 0 0]' + T;
        TY = R*[0 len 0]' + T;
        TZ = R*[0 0 len]'  +T;        
        Xs = [Xs, T, TX, T];
        Ys = [Ys, T, TY, T];
        Zs = [Zs, T, TZ, T];
        Ts = [Ts, T];
        
    end
    
    plot3(Xs(1,:), Xs(2,:), Xs(3,:), 'r');
    hold on
    plot3(Ys(1,:), Ys(2,:), Ys(3,:), 'g');
    hold on
    plot3(Zs(1,:), Zs(2,:), Zs(3,:), 'b');
    hold on
    plot3(Ts(1,:), Ts(2,:), Ts(3,:), 'k');
    hold on
    O = [0 0 0; 100 0 0; 0 0 0; 0 100 0; 0 0 0; 0 0 100];
    plot3(O(:, 1), O(:, 2), O(:, 3), 'k');
    axis equal
    %a = axes(f);
    %set(a,'YDir','reverse');
    %set(a,'XAxisLocation','top','YAxisLocation', 'left','YDir','reverse');
        
    %xlim([0 max(RTs(1, 4, :), [],3)])
    %ylim([min(RTs(2, 4, :), [],3) 0])
    %zlim([0 max(RTs(3, 4, :), [], 3)])
    %zlim([min(RTs(3, 4, :), [], 3) 0])
    xlabel('x');
    ylabel('y');
    zlabel('z');

    
    

    
end