function application_tracking(hObject, eventdata, handles)
    disp('application_tracking');
    if handles.mode ~= 4
        return;
    end
    stereoParams = handles.stereoParams;
    %Image2 = getdata(hObject, 1);
    h = handles.h;
    w = handles.w;
    Image1 = (getdata(handles.vid1, 1));
    Image1 = imresize(Image1, [h w], 'bicubic');
    Image2 = (getdata(handles.vid2, 1));
    Image2 = imresize(Image2, [h w], 'bicubic');
    L = single(undistortImage(Image1, stereoParams.CameraParameters1));
    R = single(undistortImage(Image2, stereoParams.CameraParameters2));
    L = imresize(mean(L, 3), [h w]);
    R = imresize(mean(R, 3), [h w]);
    R = mean(L(:))*R./mean(R(:));

    flushdata(handles.vid1);
    flushdata(handles.vid2);

    %Image1 = getsnapshot(handles.vid1);
    %Image2 = getsnapshot(handles.vid2);


    KL = stereoParams.CameraParameters1.IntrinsicMatrix';
    KR = stereoParams.CameraParameters2.IntrinsicMatrix';
    RT = [stereoParams.RotationOfCamera2', stereoParams.TranslationOfCamera2'];
    CL = single(KL*[eye(3), [0 0 0]']);
    CR = single(KR*RT);
    minZ = handles.minZ;
    maxZ = handles.maxZ;
    layers = handles.layers;
    %threshG = 3;
    %threshC = 30;
    threshG = 5;
    threshC = 40;    
    weighted_sampling = 0.05;
    
    f = handles.f;
    v = handles.v;
    
    [ZL, ~] = estimate_iter_depth(L, R, CL, CR, minZ, maxZ, threshG, threshC, layers, 0.02, 0.8, weighted_sampling);    
    

    %figure(af); imshow(ZL, [minZ maxZ]); colormap(pink); title('Confident Left Depth'); drawnow;
    XYZl = backproject_Z(ZL, CL);
    XYZlm = median(XYZl); 

    [a, b, c] = find_major_plane(XYZl(:,1), XYZl(:,2), XYZl(:,3), 1000, 20);
    thrPlane = 100;
    Err = abs(XYZl(:,1).*a+XYZl(:,2).*b + c - XYZl(:,3));%    
    XYZl = XYZl(Err < thrPlane, :);    

    Rini = rodrigues([atan(b), -atan(a), 0]);       
    Tini = [1 0 0 XYZlm(1); 0 1 0 XYZlm(2); 0 0 1 XYZlm(3); 0 0 0 1];
    Mnew = Tini*[Rini, [0 0 0]'; 0 0 0 1];

    light = 1000;
    [Zx, Ix, ~] = render_CAD_model(f, v, CL, CR, Mnew, h, w, minZ, maxZ, 1/light);
    Zx(Zx > maxZ) = nan;
    Ix = uint8(Ix(:,:,1)*255);
    Zx(Ix < threshC) = nan;
        

    if weighted_sampling > 0
        Ix(isnan(Ix)) = 0;
        Ix(Ix > 255) = 255;
        [Lx, Ly] = gradient(single(Ix));
        Ig = sqrt(Lx.^2 + Ly.^2);
        clear Lx Ly;
        Mask = imdilate(Ig>=threshG, [1 1 1; 1 1 1; 1 1 1]);

        Z2x = Zx;
        Z2x(rand(size(Z2x)) > weighted_sampling) = nan;
        Zx(~Mask) = nan;
        Zx(~isnan(Z2x)) = Z2x(~isnan(Z2x));
        clear Z2x Mask;
    
    end
        %
    XYZm = backproject_Z(Zx, CL);
        

    XYZm = XYZm(:,1:3);
    XYZl = XYZl(:,1:3);

    tic
        [TR, TT, ~, ~] = icp2(XYZl', XYZm', 100, 'Verbose', true, 'WorstRejection', 0.2, 'Matching', 'kDtree');
    toc
    M = [TR, TT; [0 0 0 1]];

    angles = rodrigues(TR);
    handles.editX.String = num2str(TT(1));
    handles.editY.String = num2str(TT(2));
    handles.editZ.String = num2str(TT(3));
    
    handles.editAX.String = num2str(angles(1));
    handles.editAY.String = num2str(angles(2));
    handles.editAZ.String = num2str(angles(3));
    
    % Visualization
    light = 1000; % lumen (kinda lamp strength)
    [~, Iz, ~] = render_CAD_model(f, v, CL, CR, M*Mnew, h, w, minZ, maxZ, 1/light);
    Iz(isnan(Iz)) = 0;
    Iz(Iz > 1) = 1;
    Iv = L/255;
    Iv(:,:,3) = Iz(:,:,1);
    Iv(:,:,2) = (L/255 + Iz(:,:,1))/2;
    

    axes(handles.axes1);    
    imshow(ZL, [minZ maxZ]); colormap(pink);
    drawnow;
    axis off;

    XYZm(:,4) = 1;
    XYZm = XYZm*M';
    
    axes(handles.axes2);    
    scatter3(-XYZl(:,1), XYZl(:,3), XYZl(:,2), '.', 'r');
    hold on
    scatter3(-XYZm(:,1), XYZm(:,3), XYZm(:,2), '.', 'b');
    xlim([-500 500]);
    axis equal    
    drawnow;    
    axis off;
    
    
    axes(handles.axes3);
    imshow(Iv);
    drawnow;
    axis off;
    
    toc
    if strcmp(handles.vid1.Running, 'on') ~= 0
        tic
        trigger(handles.vid1);
    end
    %if strcmp(handles.vid2.Running, 'on') ~= 0
    %    trigger(handles.vid2);
    %end
    

return;
