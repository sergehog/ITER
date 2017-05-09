function application_tracking(hObject, eventdata, handles)
    disp('application_tracking');
    if handles.mode ~= 4
        return;
    end
    
    % Main Parameters
    WorstRejection = handles.sliderRej.Value;
    threshG = 25*handles.sliderGrad.Value;
    threshC = 50*handles.SliderColor.Value;    
    thrPlane = 100*handles.sliderPlane.Value;    
    robust_sampling = 0.01;
    adaptive_thr = 0;    
    thrNegPlane = thrPlane;
    
    
    stereoParams = handles.stereoParams;
    [CL, CR] = getCMatrices(stereoParams);
    
    h = handles.h;
    w = handles.w;
    
    Image1 = (getdata(handles.vid1, 1));    
    Image2 = (getdata(handles.vid2, 1));
    
    if size(Image1,1) ~= h
        Image1 = imresize(Image1, [h w], 'bicubic');
        Image2 = imresize(Image2, [h w], 'bicubic');
    end
    L = single(undistortImage(Image1, stereoParams.CameraParameters1));
    R = single(undistortImage(Image2, stereoParams.CameraParameters2));
    L = mean(L, 3);
    R = mean(R, 3);
    R = mean(L(:))*R./mean(R(:));

    flushdata(handles.vid1);
    flushdata(handles.vid2);

    minZ = handles.minZ;
    maxZ = handles.maxZ;
    layers = handles.layers;
    
    f = handles.f;
    v = handles.v;
        
    [M, ZL] = full_ICP(f, v, L, R, CL, CR, minZ, maxZ, layers, threshG, threshC, robust_sampling, adaptive_thr, WorstRejection, thrPlane, thrNegPlane);
    
    
    %Mi = [TR, TT; [0 0 0 1]];
    %M = inv(Mi);
    handles.M = M;
    % Frame of interest in the tool local coordinates
    Pose = (handles.Tool)*(handles.X)*M*(handles.FrameOfInterest);
        
    translation = round(100*Pose(1:3,4))/100;
    angles = round(100*180*rodrigues(Pose(1:3, 1:3))/pi)/100;    
    
    handles.editX1.String = num2str(translation(1));
    handles.editY1.String = num2str(translation(2));
    handles.editZ1.String = num2str(translation(3));
    
    handles.editAX1.String = num2str(angles(1));
    handles.editAY1.String = num2str(angles(2));
    handles.editAZ1.String = num2str(angles(3));
    
    guidata(handles.editX1, handles);
    
    % Visualization    
    [~, Iz, ~] = render_CAD_model(f, v, CL, CR, M, h, w);    
    Iz(isnan(Iz)) = 0;
    Iz = Iz./max(Iz(:));    
    Iv = 2*L/255;
    Iv(:,:,3) = Iz(:,:,1);
    Iv(:,:,2) = (L/255 + Iz(:,:,1))/2;
    

    axes(handles.axes1);    
    imshow(ZL, [minZ maxZ]); colormap(pink);
    drawnow;
    axis off;

    %XYZm(:,4) = 1;
    %XYZm = (M'*XYZm')';
    
    %axes(handles.axes2);    
    %cla
    %scatter3(XYZl(:,1), XYZl(:,2), XYZl(:,3), '.', 'r');
    %scatter(XYZl(:,1), XYZl(:,2), '.', 'r');
    %hold on
    %scatter3(XYZm(:,1), XYZm(:,2), XYZm(:,3), '.', 'b');
    %scatter(XYZm(:,1), XYZm(:,2), '.', 'b');
    %xlim([-500 500]);
    %axis equal    
    %drawnow;    
    %axis off;
    
    
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
