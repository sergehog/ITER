function application_fast(hObject, eventdata, figHandle)
    
    disp('application_fast');
    handles = guidata(figHandle);
    
    if handles.mode ~= 7
        return;
    end    
    
    
    % Main Parameters
    WorstRejection = handles.sliderRej.Value;
    threshG = 25*handles.sliderGrad.Value;
    threshC = 50*handles.SliderColor.Value;
    thrPlane = 100*handles.sliderPlane.Value;    
    %WorstRejection = 0.02;
    %threshG = 5;    
    %threshC = 10;
    %thrPlane = 40;
    
    robust_sampling = 0.001;
    adaptive_thr = 0;
    
    
    stereoParams = handles.stereoParams;
    [CL, CR] = getCMatrices(stereoParams);
    
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
    XYZABC = [];
    
    global updatePosition;
    if updatePosition == 1
        hudpr = dsp.UDPReceiver('LocalIPPort',35001,'ReceiveBufferSize', 100);
        setup(hudpr);
        dataReceived = [1];
        % remove all UDP packets from the buffer
        while (numel(dataReceived) > 0)
            dataReceived = step(hudpr);
        end  
        % now get one correct
        while (numel(dataReceived) ~= 96)
            dataReceived = step(hudpr);            
        end
        
        
        XYZABC = typecast(dataReceived, 'double')';    
        XYZABC(1:3) = XYZABC(1:3) * 1000;                        
        disp(['XYZABC = ', num2str(XYZABC)]);
        release(hudpr);
    end
    %Image1 = getsnapshot(handles.vid1);
    %Image2 = getsnapshot(handles.vid2);

    
    minZ = handles.minZ;
    maxZ = handles.maxZ;
    layers = handles.layers;   
    
    XYZm = handles.XYZm;
    
    [M, XYZl, ZL] = simplified_ICP(L, R, CL, CR, minZ, maxZ, layers, threshG, threshC, robust_sampling, adaptive_thr, thrPlane, XYZm, WorstRejection);
    
    M(4,:) = [0 0 0 1];
    
    handles.M = M;

    % handles.FrameOfInterest is position and orientation of a
    % point-of-interest in the CAD model space
    %
    % M - just found transform from Model space to Camera space
    %
    % handles.X - hand2eye calibration matrix (transform from camera to hand)
    %
    % handles.Tool - tool center point transform. Different for different
    % tools
    
    
    % Position of a point of interest in the Tool frame 
    Pose = (handles.Tool)*(handles.X)*M*(handles.FrameOfInterest);
     
    xyzabc = rtmat2xyzabc(Pose);
    translation = round(100*xyzabc(1:3))/100;
    angles = round(100*xyzabc(4:6))/100;
    %translation = round(100*Pose(1:3,4))/100;
    %angles = round(100*180*rodrigues(Pose(1:3, 1:3))/pi)/100;    
    
    handles.editX1.String = num2str(translation(1));
    handles.editY1.String = num2str(translation(2));
    handles.editZ1.String = num2str(translation(3));
    
    handles.editAX1.String = num2str(angles(1));
    handles.editAY1.String = num2str(angles(2));
    handles.editAZ1.String = num2str(angles(3));
    
    if updatePosition == 1
        CurrentPose = xyzabc2rtmat(XYZABC);  
        
        % Transform goes like that: Model (origin) -> Camera -> Hand -> Robot World 
        KnucklePosition = CurrentPose*(handles.X)*(handles.M);
        knuckle = rtmat2xyzabc(KnucklePosition);
        %handles.KnucklePosition = (handles.SavedPose1)*(handles.X)*(handles.M);
        disp(['New position: ' , num2str(knuckle)]);
        if handles.positions > 0
            disp(['Old position: ' , num2str(handles.KnucklePosition)]);            
            handles.KnucklePosition = (handles.KnucklePosition * handles.positions + knuckle) / (handles.positions + 1);
            disp(['Updated position: ' , num2str(handles.KnucklePosition)]);
        else
            handles.KnucklePosition = knuckle;
        end
        
        %Interest = (handles.KnucklePosition)*(handles.FrameOfInterest);
        Interest = xyzabc2rtmat(knuckle)*(handles.FrameOfInterest);
        
        xyzabc = rtmat2xyzabc(Interest);
        xyzabc = round(xyzabc*100)/100;
        handles.editX.String = num2str(xyzabc(1));
        handles.editY.String = num2str(xyzabc(2));
        handles.editZ.String = num2str(xyzabc(3));  
        handles.editAX.String = num2str(xyzabc(4));
        handles.editAY.String = num2str(xyzabc(5));
        handles.editAZ.String = num2str(xyzabc(6));
        
        handles.positions = handles.positions + 1;
    end
    updatePosition = 0;
    
    guidata(figHandle, handles);
    
    % Visualization
    light = 1000; % lumen (kinda lamp strength)
    [~, Iz, ~] = render_CAD_model(handles.f, handles.v, CL, CR, inv(M), h, w, minZ, maxZ, 1/light);
    Iz(isnan(Iz)) = 0;
    Iz = Iz(:,:,1)./max(max(Iz(:,:,1)));    
    Iv = single(L)/max(L(:))*2;
    Iv(:,:,2) = (ZL-minZ)/(maxZ-minZ);
    Iv(:,:,3) = Iz;    
    

    axes(handles.axes1);    
    %imshow(ZL, [minZ maxZ]); colormap(gca, jet);
    imshow(uint8(L));
    drawnow;
    axis off;

    %XYZm(:,4) = 1;
    %XYZm = XYZm*M';
    
    axes(handles.axes2);    
    cla
    scatter3(XYZl(:,1), XYZl(:,2), XYZl(:,3), '.', 'r');
    hold on
    scatter3(XYZm(:,1), XYZm(:,2), XYZm(:,3), '.', 'b');
    %xlim([-500 500]);    
    axis equal    
    hold off;
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
