function application_fast2(hObject, eventdata, figHandle)
    
    disp('application_fast2');
    
    handles = guidata(figHandle);
    disp(num2str(handles.mode));
    
    if handles.mode ~= 7
        return;
    end    
    
            
    stereoParams = handles.stereoParams;
    [CL, ~] = getCMatrices(stereoParams);
    
    
    Image1 = (getdata(handles.vid1, 1));
    Image1 = imresize(Image1, [h w], 'bicubic');
    
    L = single(undistortImage(Image1, stereoParams.CameraParameters1));        
    flushdata(handles.vid1);
    flushdata(handles.vid2);
    
    XYZABC = [];
    
    
    hudpr = dsp.UDPReceiver('LocalIPPort',51002,'ReceiveBufferSize', 100);
    setup(hudpr);
    dataReceived = [1];
    % remove all UDP packets from the buffer
    while (numel(dataReceived) > 0)
        dataReceived = step(hudpr);
    end  
`   % now get one correct
    while (numel(dataReceived) ~= 96)
        dataReceived = step(hudpr);            
    end
        
    XYZABC = str2num(char(dataReceived'));
    disp(['UDP recieved: ', num2str(XYZABC)]);             
        
    release(hudpr);
        
    Pose = xyzaer2rotm(XYZABC);
    M = inv(handles.X) * inv(Pose) * handles.Xworld;
    
    
    
    minZ = handles.minZ;
    maxZ = handles.maxZ;
    layers = handles.layers;   
       
    
    % handles.FrameOfInterest is position and orientation of a
    % point-of-interest in the CAD model space
    %
    % M - just found transform from Model space to Camera space
    %
    % handles.X - hand2eye calibration matrix (transform from camera to hand)
    %
    % handles.Tool - tool center point transform. Different for different
    % tools
    
    [DesiredInTool] = DesiredInToolCoords(handles.Xworld, XYZABC, handles.Tool, handles.FrameOfInterest);    
    XYZAERtool = rotm2xyzaer(DesiredInTool);
    handles.editX1.String = num2str(XYZAERtool(1));
    handles.editY1.String = num2str(XYZAERtool(2));
    handles.editZ1.String = num2str(XYZAERtool(3));
    handles.editAX1.String = num2str(XYZAERtool(4));
    handles.editAY1.String = num2str(XYZAERtool(5));
    handles.editAZ1.String = num2str(XYZAERtool(6));     
        
    % Position of a point of interest in the Tool frame 
    %Pose = (handles.Tool)*(handles.X)*M*(handles.FrameOfInterest);
     
    %xyzabc = rtmat2xyzabc(Pose);
    %translation = round(100*xyzabc(1:3))/100;
    %angles = round(100*xyzabc(4:6))/100;
    %translation = round(100*Pose(1:3,4))/100;
    %angles = round(100*180*rodrigues(Pose(1:3, 1:3))/pi)/100;    
    
    %handles.editX1.String = num2str(translation(1));
    %handles.editY1.String = num2str(translation(2));
    %handles.editZ1.String = num2str(translation(3));    
    %handles.editAX1.String = num2str(angles(1));
    %handles.editAY1.String = num2str(angles(2));
    %handles.editAZ1.String = num2str(angles(3));
    
    
    guidata(figHandle, handles);
    
    % Visualization
    light = 1000; % lumen (kinda lamp strength)
    [Zm, Iz, ~] = render_CAD_model(handles.f, handles.v, CL, CR, inv(M), h, w, minZ, maxZ, 1/light);
    Iz(isnan(Iz)) = 0;
    Iz = Iz(:,:,1)./max(max(Iz(:,:,1)));    
    Iv = single(L)/max(L(:))*2;
    Iv(:,:,2) = (ZL-minZ)/(maxZ-minZ);
    Iv(:,:,3) = Iz;    
    

    axes(handles.axes1);    
    %imshow(ZL, [minZ maxZ]); colormap(gca, jet);
    imshow(Iv);
    drawnow;
    axis off;

    %XYZm(:,4) = 1;
    %XYZm = XYZm*M';
    
    axes(handles.axes2);    
    imshow(ZL, [minZ maxZ]); colormap(gca, jet);
    %cla
    %scatter3(XYZl(:,1), XYZl(:,2), XYZl(:,3), '.', 'r');
    %hold on
    %scatter3(XYZm(:,1), XYZm(:,2), XYZm(:,3), '.', 'b');
    %xlim([-500 500]);    
    %axis equal    
    %hold off;
    drawnow;    
    axis off;
    
    
    axes(handles.axes3);
    %imshow(Iv);
    imshow(Zm, [minZ maxZ]); colormap(gca, jet);
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
