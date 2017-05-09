function application_detection(hObject, eventdata, handles)
    disp('application_detection');
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
    XYZABC= [];
    if handles.takecoords == 1
            hudpr = dsp.UDPReceiver('LocalIPPort',51002,'ReceiveBufferSize', 100);
            setup(hudpr);
            dataReceived = [1];
            % remove all UDP packets from the buffer
            while (numel(dataReceived) > 0)
                dataReceived = step(hudpr);
            end  
            % now get one correct
            %while (numel(dataReceived) ~= 48)
            while (numel(dataReceived) == 0)
                dataReceived = step(hudpr);            
            end 
            release(hudpr);

            XYZABC = str2num(char(dataReceived'));
            disp(['UDP recieved: ', num2str(XYZABC)]);   
    end
        
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
        
    [M, ZL, ~, Points] = full_ICP(f, v, L, R, CL, CR, minZ, maxZ, layers, threshG, threshC, robust_sampling, adaptive_thr, WorstRejection, thrPlane, thrNegPlane);
    
    
    if numel(XYZABC) == 6
        handles.Xworld = WorldFromICPandXYZAER(M, XYZABC, handles.X);
        
        updateDesiredCoordinates(handles);
        
        %[DesiredInTool] = DesiredInToolCoords(handles.Xworld, XYZABC, handles.Tool, handles.FrameOfInterest);    
        %XYZAERtool = rotm2xyzaer_zyz(DesiredInTool);
        %handles.editX1.String = num2str(XYZAERtool(1));
        %handles.editY1.String = num2str(XYZAERtool(2));
        %handles.editZ1.String = num2str(XYZAERtool(3));
        %handles.editAX1.String = num2str(XYZAERtool(4));
        %handles.editAY1.String = num2str(XYZAERtool(5));
        %handles.editAZ1.String = num2str(XYZAERtool(6));     
        
        
    end
    
    %handles.M = M;           
    
    
    % Visualization    
    [Zm, Iz, ~] = render_CAD_model(f, v, CL, CR, M, h, w);    
    Iz(isnan(Iz)) = 0;
    Iz = Iz./max(Iz(:));    
    Iv = 2*L/255;
    Iv(:,:,3) = Iz(:,:,1);
    Iv(:,:,2) = (L/255 + Iz(:,:,1))/2;
    
    % show augmenter image
    axes(handles.axes1);
    imshow(Iv);    
    axis off;

    % show estimated/filtered Depth Map
    axes(handles.axes2);    
    imshow(ZL, [min(ZL(:))*0.9 max(ZL(:))*1.1]); colormap(gca, jet);    
    axis off;
                  
    % show aligned points (+ unaligned)
    axes(handles.axes3);    
    cla
    XYZm = Points{1};
    XYZl = Points{2};
    %XYZl2 = Points{3};
    scatter3(XYZm(:,1), XYZm(:,3), -XYZm(:,2), '.', 'b');    
    hold on
    scatter3(XYZl(:,1), XYZl(:,3), -XYZl(:,2), '.', 'r');
    axis equal
    drawnow;
    axis off;
    
    
    %toc
    %if strcmp(handles.vid1.Running, 'on') ~= 0
    %    tic
    %    trigger(handles.vid1);
    %end
    %if strcmp(handles.vid2.Running, 'on') ~= 0
    %    trigger(handles.vid2);
    %end
    stop(handles.vid1);
    stop(handles.vid2);
    
    if handles.images > 5
        handles.pushbutton1.Enable = 'on';
    end
    handles.pushbutton2.Enable = 'on';
    handles.pushbutton3.Enable = 'on';
    handles.pushbutton4.String = 'Detect Target';    
    handles.pushbutton5.Enable = 'on';
    handles.pushbutton7.Enable = 'on';
    handles.mode = 0;
    
    guidata(handles.editX1, handles);

return;
