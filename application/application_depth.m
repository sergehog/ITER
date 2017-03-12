function application_depth(hObject, eventdata, handles)
    disp('application_depth');
    
    if handles.mode ~= 2
        return;
    end
    
    threshG = 25*handles.sliderGrad.Value;
    threshC = 50*handles.SliderColor.Value;
    weighted_sampling = 0.01;
    
    minZ = handles.minZ;
    maxZ = handles.maxZ;
    layers = handles.layers;    

    
    stereoParams = handles.stereoParams;
    [CL, CR] = getCMatrices(stereoParams);
    
    h = handles.h; 
    w = handles.w;
    Image1 = (getdata(handles.vid1, 1));
    %Image1 = imresize(Image1, [h w], 'bicubic');
    Image2 = (getdata(handles.vid2, 1));
    %Image2 = imresize(Image2, [h w], 'bicubic');
    L = single(undistortImage(Image1, stereoParams.CameraParameters1));
    R = single(undistortImage(Image2, stereoParams.CameraParameters2));
    L = imresize(mean(L, 3), [h w]);
    R = imresize(mean(R, 3), [h w]);
    R = mean(L(:))*R./mean(R(:));

    flushdata(handles.vid1);
    flushdata(handles.vid2);

    %tic
    [ZL, ~] = estimate_iter_depth(L, R, CL, CR, minZ, maxZ, layers, threshG, threshC, weighted_sampling);
    %toc
    
    axes(handles.axes1);
    imshow(uint8(L));
    drawnow
    axis off;

    axes(handles.axes2);
    imshow(uint8(R));
    drawnow
    axis off;

    axes(handles.axes3);
    colormap(handles.axes3, jet)
    imshow(ZL, [minZ maxZ]);    
    colormap(gca, jet);
    drawnow;
    axis off;
        
    toc
    if handles.mode == 2 && strcmp(handles.vid1.Running, 'on') && strcmp(handles.vid2.Running, 'on')
        tic
        trigger(handles.vid1);    
        %trigger(handles.vid2);
    end
    
    

return;
