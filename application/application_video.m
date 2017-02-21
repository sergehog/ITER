function application_video(hObject, eventdata, figHandle)
    %disp('application_video');
    
    
    handles = guidata(figHandle);
    disp(handles)
    if handles.mode ~= 5
        disp('wrong mode');
        return;
    end
    
    Image1 = (getdata(handles.vid1, 1));    
    Image2 = (getdata(handles.vid2, 1));
    
    flushdata(handles.vid1);
    flushdata(handles.vid2);
    
    axes(handles.axes1);
    imshow(uint8(Image1));
    %drawnow
    axis off;

    axes(handles.axes2);
    imshow(uint8(Image2));
    %drawnow
    axis off;
    %disp(handles.takeImages);
    global takeImages;
    if takeImages == 1
        takeImages = 0;
        filename1 = [handles.imageDir,'/1_', num2str(handles.images),'.png'];
        filename2 = [handles.imageDir,'/2_', num2str(handles.images),'.png'];
        imwrite(uint8(Image1), filename1, 'png');
        imwrite(uint8(Image2), filename2, 'png');        
        handles.images = handles.images + 1;        
    end
    
    
    %toc
    if handles.mode == 5 && strcmp(handles.vid1.Running, 'on') ~= 0 && strcmp(handles.vid2.Running, 'on') ~= 0
        tic
        trigger(handles.vid1);    
        %trigger(handles.vid2);
    end
    
    guidata(figHandle, handles);
    

return;
