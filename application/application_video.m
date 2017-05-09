function application_video(hObject, eventdata, figHandle)
    %disp('application_video');
    
    
    handles = guidata(figHandle);
    %disp(handles)
    if handles.mode ~= 5
        disp('wrong mode');
        return;
    end
    
    Image1 = (getdata(handles.vid1, 1));    
    Image2 = (getdata(handles.vid2, 1));
    
    flushdata(handles.vid1);
    flushdata(handles.vid2);
    
    axes(handles.axes1);
    imshow(imrotate(uint8(Image1), 90));
    %drawnow
    axis off;

    axes(handles.axes2);
    imshow(imrotate(uint8(Image2), 90));
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
        
        if handles.takecoords == 1
            %packet = step(handles.hudpr);

            %disp(num2str(packet'));
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

            %XYZABC = typecast(dataReceived, 'double')';
            handles.XYZABC(handles.images, :) = XYZABC;

            disp(['UDP recieved: ', num2str(XYZABC)]);   

            XYZABC = handles.XYZABC;
            save([handles.imageDir,'/XYZABC.mat'], 'XYZABC');
        end
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
