% ITER Computer-Assisted Teleoperation using Stereo Reconstruction and ICP
%  
% Application requires two connected GENTL cameras. 
% It has several modes: 
%  - Video mode shows output from the cameras as well as allow to make
%    shots  (those are required for Stereoscopic and Hand-to-eye calibration)
%  - Depth mode uses Stereoscopic calibration parameters to estimate Depth
%    of the scene and show it in a separate placeholder. It could be used to
%  validate correctness of calibration.
%  - Tracking mode uses full chain alignment in order to discover Knuckle 
%   in the scene and estimate it's pose. 
%  - Fast tracking only uses ICP with pre-calulated model point cloud. 
% 
%   Tracking provides coordinates of the interest point in tool coordinate system.   
%   If tracking is succesfull, first placeholder will show an aquired image 
%   well aligned with the rendered CAD-model image. 
% 
%   Tool can be selected with a separate radio-box. Inspection mode assumes
%   no tool. Hence, found knuckle position is shown in Flange (last-joint) 
%   coordinate system. There is also no specific interest point, so origin
%   of a CAD model is assumed.

function varargout = application(varargin)

clc
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @application_OpeningFcn, ...
                   'gui_OutputFcn',  @application_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before application is made visible.
function application_OpeningFcn(hObject, eventdata, handles, varargin)

    addpath(genpath('..\libs'));

    handles.setup = 1; % my stereo-camera setup
    %handles.setup = 2; % hydraulics Lab stereo-camera setup

    % Choose default command line output for application
    handles.output = hObject;
    %handles.hudpr = dsp.UDPReceiver('LocalIPPort',35001);
    %setup(handles.hudpr);
    handles.mode = 0;
    handles.takeImages = 0;
    handles.images = 1;
    handles.XYZABC = [];
    
    loadedValues = load(['stereoParams',num2str(handles.setup),'.mat'], 'stereoParams');
    handles.stereoParams = loadedValues.stereoParams;        
    
    params2 = load(['hand2eye',num2str(handles.setup),'.mat']); 
    %params2 = load(['hand2eye.mat']); 
    handles.X = params2.X;
    handles.takecoords = 0;

    %filename = '..\STL\Knuckle_cut_to_size2.stl'; 
    filename = '..\STL\knuckle+Cassete.stl'; 
    RTmat = [rodrigues([0 0 0]), [0 0 0]'; 0 0 0 1];
    [f, v] = load_CAD_model(filename, RTmat);
    handles.XYZm = load3dPoints('..\STL\knuckle+Cassete.bin');
    handles.f = f;
    handles.v = v;
    handles.w = 960;
    handles.h = 540;
    %handles.w = 1920;
    %handles.h = 1080;
    handles.minZ = 300;
    handles.maxZ = 5000;
    handles.layers = 100;
    
    %system('ITERSetup.exe');
    %handles.vid1 = videoinput('gentl', 1, 'RGB8Packed');
    handles.vid1 = videoinput('gentl', 1, 'Mono8');
    handles.vid1.FramesPerTrigger = 1;
    triggerconfig(handles.vid1, 'Manual');
    handles.vid1.TriggerRepeat = Inf;

    %handles.vid2 = videoinput('gentl', 2, 'RGB8Packed');
    handles.vid2 = videoinput('gentl', 2, 'Mono8');
    handles.vid2.FramesPerTrigger = 1;
    triggerconfig(handles.vid2, 'hardware', 'DeviceSpecific', 'DeviceSpecific');
    %triggerconfig(handles.vid2, 'Manual');
    handles.vid2.TriggerRepeat = Inf;

    %vid.TriggerRepeat = Inf;
    handles.imageDir = ['images/',date(),'-', num2str(round(rand()*1000000))];
    status = mkdir(handles.imageDir);
    disp(handles.imageDir);
    disp(status);
    %src.FrameStartTriggerMode = 'On';
    %src.FrameStartTriggerActivation = 'RisingEdge';

    handles.Tool = eye(4); % Flange to tool transform
    handles.Tool1 = inv([rodrigues([0 0 pi/4]), rodrigues([0 0 pi/4])*[40, 0, 20]'; 0 0 0 1]);
    handles.Tool2 = inv([rodrigues([0 0 0.52]), rodrigues([0 0 0.52])*[50, 0, 20]'; 0 0 0 1]);
    
    handles.FrameOfInterest = eye(4); % center of a model
    handles.Frame1 = [eye(3), [140, 130, -40]'; 0 0 0 1]; % point of interest for wrench tool
    handles.Frame2 = [rodrigues([0 0 pi*20/180]), [[-80, 250, -40]]'; 0 0 0 1]; % point of interest for second tool
    
    handles.KnucklePosition = [0 0 0 0 0 0];
    %handles.xyzabc2rtmat = @(x) ([eul2rotm([x(4) x(5) x(6)], 'ZYZ'), [x(1) x(2) x(3)]'; 0 0 0 1]);
    %handles.rtmat2xyzabc = @(x) ([x(1,4) x(2,4) x(3,4), rotm2eul([x(1:3,1:3)], 'ZYZ')]);
    %handles.xyzaer2rotm = @(x) ([eul2rotm(pi * [x(4) x(5) x(6)] / 180, 'ZYZ'), [x(1) x(2) x(3)]'; 0 0 0 1]);
    %handles.SavedPose1 = xyzaer2rotm([1212.763 1258.115 1108.814 51.970 130.230 113.008]);
    
    handles.updatePosition = 0;
    handles.positions = 0;
    
    % Update handles structure
    guidata(hObject, handles);

return;

% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
    %release(handles.hudpr);    
    handles.mode = 0;
    guidata(hObject, handles);
    stop(handles.vid1);
    stop(handles.vid2);        
    pause(3);
    try  
        delete(handles.vid1);
        delete(handles.vid2);
    catch
    end
    
    delete(hObject);
return;


% ---  Calibration
function pushbutton1_Callback(hObject, eventdata, handles)
    
    h = handles.h;
    w = handles.w;
    
    % calibrate and save params
    if handles.images >= 5
        
            handles.pushbutton1.String = 'Please Wait ...';            
            handles.pushbutton2.Enable = 'off';
            handles.pushbutton3.Enable = 'off';
            handles.pushbutton4.Enable = 'off';
            handles.pushbutton5.Enable = 'off';
            handles.pushbutton7.Enable = 'off';
            
            drawnow;
            imageFileNames1 = {};
            imageFileNames2 = {};
            for i=1:(handles.images-1)
                imageFileNames1{i} = [handles.imageDir,'/1_', num2str(i),'.png'];
                imageFileNames2{i} = [handles.imageDir,'/2_', num2str(i),'.png'];
            end
            [imagePoints, boardSize, pairsUsed] = detectCheckerboardPoints(imageFileNames1, imageFileNames2);
            disp('boardSize');   
            disp(boardSize);
            % Generate world coordinates of the checkerboard keypoints
            squareSize = 25;  % in units of 'mm'
            worldPoints = generateCheckerboardPoints(boardSize, squareSize);

            % Calibrate the camera
            [stereoParams, pairsUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
                'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
                'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'mm');

            save(['stereoParams',num2str(handles.setup),'.mat'], 'stereoParams');
            handles.stereoParams = stereoParams;
            
            handles.mode = 0;
            handles.pushbutton1.String = 'Calibration'; 
            handles.pushbutton2.Enable = 'on';
            handles.pushbutton3.Enable = 'on';
            handles.pushbutton4.Enable = 'on';
            handles.pushbutton5.Enable = 'on';
            handles.pushbutton7.Enable = 'on';
            
            saveStereoParameters(stereoParams, ['prosilica_cameras',num2str(handles.setup),'.txt']);            
            
            
            
    end    
    
    guidata(hObject, handles);
   
return;


% --- Start Depth.
function pushbutton2_Callback(hObject, eventdata, handles)
    
    if handles.mode == 0
        handles.pushbutton1.Enable = 'off';
        handles.pushbutton2.String = 'Stop Depth';    
        handles.pushbutton3.Enable = 'off';        
        handles.pushbutton4.Enable = 'off';
        handles.pushbutton5.Enable = 'off';
        handles.pushbutton7.Enable = 'off';
        handles.mode = 2;

        handles.vid2.FramesAcquiredFcnCount = 1;
        handles.vid2.FramesAcquiredFcn = {'application_depth', handles};
        start(handles.vid1);
        start(handles.vid2);
        tic;
        trigger(handles.vid1);    

    elseif handles.mode == 2
        handles.pushbutton1.Enable = 'on';
        handles.pushbutton2.String = 'Start Depth';            
        handles.pushbutton3.Enable = 'on';
        handles.pushbutton4.Enable = 'on';        
        handles.pushbutton5.Enable = 'on';
        handles.pushbutton7.Enable = 'on';
        handles.mode = 0;
        stop(handles.vid1);
        stop(handles.vid2);
    end

    guidata(hObject, handles);

return;

        
% --- FAST TRACKING
function pushbutton7_Callback(hObject, eventdata, handles)
    if handles.mode == 0
        handles.pushbutton1.Enable = 'off';
        handles.pushbutton2.Enable = 'off';
        handles.pushbutton3.Enable = 'off';
        handles.pushbutton4.Enable = 'off';    
        handles.pushbutton5.Enable = 'off';
        handles.pushbutton7.String = 'Stop Tracking';         
        handles.mode = 7;
        handles.vid2.FramesAcquiredFcnCount = 1;
        handles.vid2.FramesAcquiredFcn = {'application_fast', hObject};
        start(handles.vid1);
        start(handles.vid2);
        tic
        trigger(handles.vid1);    
    elseif handles.mode == 7
        handles.pushbutton1.Enable = 'on';
        handles.pushbutton2.Enable = 'on';
        handles.pushbutton3.Enable = 'on';
        handles.pushbutton4.Enable = 'on';
        handles.pushbutton5.Enable = 'on';        
        handles.pushbutton7.String = 'Fast Tracking';    
        handles.mode = 0;
        stop(handles.vid1);
        stop(handles.vid2);
    end
    guidata(hObject, handles);
return;

% --- Start Tracking 
function pushbutton4_Callback(hObject, eventdata, handles)
    if handles.mode == 0
        handles.pushbutton1.Enable = 'off';
        handles.pushbutton2.Enable = 'off';
        handles.pushbutton3.Enable = 'off';
        handles.pushbutton4.String = 'Stop Tracking';    
        handles.pushbutton5.Enable = 'off';
        handles.pushbutton7.Enable = 'off';
        
        handles.mode = 4;

        %handles.vid2.TriggerMode = 'On';
        %triggerconfig(handles.vid2, 'hardware');
        %handles.vid1.FramesPerTrigger = 1;
        %handles.vid1.TriggerRepeat = 10000;
        %handles.vid2.FramesPerTrigger = 1;
        %handles.vid2.TriggerRepeat = 10000;
        handles.vid2.FramesAcquiredFcnCount = 1;
        handles.vid2.FramesAcquiredFcn = {'application_tracking', handles};
        start(handles.vid1);
        start(handles.vid2);
        tic
        trigger(handles.vid1);    
        %trigger(handles.vid2);    
    elseif handles.mode == 4
        handles.pushbutton1.Enable = 'on';
        handles.pushbutton2.Enable = 'on';
        handles.pushbutton3.Enable = 'on';
        handles.pushbutton4.String = 'Start Tracking';    
        handles.pushbutton5.Enable = 'on';
        handles.pushbutton7.Enable = 'on';
        handles.mode = 0;
        stop(handles.vid1);
        stop(handles.vid2);
    end

    guidata(hObject, handles);
return;


% --- Start Video button 
function pushbutton5_Callback(hObject, eventdata, handles)
    if handles.mode == 0
        handles.pushbutton1.Enable = 'off';
        handles.pushbutton2.Enable = 'off';
        handles.pushbutton3.Enable = 'off';        
        handles.pushbutton4.Enable = 'off';
        handles.pushbutton5.String = 'Stop Video';    
        handles.pushbutton6.Enable = 'on';
        handles.pushbutton7.Enable = 'off';
        handles.mode = 5;

        tic
        handles.vid2.FramesAcquiredFcnCount = 1;
        handles.vid2.FramesAcquiredFcn = {'application_video', hObject};
        start(handles.vid1);
        start(handles.vid2);
        trigger(handles.vid1);    
        %trigger(handles.vid2);    
    elseif handles.mode == 5
        handles.pushbutton1.Enable = 'on';
        handles.pushbutton2.Enable = 'on';
        handles.pushbutton3.Enable = 'on';        
        handles.pushbutton4.Enable = 'on';
        handles.pushbutton5.String = 'Start Video';    
        handles.pushbutton6.Enable = 'off';
        handles.pushbutton7.Enable = 'on';
        handles.mode = 0;
        stop(handles.vid1);
        stop(handles.vid2);
    end

    guidata(hObject, handles);
return;

% --- TAKE IMAGE
function pushbutton6_Callback(hObject, eventdata, handles)
    handles = guidata(hObject);
    global takeImages;
    takeImages = 1;
    %handles.takeImages = 1;
    handles.pushbutton6.String = ['Take Image #', num2str(handles.images+1)];
    guidata(hObject, handles);
return;


% --- UPDATE Button
function pushbutton8_Callback(hObject, eventdata, handles)
    disp(num2str(handles.mode))
    global updatePosition;
    if handles.mode == 7        
        updatePosition = 1;
    end
    %handles.KnucklePosition = (handles.SavedPose1)*(handles.X)*(handles.M);
    %updateDesiredCoordinates(handles);
    guidata(hObject, handles);
return;

function updateDesiredCoordinates(handles)
    rtmat2xyzear = @(x) [x(1,4), x(2,4), x(3,4), 180*rotm2eul(x(1:3,1:3), 'ZYZ')/pi];
    if ~isnan(handles.KnucklePosition(1,1))
        Interest = (handles.KnucklePosition)*(handles.FrameOfInterest);
        xyzaer = rtmat2xyzear(Interest);
        xyzaer = round(xyzaer*100)/100;
        handles.editX.String = num2str(xyzaer(1));
        handles.editY.String = num2str(xyzaer(2));
        handles.editZ.String = num2str(xyzaer(3));  
        handles.editAX.String = num2str(xyzaer(4));
        handles.editAY.String = num2str(xyzaer(5));
        handles.editAZ.String = num2str(xyzaer(6));
        guidata(handles.editX1, handles);
    end
return;


% --- Executes on button press in radiobutton1.
function radiobutton1_Callback(hObject, eventdata, handles)
    if get(hObject,'Value') == 1
        handles.Tool = eye(4);
        handles.FrameOfInterest = eye(4);
        updateDesiredCoordinates(handles);
    end
    guidata(hObject, handles);
return;

% --- Executes on button press in radiobutton2.
function radiobutton2_Callback(hObject, eventdata, handles)
    if get(hObject,'Value') == 1
        handles.Tool = handles.Tool1;
        handles.FrameOfInterest = handles.Frame1;
        updateDesiredCoordinates(handles);
    end
    guidata(hObject, handles);
return;

% --- Executes on button press in radiobutton3.
function radiobutton3_Callback(hObject, eventdata, handles)
    if get(hObject,'Value') == 1
        handles.Tool = handles.Tool2;
        handles.FrameOfInterest = handles.Frame2;
        updateDesiredCoordinates(handles);
    end
    guidata(hObject, handles);
return;    



% --- Executes during object creation, after setting all properties.
function editX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editY_Callback(hObject, eventdata, handles)
% hObject    handle to editY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editY as text
%        str2double(get(hObject,'String')) returns contents of editY as a double


% --- Executes during object creation, after setting all properties.
function editY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function editZ_Callback(hObject, eventdata, handles)
% hObject    handle to editZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editZ as text
%        str2double(get(hObject,'String')) returns contents of editZ as a double


% --- Executes during object creation, after setting all properties.
function editZ_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function editAX_Callback(hObject, eventdata, handles)
% hObject    handle to editAX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editAX as text
%        str2double(get(hObject,'String')) returns contents of editAX as a double


% --- Executes during object creation, after setting all properties.
function editAX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editAX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function editAY_Callback(hObject, eventdata, handles)
% hObject    handle to editAY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editAY as text
%        str2double(get(hObject,'String')) returns contents of editAY as a double


% --- Executes during object creation, after setting all properties.
function editAY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editAY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function editAZ_Callback(hObject, eventdata, handles)
% hObject    handle to editAZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editAZ as text
%        str2double(get(hObject,'String')) returns contents of editAZ as a double


% --- Executes during object creation, after setting all properties.
function editAZ_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editAZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Outputs from this function are returned to the command line.
function varargout = application_OutputFcn(hObject, eventdata, handles) 
varargout{1} = handles.output;



% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)





% --- Hand2Eye Tool Calibration.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    h = handles.h;
    w = handles.w;
    triggerconfig(handles.vid2, 'Manual');
    
    Image1 = getsnapshot(handles.vid1);    
    Image2 = getsnapshot(handles.vid2);                
        
    Image1 = imresize(Image1, [h w], 'bicubic');
    Image2 = imresize(Image2, [h w], 'bicubic');
        
    axes(handles.axes1);
    imshow(Image1);
    axis off;

    axes(handles.axes2);
    imshow(Image2);
    axis off;

    drawnow;
        
    if handles.mode == 0
        handles.mode = 3;        
        handles.frame = 1;
        
        handles.pushbutton1.Enable = 'off';
        handles.pushbutton2.Enable = 'off';                
        handles.pushbutton3.String = 'Take Frame 2';
        handles.pushbutton4.Enable = 'off';
        handles.pushbutton5.Enable = 'off';
                                                
        imwrite(uint8(Image1), ['hand2eye\1\', num2str(handles.frame),'.png']);
        imwrite(uint8(Image2), ['hand2eye\2\', num2str(handles.frame),'.png']);
        
        handles.frame = 2;        
        
        
    elseif handles.mode == 3
                 
        
        imwrite(uint8(Image1), ['hand2eye\1\', num2str(handles.frame),'.png']);
        imwrite(uint8(Image2), ['hand2eye\2\', num2str(handles.frame),'.png']);
                
        if  handles.frame == 2
                                
            handles.pushbutton3.String = sprintf('Take Frame 3', handles.frame);
            handles.frame = 3;            
            
        elseif  handles.frame == 3
                                
            handles.pushbutton3.String = sprintf('Take Frame 4', handles.frame);
            handles.frame = 4;            
        elseif  handles.frame == 4
                                
            handles.pushbutton3.String = sprintf('Take Frame 5', handles.frame);
            handles.frame = 5;            
            
        else
            
            handles.pushbutton3.String = 'Please Wait ...';
            handles.pushbutton3.Enable = 'off';            
            drawnow;
            
            imageFileNames1 = {};
            imageFileNames2 = {};
            for i=1:5
                imageFileNames1{i} = ['hand2eye\1\', num2str(i),'.png'];
                imageFileNames2{i} = ['hand2eye\2\', num2str(i),'.png'];
            end
            
            [imagePoints, boardSize] = detectCheckerboardPoints(imageFileNames1);
            disp(['boardSize = ', num2str(boardSize)]);   
            
            % Generate world coordinates of the checkerboard keypoints
            squareSize = 25;  % in units of 'mm'
            worldPoints = generateCheckerboardPoints(boardSize, squareSize);

            [params, ~, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints);
                        
            
            handles.hand2eyeParams = params;
            
            R = params.RotationMatrices;
            T = params.TranslationVectors;                        
            
            wHc(:,:,1) = [R(:,:,1), T(1,:)'; 0 0 0 1];
            wHc(:,:,2) = [R(:,:,2), T(2,:)'; 0 0 0 1];
            wHc(:,:,3) = [R(:,:,3), T(3,:)'; 0 0 0 1];
            
            bHg(:,:,1) = eye(4);
            bHg(:,:,2) = [rodrigues([0 0 pi/90]), [0 0 0]'; 0 0 0 1];
            bHg(:,:,3) = [rodrigues([0 pi/90 0])*rodrigues([0 0 pi/90]), [0 0 0]'; 0 0 0 1];
                        
            gHc = handEye(bHg, bHg);
            
            save('hand2eyeParams.mat', 'params', 'gHc');
            
            handles.gHc = gHc;            
            handles.frame = 0;
            handles.mode = 0;
            
            handles.pushbutton1.Enable = 'on';            
            handles.pushbutton2.Enable = 'on';
            handles.pushbutton3.String = 'Hand2Eye Calibr'; 
            handles.pushbutton4.Enable = 'on';
            handles.pushbutton5.Enable = 'on';  
            handles.pushbutton7.Enable = 'on';
        end
                        
    end
    
    guidata(hObject, handles);



function editX1_Callback(hObject, eventdata, handles)
% hObject    handle to editX1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editX1 as text
%        str2double(get(hObject,'String')) returns contents of editX1 as a double


% --- Executes during object creation, after setting all properties.
function editX1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editX1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editY1_Callback(hObject, eventdata, handles)
% hObject    handle to editY1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editY1 as text
%        str2double(get(hObject,'String')) returns contents of editY1 as a double


% --- Executes during object creation, after setting all properties.
function editY1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editY1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editZ1_Callback(hObject, eventdata, handles)
% hObject    handle to editZ1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editZ1 as text
%        str2double(get(hObject,'String')) returns contents of editZ1 as a double


% --- Executes during object creation, after setting all properties.
function editZ1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editZ1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editAX1_Callback(hObject, eventdata, handles)
% hObject    handle to editAX1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editAX1 as text
%        str2double(get(hObject,'String')) returns contents of editAX1 as a double


% --- Executes during object creation, after setting all properties.
function editAX1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editAX1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editAY1_Callback(hObject, eventdata, handles)
% hObject    handle to editAY1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editAY1 as text
%        str2double(get(hObject,'String')) returns contents of editAY1 as a double


% --- Executes during object creation, after setting all properties.
function editAY1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editAY1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editAZ1_Callback(hObject, eventdata, handles)
% hObject    handle to editAZ1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editAZ1 as text
%        str2double(get(hObject,'String')) returns contents of editAZ1 as a double


% --- Executes during object creation, after setting all properties.
function editAZ1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editAZ1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function sliderRej_Callback(hObject, eventdata, handles)
% hObject    handle to sliderRej (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function sliderRej_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sliderRej (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function sliderGrad_Callback(hObject, eventdata, handles)
% hObject    handle to sliderGrad (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function sliderGrad_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sliderGrad (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function SliderColor_Callback(hObject, eventdata, handles)
% hObject    handle to SliderColor (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function SliderColor_CreateFcn(hObject, eventdata, handles)
% hObject    handle to SliderColor (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function sliderPlane_Callback(hObject, eventdata, handles)
% hObject    handle to sliderPlane (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function sliderPlane_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sliderPlane (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
