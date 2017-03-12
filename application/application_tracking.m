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
    weighted_sampling = 0.01;

    
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

    minZ = handles.minZ;
    maxZ = handles.maxZ;
    layers = handles.layers;
    
    f = handles.f;
    v = handles.v;
        
    [ZL, ~] = estimate_iter_depth(L, R, CL, CR, minZ, maxZ, layers, threshG, threshC, weighted_sampling);
    

    XYZl = backproject_Z(ZL, CL);
    XYZlm = median(XYZl); 

    [a, b, c] = find_major_plane(XYZl(:,1), XYZl(:,2), XYZl(:,3), 1000, 20);
    
    Err = abs(XYZl(:,1).*a+XYZl(:,2).*b + c - XYZl(:,3));%    
    XYZl = XYZl(Err < thrPlane, :);    

    Rini = rodrigues([atan(b), -atan(a), 0]);       
    Tini = [1 0 0 XYZlm(1); 0 1 0 XYZlm(2); 0 0 1 XYZlm(3); 0 0 0 1];
    Mnew = Tini*[Rini, [0 0 0]'; 0 0 0 1];

    light = 1000;
    [Zx, Ix, ~] = render_CAD_model(f, v, CL, CR, Mnew, h, w, minZ, maxZ, 1/light, 1);
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
    Pm = pointCloud(XYZm);    
    Pm = pcdownsample(Pm, 'gridAverage', 4);
    if Pm.Count > 2000
        Pm = pcdownsample(Pm, 'random', 2000/Pm.Count);
    end
    XYZm = Pm.Location;
    clear Pm;


    XYZm = XYZm(:,1:3);
    XYZl = XYZl(:,1:3);

    tic
        %[TR, TT, ~, ~] = icp2(XYZl', XYZm', 100, 'Verbose', true, 'WorstRejection', WorstRejection, 'Matching', 'kDtree');
        [TR, TT, ~, ~] = icp2(XYZl', XYZm', 100, 'Verbose', true, 'WorstRejection', WorstRejection, 'Matching', 'kDtree');
    toc
    disp(num2str([TR, TT]));
    M = Mnew*([TR, TT; [0 0 0 1]]);
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
    light = 1000; % lumen (kinda lamp strength)
    [~, Iz, ~] = render_CAD_model(f, v, CL, CR, M, h, w, minZ, maxZ, 1/light);    
    Iz(isnan(Iz)) = 0;
    Iz = Iz./max(Iz(:));    
    Iv = L/255;
    Iv(:,:,3) = Iz(:,:,1);
    Iv(:,:,2) = (L/255 + Iz(:,:,1))/2;
    

    axes(handles.axes1);    
    imshow(ZL, [minZ maxZ]); colormap(pink);
    drawnow;
    axis off;

    XYZm(:,4) = 1;
    XYZm = ((inv(Mnew)*M)*XYZm')';
    
    axes(handles.axes2);    
    cla
    %scatter3(XYZl(:,1), XYZl(:,2), XYZl(:,3), '.', 'r');
    scatter(XYZl(:,1), XYZl(:,2), '.', 'r');
    hold on
    %scatter3(XYZm(:,1), XYZm(:,2), XYZm(:,3), '.', 'b');
    scatter(XYZm(:,1), XYZm(:,2), '.', 'b');
    %xlim([-500 500]);
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
