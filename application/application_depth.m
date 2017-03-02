function application_depth(hObject, eventdata, handles)
    disp('application_depth');
    
    if handles.mode ~= 2
        return;
    end
    stereoParams = handles.stereoParams;
    %Image2 = getdata(hObject, 1);
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
    threshG = 0;
    threshC = 0;
    weighted_sampling = 0.0;
    %threshG = 4;
    %threshC = 40;    
    %weighted_sampling = 0.05;
    
    f = handles.f;
    v = handles.v;

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
    drawnow;
    axis off;
    
if 1==2
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
        %figure; imshow(uint8(Ix)); title('Rendered image');
        %figure; imshow(Zx, [minZ maxZ]); colormap(pink); title('Model Depth Map');

    if weighted_sampling > 0
    Ix(isnan(Ix)) = 0;
    Ix(Ix > 255) = 255;
    [Lx, Ly] = gradient(single(Ix));
    Ig = sqrt(Lx.^2 + Ly.^2);
    clear Lx Ly;
    Mask = imdilate(Ig>=threshG, [1 1 1; 1 1 1; 1 1 1]);
    %figure; imshow(Ig>=threshG, [])    
    Z2x = Zx;
    Z2x(rand(size(Z2x)) > weighted_sampling) = nan;
    Zx(~Mask) = nan;
    Zx(~isnan(Z2x)) = Z2x(~isnan(Z2x));
    clear Z2x Mask;
    %figure(af); imshow(Zx, [minZ maxZ]); colormap(pink); title('Sampled Model Depth Map'); drawnow;
    end
        %
    XYZm = backproject_Z(Zx, CL);
        %figure; 
        %scatter3(XYZl(:,1), XYZl(:,3), -XYZl(:,2), '.', 'r');
        %hold on
        %scatter3(XYZm(:,1), XYZm(:,3), -XYZm(:,2), '.', 'b');
        %axis equal
        %title('Sensed Point Cloud + CAD-Model points');    
        % Best ICP method

    XYZm = XYZm(:,1:3);
    XYZl = XYZl(:,1:3);

    tic
        [TR, TT, ~, ~] = icp2(XYZl', XYZm', 100, 'Verbose', true, 'WorstRejection', 0.2, 'Matching', 'kDtree');
    toc
    M = [TR, TT; [0 0 0 1]];

    %XYZl1 = XYZm;
    %XYZl1(:,4) = 1;
    %XYZ = XYZl1*M';
    %clear XYZl1; 

    %figure(c); 
    %scatter3(XYZl(:,1), XYZl(:,3), -XYZl(:,2), '.', 'r');
    %hold on
    %scatter3(XYZ(:,1), XYZ(:,3), -XYZ(:,2), '.', 'b');
    %axis equal
    %title(['After Alignment for dataset ', dataset]);

    %RTx = M*Mnew;
    %RTfound{number} = RTx;
    %XYZfound(number, 1) = RTx(1,4);
    %XYZfound(number, 2) = RTx(2,4);
    %XYZfound(number, 3) = RTx(3,4);
    %figure; scatter3(XYZfound(:,1), XYZfound(:,3), XYZfound(:,2), '.', 'b'); axis equal
    
    % Visualization
    light = 1000; % lumen (kinda lamp strength)
    [~, Iz, ~] = render_CAD_model(f, v, CL, CR, M*Mnew, h, w, minZ, maxZ, 1/light);
    Iz(isnan(Iz)) = 0;
    Iz(Iz > 1) = 1;
    Iv = L/255;
    Iv(:,:,3) = Iz(:,:,1);
    Iv(:,:,2) = (L/255 + Iz(:,:,1))/2;
    axes(handles.axes3);
    imshow(Iv);
    drawnow;
    axis off;
    
end
    
toc
    if handles.mode == 2 && strcmp(handles.vid1.Running, 'on') ~= 0 && strcmp(handles.vid2.Running, 'on') ~= 0
        tic
        trigger(handles.vid1);    
        %trigger(handles.vid2);
    end
    %if 
    %    trigger(handles.vid1);
    %end
    %
    %    trigger(handles.vid2);
    %end
    

return;
