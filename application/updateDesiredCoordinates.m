function updateDesiredCoordinates(handles)
    
    if ~isnan(handles.Xworld)
        
        XYZAERworld = rotm2xyzaer_zyz(handles.Xworld*handles.Tool);
        disp('UFRAME position:')
        disp(num2str(XYZAERworld));
        handles.editX.String = num2str(XYZAERworld(1));
        handles.editY.String = num2str(XYZAERworld(2));
        handles.editZ.String = num2str(XYZAERworld(3));
        handles.editAX.String = num2str(XYZAERworld(4));
        handles.editAY.String = num2str(XYZAERworld(5));
        handles.editAZ.String = num2str(XYZAERworld(6));
        
        guidata(handles.editX1, handles);
    end
end