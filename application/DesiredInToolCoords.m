function  [DesiredInTool] = DesiredInToolCoords(Xworld, XYZAERfeedback, ToolMatrix, FrameOfInterest)

HandPosition = xyzaer2rotm(XYZAERfeedback);

% FrameOfInterest IN 3D model coordinates goes to
% 3d wORLD coordinates
% then goes to Robot Wrist coordinates
% and finally in the Tool Coordinates

DesiredInTool = ToolMatrix*inv(HandPosition)*Xworld*FrameOfInterest;



