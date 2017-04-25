function M = xyzabc2rtm(data)

roll = deg2rad(data(4));
pitch = deg2rad(data(5));
yaw = deg2rad(data(6)); 

 
dcm = angle2dcm( yaw, pitch, roll ); % this gives rotation matrix(3x3) for the given RPY
 
