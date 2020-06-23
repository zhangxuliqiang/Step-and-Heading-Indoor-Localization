function [positions,step_orient] = plotTrajectory(orient_estimate,shs)

euler_angles = timetable(orient_estimate.Time);

[euler_angles.yaw, euler_angles.pitch, euler_angles.roll] =  ...
    quat2angle([orient_estimate.est{:,:}]');

euler_angles.yaw = euler_angles.yaw +pi/2;

step_orient = unique(euler_angles(shs.steps.data.Time, :));

positions = [];
prev_x = 0;
prev_y = 0;

for i = 1: height(shs.steps.data)
   pos.x = prev_x + cos(step_orient(i,:).yaw).* shs.steps.data.step_length(i); 
   prev_x = pos.x;
   
   pos.y = prev_y + sin(step_orient(i,:).yaw).* shs.steps.data.step_length(i); 
   prev_y = pos.y;   
   
   positions = [positions, pos];
end

% plot the trajectory
figure()
x = [positions.x];
y = [positions.y];
z = [seconds(shs.steps.data.Time)]';
col = z;  % This is the color, vary with x in this case.
surface([x;x],[y;y],[z;z],[col;col],...
        'facecol','no',...
        'edgecol','interp',...
        'linew',2);
hcb = colorbar;
title(hcb,'Time (sec)')
xlabel('X position from initial (meters)')
ylabel('Y position from initial (meters)')
title('step and heading system walking around the block')
% ylim([-50, 150])
% xlim([-150,50])

end