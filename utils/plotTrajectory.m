function [positions,step_orient] = plotTrajectory(orient_estimate,shs, door_handle_use,plot_flag)

if nargin<4
  door_handle_use = [];
  plot_flag = [];
elseif nargin < 3
  plot_flag = [];
end

euler_angles = timetable(orient_estimate.Time);

[euler_angles.yaw, euler_angles.pitch, euler_angles.roll] =  ...
    quat2angle([orient_estimate.est]);

euler_angles.yaw = euler_angles.yaw +pi/2;

%%
temp_data = euler_angles(shs.steps.data.Time,:);

[~,ia,~] = unique(temp_data.Time,'first');

step_orient = temp_data(ia,:);

step_orient.step_length = shs.steps.data.step_length;


positions = [];
prev_x = 0;
prev_y = 0;

for i = 1: height(shs.steps.data)
   pos.x = prev_x + cos(step_orient(i,:).yaw).* shs.steps.data.step_length(i); 
   prev_x = pos.x;
   
   pos.y = prev_y + sin(step_orient(i,:).yaw).* shs.steps.data.step_length(i); 
   prev_y = pos.y;  
   
   pos.time = seconds(shs.steps.data.Time(i));
   
   positions = [positions, pos];
end

positions = struct2table(positions);
positions.time = seconds(positions.time);
positions = table2timetable(positions);

door_detections =[];

if ~isempty(door_handle_use)
    for i = 1: height(door_handle_use)
      closest_point = retime(positions,door_handle_use(i,:).elapsed,'nearest');
      door_detections = [door_detections; closest_point];
    end
end

% plot the trajectory

if isempty(plot_flag)
    figure()
    x = [positions.x]';
    y = [positions.y]';
    z = [seconds(positions.time)]';
    col = z;  % This is the color, vary with x in this case.
    hold on
    surface([x;x],[y;y],[z;z],[col;col],...
        'facecol','no',...
        'edgecol','interp',...
        'linew',2);
    hcb = colorbar;
    
    if ~isempty(door_handle_use)
        scatter3(door_detections.x, door_detections.y, seconds(door_detections.time))
    end
    
    title(hcb,'Time (sec)')
    xlabel('X position from initial (meters)')
    ylabel('Y position from initial (meters)')
    % title('step and heading system walking around the block')
    % ylim([-50, 150])
    % xlim([-150,50])
end
end