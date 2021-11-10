function [u, data] = FeedbackControl(data,settings)
% coil current order SWEN

th = data.curr_theta;
% TODO: create your PID controller here, the input is data and settings and
% the output should be u (the control efforts) and data. You should update
% all the values in data class (position, error, etc.) and your contorl
% effort should be in the form of [south west east north].
kp = 0.15*0.1e3;
ki = 0.02*0.1e3;
kd = 0.1*0.1e3;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Use settings.p_control, settings.i_control, settings.d_control in your pid control equation
% INSERT YOUR CODE HERE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
south = 0;
west = 0;
east = 0;
north = 0;
data.err_xPos = data.desired_x - data.curr_x;
data.err_yPos = data.desired_y - data.curr_y;

if abs(ki * data.sum_err_x) < 0.8 
    data.sum_err_x = data.sum_err_x + data.err_xPos;
else
    disp("anti-windup engaged");
end
if abs(ki * data.sum_err_y) < 0.8 
    data.sum_err_y = data.sum_err_y + data.err_yPos;
else
    disp("anti-windup engaged");
end

if settings.p_control
    east = -kp*data.err_xPos;
    west = -east;
    south = -kp*data.err_yPos;
    north = -south;
end
if settings.d_control
    east = east - kd * data.xVel;
    west = -east;
    south = south - kd * data.yVel;
    north = -south;
end
if settings.i_control
    x_e = ki * data.sum_err_x;
    y_e = ki * data.sum_err_y;
    
    east = east - x_e;
    west = -east;
    south = south - y_e;
    north = -south;
end

u = [south west east north]; %coil currents
u(u > 0) = 0;

% adjusting the currents based on the maximum current of power supply
MAX_CURR = 1;
curr_sum = norm(u);
if curr_sum >MAX_CURR
    disp('Max Current Exceeded');
    u = u/curr_sum*MAX_CURR;
end

end

% for reference:
% handles.data.xVel = 0;  % velocity in x direction 
% handles.data.yVel = 0;  % velocity in y direction 
% handles.data.thetaVel = 0;  % angular velocity
% handles.data.prevXpos = 0;  % previous x position 
% handles.data.prevYpos = 0;  % previous y position 
% handles.data.err_prev_x = 0;% previous position error in x direction 
% handles.data.err_prev_y = 0;% previous position error in y direction 
% handles.data.sum_err_x = 0; % sum of position error in x coordinates 
% handles.data.sum_err_y = 0; % sum of position error in y coordinates 
% handles.data.last_t = 0; % current time
% handles.data.dt = handles.data.last_t - handles.data.prev_t;     % delta_t 
% handles.data.goalReached = 0; % boolean to determine if the target is reached