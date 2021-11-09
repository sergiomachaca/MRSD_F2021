function [coil_currents] = MapInputtoCoilCurrents(u, settings)

% find the actual coil current command

if (settings.closedloop_control_on) 
    south_c = u(1);
    west_c = u(2);
    east_c = u(3);
    north_c = u(4); 
    % TODO: this should be either positive or negative depending on how you
    % define error, you should experiment yourself. 
%     coil_currents = '(+ or -)' [south, west, east, north];
    coil_currents = [south_c, west_c, east_c, north_c];
    
else % Joystick control on
    lh = u(1); % left horizontal joystick 
    lv = u(2); % left vertical joystick 
%     rh = u(3); % right horizontal joystick 
%     rv = u(4); % right vertical joystick  
    
    scale = 0.8; % scale down the current output
    
% control with two joysticks 
%     south_c = (max(0.0, -lv) + min(0.0, rv))*scale;
%     north_c = (max(0.0, lv) + min(0.0, -rv))*scale;
%     east_c = (max(0.0, -lh) + min(0.0, rh))*scale;
%     west_c = (max(0.0, lh) + min(0.0, -rh))*scale;
    

    south_c = max(0.0, lv)*scale;
    north_c = max(0.0, -lv) *scale;
    east_c = max(0.0, lh) *scale;
    west_c = max(0.0, -lh)  *scale;
    
end
    % current projection
    MAX_CURR = 1; % maximum current
    current = [south_c, west_c, east_c, north_c];
    curr_sum = norm(current);
    if curr_sum >MAX_CURR
        coil_currents = current/curr_sum*MAX_CURR;
    else
        coil_currents = current;
    end 

end
