%% This function adds measurement noise to the sensor output
function y=op_func(q)

global var_vo;
global var_ins;
global var_steer;


y=[];

% visual odometry/VO
if var_vo==1
          
    y=[y;q(1:2)];
    
end
% ins
if var_ins==1
    
    y=[y;q(3:3)];
    
end
% steering
if var_steer==1
    
    y=[y;q(4:4)];
    
end
