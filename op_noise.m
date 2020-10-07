%% This function adds measurement noise to the sensor output
function y=op_noise(q,qdot,iteration)

global var_vo;
global var_ins;
global var_steer;


global sig_vo;
global sig_ins;
global sig_steer;


y=[];

% visual odometry/VO
if var_vo==1
          
    y=[y;q(1:2)+sig_vo*randn(2,1)];
    
end
% ins
if var_ins==1
    
    y=[y;q(3:3)+sig_ins*randn(1,1)];
    
end
% steering
if var_steer==1
    
    y=[y;q(4:4)+sig_steer*randn(1,1)];
    
end

