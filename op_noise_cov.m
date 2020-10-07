%% This function adds measurement noise to the sensor output
function R=op_noise_cov(q)

global var_vo;
global var_ins;
global var_steer

global sig_vo;
global sig_ins;
global sig_steer;

R=[];

% visual odometry/VO
if var_vo==1
          
    R=blkdiag(R,sig_vo(1)^2);
    R=blkdiag(R,sig_vo(2)^2);
    
end
% ins
if var_ins==1
    
    R=blkdiag(R,sig_ins^2);
    
end
% steering
if var_steer==1
    
    R=blkdiag(R,sig_steer^2);
    
end

