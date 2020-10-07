close all

clc

PLOT=0;

Param

T=1; % sampling time/interval

x=0; y=0; theta=pi/6; phi=pi/6; % initialize state variables
X=[x,y,theta,phi]'; % initial state vector

n=length(X); % length of the state vector

% N_p=[0.5 0.5 1.0472 1.0472]; % process noise covaiances
N_p=[0.001 0.001 0.0017 0.0017]; % process noise covaiances
Q=diag(N_p); % process noise covariance matrix

X_n=X+sqrt(Q)*randn(length(X),1)*T; % initial noisy state vector

X_p=X;
P_cov=blkdiag(eye(2),.0012*eye(2));





v=.5; w=0; % initial linear and angular speed
u=[v,w]'; % speed vector




samples=100; % number of samples
t=0; % initial time

for i=1:T:samples
    
    f_v=[cos(X(3,i)), sin(X(3,i)), (1/d)*tan(X(4,i)), 0];
    f_w=[0,0,0,1];
    f=[f_v' f_w'];
    
    X(:,i+1)=X(:,i)+T*f*u; % true sates
    X_n(:,i+1)=X_n(:,i)+T*f*u+sqrt(Q)*randn(n,1)*T; % noisy states
    
    
    X_p(:,i+1)=X_p(:,i)+T*f*u; % predicted states
    
    % Jacobian of f(*) w.r.t the state before prediction
    delta=0.00001;   
    for ii=1:n
        
        Xper=X_p(:,i);
        Xper(ii)=Xper(ii)+delta;
            
        f_vper=[cos(Xper(3)), sin(Xper(3)), (1/d)*tan(Xper(4)), 0];
        f_wper=[0,0,0,1];
        f_per=[f_vper' f_wper'];
             
        F(:,ii)=(f_per-f)*u;
            
    end
    
    P_cov=F*P_cov*F'+Q; % predicted process covariance
    
    % Jacobian of Measurement Matrix
    delta=0.00001;
    for ii=1:n
        Xper=X_p(:,i+1);
        Xper(ii)=Xper(ii)+delta;
            
        H(:,ii)=(op_func(Xper)-op_func(X_p(:,i+1)))/delta;
    end
    
    Rv=op_noise_cov(X_n(:,i)); % measurement noise covariance
       
    K=P_cov*H'*inv(H*P_cov*H'+Rv); % Kalman Gain computation
        
    delta_y=op_noise(X_n(:,i+1))-op_func(X_p(:,i+1)); % measurement noise
        
    X_p(:,i+1)=X_p(:,i+1)+K*delta_y; % update state prediction (noise filtering using kalman gain)
        
    P_cov=(eye(n)-K*H)*P_cov; % process covariance update using kalman gain
    t(:,i+1)=i;
    i
end

plot(X(1,:),X(2,:),'--o')
figure
plot(X_n(1,:),X_n(2,:),'--go')
figure
plot(X_p(1,:),X_p(2,:),'--ro')
figure
plot(t,(X_n(1,:)-X_p(1,:)).^2,t,(X_n(2,:)-X_p(2,:)).^2)


if PLOT==1
    figure
    plot(t,X(1,:))
    
    figure
    plot(t,X(2,:))

    figure
    plot(t,X(3,:))

    figure
    plot(t,X(4,:))
end