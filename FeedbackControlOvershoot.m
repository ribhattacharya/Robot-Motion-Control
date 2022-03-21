%% FeedbackControl function specifically for overshoot calculation. The difference between this function and 
%FeedbackControl is that this uses only feedback control and does NOT
%implement feedforward to calculate twist to obtain overshoot

%% Sample inputs

% Xd = [0 0 1 0.5
%       0 1 0 0
%      -1 0 0 0.5
%       0 0 0 1];
% 
% Xd_next = [0 0 1 0.6
%            0 1 0 0
%           -1 0 0 0.3
%            0 0 0 1];
% 
% X = [0.170 0 0.985 0.387
%      0 1 0 0
%     -0.985 0 0.170 0.570
%      0 0 0 1];
% 
% Kp = zeros(6);
% Ki = zeros(6);
% 
% t = 0.01;
% 
% thetalist = [0 0 0.2 -1.6 0];
% 
%[V, u, theta_dot, T0e, Xerr] = FeedbackControl(X, Xd, Xd_next, Kp, Ki, t, thetalist)

% Sample Outputs 
% 
% V =
% 
%          0
%          0
%          0
%    21.4000
%          0
%     6.4500
% 
% u =
% 
%   157.1068
%   157.1068
%   157.1068
%   157.1068
% 
% 
% theta_dot =
% 
%    1.0e+03 *
% 
%    -0.0000
%    -0.6526
%     1.3980
%    -0.7454
%     0.0000
%     
% T0e =
% 
%   Columns 1 through 3
% 
%     0.1700         0    0.9854
%          0    1.0000         0
%    -0.9854         0    0.1700
%          0         0         0
% 
%   Column 4
% 
%     0.2206
%          0
%     0.4707
%     1.0000
%     
% Xerr =
% 
%          0
%     0.1709
%          0
%     0.0795
%          0
%     0.1067

%%   Function body
function [V, u, theta_dot, T0e, Xerr] = FeedbackControlOvershoot(X, Xd, Xd_next, Kp, Ki, t, thetalist)
    
%% Finding twist

    % Calculating error matrix from current end-effector configuration and
    % current desired end-effector configuration
    Xerr_skew = MatrixLog6((TransInv(X))*Xd);
    Xerr = se3ToVec(Xerr_skew);    

    %Twist calculated using only feedback function
    V = Kp*Xerr + Ki*Xerr*t;
    
%% Finding wheel and arm joint velocities

    % Body frame screw axes of arm
    Blist = [0         0    1.0000         0    0.0330         0;
             0   -1.0000         0   -0.5076         0         0;
             0   -1.0000         0   -0.3526         0         0;
             0   -1.0000         0   -0.2176         0         0;
             0         0    1.0000         0         0         0]';

    % End-effector home configuration
    M = [1 0 0 0.033
           0 1 0 0
           0 0 1 0.654
           0 0 0 1];

    % Jacobian of Arm
    Jb_arm = JacobianBody(Blist,thetalist); 

    %Finding base Jacobian
    % Youbot dimensions
    r = 0.0475;
    l = 0.47/2;
    w = 0.3/2;
    
    % Pseudoinverse of H(0)
    F = [-1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w)
              1 1 1 1
              -1 1 -1 1]*r/4;
    
    % Converting F into 6x4 matrix, 1 column for each individual wheel
    F6 = zeros(6,4);
    F6(3:5, :) = F;
    
    
    
    % End-effector configuration with respect to base frame {0}
    T0e = FKinBody(M,Blist,thetalist');
    
    % Base configuration with respect to chassis frame {b} 
    Tb0 = [1 0 0 0.1662
           0 1 0 0
           0 0 1 0.0026
           0 0 0 1];
    
    % Jacobian of chassis
    Jb_base = (Adjoint(TransInv(T0e)*TransInv(Tb0)))*F6;
    
    % Merging base and arm Jacobian
    Je = zeros(6, 9);
    Je(:, 1:4) = Jb_base;
    Je(:, 5:9) = Jb_arm;
    
    % Finding wheel speeds(u) and arm joint speeds(theta_dot)
    u = zeros(4,1);
    theta_dot = zeros(5,1);
    Speeds = pinv(Je,1e-4)*V;
    u = Speeds(1:4);
    theta_dot = Speeds(5:9);
end