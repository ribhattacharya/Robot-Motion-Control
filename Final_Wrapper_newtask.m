clc;
clear all;

%% This function is used to implement the "new task" portion of the project. 
% We set our new initial position for the cube position in the T_sc_init
% matrix, and our new final position as a rotation by -90 degrees from the
% new initial position. 

%% Initializing required inputs to move cube from initial position to new initial position

% Transformation matrix of initial end-effector position in terms of space
% frame
T_se_init = [0 0 1 0; 
             0 1 0 0; 
            -1 0 0 0.5; 
             0 0 0 1];

% Transformation matrix of new initial cube position in terms of space
% frame
T_sc_init = [ 1 0 0 1 
              0 1 0 1 
              0 0 1 0.025
              0 0 0 1];

% Transformation matrix of new final position in terms of space
% frame calculated by rotating the initial position by -90 degrees along
% z-axis
T_sc_final = [0 1 0 0; 
              -1 0 0 0;
              0 0 1 0;
              0 0 0 1] * T_sc_init;

% Setting angle at which end-effector grasps cube
grasp_angle = 2*pi/3;

% Transformation matrix of end-effector in terms of cube frame while
% grasping
T_ce_grasp = [cos(grasp_angle) 0 sin(grasp_angle) 0; 
                0 1 0 0; 
                -sin(grasp_angle) 0 cos(grasp_angle) 0; 
                0 0 0 1];

% Transformation matrix of end-effector in terms of cube frame at standoff
% position above T_ce_grasp by 0.2m
T_ce_standoff = [cos(grasp_angle) 0 sin(grasp_angle) 0; 
                0 1 0 0; 
                -sin(grasp_angle) 0 cos(grasp_angle) 0.2; 
                0 0 0 1];

k = 1;

% Initializing Proportional and Integral coefficients for feedback
% controller
Kp = eye(6)*1.5;
Ki = eye(6)*20;

% Setting time step at 0.01
t = 0.01;

% End-effector transformation matrix with respect to base frame {0} at home
% position
M = [1 0 0 0.033
     0 1 0 0
     0 0 1 0.6546
     0 0 0 1];

% Setting initial state of youBot
Cur_state = [pi/3, -0.2, -0.2, 0, 0, 0.2, -1.6, 0, 0, 0, 0, 0];

% Obtaining arm angles and chassis orientation
thetalist = Cur_state(4:8)';
phi = Cur_state(1);
x = Cur_state(2);
y = Cur_state(3);

% Calculating transformation matrices from {s} to {b}, and {b} to {0}
Tsb = [cos(phi) -1*sin(phi) 0   x
           sin(phi) cos(phi)    0   y
           0            0       1   0.0963
           0            0       0   1];

Tb0 =  [1   0   0   0.1662
        0   1   0   0
        0   0   1   0.0026  
        0   0   0   1];

% Body frame screw axes
Blist = [0         0    1.0000         0    0.0330         0;
         0   -1.0000         0   -0.5076         0         0;
         0   -1.0000         0   -0.3526         0         0;
         0   -1.0000         0   -0.2176         0         0;
         0         0    1.0000         0         0         0]';

%Finding end-effector tranformation matrix with respect to {0} using
%forward kinematics
T0e = FKinBody(M,Blist,thetalist);

% Calculating initial configuration of end-effector with respect to space
% frame {s}
X = Tsb * Tb0 * T0e;



%% Calling trajectory generatorto obtain all sets of end-effector configurations 

final_trajectory = TrajectoryGenerator(T_se_init, T_sc_init, T_sc_final, ...
    T_ce_grasp, T_ce_standoff, k);


%% finding required values for loop

%Finding number of trajectories obtained
[l, w] = size(final_trajectory);

%Initializing twist matrix for all sets of trajectories
Vlist = zeros(l-1, 6);


% Initializing Configuration matrix for all sets of trajectories
Config_list = zeros(l, 13);
Config_list(1, 1:12) = Cur_state;
Config_list(1, 13) = 0;

% Setting max velocity of joints
Vmax = 15;

% Initializing error matrix for all sets of trajectories
Xerr_list = zeros(l-1,6);

%% Looping through each set of Xd and Xd_next

for i = 1:l-1
    
    % Setting desired end-effector position in current iteration of
    % trajectory
    Xd = [final_trajectory(i, 1) final_trajectory(i, 2) final_trajectory(i, 3) final_trajectory(i, 10)
          final_trajectory(i, 4) final_trajectory(i, 5) final_trajectory(i, 6) final_trajectory(i, 11)
          final_trajectory(i, 7) final_trajectory(i, 8) final_trajectory(i, 9) final_trajectory(i, 12)
          0                      0                      0                       1];

    % Setting next desired end-effector position in current iteration of
    % trajectory
    Xd_next = [final_trajectory(i+1, 1) final_trajectory(i+1, 2) final_trajectory(i+1, 3) final_trajectory(i+1, 10)
               final_trajectory(i+1, 4) final_trajectory(i+1, 5) final_trajectory(i+1, 6) final_trajectory(i+1, 11)
               final_trajectory(i+1, 7) final_trajectory(i+1, 8) final_trajectory(i+1, 9) final_trajectory(i+1, 12)
               0                      0                      0                       1];
    
    % Running FeedbackControl to obtain Twist, wheel velocities, arm joint velocities, end-effector
    % tranformation matrix with respect to {0}, and error
    [V, u, theta_dot, T0e, Xerr] = FeedbackControl(X, Xd, Xd_next, Kp, Ki, t, thetalist);
    Vlist(i, :) = V;

    % Setting wheel and arm joint velocities in a single vector
    Vel = zeros(9,1);
    Vel(1:5) = theta_dot;
    Vel(6:9) = u;
    
    % Obtaining the next configuration of the robot when given Velocity Vel
    New_state = NextState(Cur_state, Vel', t, Vmax);

    % Getting chassis configuration
    Cur_state = New_state;
    phi = Cur_state(1);
    x = Cur_state(2);
    y = Cur_state(3);

    % Calculating transformation matrix from {s} to {b} for new chassis
    % configuration
    Tsb = [cos(phi) -1*sin(phi) 0   x
           sin(phi) cos(phi)    0   y
           0            0       1   0.0926
           0            0       0   1];

    % Calculating end-effector transformation matrix in new
    % configuration
    X = Tsb*Tb0*T0e;

    % Obtaining arm joint angles for next iteration of loop
    thetalist = Cur_state(4:8);
    
    % Saving current configuration to file
    Config_list(i+1, 1:12) = Cur_state;
    Config_list(i+1, 13) = final_trajectory(i+1, 13);

    % Saving current error to file
    Xerr_list(i,:) = Xerr';

end

% Saving configurations as .csv
writematrix(Config_list,'newtask.csv')

%% Plotting Error graph

% Setting x-axis
index = floor(l/1) - 1;
time_plot = 0:t:t*(index-1);

% Plotting all 6 Xerr values with respect to t
plot(time_plot, Xerr_list(1:index,1),'LineWidth',2)
hold on
plot(time_plot,Xerr_list(1:index,2),'LineWidth',2)
hold on
plot(time_plot,Xerr_list(1:index,3),'LineWidth',2)
hold on
plot(time_plot,Xerr_list(1:index,4),'LineWidth',2)
hold on
plot(time_plot,Xerr_list(1:index,5),'LineWidth',2)
hold on
plot(time_plot,Xerr_list(1:index,6),'LineWidth',2)
title('X_{Error} vs. Time for newtask')
legend('w_x', 'w_y', 'w_z', 'v_x', 'v_y', 'v_z')
xlabel('Time (s)')
ylabel('Error')


















