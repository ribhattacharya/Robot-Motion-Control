%% Trajectory Generator function

%% Sample inputs
% T_se_init = [0 0 1 0; 
%              0 1 0 0; 
%             -1 0 0 0.5; 
%              0 0 0 1];
% 
% T_sc_init = [eye(3) [1 0 0.025]';
%                 0 0 0 1];
% 
% T_sc_final = [0 1 0 0; 
%              -1 0 0 0;
%               0 0 1 0;
%               0 0 0 1] * T_sc_init;
% 
% % Chosen arbitrarily for Milestone 2
% T_ce_grasp = [0 0 1 0; 
%               0 1 0 0; 
%              -1 0 0 0; 
%               0 0 0 1];
% 
% T_ce_standoff = [0 0 1 0; 
%                  0 1 0 0; 
%                 -1 0 0 0.1; % effector is 0.1 m above cube at standoff 
%                  0 0 0 1];
% 
% k = 1;
% 
% trajectory = TrajectoryGenerator(T_se_init, T_sc_init, T_sc_final, ...
%     T_ce_grasp, T_ce_standoff, k);

% Output
% First 2 rows = 
% 0         0    1.0000         0    1.0000         0   -1.0000         0         0         0         0    0.5000         0
% 0         0    1.0000         0    1.0000         0   -1.0000         0         0    0.0000         0    0.5000         0


%% Function body

function final_trajectory = TrajectoryGenerator(T_se_init, T_sc_init, T_sc_final, ...
    T_ce_grasp, T_ce_standoff, k)
    
    % Calculate required transformation matrices
    
    % effector wrt world at point A
    T_se_cube_1 = T_sc_init * T_ce_grasp;
    
    % effector standoff position before reaching A
    T_standoff_1 = T_sc_init * T_ce_standoff;
    
    % effector standoff position before reaching B
    T_standoff_2 = T_sc_final * T_ce_standoff;

    % effector wrt world at point B
    T_se_cube_2 = T_sc_final * T_ce_grasp;


    T = 5;
    N = k * T / 0.01;

     
    % Compute all trajectories
    traj_1 = CartesianTrajectory(T_se_init, T_standoff_1, T, N, 5);
    t1 = SE3_2_vector(traj_1, 0);
    
    
    traj_2 = CartesianTrajectory(T_standoff_1, T_se_cube_1, T, N, 5);
    t2 = SE3_2_vector(traj_2, 0);
    
    % gripping action
    traj_3 = CartesianTrajectory(T_se_cube_1, T_se_cube_1, 0.63, 63, 5);
    t3 = SE3_2_vector(traj_3, 1);
    
    
    traj_4 = CartesianTrajectory(T_se_cube_1, T_standoff_1, T, N, 5);
    t4 = SE3_2_vector(traj_4, 1);
    
    
    traj_5 = CartesianTrajectory(T_standoff_1, T_standoff_2, T, N, 5);
    t5 = SE3_2_vector(traj_5, 1);
    
    
    traj_6 = CartesianTrajectory(T_standoff_2, T_se_cube_2, T, N, 5);
    t6 = SE3_2_vector(traj_6, 1);
    
    % releasing grip action
    traj_7 = CartesianTrajectory(T_se_cube_2, T_se_cube_2, 0.63, 63, 5);
    t7 = SE3_2_vector(traj_7, 0);
    
    
    traj_8 = CartesianTrajectory(T_se_cube_2, T_standoff_2, T, N, 5);
    t8 = SE3_2_vector(traj_8, 0);
    
    % concatenate all trajectories into one single array
    final_trajectory = [t1;t2;t3;t4;t5;t6;t7;t8]; 

    % Function to convert SE3 matrices (4x4) and gripper status (0/1) to row form (13x1) in csv
    function trajectory = SE3_2_vector(traj, gripper)
        trajectory = zeros(length(traj),13);
        for i = 1:length(traj)
            trajectory(i,:) = ...
                     [  traj{1,i}(1,1) traj{1,i}(1,2) traj{1,i}(1,3) ... 
                        traj{1,i}(2,1) traj{1,i}(2,2) traj{1,i}(2,3) ...
                        traj{1,i}(3,1) traj{1,i}(3,2) traj{1,i}(3,3) ...
                        traj{1,i}(1,4) traj{1,i}(2,4) traj{1,i}(3,4) gripper ...
                     ];
        end
    end

end