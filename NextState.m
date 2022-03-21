%% Next State function- used to find the new configuration after wheels and arm joints are given velocities Vel for t seconds

%% Sample inputs

% set current state variables to 0
% current_state = zeros(1, 12);
% 
% % input velocities for robot spinning
% Vel = zeros(1, 9);
% Vel(1, 6:9) = [-10 10 10 -10];
% 
% % delta t - time step
% t = 0.01;
% 
% Vmax = 15;
% 
% file = zeros(101, 13);
% 
% % set first state to all zeros
% file(1,1:12) = current_state;
% 
% 
% state = NextState(current_state, Vel, t, Vmax);
% 
% Sample output
% 
% First 3 lines
% 0	0	0	0	0	0	0	0	0	0	0	0	0
% 0	0.00475	0	0	0	0	0	0	0.1	0.1	0.1	0.1	0
% 0	0.0095	0	0	0	0	0	0	0.2	0.2	0.2	0.2	0


%% Function

function New_state = NextState(Cur_state, Vel, t, Vmax)
    
    % robot dimensions
    r = 0.0475;
    l = 0.47/2;
    w = 0.3/2;

    % truncate velocities to Vmax
    Vel(Vel > Vmax) = Vmax;

    % initialize next state
    New_state = zeros(1, 12);
    
    % joint and wheel states (last 9 states except gripper)
    New_state(4:12) = Cur_state(4:12) + Vel*t;

    F = [-1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w)
          1 1 1 1
          -1 1 -1 1]*r/4;
    
    % delta theta
    theta_delta = (Vel(6:9)*t)';
    
    % body twist
    V_b = F*theta_delta;
    
    
    % change in chassis orientation wrt body frame
    if V_b(1) ~= 0 
        qb_delta = [V_b(1)
                    (V_b(2)*sin(V_b(1)) + V_b(3)*(cos(V_b(1)) - 1))/V_b(1)
                    (V_b(3)*sin(V_b(1)) + V_b(2)*(1 - cos(V_b(1))))/V_b(1)];
    else
        qb_delta = V_b;
    end
    
    % phi of chassis (first state)
    phi_k = Cur_state(1);
    
    % change in chassis orientation wrt space frame 
    q_delta = [1 0 0
               0 cos(phi_k) -sin(phi_k)
               0 sin(phi_k) cos(phi_k)]*qb_delta;
    
    
    % phi, x, y for the chassis
    New_state(1:3) =Cur_state(1:3) + q_delta';   

end
