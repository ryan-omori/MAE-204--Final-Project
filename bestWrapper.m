clc; clear; close;

%% Initial configuration
initial_pos = [pi/6 0 -.2];
theta0 = [0, -0.5, -0.5, -0.5, 0];
wheel0 = [-pi/4 pi/4 -pi/4 pi/4];
state  = [initial_pos theta0 wheel0]';

dt = 0.01;
Tf = 10;
max_speed = 10;
% Best K Values So far
Kp = 2 * eye(6);
Ki = 0.001 * eye(6);

Xerr_int = zeros(6,1);

%% Robot parameters
r = 0.0475; l = 0.235; w = 0.15;

Blist = [0 0 1 0 0.033 0;
         0 -1 0 -0.5076 0 0;
         0 -1 0 -0.3526 0 0;
         0 -1 0 -0.2176 0 0;
         0 0 1 0 0 0]';

M0e = [1 0 0 0.033; 0 1 0 0; 0 0 1 0.6546; 0 0 0 1];
Tb0 = [1 0 0 0.1662; 0 1 0 0; 0 0 1 0.0026; 0 0 0 1];

cube_int=[0 1 0];
Tsc_initial = [cos(cube_int(1)) -sin(cube_int(1)) 0 cube_int(2); 
               sin(cube_int(1)) cos(cube_int(1)) 0 cube_int(3); 
               0 0 1 0.025; 
               0 0 0 1];

cube_goal=[-pi/2 0 -1];
Tsc_final   = [cos(cube_goal(1)) -sin(cube_goal(1)) 0 cube_goal(2); 
               sin(cube_goal(1)) cos(cube_goal(1)) 0 cube_goal(3); 
               0 0 1 0.025; 
               0 0 0 1];

% Textbook hardcoded value - px=0 not 1
angle_attack = 3*pi/4;
Tce_grasp    = [cos(angle_attack) 0 sin(angle_attack) 0;
                0 1 0 0;
               -sin(angle_attack) 0 cos(angle_attack) 0;
                0 0 0 1];

Tce_standoff = [cos(angle_attack) 0 sin(angle_attack) 0;
                0 1 0 0;
               -sin(angle_attack) 0 cos(angle_attack) 0.1;
                0 0 0 1];
%% Compute X_init from FK at initial state
phi_init   = state(1);
x_init     = state(2);
y_init     = state(3);
theta_init = state(4:8);
T0e_init   = FKinBody(M0e, Blist, theta_init);
Tsb_init   = [cos(phi_init) -sin(phi_init) 0 x_init;
              sin(phi_init)  cos(phi_init) 0 y_init;
              0              0             1 0.0963;
              0              0             0 1];
Tse_initial = Tsb_init * Tb0 * T0e_init;
%Tse_initial = [0 0 1 0; 0 1 0 0; -1 0 0 0.5; 0 0 0 1];
%% Generate trajectory - returns cell array
traj_cell = TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_final, ...
                                Tce_grasp, Tce_standoff, Tf, dt);

%% Convert cell array to matrix
M = length(traj_cell);
seg_N = M / 8;  % steps per segment

traj = zeros(M, 13);
for i = 1:M
    T = traj_cell{i};
    traj(i,1:9)  = [T(1,1) T(1,2) T(1,3) ...
                    T(2,1) T(2,2) T(2,3) ...
                    T(3,1) T(3,2) T(3,3)];
    traj(i,10:12) = [T(1,4) T(2,4) T(3,4)];
end

% Gripper closed during segments 3-6 (indices 2*seg_N+1 to 6*seg_N)
traj(:,13) = 0;
traj(2*seg_N+1 : 6*seg_N, 13) = 1;

N = M;  % total number of trajectory points
robot_traj = zeros(N, 13);
Xerr_log   = zeros(N, 6);
% Joint limits (radians) - tune to your arm's actual limits
joint_min = [-2.5, -1.8, -1.8, -1.8, -2.5];
joint_max = [ 2.5,  1.8,  1.8,  1.8,  2.5];

%% MAIN LOOP
for i = 1:N-1
    phi   = state(1);
    x     = state(2);
    y     = state(3);
    theta = state(4:8);

    T0e = FKinBody(M0e, Blist, theta);
    Tsb = [cos(phi) -sin(phi) 0 x;
           sin(phi)  cos(phi) 0 y;
           0         0        1 0.0963;
           0         0        0 1];
    X = Tsb * Tb0 * T0e;

    R      = reshape(traj(i,1:9), 3, 3)';
    p      = traj(i,10:12)';
    Xd     = [R p; 0 0 0 1];

    R_next = reshape(traj(i+1,1:9), 3, 3)';
    p_next = traj(i+1,10:12)';
    Xd_next = [R_next p_next; 0 0 0 1];

    gripper = traj(i,13);

    % --- Compute full Jacobian ---
    Je = CalcJacobian(Blist, M0e, Tb0, r, l, w, state);
    % --- Feedback control ---
    Xerr_int = max(min(Xerr_int, 0.05), -0.05);
    [V, Vd, Xerr, Xerr_int, Ad] = FeedbackControl(...
        X, Xd, Xd_next, Kp, Ki, dt, Xerr_int, Je);



    % --- Joint limit check ---
    controls_test = pinv(Je) * V;
    theta_next_test = theta + controls_test(5:9) * dt;
    violated = testJointLimits(theta_next_test);

    % Zero out columns of Je for violated joints (arm joints = cols 5-9)
    Je_limited = Je;
    for j = 1:5
        if violated(j)
            Je_limited(:, j+4) = 0;  % arm joints are cols 5-9 in Je
        end
    end

    % --- Damped pseudoinverse on potentially modified Jacobian ---
    lambda = 0.01;
    [U, S, V_svd] = svd(Je_limited, 'econ');
    s = diag(S);
    s_damp = s ./ (s.^2 + lambda^2);
    Je_pinv = V_svd * diag(s_damp) * U';

    controls = Je_pinv * V;

    wheel_speeds = controls(1:4);
    joint_speeds = controls(5:9);
    speeds = [joint_speeds; wheel_speeds];

    robot_traj(i,1:12) = state(1:12)';
    robot_traj(i,13)   = gripper;
    Xerr_log(i,:)      = Xerr';

    state = NextState(state, speeds, dt, max_speed, r, l, w);
    % Hard clamp as a safety net (secondary enforcement)
%    state(4:8) = max(min(state(4:8), [2.5 1.8 1.8 1.8 2.5]'), ...
 %                                    [-2.5 -1.8 -0.2 -0.2 -2.5]');
end
robot_traj(N,1:12) = state(1:12)';
robot_traj(N,13)   = traj(N,13);

%% Write CSV
writematrix(robot_traj, 'BestSim.csv')

%% Plot error
figure
plot(Xerr_log)
title("End Effector Error")
xlabel("Time Step")
ylabel("Error")
legend("wx","wy","wz","vx","vy","vz")

function violated = testJointLimits(theta)

joint_min = [-2.5, -1.8, -1.8, -1.8, -2.5];
joint_max = [ 2.5,  1.8, -0.2, -0.2,  2.5];  % joints 3,4 must stay < -0.2
violated = (theta' < joint_min) | (theta' > joint_max);
end