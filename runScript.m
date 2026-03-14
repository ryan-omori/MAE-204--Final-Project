clc;clear;close
% Run Trajectory Generator

Tse_initial  = [0 0 1 0; 0 1 0 0; -1 0 0 0.5; 0 0 0 1];
Tsc_initial  = [1 0 0 1; 0 1 0 0;  0 0 1 0.025; 0 0 0 1];
Tsc_final    = [0 1 0 0; -1 0 0 -1; 0 0 1 0.025; 0 0 0 1];
Tce_grasp = [cos(3*pi/4) 0 sin(3*pi/4) 0; 
    0 1 0 0; 
    -sin(3*pi/4) 0 cos(3*pi/4) 0; 
    0 0 0 1];

Tce_standoff = [cos(3*pi/4) 0 sin(3*pi/4) 0; 
    0 1 0 0; 
    -sin(3*pi/4) 0 cos(3*pi/4) 0.1; 
    0 0 0 1];

dt = 0.01;
Tf = 4;

traj = TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_final, ...
                            Tce_grasp, Tce_standoff, Tf, dt);

writematrix(traj, 'trajectory.csv');


%% Run Next State
clc;clear;close
dt = 0.01;
% Robot parameters
r = 0.0475;
l = 0.235;
w = 0.15;

max_speed = 10;
state = zeros(12,1);
speeds = [0 0 0 0 0 -10 10 10 -10];  % wheel motion
N = 1000;
kin_traj = zeros(N,13);

for i = 1:N
    state = NextState(state, speeds, dt, max_speed,r,l,w);
    kin_traj(i,1:12) = state';
    kin_traj(i,13) = 0; % gripper open
end
writematrix(kin_traj,'Kin_test.csv')


%% Run Feedback
clc;clear;close
% Robot parameters
r = 0.0475;
l = 0.235;
w = 0.15;

% Robot geometry (given in project)
Blist = [ ...
0 0 1 0 0.033 0;
0 -1 0 -0.5076 0 0;
0 -1 0 -0.3526 0 0;
0 -1 0 -0.2176 0 0;
0 0 1 0 0 0]' ;

M0e = [1 0 0 0.033;
       0 1 0 0;
       0 0 1 0.6546;
       0 0 0 1];

Tb0 = [1 0 0 0.1662;
       0 1 0 0;
       0 0 1 0.0026;
       0 0 0 1];

% Parameters
state=[0 0 0 0 0 0.2 -1.6 0 0 0 0 0]'

Xd=[0 0 1 0.5; 0 1 0 0; -1 0 0 0.5; 0 0 0 1;];
Xd_next=[0 0 1 0.6; 0 1 0 0;-1 0 0 0.3;0 0 0 1;];
X=[0.170 0 0.985 0.387; 0 1 0 0; -0.985 0 0.170 0.570;0 0 0 1];
dt=.01;

% Initialize
Kp=zeros(6);
Ki=zeros(6);
Xerr_int = zeros(6,1);

%Calc Jacobian
Je=CalcJacobian(Blist,M0e,Tb0,r,l,w,state);
Je_pinv=pinv(Je,1e-4);
% Call FeedbackControl
[Vb, Vd, Xerr, Xerr_int, Ad]= FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, Xerr_int,Je);

controls = Je_pinv * Vb;
% Results
Ad*Vb
% Display Feedforward Reference twist
disp(Vd)

% Display End-Effector Twist
disp(Vb)

% Display Error
disp("Error Twist Xerr:")
disp(Xerr)

% Display Jacobian
disp("Jacobian Matrix:")
disp(Je)

% Display Linear and Angular Velocity [u,thetadot]
disp("Control Speeds [wheel speeds; joint speeds]:")
disp(controls)
