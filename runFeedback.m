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
state=[0 0 0 0 0 0.2 -1.6 0]

Xd=[0 0 1 0.5; 0 1 0 0; -1 0 0 0.5; 0 0 0 1;];
Xd_next=[0 0 1 0.6; 0 1 0 0;-1 0 0 0.3;0 0 0 1;];
X=[0.170 0 0.985 0.387; 0 1 0 0; -0.985 0 0.170 0.570;0 0 0 1];
dt=.01;

% Initialize
Kp=zeros(6);
Ki=zeros(6);
Xerr_int = zeros(6,1);

%Calc Jacobian
Je=CalcJacobian(Blist,M0e,Tb0,r,l,w,state)

% Call FeedbackControl
[Vd,V, controls, Xerr, Xerr_int,Ad] = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, Xerr_int,Je);

% Results
% Display Feedforward Reference twist
disp(Vd)

% Display End-Effector Twist

disp(V)

% Display Error
disp("Error Twist Xerr:")
disp(Xerr)

% Display Jacobian
disp("Jacobian Matrix:")
disp(Je)

% Display Linear and Angular Velocity [u,thetadot]
disp("Control Speeds [wheel speeds; joint speeds]:")
disp(controls)
