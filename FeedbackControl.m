% Function for Feedforward and PI Feedback Control of a robot end-effector
% Inputs:
% X - Current end-effector configuration (4x4 SE(3) matrix)
% Xd - Desired end-effector configuration at current time step
% Xd_next - Desired end-effector configuration at next time step
% Kp - Proportional gain (6x6 or scalar)
% Ki - Integral gain (6x6 or scalar)
% dt - Time step duration
% Xerr_int - Accumulated integral of error (6x1 vector)
%
% Outputs:
% Vb - Body-frame twist command to apply (6x1 vector)
% Vd - Feedforward twist in desired frame (6x1 vector)
% Xerr - Current configuration error (6x1 vector)
% Xerr_int - Updated integral of error (6x1 vector)
% Ad - Adjoint transformation from desired frame to body frame

function [Vb,Vd, Xerr, Xerr_int,Ad] = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, Xerr_int)
Xerr = se3ToVec(MatrixLog6(TransInv(X)*Xd));
Xerr_int = Xerr_int + Xerr*dt;
Vd = se3ToVec((1/dt)*MatrixLog6(TransInv(Xd)*Xd_next));
Ad = Adjoint(TransInv(X)*Xd);
Vb = Ad*Vd + Kp*Xerr + Ki*Xerr_int;
end