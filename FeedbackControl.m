function [Vb,Vd, Xerr, Xerr_int,Je,Ad] = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, Xerr_int, Je)

Xerr = se3ToVec(MatrixLog6(TransInv(X)*Xd));
Xerr_int = Xerr_int + Xerr*dt;
Vd = se3ToVec((1/dt)*MatrixLog6(TransInv(Xd)*Xd_next));
Ad = Adjoint(TransInv(X)*Xd);
Vb = Ad*Vd + Kp*Xerr + Ki*Xerr_int;


end