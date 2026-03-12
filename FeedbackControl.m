function [Vb,Vd, controls, Xerr, Xerr_int,Je,Ad] = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, Xerr_int, Je)

Xerr = se3ToVec(MatrixLog6(TransInv(X)*Xd));
Xerr_int = Xerr_int + Xerr*dt;
Vd = se3ToVec((1/dt)*MatrixLog6(TransInv(Xd)*Xd_next));
Ad = Adjoint(TransInv(X)*Xd);
Vb = Ad*Vd + Kp*Xerr + Ki*Xerr_int;

% Damped least squares for non-square Je (6x9)
[U, S, V_svd] = svd(Je, 'econ');  % U:6x6, S:6x6, V_svd:9x6
lambda = 0.05;
s = diag(S);                                          % 6x1 singular values
s_damp = s ./ (s.^2 + lambda^2);                     % 6x1 damped reciprocals
Je_inv = V_svd * diag(s_damp) * U';                  % 9x6 * 6x6 * 6x6 = 9x6
controls = Je_inv * Vb;                               % 9x6 * 6x1 = 9x1

end