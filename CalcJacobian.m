% CalcJacobian computes the full mobile manipulator Jacobian
% for the KUKA youBot, combining the base (chassis) and arm contributions.
%
% Inputs:
% Blist - screw axes of the arm in body frame (6x5)
% M0e - home configuration of the end-effector relative to arm base (4x4)
% Tb0 - transformation from chassis frame to arm base (4x4)
% r - wheel radius (meters)
% l - distance from robot center to wheel along x-axis (meters)
% w - distance from robot center to wheel along y-axis (meters)
% state - current robot state (phi, x, y, theta1..theta5, wheel angles)
%
% Output:
% Je - full 6x9 Jacobian (3 for angular velocity + 3 for linear velocity)

function Je=CalcJacobian(Blist, M0e,Tb0,r,l,w,state)

F = (r/4)*[-1/(l+w)  1/(l+w)  1/(l+w) -1/(l+w);...
     1        1        1        1;
    -1        1       -1        1];

F6 = [zeros(2,4); F; zeros(1,4)];

% Extract arm configuration
theta = state(4:8);

% Compute Jacobian
T0e = FKinBody(M0e,Blist,theta);
Tbe = Tb0 * T0e;
Teb = TransInv(Tbe);
Jarm = JacobianBody(Blist,theta);
Jbase = Adjoint(Teb)*F6;
Je = [Jbase Jarm];

end