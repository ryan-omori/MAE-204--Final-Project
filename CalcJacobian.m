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