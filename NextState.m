function next_state = NextState(state, speeds, dt, max_speed,r,l,w)
% Force all inputs to column vectors
state  = state(:);
speeds = speeds(:);
% Split state
phi = state(1);
x   = state(2);
y   = state(3);

arm = state(4:8);
wheels = state(9:12);

% Split speeds
Joint_speeds   = speeds(1:5);
wheel_speeds = speeds(6:9);

% Limit speeds
Joint_speeds   = max(min(Joint_speeds, max_speed), -max_speed);
wheel_speeds = max(min(wheel_speeds, max_speed), -max_speed);

% Update joints
arm_next = arm(:) + Joint_speeds * dt;
wheels_next = wheels(:) + wheel_speeds * dt;

F = (r/4)*[
    -1/(l+w)  1/(l+w)  1/(l+w) -1/(l+w);
     1        1        1        1;
    -1        1       -1        1];


delta_theta = wheel_speeds * dt;
Vb = F * delta_theta;

wbz = Vb(1);
vbx = Vb(2);
vby = Vb(3);

% Body twist integration
if abs(wbz) < 1e-5
    delta_qb = [0;
                vbx;
                vby];
else
    delta_qb = [
        wbz;
        (vbx*sin(wbz) + vby*(cos(wbz)-1))/wbz;
        (vby*sin(wbz) + vbx*(1-cos(wbz)))/wbz
    ];
end

% Convert body twist to space frame
rot = [
    1 0 0;
    0 cos(phi) -sin(phi);
    0 sin(phi)  cos(phi)
];

delta_q = rot * delta_qb;

phi_next = phi + delta_q(1);
x_next   = x   + delta_q(2);
y_next   = y   + delta_q(3);
% Add at end of NextState.m before return
phi_next = atan2(sin(phi_next), cos(phi_next));  % wrap to [-pi, pi]
next_state = [phi_next; x_next; y_next; arm_next; wheels_next];

end