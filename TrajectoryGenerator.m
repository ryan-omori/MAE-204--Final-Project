% Function for creating Trajectory
function traj = TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_final, ...
                                    Tce_grasp, Tce_standoff,Tf,dt)
k=1;
method=5;

N = (Tf*k)/dt;

% Compute useful transforms
Tse_standoff_initial = Tsc_initial * Tce_standoff;
Tse_grasp = Tsc_initial * Tce_grasp;

Tse_standoff_final = Tsc_final * Tce_standoff;
Tse_release = Tsc_final * Tce_grasp;

% Segment 1: move to initial standoff
traj1 = ScrewTrajectory(Tse_initial, Tse_standoff_initial, k, N, method);

% Segment 2: move to grasp
traj2 = ScrewTrajectory(Tse_standoff_initial, Tse_grasp, k, N, method);

% Segment 3: close gripper (hold pose at grasp)
close_pose = traj2{N};
traj3 = cell(1, N);
for i = 1:N
    traj3{i} = close_pose;
end

% Segment 4: back to standoff
traj4 = ScrewTrajectory(Tse_grasp, Tse_standoff_initial, k, N, method);

% Segment 5: move to final standoff
traj5 = ScrewTrajectory(Tse_standoff_initial, Tse_standoff_final, k, N, method);

% Segment 6: move to release
traj6 = ScrewTrajectory(Tse_standoff_final, Tse_release, k, N, method);

% Segment 7: open gripper (hold pose at release)
open_pose = traj6{N};
traj7 = cell(1, N);
for i = 1:N
    traj7{i} = open_pose;
end
% Segment 8: back to standoff
traj8 = ScrewTrajectory(Tse_release, Tse_standoff_final, k, N, method);

% Combine Into Final Segment
traj_cell=[traj1 traj2 traj3 traj4 traj5 traj6 traj7 traj8];
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


end
