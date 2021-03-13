function [ref_traj, size_traj] = TrajectoryGenerator(end_effector_init, cube_init,cube_final,k)
%end_effector_init: The initial configuration of the end-effector in the reference trajectory: Tse,initial.
%cube_init: The cube's initial configuration: Tsc,initial.
%cube_final: The cube's desired final configuration: Tsc,final.
%The end-effector's configuration relative to the cube when it is grasping the cube: Tce,grasp.
%The end-effector's standoff configuration above the cube, before and after grasping, relative to the cube: Tce,standoff. This specifies the configuration of the end-effector {e} relative to the cube frame {c} before lowering to the grasp configuration Tce,grasp, for example.
%k: The number of trajectory reference configurations per 0.01 seconds

%Output%
%Tse: Representation of end-effector frame in space frame, size(N*13)
%CSV file depicting the concatenated eight-segment reference trajectory
%r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, gripper state
y_rot = [cos(3*pi/4) 0 sin(3*pi/4); 0 1 0; -sin(3*pi/4) 0 cos(3*pi/4)];
init_orient = [cos(cube_init(1,3)) -sin(cube_init(1,3)) 0; sin(cube_init(1,3)) cos(cube_init(1,3)) 0 ; 0 0 1 ];
final_orient = [cos(cube_final(1,3)) -sin(cube_final(1,3)) 0; sin(cube_final(1,3)) cos(cube_final(1,3)) 0; 0 0 1];
init_pos = [cube_init(1,1); cube_init(1,2); 0.025; 1];
final_pos = [cube_final(1,1);cube_final(1,2);0.025; 1];
cube_init_rot = init_orient*y_rot ;
cube_final_rot = final_orient*y_rot;
cube_init = [cube_init_rot; 0 0 0];
cube_init = [cube_init init_pos];
cube_final = [cube_final_rot; 0 0 0];
cube_final = [cube_final final_pos];

%z-offset at stand-off position
stand_off_config = [0 0 0 0; 0 0 0 0; 0 0 0 0.1; 0 0 0 0];
N = k/0.01; % Number of reference points
method = 5; %quintic (fifth-order polynomial) time scaling
max_lin_vel = 0.35; %Max liner velocity 0.35 m/s
closure_open = 0.5* k/0.01; %Number of lines required for end-effector to open/close
traj_config = [];
output = [];

 
%A trajectory to move the gripper from its initial configuration to a "standoff" configuration a few cm above the block.
stand_off_init = cube_init + stand_off_config;
pts1 = end_effector_init(1:3,4);
pts2 = stand_off_init(1:3,4);
Tf = abs(sqrt(sum((pts1 - pts2 ) .^ 2))/max_lin_vel);
traj_1 = ScrewTrajectory(end_effector_init, stand_off_init, Tf, N, method);


for j = 1:size(traj_1,2)

    z = traj_1{j};
    traj_config = [z(1,1:3), z(2,1:3), z(3,1:3), transpose(z(1:3,4))];
    output = [output; traj_config 0];
    
end


%A trajectory to move the gripper down to the grasp position.
pts1 = stand_off_init(1:3,4);
pts2 = cube_init(1:3,4);
Tf = abs(sqrt(sum((pts1 - pts2 ) .^ 2))/max_lin_vel); 
traj_2 = ScrewTrajectory(stand_off_init, cube_init, Tf, N, method);

for j = 1:size(traj_2,2)

    z = traj_2{j};
    traj_config = [z(1,1:3), z(2,1:3), z(3,1:3), transpose(z(1:3,4))];
    output = [output; traj_config 0];
    
end

%Closing of the gripper.
last_row_output = output(size(output,1),1:12);

for i=1:closure_open
    output = [output; last_row_output 1];
end

%A trajectory to move the gripper back up to the "standoff" configuration.
pts1 = cube_init(1:3,4);
pts2 = stand_off_init(1:3,4);
Tf = abs(sqrt(sum((pts1 - pts2 ) .^ 2))/max_lin_vel); 
traj_4 = ScrewTrajectory(cube_init,stand_off_init , Tf, N, method);

for j = 1:size(traj_4,2)

    z = traj_4{j};
    traj_config = [z(1,1:3), z(2,1:3), z(3,1:3), transpose(z(1:3,4))];
    output = [output; traj_config 1];
    
end

%A trajectory to move the gripper to a "standoff" configuration above the final configuration.
stand_off_final = cube_final + stand_off_config;
pts1 = stand_off_init(1:3,4);
pts2 = stand_off_final(1:3,4);
Tf = abs(sqrt(sum((pts1 - pts2 ) .^ 2))/max_lin_vel); 
traj_5 = ScrewTrajectory(stand_off_init,stand_off_final , Tf, N, method);

for j = 1:size(traj_5,2)

    z = traj_5{j};
    traj_config = [z(1,1:3), z(2,1:3), z(3,1:3), transpose(z(1:3,4))];
    output = [output; traj_config 1];
    
end

%A trajectory to move the gripper to the final configuration of the object.
pts1 = stand_off_final(1:3,4);
pts2 = cube_final(1:3,4);
Tf = abs(sqrt(sum((pts1 - pts2 ) .^ 2))/max_lin_vel); 
traj_6 = ScrewTrajectory(stand_off_final, cube_final, Tf, N, method);

for j = 1:size(traj_6,2)

    z = traj_6{j};
    traj_config = [z(1,1:3), z(2,1:3), z(3,1:3), transpose(z(1:3,4))];
    output = [output; traj_config 1];
    
end

%Opening of the gripper.

last_row_output = output(size(output,1),1:12);

for i=1:closure_open
    output = [output; last_row_output 0];
end

%A trajectory to move the gripper back to the "standoff" configuration.
pts1 = cube_final(1:3,4);
pts2 = stand_off_final(1:3,4);
Tf = abs(sqrt(sum((pts1 - pts2 ) .^ 2))/max_lin_vel); 
traj_8 = ScrewTrajectory(cube_final,stand_off_final, Tf, N, method);

for j = 1:size(traj_8,2)

    z = traj_8{j};
    traj_config = [z(1,1:3), z(2,1:3), z(3,1:3), transpose(z(1:3,4))];
    output = [output; traj_config 0];
    
end
ref_traj = output;
size_traj = size(output);
csvwrite('TrajectoryGenerator.csv', output, 0, 0);


end

