clear; clc;
disp('Generating animation csv file..')
pause(1);
error_plot = [];
delta_t = 0.01;
Tb0 = [1 0 0 0.1662; 0 1 0 0; 0 0 1 0.0026; 0 0 0 1];
M0e = [1 0 0 0.033; 0 1 0 0; 0 0 1 0.6546; 0 0 0 1];%forward kinematics
init_config_vector = csvread('Input.csv',2 , 0  , [2 0 2 11]);
Config_list = [init_config_vector 0];
time_step = csvread('Input.csv', 9, 0, [9 0 9 0]);
speed_limit = csvread('Input.csv', 9, 1, [9 1 9 1]);
%%initial configuration of the end-effector reference trajectory
end_effector_init = [0 0 1 0; 0 1 0 0; -1 0 0 0.5; 0 0 0 1]; %Tse_initial

%initial configuration of the youbot
X = end_effector_init;
cube_init = csvread('Input.csv', 13, 0, [13 0 13 2]);
cube_final = csvread('Input.csv', 16, 0, [16 0 16 2]);
k = 10; %number of trajectory reference configurations per 0.01 seconds
%Trajectory generation
[ref_traj, size_traj] = TrajectoryGenerator(end_effector_init, cube_init,cube_final,k);

%robot dimensions
l = 0.235;
w = 0.15;
r = 0.0475;

%Je = [Jbase Jbody] // Jacobian Calculation
Blist = [0 0 0 0 0; 0 -1 -1 -1 0; 1 0 0 0 1; 0 -0.5076 -0.3526 -0.2176 0; 0.033 0 0 0 0; 0 0 0 0 0];
v_b = (r/4)*[-1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w); 1 1 1 1; -1 1 -1 1];
F6 = [0 0 0 0; 0 0 0 0; v_b; 0 0 0 0];
Kp = (csvread('Input.csv', 6, 0, [6 0 6 0]))*eye(6);
Ki = (csvread('Input.csv', 6, 1, [6 1 6 1]))*eye(6);

for j=1:(size_traj(1,1)-1)
    
    Xd = Vec13ToMat(ref_traj(j,:));
    Xd_next = Vec13ToMat(ref_traj(j+1,:));
    
    if j == 1
        
        thetalist = transpose(init_config_vector(1,4:8));
        T0e = FKinBody(M0e, Blist, thetalist);
        phi = init_config_vector(1,1); x = init_config_vector(1,2); y = init_config_vector(1,3);
        Tsb = [cos(phi) -sin(phi) 0 x; sin(phi) cos(phi) 0 y; 0 0 1 0.0963; 0 0 0 1];
        X = Tsb*Tb0*T0e;
        Jbody = JacobianBody(Blist, thetalist);
        Jbase = Adjoint(inv(T0e)*inv(Tb0))*F6; 
        Je = [Jbase Jbody];
        [speed_vector, error,Xerr_tot] = FeedbackControl (X, Xd,Xd_next,Kp,Ki,time_step,Je,0);        
        %calculating new configuration with the calc. speed vector
        next_config = NextState(init_config_vector, speed_vector,time_step, speed_limit);
        
    else

        thetalist = transpose(next_config(1,4:8));
        T0e = FKinBody(M0e, Blist, thetalist);
        phi = next_config(1,1); x = next_config(1,2); y = next_config(1,3);
        Tsb = [cos(phi) -sin(phi) 0 x; sin(phi) cos(phi) 0 y; 0 0 1 0.0963; 0 0 0 1];
        X = Tsb*Tb0*T0e;
        Jbody = JacobianBody(Blist, thetalist);
        Jbase = Adjoint(inv(T0e)*inv(Tb0))*F6; 
        Je = [Jbase Jbody];
        [speed_vector, error, Xerr_tot] = FeedbackControl (X, Xd,Xd_next,Kp,Ki,time_step, Je, Xerr_tot);
        next_config = NextState(next_config, speed_vector,time_step, speed_limit);
    end
    %creating error and configuration matrices
    error_plot = [error_plot; transpose(error)];
    Config_list = [Config_list; next_config ref_traj(j,13)];
    fprintf('Percent %% %d', int8((j/(size_traj(1,1)-1))*100));
    clc;
    
end

%printing outputs
csvwrite('Output.csv',Config_list);
disp('Percent 100%')
disp('Animation csv file created')
disp('Writing error plot data')
csvwrite('error_plot.csv',error_plot);
disp('Done!')

