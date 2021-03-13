function [next_config] = NextState(config_vector, speed_vector,time_step, speed_limit)

%Input%
%config_vector: 12 vector [chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4]
% where 'J' is arm joint angle & 'W' is wheel angle // gripper state: '0'
% for opened and '1' for closed
%speed_vector: 9 vector indicating arm joint speeds 'theta_dot' and wheel speeds 'U'
%speed_vector: [W1,..,W4,theta_dot1,...,theta_dot5]
%time_step: 0.01 sec.
%speed_limit: A positive real value indicating the maximum angular speed of
%the arm joints and the wheels (rad/sec)

%Output%
%A 12-vector representing the configuration of the robot time delta_t later
%Next_config = [chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4]
% new arm joint angles = (old arm joint angles) + (joint speeds) * delta_t
%new wheel angles = (old wheel angles) + (wheel speeds) * delta_t
%new chassis configuration is obtained from odometry

delta_theta = [];
next_config = config_vector;
%robot dimensions
l = 0.235;
w = 0.15;
r = 0.0475;


%Making sure that maximum speed is not exceeded
for i=1:9
   if speed_vector(1,i)>speed_limit
        speed_vector(1,i) = speed_limit;
        disp('speed was set to max limit')
    else if speed_vector(1,i) < -1*speed_limit
        speed_vector(1,i) = -1*speed_limit;
        disp('speed was set to max limit')
        end
   end
end
    
%calculating new arm angles
for i=5:9

        arm_angle = config_vector(1,i-1) + speed_vector(1,i)*time_step;
        next_config(1,i-1) = arm_angle;

end

%calculating new wheel angles
for i=1:4

        delta_theta(i,1) = speed_vector(1,i)*time_step;
        wheel_angle = config_vector(1,i+8) + delta_theta(i,1);
        next_config(1,i+8) = wheel_angle;

end

%calculating the chassis configuration
v_b = (r/4)*[-1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w); 1 1 1 1; -1 1 -1 1]*delta_theta;

if v_b(1,1) == 0;

    delta_config = [0, v_b(2,1), v_b(3,1)];

else
    delta_phi = v_b(1,1);
    delta_x = (v_b(2,1)*sin(delta_phi) + (v_b(3,1)*(cos(delta_phi)-1)))/delta_phi;
    delta_y = (v_b(3,1)*sin(delta_phi) + (v_b(2,1)*(1-cos(delta_phi))))/delta_phi;
    delta_config = [delta_phi, delta_x, delta_y];

end

next_config(1,1:3) = config_vector(1,1:3) + delta_config;

end

