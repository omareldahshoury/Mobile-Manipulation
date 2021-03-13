function [U_theta_dot,Xerr,Xerr_tot] = FeedbackControl (X, Xd,Xd_next,Kp,Ki,time_step, Je, Xerr_tot)
%inputs%
% X: The current actual end-effector configuration X (also written Tse).
% Xd: The current end-effector reference configuration Xd (i.e., Tse,d).
% Xd_next: The end-effector reference configuration at the next timestep in the reference trajectory, Xd,next (i.e., Tse,d,next), at a time ?t later.
% The PI gain matrices Kp and Ki.
% The timestep delta_t between reference trajectory configurations.

%Output%
%The commanded end-effector twist {V} expressed in the end-effector frame {e}.
%U_theta_dot: Speed Vector [W1,..,W4,theta_dota1,...,theta_dot5]
speed_limit = csvread('Input.csv', 9, 1, [9 1 9 1]);
Xerr_tot = 0;
Xerr = se3ToVec(MatrixLog6(inv(X) * Xd));
Vd = se3ToVec(MatrixLog6(inv(Xd) * Xd_next)/time_step);
Xerr_tot = Xerr_tot + Xerr;
V = Adjoint(inv(X) * Xd)*Vd + Kp*Xerr + Ki*Xerr_tot*time_step;
U_theta_dot = transpose(pinv(Je,1e-2) * V);

end
            