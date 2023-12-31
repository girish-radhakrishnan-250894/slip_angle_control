function [Q_dot,O_simulator,O_model] = slip_angle_controller(t, Q,input)
%slip_angle_controller Simulator function that runs vehicle simulations
%   This is a wrapping function that is called by the numerical integrator.
%   It knows the current time-step and using it, it interpolates all the
%   inputs to be tracked. It also calculates the steering and throttle
%   control action needed and it passes these as scalar values to the
%   vehicle model.

% The goal of the slip angle controller is to keep the slip angle within a 
% threshold of the peak slip angle. This allows to improve vehicle performance
% and reduce tire wear at the front. 

% To utilize more of control theory and observre knowledge, I assume that
% the the slip angle is not measureable. Therefore it must be estimated. 
% The slip angle is estimated indirectly by estimating yaw rate and lateral 
% velocity. The lateral velocity is assumed to be measurable and yaw rate is 
% assumed to be not measureable. 

% A non-linear state estimator is implemented which estimates yaw rate and 
% lateral velocity. These estimates are then used to estimate the slip angle. 

% The estimated slip angle is then controlled.

%% Initialization : State Observer variables (only those required to calculate the necessary observer actions)

v_hat = Q(29); % Estimate of lateral velocity - v
r_hat = Q(30); % Estimate of yaq rate - r

%% Initialization : Controller variables (only those required to calculate the necessary observer actions)

Zc_alpha = [Q(31:33)];

%% Initialization : Filtered error

e_alpha = Q(34);

Zc_error = e_alpha;
%% Initialization : Inputs (reference inputs to be tracked by controller)
delta_steering_input = interp1(input.time, input.delta, t, 'pchip');

delta_controller_unsaturated = input.C_alpha * Zc_alpha;
delta_controller = 0*F04_SATURATION_1(-deg2rad(5),deg2rad(5),delta_controller_unsaturated,0.05,1);

if abs(e_alpha) < deg2rad(0.001)
    delta_controller = 0;
end

delta_c = delta_steering_input + delta_controller;

m_d_c = 0;

%% Measurement data 
% NOTE - Assuming that the four-wheel model's results can be assumed as a
% sensor data which will be fed to the state-estimator
q = Q(1:28);
[q_dot , ~ , ~ ,O_model] = vehicle_model_fw_simplified(q,input,delta_c,m_d_c);


%% Initialization : Measured state

% This is the measured signal that the estimator tries to track
% This could be a signal coming from a sensor but in this code it is coming
% from the four-wheel model that is significantly more complex than the
% estimator model. Therefore, it can be considered to be sensor measurement
r_measured = q_dot(6);

v_measured = O_model(2); % The lateral velocity state variable is in the world frame. Hence, inside the vehicle model, it is converted to the chassis frame and stored in the O_model vector

%% Initialization : State Observer 

% Parameters
m = input.m_s;
Izz = input.J_z;
C1 = input.C1;
C2 = input.C2;
u = O_model(1);
a = input.a_1;
b = abs(input.a_3);
l = a + b;
g = 9.81;
%% Observer Dynamics : Slip Angles

% Front slip angle
alpha_1_hat = -atan( ( -u*sin(delta_c) + (v_hat + a*r_hat)*cos(delta_c) ) / ( u*cos(delta_c) + (v_hat + a*r_hat)*sin(delta_c)  ));

% Rear slip angle
alpha_2_hat = -atan( ( -u*sin(0) + (v_hat - b*r_hat)*cos(0) ) / ( u*cos(0) + (v_hat - b*r_hat)*sin(0)  ));

%% Controller Dynamics : Error Calculation

target_alpha_1 = input.peak_alpha_1;

e_alpha_unfiltered = 0;

if (abs(target_alpha_1) - abs(alpha_1_hat) < -deg2rad(0.1) )
    e_alpha_unfiltered = abs(target_alpha_1) - abs(alpha_1_hat);
end

% e_alpha = min(-deg2rad(0.5), e_alpha);
% e_alpha = target_alpha_1 - alpha_1_hat;
% 
% e_alpha = min(0,e_alpha);

% if (r_hat > 0) % Left turn & Slip Angle is positive (in adapted ISO which is what is used in calculations above)
%     e_alpha =  e_alpha;
% elseif (r_hat < 0) % Right turn & slip angle is negative (in adapted ISO which is what is used in calculations above)
%     e_alpha = - e_alpha;
% end


%% Observer Dynamics : Forces

Fz_1 = m*g*b/(l);
Fz_2 = m*g*a/(l);

mf_input_1 = [Fz_1/2, 0, -alpha_1_hat, 0, 0, u];
mf_input_2 = [Fz_2/2, 0, -alpha_2_hat, 0, 0, u];

mf_output_1 = mfeval(input.tirFile_1, mf_input_1, 211);
mf_output_2 = mfeval(input.tirFile_2, mf_input_2, 211);

Fy_1__1 = mf_output_1(2)*2; % In Tire frame of refernecce 
Fy_2__2 = mf_output_2(2)*2; % In Tire frame of refernecce 

Fy_1 = Fy_1__1 * cos(delta_c); % In chassis frame of reference 
Fy_2 = Fy_2__2; % No steering in rear sore tire frame = chassis frame



%% Initializing : Measured output and estimator output

y = v_measured;

y_hat = v_hat;

%% Estimator Dynamics

% x_hat_dot = A*x_hat + B*u + L*(y - y_hat);

% Estimator Mass Matrix
M_hat = [m 0
         0 Izz];

% Estimator Force & Moments vector
f_hat_qd_q_u = [Fy_1 + Fy_2 - m*u*r_hat;
                a*Fy_1 - b*Fy_2
                ];

%% Controller Dynamics


Zc_dot_alpha = input.A_alpha*Zc_alpha + input.B_alpha*e_alpha;

%% Filtering the Error

Zc_dot_error = input.A_lpf*Zc_error + input.B_lpf*e_alpha_unfiltered;


%% Augmented system dynamics ([model estimator])

Q_dot = [q_dot;
         (M_hat\f_hat_qd_q_u) + input.L*(y - y_hat);
         Zc_dot_alpha;
         Zc_dot_error
         ];

%% Initializing outputs to be logged

O_simulator = [O_model(2);
               (O_model(3) + O_model(4))/2;
               alpha_1_hat;
               e_alpha;
               delta_controller
               ];








end