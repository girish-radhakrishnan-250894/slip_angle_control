%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%% OBSERVER GAIN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% -> This script calculates the observer gain matrix L using a linearized
%    version of the bicycle model 

%% Initialization : Estimator Parameters
% Parameters
m = input.m_s + input.m_u_1 + input.m_u_2 + input.m_u_3 + input.m_u_4;
Izz = input.J_z;
C1 = input.C1;
C2 = input.C2;
u = input.u_start;
a = input.a_1;
b = abs(input.a_3);
l = a + b;
g = -9.81;

%% State Space Respresentation

% System Matrix
A = -[(C1 + C2)/(m*u),              u + (a*C1 - b*C2)/(m*u);
      (a*C1 - b*C2)/(Izz*u),    (a^2*C1 + b^2*C2)/(Izz*u)];

% Input Matrix
B = [C1/m;
     a*C1/Izz];

% Output Matrix
% NOTE - Assuming that lateral velocity is the output that can be measured 
C = [1/u a/u];

D = [-1];

%% Transfer Function : alpha/delta

[num,den] = ss2tf(A,B,C,D);

tf_alpha_delta = tf(num,den);
