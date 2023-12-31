# slip_angle_control
Controller that keeps the front slip angle within a threshold of the peak slip angle of the tire. Uses estimator to estimate slip angle

# RUNNING THE MODEL
a_entry_point.m                 -> As the name suggests, this is the entry script. Simply hitting PLAY inside this scrpt wil result running the observer simulation and plots the results
input_script.m                  -> To edit inputs
esc_controller.m                -> Wrapper function which is called by the ODE function. This function contains ESC algorithm. This function calls the core vehicle model from within
vehicle_model_fw_simplified.m   -> Core function containing the equations of motion of the vehicle model. This is function has only 1 role, accept some inputs and states, and calculate the forces and accelerations

# DESCRIPTION
This repository contains the control algorithm for a slip angle
controller AND a non-linear state observer. 
Apart from this, the repository also contains the algorithm of a 
four-wheel model. 

The goal of the slip angle controller is to keep the slip angle within a 
threshold of the peak slip angle. This allows to improve vehicle performance
and reduce tire wear at the front. 

To utilize more of control theory and observre knowledge, I assume that
the the slip angle is not measureable. Therefore it must be estimated. 
The slip angle is estimated indirectly by estimating yaw rate and lateral 
velocity. The lateral velocity is assumed to be measurable and yaw rate is 
assumed to be not measureable. 

A non-linear state estimator is implemented which estimates yaw rate and 
lateral velocity. These estimates are then used to estimate the slip angle. 

The estimated slip angle is then controlled.

A linearized bicycle model along with pole placement technique is used to 
calculate the observer gains.
