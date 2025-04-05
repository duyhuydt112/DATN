import time
import numpy as np
from lib.utils import plot_line
from gpio_module import PID, Motor

#init parameters
operate_time_s = 2      
sample_time_s = 0.01
theta_max_deg = 180

# Pre-allocating output arrays
time_list = []  # Time (s)
theta_list = []  # Measured shaft position (deg)
theta_setpoint_list = []  # Set point shft position (deg)
output_list = []  # Controler output

# Setting motion parameters
# (Valid options: 'sin', 'cos')
option = 'cos'
if option == 'sin':
    T = 2*operate_time_s  # Period of sine wave (s)
    theta0 = theta_max_deg  # Reference angle
elif option == 'cos':
    T = operate_time_s  # Period of cosine wave (s)
    theta0 = 0.5*theta_max_deg  # Reference angle

#PID controller object
kp = 0.036
ki = 0.379
kd = 0.0009
tau_pid = 0.01
pid = PID(sample_time_s, kp, kd, ki, tau=tau_pid)

my_motor = Motor(enable1=16, enable2=17, pwm1=18)
my_motor.reset_angle()

# Initializing variables and starting clock
pre_theta_deg = 0
pre_time_s = 0
curr_time_s = 0
start_time_s = time.perf_counter()

while(curr_time_s < operate_time_s):
    time.sleep(sample_time_s)       # Pausing for `tsample` to give CPU time to process encoder signal
    curr_time_s = time.perf_counter() - start_time_s
    curr_theta = my_motor.get_angle()
    if option == 'sin':
        curr_theta_setpoint = theta0 * np.sin((2*np.pi/T) * curr_time_s)
    elif option == 'cos':
        curr_theta_setpoint = theta0 * (1-np.cos((2*np.pi/T) * curr_time_s))
    curr_output = pid.control(curr_theta_setpoint, curr_theta)
    my_motor.set_output(curr_output)

    # Updating output arrays
    time_list.append(curr_time_s)
    theta_setpoint_list.append(curr_theta_setpoint)
    theta_list.append(curr_theta)
    output_list.append(curr_output)

    # Updating previous values
    pre_theta_deg = curr_theta
    pre_time_s = curr_time_s

print('Done.')
# Stopping motor and releasing GPIO pins
my_motor.set_output(0, brake=True)
del my_motor


# Calculating motor angular velocity (rad/s)
w_setpoint = np.pi/180 * np.gradient(theta_setpoint_list, time_list)
w = np.pi/180 * np.gradient(theta_list, time_list)

# Plotting results
plot_line(
    [time_list]*2, [theta_setpoint_list, theta_list], marker=True, yname=['Shaft Position (deg.)'],
    legend=['Set Point', 'Actual'])
plot_line(
    [time_list]*2, [w_setpoint, w], marker=True, yname=['Shaft Speed (rad/s)'],
    legend=['Set Point', 'Actual'])
plot_line(
    [time_list[1::], time_list], [1000*np.diff(time_list), output_list], marker=True, axes='multi',
    yname=['Sampling Period (ms)', 'Control Output (-)'])