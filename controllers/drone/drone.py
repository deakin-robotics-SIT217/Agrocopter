# mavic2pro controller
# rewritten in Python from the original C sample source from webots (which is licensed under Apache 2.0)

# Import classes
from controller import Robot
from controller import InertialUnit
from controller import Gyro
from controller import Motor
from controller import GPS
import math

#Function definitions
def clamp(n, smallest, largest): 
    return max(smallest, min(n, largest))

# create the Robot instance.
agro_drone = Robot()

# get the time step of the current world.
timestep = int(agro_drone.getBasicTimeStep())

# Initialise devices
imu = agro_drone.getInertialUnit('inertial unit')
imu.enable(timestep)

gps = agro_drone.getGPS('gps')
gps.enable(timestep)

gyro = agro_drone.getGyro('gyro')
gyro.enable(timestep)

front_left_motor = agro_drone.getMotor("front left propeller")
front_right_motor = agro_drone.getMotor("front right propeller")
rear_left_motor = agro_drone.getMotor("rear left propeller")
rear_right_motor = agro_drone.getMotor("rear right propeller")
motors = [front_left_motor, front_right_motor, rear_left_motor, rear_right_motor]
for motor in motors:
    motor.setPosition(float('inf'))
    motor.setVelocity(1.0)

# Balance variables
k_vertical_thrust = 68.5;  # with this thrust, the drone lifts.
k_vertical_offset = 0.6;   # Vertical offset where the robot actually targets to stabilize itself.
k_vertical_p = 3.0;        # P constant of the vertical PID.
k_roll_p = 50.0;           # P constant of the roll PID.
k_pitch_p = 30.0;          # P constant of the pitch PID.

target_altitude = 1.0

# Main loop:
while agro_drone.step(timestep) != -1:
    # SENSE
    time = agro_drone.getTime()
    
    imu_roll_pitch_yaw = imu.getRollPitchYaw()
    
    roll = imu_roll_pitch_yaw[0] + math.pi / 2.0
    pitch = imu_roll_pitch_yaw[1]
    
    gyro_x_y_z = gyro.getValues()

    roll_acceleration = gyro_x_y_z[0]
    pitch_acceleration = gyro_x_y_z[1]

    gps_values = gps.getValues()
    # These variables will be used to affect movement in X & Z
    roll_disturbance = 0.0
    pitch_disturbance = 0.0
    yaw_disturbance = 0.0
    
    # This is where the robot thinks it is in Y
    altitude = 70.0

    pitch_disturbance = -2.0
    if(gps_values[0]< -32):
    
        front_left_motor_input = 0
        front_right_motor_input = 0
        rear_left_motor_input = 0
        rear_right_motor_input = 0
        print(str(gps_values[0])+","+str(gps_values[1])+","+str(gps_values[2]))
        yaw_disturbance = 20

    if(gps_values[0] >-7):
        
        print(str(gps_values[0])+","+str(gps_values[1])+","+str(gps_values[2]))
        yaw_disturbance = -1.3

    if(gps_values[2] <-6):
        
        print(str(gps_values[0])+","+str(gps_values[1])+","+str(gps_values[2]))
        front_left_motor_input = 0
        front_right_motor_input = 0
        rear_left_motor_input = 0
        rear_right_motor_input = 0
    

    # THINK
    roll_input = k_roll_p * clamp(roll, -1.0, 1.0) + roll_acceleration + roll_disturbance
    pitch_input = k_pitch_p * clamp(pitch, -1.0, 1.0) - pitch_acceleration + pitch_disturbance
    yaw_input = yaw_disturbance
    clamped_difference_altitude = clamp(target_altitude - altitude + k_vertical_offset, -1.0, 1.0)
    vertical_input = k_vertical_p * pow(clamped_difference_altitude, 3.0)

    # ACT
    front_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input
    front_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input
    rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input
    rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input
    
    front_left_motor.setVelocity(front_left_motor_input)
    front_right_motor.setVelocity(-front_right_motor_input)
    rear_left_motor.setVelocity(-rear_left_motor_input)
    rear_right_motor.setVelocity(rear_right_motor_input)
    
    pass
