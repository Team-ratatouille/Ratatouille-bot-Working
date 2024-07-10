"""my_controller_wall_follower controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

def run_robot(robot):
    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    max_speed = 6.28
    max_speed = 3
    
    #enable motors
    left_motor = robot.getMotor('left wheel')
    right_motor = robot.getMotor('right wheel')
    
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)

    #2 distance sensors ds_left and ds_right
    # ds_left = robot.getDistanceSensor('ds_left')
    # ds_left.enable(timestep)
    # ds_right = robot.getDistanceSensor('ds_right')
    # ds_right.enable(timestep)
        
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        # Read the sensors:
        # left_sensor_value = ds_left.getValue()
        # right_sensor_value = ds_right.getValue()
    
        # Process sensor data here.
        # left_wall = left_sensor_value > 80
        # right_wall = right_sensor_value > 80


        left_speed = max_speed
        right_speed = max_speed

    
        # Enter here functions to send actuator commands, like:
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

        print('left_speed: ' + str(left_speed) + ' right_speed: ' + str(right_speed))

# Enter here exit cleanup code.

if __name__ == "__main__":

    # create the Robot instance.
    my_robot = Robot() 
    run_robot(my_robot)
