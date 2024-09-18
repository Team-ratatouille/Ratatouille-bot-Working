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
    left_motor = robot.getMotor('left wheel motor')
    right_motor = robot.getMotor('right wheel motor')
    
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)

    #enable proximity sensors
    proximity_sensors = []
    for i in range(8):
        sensor = robot.getDistanceSensor('ps' + str(i))
        sensor.enable(timestep)
        proximity_sensors.append(sensor)
    
    
        
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        # Read the sensors:
        for i in range(8):
            sensor_value = proximity_sensors[i].getValue()
            print('Sensor ' + str(i) + ' value is ' + str(sensor_value))
    
        # Process sensor data here.
        left_wall = proximity_sensors[5].getValue() > 80
        left_corner = proximity_sensors[6].getValue() > 80
        front_wall = proximity_sensors[7].getValue() > 80

        if front_wall:
            print('turn right')
            left_motor.setVelocity(max_speed)
            right_motor.setVelocity(-max_speed)
        else:
            if left_wall:
                print('drive forward')
                left_motor.setVelocity(max_speed)
                right_motor.setVelocity(max_speed)
            else:
                print('turn left')
                left_motor.setVelocity(max_speed/8)
                right_motor.setVelocity(max_speed)
            if left_corner:
                print('turn right')
                left_motor.setVelocity(max_speed)
                right_motor.setVelocity(max_speed/8)
    
        # Enter here functions to send actuator commands, like:
        # left_motor.setVelocity(max_speed)
        # right_motor.setVelocity(max_speed)

# Enter here exit cleanup code.

if __name__ == "__main__":

    # create the Robot instance.
    my_robot = Robot() 
    run_robot(my_robot)
