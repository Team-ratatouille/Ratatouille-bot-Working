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

    #4 distance sensors IR_L1 L2 R1 R2
    IR_array = []
    IR_array.append(robot.getDistanceSensor('IR_R2'))
    IR_array.append(robot.getDistanceSensor('IR_R1'))
    IR_array.append(robot.getDistanceSensor('IR_L1'))
    IR_array.append(robot.getDistanceSensor('IR_L2'))
    for i in range(4):
        IR_array[i].enable(timestep)
        
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        # Read the sensors:
        IR_values = []
        for i in range(4):
            IR_values.append(IR_array[i].getValue())
    
        # Process sensor data here.
        #ir value 500 is the 0 speed if decrease motor speed negative else positive linear velocity
        right_speed = (IR_values[0] - 500)/10
        left_speed = (IR_values[3] - 500)/10

        #limit speeds to -10 and +10
        if right_speed > 10:
            right_speed = 10
        elif right_speed < -10:
            right_speed = -10
        if left_speed > 10:
            left_speed = 10
        elif left_speed < -10:
            left_speed = -10

        # left_speed = max_speed
        # right_speed = max_speed

    
        # Enter here functions to send actuator commands, like:
        #forward if speeds are positive else reverse
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)


        print('left_speed: ' + str(left_speed) + ' right_speed: ' + str(right_speed))
        print('IR_values: ' + str(IR_values))
# Enter here exit cleanup code.

if __name__ == "__main__":

    # create the Robot instance.
    my_robot = Robot() 
    run_robot(my_robot)
