"""my_controller_wall_follower controller."""

from controller import Robot
import time

def update_encoder_init_values(left_motor_encoder, right_motor_encoder):
    left_encoder_init = left_motor_encoder.getValue()
    right_encoder_init = right_motor_encoder.getValue()
    return left_encoder_init, right_encoder_init

def stop_robot(left_motor, right_motor):
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)

def check_aval(IR_values, traverse=[]):
    left_aval = False
    right_aval = False
    if IR_values[2] == 1000:
        left_aval = True
    if IR_values[1] == 1000:
        right_aval = True

    if left_aval and right_aval:
        print('left and right available')
        traverse.append("left")
        traverse.append("forward")
    elif left_aval:
        print('left available')
        traverse.append("left")
        traverse.append("forward")
    elif right_aval:
        print('right available')
        traverse.append("right")
        traverse.append("forward")
    else:
        traverse.append("stop")

    return traverse

def run_robot(robot):
    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    max_speed = 6.28
    max_speed = 3
    
    #enable motors
    left_motor = robot.getDevice('left wheel')
    right_motor = robot.getDevice('right wheel')
    
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)

    #left and right motor encoders
    left_motor_encoder = robot.getDevice('left wheel encoder')
    right_motor_encoder = robot.getDevice('right wheel encoder')
    left_motor_encoder.enable(timestep)
    right_motor_encoder.enable(timestep)

    #4 distance sensors IR_L1 L2 R1 R2
    IR_array = []
    IR_array.append(robot.getDevice('IR_R2'))
    IR_array.append(robot.getDevice('IR_R1'))
    IR_array.append(robot.getDevice('IR_L1'))
    IR_array.append(robot.getDevice('IR_L2'))
    for i in range(4):
        IR_array[i].enable(timestep)

    #gyro
    gyro = robot.getDevice('gyro')
    gyro.enable(timestep)

    #get intial encoder values
    # left_encoder_init = left_motor_encoder.getValue()
    # right_encoder_init = right_motor_encoder.getValue()

    # print('left_encoder_init: ' + str(left_encoder_init) + ' right_encoder_init: ' + str(right_encoder_init))

    # mode of operation
    mode = "right"
    traverse = ["forward"]
    index = 0
    command_count = 1

    speed_factor = 2
    P = 0.25

    cell_size = 100 #12.8
    turn_size = 25

    wall_triggerd = False

    damp_speed = 8
    damp_speed_rotation = 1

    encoder_init = False

    #set motor speeds to 0
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)

    # time.sleep(3)

    angle = 0
    integral_start = time.time()
        
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    ###########################################################################################
    ########################################LOOP###############################################
    ###########################################################################################
    while robot.step(timestep) != -1:
        # set mode of operation
        mode = traverse[index]

        # Read the IR sensors:
        IR_values = []
        for i in range(4):
            # 2 decimal points
            IR_values.append(round(IR_array[i].getValue(), 2))

        # if front distance sensors triggered next mode
        if IR_values[0] < 750 and IR_values[3] < 750:
            if not wall_triggerd:

                #check if wall is available
                traverse = check_aval(IR_values, traverse)

                wall_triggerd = True
                index += 1
                mode = traverse[index]

                #reset encoder values
                encoder_init = False
                command_count = 1
                print('wall triggerd mode: ' + mode)
        else:
            wall_triggerd = False

        # Read if the command array has the same command consecutively
        while True:
            if index + 1 < len(traverse) and traverse[index] == traverse[index + 1]:
                command_count += 1
                index += 1
            else:
                break

        # Read the gyro sensors:
        gyro_values = []
        for i in range(3):
            gyro_values.append(round(gyro.getValues()[i], 2))

        # print('gyro_values: ' + str(gyro_values))

        if not encoder_init:
            left_encoder_init, right_encoder_init = update_encoder_init_values(left_motor_encoder, right_motor_encoder)
            encoder_init = True

        # Read encoders with 2 decimal points
        left_encoder = round(left_motor_encoder.getValue(), 2) - left_encoder_init
        right_encoder = round(right_motor_encoder.getValue(), 2) - right_encoder_init
    
        # Process sensor data here.
        #ir value 500 is the 0 speed if decrease motor speed negative else positive linear velocity
        # right_speed = (IR_values[0] - 500)/10
        # left_speed = (IR_values[3] - 500)/10

        # stop when counted 10 rotations
        # if left_encoder > 13.6:
        #     left_speed = 0
        # else:
        #     left_speed = 3

        # if right_encoder > 13.6:
        #     right_speed = 0
        # else:
        #     right_speed = 3

        #integrate gyro reading to get the angle (yawn)
        if (time.time() - integral_start) > 0.05:
            angle += gyro_values[2]
            integral_start = time.time()
        
        # print('angle: ' + str(angle) + ' mode: ' + mode + " left encoder" + str(left_encoder) + " right encoder" + str(right_encoder))

        if mode == "right":
            # rotate 90 degrees right using gyro anglr > 25
            if angle > -turn_size * command_count:
                left_speed = (turn_size * command_count + angle) * P + damp_speed_rotation
                right_speed = -(turn_size * command_count + angle) * P - damp_speed_rotation
            else:
                left_speed = 0
                right_speed = 0
                angle = 0
                # time.sleep(1)
                index += 1
                #reset encoder values
                encoder_init = False
                command_count = 1
                        
        elif mode == "left":
            # rotate 90 degrees left
            if angle < turn_size * command_count:
                left_speed = -(turn_size * command_count - angle) * P - damp_speed_rotation
                right_speed = (turn_size * command_count - angle) * P + damp_speed_rotation
            else:
                left_speed = 0
                right_speed = 0
                angle = 0
                # time.sleep(1)
                index += 1
                #reset encoder values
                encoder_init = False
                command_count = 1

        elif mode == "stop":
            left_speed = 0
            right_speed = 0

        else:
            # Forward
            if left_encoder > cell_size * command_count:
                left_speed = 0
            else:
                left_speed = cell_size * command_count - left_encoder + damp_speed

            if right_encoder > cell_size * command_count:
                right_speed = 0
            else:
                right_speed = cell_size * command_count - right_encoder + damp_speed

            # if angle > 0:
            #     left_speed = left_speed + angle/3
            # else:
            #     right_speed = right_speed + angle/3*(-1)


            if right_encoder > cell_size * command_count and left_encoder > cell_size * command_count:

                left_speed = 0
                right_speed = 0
                angle = 0
                # time.sleep(1)
                index += 1

                #reset encoder values
                encoder_init = False
                command_count = 1

                #check if wall is available
                # check_aval(IR_values)

        # avoid hitting the walls
        if IR_values[1] < 300:
            print('right')
            left_speed = left_speed - 1
        if IR_values[2] < 300:
            print('left')
            right_speed = right_speed - 1
                
                
        # rotate 90 degrees left
        # while left_encoder < 4.46:
        #     left_speed = -3
        #     right_speed = 3

        #limit speeds to -10 and +10
        try:
            if right_speed > 10:
                right_speed = 10
            elif right_speed < -10:
                right_speed = -10
            if left_speed > 10:
                left_speed = 10
            elif left_speed < -10:
                left_speed = -10

            # speeds 2 decimal points
            left_speed = round(left_speed, 2)
            right_speed = round(right_speed, 2)

            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)

            # print('left_speed: ' + str(left_speed) + ' right_speed: ' + str(right_speed))
            # print('IR_values: ' + str(IR_values))
            # print ('left_encoder: ' + str(left_encoder) + ' right_encoder: ' + str(right_encoder))
            # print all these in same line
            # print('left_speed: ' + str(left_speed) + ' right_speed: ' + str(right_speed) + ' IR_values: ' + str(IR_values) + ' left_encoder: ' + str(left_encoder) + ' right_encoder: ' + str(right_encoder))

            #set speeds to 0
            left_speed = 0
            right_speed = 0

        except:
            pass

# Enter here exit cleanup code.

if __name__ == "__main__":

    # create the Robot instance.
    my_robot = Robot() 
    run_robot(my_robot)
