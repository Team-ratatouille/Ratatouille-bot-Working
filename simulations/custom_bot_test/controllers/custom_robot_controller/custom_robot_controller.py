"""my_controller_wall_follower controller."""

from controller import Robot
import time

# 31x31 grid of -1 s
maze_discovered = [[-1 for i in range(31)] for j in range(31)]
location = [30, 0]
maze_discovered[location[0]][location[1]] = 0
heading = 0 # 0: North, 1: East, 2: South, 3: West

#save the grid in a txt file
def save_maze():
    global maze_discovered
    with open('maze_discovered.txt', 'w') as f:
        for row in maze_discovered:
            f.write(' '.join([str(elem) for elem in row]))
            f.write('\n')

def update_encoder_init_values(left_motor_encoder, right_motor_encoder):
    left_encoder_init = left_motor_encoder.getValue()
    right_encoder_init = right_motor_encoder.getValue()
    return left_encoder_init, right_encoder_init

def stop_robot(left_motor, right_motor):
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)

def check_aval(IR_values, traverse=[]):
    global maze_discovered
    global location
    global heading

    left_aval = False
    right_aval = False
    front_aval = False

    if IR_values[2] == 1000:
        left_aval = True
    if IR_values[1] == 1000:
        right_aval = True
    if IR_values[0] >= 800 and IR_values[3] >= 800:
        front_aval = True

    if left_aval:
        print('left available')
    if right_aval:
        print('right available')
    if front_aval:
        print('front available')

    
    if front_aval:
        traverse.append("forward")
        print('forward')
    elif left_aval:
        traverse.append("left")
        traverse.append("forward")
        print('left')
    elif right_aval:
        traverse.append("right")
        traverse.append("forward")
        print('right')

    #update the discovered maze with the findings
    if heading == 0:
        if left_aval:
            if location[1] - 2 >= 0:
                maze_discovered[location[0]][location[1] - 1] = 0
                maze_discovered[location[0]][location[1] - 2] = 0
        if right_aval:
            if location[1] + 2 <= 30:
                maze_discovered[location[0]][location[1] + 1] = 0
                maze_discovered[location[0]][location[1] + 2] = 0
        if front_aval:
            if location[0] - 2 >= 0:
                maze_discovered[location[0] - 1][location[1]] = 0
                maze_discovered[location[0] - 2][location[1]] = 0
    elif heading == 1:
        if left_aval:
            if location[0] - 2 >= 0:
                maze_discovered[location[0] - 1][location[1]] = 0
                maze_discovered[location[0] - 2][location[1]] = 0
        if right_aval:
            if location[0] + 2 <= 30:
                maze_discovered[location[0] + 1][location[1]] = 0
                maze_discovered[location[0] + 2][location[1]] = 0
        if front_aval:
            if location[1] + 2 <= 30:
                maze_discovered[location[0]][location[1] + 1] = 0
                maze_discovered[location[0]][location[1] + 2] = 0
    elif heading == 2:
        if left_aval:
            if location[1] + 2 <= 30:
                maze_discovered[location[0]][location[1] + 1] = 0
                maze_discovered[location[0]][location[1] + 2] = 0
        if right_aval:
            if location[1] - 2 >= 0:
                maze_discovered[location[0]][location[1] - 1] = 0
                maze_discovered[location[0]][location[1] - 2] = 0
        if front_aval:
            if location[0] + 2 <= 30:
                maze_discovered[location[0] + 1][location[1]] = 0
                maze_discovered[location[0] + 2][location[1]] = 0
    elif heading == 3:
        if left_aval:
            if location[0] + 2 <= 30:
                maze_discovered[location[0] + 1][location[1]] = 0
                maze_discovered[location[0] + 2][location[1]] = 0
        if right_aval:
            if location[0] - 2 >= 0:
                maze_discovered[location[0] - 1][location[1]] = 0
                maze_discovered[location[0] - 2][location[1]] = 0
        if front_aval:
            if location[1] - 2 >= 0:
                maze_discovered[location[0]][location[1] - 1] = 0
                maze_discovered[location[0]][location[1] - 2] = 0

    return traverse

def run_robot(robot):
    global maze_discovered
    global location
    global heading

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
    traverse = ["forward"]
    index = 0
    command_count = 1

    speed_factor = 2
    P = 0.25

    cell_size = 12.8
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
        try:
            mode = traverse[index]
        except:
            left_speed = 0
            right_speed = 0
            save_maze()
            print('Map saved')
            break

        # Read the IR sensors:
        IR_values = []
        for i in range(4):
            # 2 decimal points
            IR_values.append(round(IR_array[i].getValue(), 2))

        # if front distance sensors triggered next mode
        # if IR_values[0] < 750 and IR_values[3] < 750:
        #     if not wall_triggerd:

        #         #check if wall is available
        #         traverse = check_aval(IR_values, traverse)

        #         wall_triggerd = True
        #         index += 1
        #         mode = traverse[index]

        #         #reset encoder values
        #         encoder_init = False
        #         command_count = 1
        #         print('wall triggerd mode: ' + mode)
        # else:
        #     wall_triggerd = False

        # Read if the command array has the same command consecutively
        # while True:
        #     if index + 1 < len(traverse) and traverse[index] == traverse[index + 1]:
        #         command_count += 1
        #         index += 1
        #     else:
        #         break

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

                #update the heading
                heading = (heading + 1) % 4
                        
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

                #update the heading
                heading = (heading - 1) % 4

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

            if right_encoder > cell_size * command_count and left_encoder > cell_size * command_count:

                left_speed = 0
                right_speed = 0
                angle = 0
                index += 1

                #reset encoder values
                encoder_init = False
                command_count = 1

                #update the location with heading and forward move
                if heading == 0:
                    maze_discovered[location[0]-1][location[1]] = 0
                    location[0] -= 2
                elif heading == 1:
                    maze_discovered[location[0]][location[1]+1] = 0
                    location[1] += 2
                elif heading == 2:
                    maze_discovered[location[0]+1][location[1]] = 0
                    location[0] += 2
                elif heading == 3:
                    maze_discovered[location[0]][location[1]-1] = 0
                    location[1] -= 2

                #update the heading
                maze_discovered[location[0]][location[1]] = 0

                #check if wall is available
                traverse = check_aval(IR_values, traverse)

        # avoid hitting the walls
        # if IR_values[1] < 300:
        #     print('right')
        #     left_speed = left_speed - 1
        # if IR_values[2] < 300:
        #     print('left')
        #     right_speed = right_speed - 1

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
