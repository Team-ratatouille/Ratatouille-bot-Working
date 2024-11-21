#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Gyro.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>
#include <ctime>

using namespace webots;
using namespace std;

const int MAZE_SIZE = 31;
int maze_discovered[MAZE_SIZE][MAZE_SIZE];
int location[2] = {30, 0};
int heading = 0; // 0: North, 1: East, 2: South, 3: West

// Initialize the maze
void initMaze() {
  for (int i = 0; i < MAZE_SIZE; i++) {
    for (int j = 0; j < MAZE_SIZE; j++) {
      maze_discovered[i][j] = -1;
    }
  }
  maze_discovered[location[0]][location[1]] = 0;
}

// Save the grid to a file
void save_maze() {
  ofstream file("maze_discovered.txt");
  if (file.is_open()) {
    for (int i = 0; i < MAZE_SIZE; i++) {
      for (int j = 0; j < MAZE_SIZE; j++) {
        file << maze_discovered[i][j] << " ";
      }
      file << "\n";
    }
    file.close();
  }
}

pair<double, double> updateEncoderInitValues(Motor *leftMotorEncoder, Motor *rightMotorEncoder) {
  return {leftMotorEncoder->getPosition(), rightMotorEncoder->getPosition()};
}


void stop_robot(Motor *leftMotor, Motor *rightMotor) {
  leftMotor->setVelocity(0);
  rightMotor->setVelocity(0);
}


vector<string> check_aval(double IR_values[], vector<string> traverse) {

  bool left_aval = IR_values[2] == 1000;
  bool right_aval = IR_values[1] == 1000;
  bool front_aval = IR_values[0] >= 800 && IR_values[3] >= 800;

  if (left_aval) traverse.push_back("left");
  if (right_aval) traverse.push_back("right");
  if (front_aval) traverse.push_back("forward");


  switch (heading) {
    case 0: // North
      if (left_aval && location[1] - 2 >= 0) maze_discovered[location[0]][location[1] - 2] = 0;
      if (right_aval && location[1] + 2 <= MAZE_SIZE - 1) maze_discovered[location[0]][location[1] + 2] = 0;
      if (front_aval && location[0] - 2 >= 0) maze_discovered[location[0] - 2][location[1]] = 0;
      break;
   
  }

  return traverse;
}

void run_robot(Robot *robot) {
  initMaze();

  int timestep = (int)robot->getBasicTimeStep();
  double max_speed = 3.0;

  // Enable motors
  Motor *leftMotor = robot->getMotor("left wheel");
  Motor *rightMotor = robot->getMotor("right wheel");

  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);

  // Enable sensors
  Motor *leftEncoder = robot->getMotor("left wheel encoder");
  Motor *rightEncoder = robot->getMotor("right wheel encoder");
  leftEncoder->enable(timestep);
  rightEncoder->enable(timestep);

  DistanceSensor *IR_array[4];
  IR_array[0] = robot->getDistanceSensor("IR_R2");
  IR_array[1] = robot->getDistanceSensor("IR_R1");
  IR_array[2] = robot->getDistanceSensor("IR_L1");
  IR_array[3] = robot->getDistanceSensor("IR_L2");

  for (int i = 0; i < 4; i++) {
    IR_array[i]->enable(timestep);
  }

  Gyro *gyro = robot->getGyro("gyro");
  gyro->enable(timestep);

  // Variables initialization
  vector<string> traverse = {"forward"};
  int index = 0;
  int command_count = 1;

  double cell_size = 12.8;
  double turn_size = 25;
  double angle = 0;
  bool encoder_init = false;

  double left_encoder_init, right_encoder_init;
  double P = 0.25, damp_speed = 8, damp_speed_rotation = 1;

  clock_t integral_start = clock();

  // Main loop
  while (robot->step(timestep) != -1) {
    string mode = traverse[index];

    double IR_values[4];
    for (int i = 0; i < 4; i++) {
      IR_values[i] = round(IR_array[i]->getValue());
    }

    // Gyro readings and angle integration
    if (((clock() - integral_start) / CLOCKS_PER_SEC) > 0.05) {
      angle += gyro->getValues()[2];
      integral_start = clock();
    }

    if (!encoder_init) {
      tie(left_encoder_init, right_encoder_init) = updateEncoderInitValues(leftEncoder, rightEncoder);
      encoder_init = true;
    }


    // Traverse maze
    traverse = check_aval(IR_values, traverse);

    // Save maze when needed
    if (mode == "stop") {
      stop_robot(leftMotor, rightMotor);
      save_maze();
      break;
    }
  }
}

int main() {
  Robot *my_robot = new Robot();
  run_robot(my_robot);
  delete my_robot;
  return 0;
}
