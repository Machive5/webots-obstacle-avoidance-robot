  // File: Robot_Controller.cpp
  // Date:
  // Description:
  // Author: Abrar Rafi Dwianto
  // Modifications:
  
  // You may need to add webots include files such as
  // <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
  // and/or to add some other includes
  #include <webots/Robot.hpp>
  #include <webots/Motor.hpp>
  #include <webots/DistanceSensor.hpp>
  #include <math.h>
  // All the webots classes are defined in the "webots" namespace
  using namespace webots;
  
  // This is the main program of your controller.
  // It creates an instance of your Robot instance, launches its
  // function(s) and destroys it at the end of the execution.
  // Note that only one instance of Robot should be created in
  // a controller program.
  // The arguments of the main function can be specified by the
  // "controllerArgs" field of the Robot node
  
//!============================== don't delete this block =========================
  Motor *RightFrontMotor;
  Motor *RightBackMotor;
  Motor *LeftFrontMotor;
  Motor *LeftBackMotor;
  Motor *USCServo;
  DistanceSensor *USCSensor;
  
  void robot_move_forward(float speed){
    RightFrontMotor->setVelocity(speed);
    RightBackMotor->setVelocity(speed);
    LeftFrontMotor->setVelocity(speed);
    LeftBackMotor->setVelocity(speed);
  }
  
  void robot_rotate_clockwise(float speed){
    RightFrontMotor->setVelocity(-speed);
    RightBackMotor->setVelocity(-speed);
    LeftFrontMotor->setVelocity(speed);
    LeftBackMotor->setVelocity(speed);
  }
  
  void robot_rotate_counter_clockwise(float speed){
    RightFrontMotor->setVelocity(speed);
    RightBackMotor->setVelocity(speed);
    LeftFrontMotor->setVelocity(-speed);
    LeftBackMotor->setVelocity(-speed);
  }
  
  double get_distance(){
    return USCSensor->getValue();
  }
  
  void move_servo(float position){
    float rad = position*M_PI/180;
    USCServo->setPosition(rad);
  }
//!=================================================================================
  
  int main(int argc, char **argv) {
//!============================== don't delete this block ==========================
    // create the Robot instance.
    Robot *robot = new Robot();
    // get the time step of the current world.
    int timeStep = (int)robot->getBasicTimeStep();
    
    RightFrontMotor = robot->getMotor("RIGHT_FRONT_MOTOR");
    RightBackMotor = robot->getMotor("RIGHT_BACK_MOTOR");
    LeftFrontMotor = robot->getMotor("LEFT_FRONT_MOTOR");
    LeftBackMotor = robot->getMotor("LEFT_BACK_MOTOR");
    USCServo = robot->getMotor("USC_SERVO");
    
    USCSensor = robot->getDistanceSensor("USC_SENSOR");
    
    RightFrontMotor->setPosition(INFINITY);
    RightBackMotor->setPosition(INFINITY);
    LeftFrontMotor->setPosition(INFINITY);
    LeftBackMotor->setPosition(INFINITY);
    
    RightFrontMotor->setVelocity(0.0);
    RightBackMotor->setVelocity(0.0);
    LeftFrontMotor->setVelocity(0.0);
    LeftBackMotor->setVelocity(0.0);
  
    
    USCSensor->enable(timeStep);
 //!================================================================================
    //*your custom initiation here
    const double OBSTACLE_THRESHOLD = 20.0;
    const double FORWARD_SPEED = 5.0;
    const double TURN_SPEED = 3.0;
    const int SERVO_WAIT = 15;
    const int TURN_DURATION = 30;

    enum State {
      FORWARD,
      SCAN_RIGHT,
      SCAN_LEFT,
      DECIDE,
      TURN_RIGHT,
      TURN_LEFT
    };

    State state = FORWARD;

    double dist_front = 0.0;
    double dist_right = 0.0;
    double dist_left = 0.0;

    int counter = 0;
  
    //* Main loop: write your logic here 
    while (robot->step(timeStep) != -1) {
      switch (state) {
        case FORWARD:
          move_servo(0.0);
          robot_move_forward(FORWARD_SPEED);
          dist_front = get_distance();
          printf("FRONT : %.2f\n", dist_front);

          if (dist_front < OBSTACLE_THRESHOLD) {
            printf("Obstacle detected!\n");
            robot_move_forward(0.0);
            counter = 0;
            state = SCAN_RIGHT;
          }

          break;
        case SCAN_RIGHT:
          move_servo(-60.0);
          counter++;
          if (counter >= SERVO_WAIT) {
            dist_right = get_distance();
            printf("RIGHT : %.2f\n", dist_right);
            counter = 0;
            state = SCAN_LEFT;
          }
          break;
        case SCAN_LEFT:
          move_servo(60.0);
          counter++;
          if (counter >= SERVO_WAIT) {
            dist_left = get_distance();
            printf("LEFT : %.2f\n", dist_left);
            counter = 0;
            state = DECIDE;
          }
          break;
        case DECIDE:
          printf(
            "Decision -> Right: %.2f | Left: %.2f\n",
            dist_right,
            dist_left
          );
          move_servo(0.0);
          if (dist_right < dist_left) {
            printf("Turn LEFT\n");
            counter = 0;
            state = TURN_LEFT;
          } else {
            printf("Turn RIGHT\n");
            counter = 0;
            state = TURN_RIGHT;
          }
          break;
        case TURN_RIGHT:
          robot_rotate_clockwise(TURN_SPEED);
          counter++;
          if (counter >= TURN_DURATION) {
            robot_move_forward(0.0);
            counter = 0;
            state = FORWARD;
          }
          break;
        case TURN_LEFT:
          robot_rotate_counter_clockwise(TURN_SPEED);
          counter++;
          if (counter >= TURN_DURATION) {
            robot_move_forward(0.0);
            counter = 0;
            state = FORWARD;
          }
          break;
      }
    };
  
    delete robot;
    return 0;
  }
  