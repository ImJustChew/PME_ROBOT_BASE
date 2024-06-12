#include <ESP32Encoder.h>
#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>


// Define motor pins and encoder pins for four motors
#define IN1_1 12 // PWM_1 pin for motor 1
#define IN1_2 13 // PWM_2 pin for motor 1
#define DC1_1 23 // Encoder pin A for motor 1
#define DC1_2 22 // Encoder pin B for motor 1

#define IN2_1 32 // PWM_1 pin for motor 2
#define IN2_2 33 // PWM_2 pin for motor 2
#define DC2_1 21 // Encoder pin A for motor 2
#define DC2_2 19 // Encoder pin B for motor 2

#define IN3_1 25 // PWM_1 pin for motor 3
#define IN3_2 26 // PWM_2 pin for motor 3
#define DC3_1 18 // Encoder pin A for motor 3
#define DC3_2 5  // Encoder pin B for motor 3

#define IN4_1 14 // PWM_1 pin for motor 4
#define IN4_2 15 // PWM_2 pin for motor 4
#define DC4_1 4  // Encoder pin A for motor 4
#define DC4_2 2  // Encoder pin B for motor 4

// Define PWM channels
#define PWM_CHANNEL_1_1 0
#define PWM_CHANNEL_1_2 1
#define PWM_CHANNEL_2_1 2
#define PWM_CHANNEL_2_2 3
#define PWM_CHANNEL_3_1 4
#define PWM_CHANNEL_3_2 5
#define PWM_CHANNEL_4_1 6
#define PWM_CHANNEL_4_2 7


// Encoder objects
ESP32Encoder encoder1;
ESP32Encoder encoder2;
ESP32Encoder encoder3;
ESP32Encoder encoder4;

// Position PID control variables
int SetpointPos1, InputPos1, OutputPos1;
int SetpointPos2, InputPos2, OutputPos2;
int SetpointPos3, InputPos3, OutputPos3;
int SetpointPos4, InputPos4, OutputPos4;

// def state
enum MotorState {
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  MOVE_LEFT = 3,
  MOVE_RIGHT = 4,
  TURN_LEFT = 5,
  TURN_RIGHT = 6
};

// declare function
void controlMotor(int output, int motor, int pwmChannel1, int pwmChannel2);
void move_straight();
void move_right();
void move_left();
void turn_right();
void turn_left();

MotorState getStateFromCommand(const std_msgs::Int32& command);
void commandCallback(const std_msgs::Int32& msg);
void executeMotorAction(MotorState state);


// ROS NodeHandle and Publisher/Subscriber
ros::NodeHandle nh;
std_msgs::Int32 motorStateMsg;
ros::Publisher motorStatePub("info_back", &motorStateMsg);
ros::Subscriber<std_msgs::Int32> commandSub("information", commandCallback);

MotorState currentState = STOP;  // initial state stop
int OutputSpeed = 0;
int speed = 75;

void setup() {
  // Initialize encoders
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder1.attachSingleEdge(DC1_1, DC1_2);
  encoder2.attachSingleEdge(DC2_1, DC2_2);
  encoder3.attachSingleEdge(DC3_1, DC3_2);
  encoder4.attachSingleEdge(DC4_1, DC4_2);

  // Set motor pins as output
  pinMode(IN1_1, OUTPUT);
  pinMode(IN1_2, OUTPUT);
  pinMode(IN2_1, OUTPUT);
  pinMode(IN2_2, OUTPUT);
  pinMode(IN3_1, OUTPUT);
  pinMode(IN3_2, OUTPUT);
  pinMode(IN4_1, OUTPUT);
  pinMode(IN4_2, OUTPUT);

  // Initialize PWM channels
  ledcSetup(PWM_CHANNEL_1_1, 5000, 8); // 5 kHz frequency, 8-bit resolution
  ledcSetup(PWM_CHANNEL_1_2, 5000, 8);
  ledcSetup(PWM_CHANNEL_2_1, 5000, 8);
  ledcSetup(PWM_CHANNEL_2_2, 5000, 8);
  ledcSetup(PWM_CHANNEL_3_1, 5000, 8);
  ledcSetup(PWM_CHANNEL_3_2, 5000, 8);
  ledcSetup(PWM_CHANNEL_4_1, 5000, 8);
  ledcSetup(PWM_CHANNEL_4_2, 5000, 8);

  // Attach PWM channels to GPIO pins
  ledcAttachPin(IN1_1, PWM_CHANNEL_1_1);
  ledcAttachPin(IN1_2, PWM_CHANNEL_1_2);
  ledcAttachPin(IN2_1, PWM_CHANNEL_2_1);
  ledcAttachPin(IN2_2, PWM_CHANNEL_2_2);
  ledcAttachPin(IN3_1, PWM_CHANNEL_3_1);
  ledcAttachPin(IN3_2, PWM_CHANNEL_3_2);
  ledcAttachPin(IN4_1, PWM_CHANNEL_4_1);
  ledcAttachPin(IN4_2, PWM_CHANNEL_4_2);

  // Initialize ROS node
  nh.initNode();
  nh.advertise(motorStatePub);
  nh.subscribe(commandSub);

}

void loop() {
  // Read current positions
  InputPos1 = encoder1.getCount();
  InputPos2 = encoder2.getCount();
  InputPos3 = encoder3.getCount();
  InputPos4 = encoder4.getCount();

  executeMotorAction(currentState);
  // Handle ROS communication
  nh.spinOnce();
  delay(10);
}

void move_straight() {
  // move forward or backwards
  controlMotor(OutputSpeed, 1, PWM_CHANNEL_1_2, PWM_CHANNEL_1_1);
  controlMotor(OutputSpeed, 2, PWM_CHANNEL_2_1, PWM_CHANNEL_2_2);
  controlMotor(OutputSpeed, 3, PWM_CHANNEL_3_1, PWM_CHANNEL_3_2);
  controlMotor(OutputSpeed, 4, PWM_CHANNEL_4_1, PWM_CHANNEL_4_2);
  Serial.println("successfully move straight");
}

void move_right() {
  // move right
  controlMotor(OutputSpeed, 1, PWM_CHANNEL_1_2, PWM_CHANNEL_1_1);
  controlMotor(OutputSpeed, 2, PWM_CHANNEL_2_2, PWM_CHANNEL_2_1);
  controlMotor(OutputSpeed, 3, PWM_CHANNEL_3_2, PWM_CHANNEL_3_1);
  controlMotor(OutputSpeed, 4, PWM_CHANNEL_4_1, PWM_CHANNEL_4_2);
  Serial.println("successfully move right");
}

void move_left() {
  // move left
  controlMotor(OutputSpeed, 1, PWM_CHANNEL_1_1, PWM_CHANNEL_1_2);
  controlMotor(OutputSpeed, 2, PWM_CHANNEL_2_1, PWM_CHANNEL_2_2);
  controlMotor(OutputSpeed, 3, PWM_CHANNEL_3_1, PWM_CHANNEL_3_2);
  controlMotor(OutputSpeed, 4, PWM_CHANNEL_4_2, PWM_CHANNEL_4_1);
  Serial.println("successfully move left");
}

void turn_left() {
  // turn left
  controlMotor(OutputSpeed, 1, PWM_CHANNEL_1_2, PWM_CHANNEL_1_1);
  controlMotor(OutputSpeed, 2, PWM_CHANNEL_2_1, PWM_CHANNEL_2_2);
  controlMotor(OutputSpeed, 3, PWM_CHANNEL_3_2, PWM_CHANNEL_3_1);
  controlMotor(OutputSpeed, 4, PWM_CHANNEL_4_2, PWM_CHANNEL_4_1);
  Serial.println("successfully turn right");
}

void turn_right() {
  // turn right
  controlMotor(OutputSpeed, 1, PWM_CHANNEL_1_1, PWM_CHANNEL_1_2);
  controlMotor(OutputSpeed, 2, PWM_CHANNEL_2_2, PWM_CHANNEL_2_1);
  controlMotor(OutputSpeed, 3, PWM_CHANNEL_3_1, PWM_CHANNEL_3_2);
  controlMotor(OutputSpeed, 4, PWM_CHANNEL_4_1, PWM_CHANNEL_4_2);
  Serial.println("successfully turn right");
}


const int motorAdj[4] = {1, 1, 1, 1};
void controlMotor(int output, int motor, int pwmChannel1, int pwmChannel2) {
  int actualOutput = output * motorAdj[motor - 1];
  if (output > 0) {
    ledcWrite(pwmChannel1, (int)actualOutput);
    ledcWrite(pwmChannel2, 0);
  } else {
    ledcWrite(pwmChannel1, 0);
    ledcWrite(pwmChannel2, (int)-actualOutput);
  }
}

// Callback function when recieved command
void commandCallback(const std_msgs::Int32& msg) {
  MotorState newMotorState = getStateFromCommand(msg);
  if (newMotorState != currentState) {
    currentState = newMotorState;  //update motor state
  }
  motorStateMsg.data = (currentState);
  motorStatePub.publish(&motorStateMsg);
}

// transfer command into new state
MotorState getStateFromCommand(const std_msgs::Int32& command) {
  if (command.data == 9) {
    return STOP;
  } 
  else if (command.data == -1) {
    return FORWARD;
  } 
  else if (command.data == -2) {
    return BACKWARD;
  } 
  else if (command.data == -3) {
    return MOVE_LEFT;
  } 
  else if (command.data == -4) {
    return MOVE_RIGHT;
  } 
  else if (command.data == -5) {
    return TURN_LEFT;
  } 
  else if (command.data == -6) {
    return TURN_RIGHT;
  } 
  else {
    return STOP; // other situations
  }
}

// activate motor from state
void executeMotorAction(MotorState state) {
  switch (state) {
    case STOP:
      OutputSpeed = 0;
      move_straight();
      break;
    case FORWARD:
      OutputSpeed = speed;
      move_straight();
      break;
    case BACKWARD:
      OutputSpeed = -speed;
      move_straight();
      break;
    case MOVE_LEFT:
      OutputSpeed = speed;
      move_left();
      break;
    case MOVE_RIGHT:
      OutputSpeed = speed;
      move_right();
      break;
    case TURN_LEFT:
      OutputSpeed = speed;
      turn_left();
      break;
    case TURN_RIGHT:
      OutputSpeed = speed;
      turn_right();
      break;
    default:
      break;
  }
}