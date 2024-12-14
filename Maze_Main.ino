// Definition of IR sensor pins
#define IR_left A0
#define IR_middle A1
#define IR_right A2
// Motor A connections
int enA = 9;
int in1 = 8;
int in2 = 7;
// Motor B connections
int enB = 3;
int in3 = 5;
int in4 = 4;
unsigned long starttime = 0;

// Direction constants
#define FORWARD 1
#define LEFT 2
#define RIGHT 3
#define BACK 4

int path[100];       // Record the robot's movement path
int index = 0;       // Record how many steps the robot has moved
bool back = false;   // Indicates whether the robot needs to backtrack

// -------------------- PID-related variables --------------------
float Kp = 0.5;   // Proportional coefficient
float Ki = 0.2;   // Integral coefficient
float Kd = 0.8;   // Derivative coefficient 

float error = 0.0;
float previous_error = 0.0;
float integral = 0.0;
float derivative = 0.0;

// Base speed setting
int baseSpeed = 200; // Base forward speed, adjust as needed

// Variables to store final PWM values
int leftMotorPWM = 200;
int rightMotorPWM = 200;
// ---------------------------------------------------------------------

void setup() {
  // Communication initialization
  Serial.begin(9600);
  Serial.println(5);

  // Sensor initialization
  pinMode(IR_left, INPUT);
  pinMode(IR_middle, INPUT);
  pinMode(IR_right, INPUT);

  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Turn off motors - initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

// Function to move forward
void moveForward(int leftPWM, int rightPWM) {
  // Move both motors forward
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, leftPWM);
  analogWrite(enB, rightPWM);
}

// Function to turn right
void turnRight(int leftPWM, int rightPWM) {
  // Make the robot turn right
  // Left motor forward, right motor backward
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, leftPWM);
  analogWrite(enB, rightPWM);
}

// Function to turn left
void turnLeft(int leftPWM, int rightPWM) {
  // Make the robot turn left
  // Left motor backward, right motor forward
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, leftPWM);
  analogWrite(enB, rightPWM);
}

// Function to stop the robot
void STOP() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void loop() {
  // Read sensor values
  int sensorValueLeft = analogRead(IR_left);
  int sensorValueMiddle = analogRead(IR_middle);
  int sensorValueRight = analogRead(IR_right);
  Serial.println(1);
  Serial.println(sensorValueLeft);
  delay(100);
  
  // -------------------- Simple PID control logic --------------------
  // error = left sensor - right sensor
  // If error > 0, left side stronger, need to turn right slightly
  // If error < 0, right side stronger, need to turn left slightly
  error = (float)sensorValueLeft - (float)sensorValueRight;
  integral += error;                     
  derivative = error - previous_error;
  float correction = Kp * error + Ki * integral + Kd * derivative;
  previous_error = error;

  // Adjust left/right motor PWM based on correction
  leftMotorPWM = baseSpeed + (int)correction;
  rightMotorPWM = baseSpeed - (int)correction;

  // Constrain PWM to 0-255
  leftMotorPWM = constrain(leftMotorPWM, 0, 255);
  rightMotorPWM = constrain(rightMotorPWM, 0, 255);
  // -----------------------------------------------------------------

  if (back) {
    // Backtracking logic (simplified)
    STOP();
  } else if (sensorValueLeft > 100) {
    // Left sensor detects line
    path[index++] = LEFT; // Record movement
    moveForward(leftMotorPWM, rightMotorPWM);
    delay(400);
    turnLeft(leftMotorPWM, rightMotorPWM);
    delay(200);
    starttime = millis();
    while (true) {
      int sensorValueMiddle = analogRead(IR_middle);
      Serial.println("turnleft");
      if (sensorValueMiddle > 100) {
        turnRight(leftMotorPWM, rightMotorPWM);
        delay(30);
        starttime = millis();
        break;
      }
    }

  } else if (sensorValueMiddle > 100) {
    // Middle sensor detects line
    path[index++] = FORWARD; // Record movement
    moveForward(leftMotorPWM, rightMotorPWM);
    starttime = millis();

  } else if (sensorValueRight > 100) {
    // Right sensor detects line
    path[index++] = RIGHT; // Record movement
    moveForward(leftMotorPWM, rightMotorPWM);
    delay(400);
    turnRight(leftMotorPWM, rightMotorPWM);
    delay(200);
    starttime = millis();
    while (true) {
      int sensorValueMiddle = analogRead(IR_middle);
      Serial.println("turnright");
      if (sensorValueMiddle > 100) {
        turnLeft(leftMotorPWM, rightMotorPWM);
        delay(40);
        starttime = millis();
        break;
      }
    }
  } else {
    // No sensor detects the line, possibly a dead end
    Serial.println("Dead end detected");
    back = true;               
    path[index++] = BACK;      
    STOP();
  }
}
