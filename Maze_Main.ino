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
int index = 0;       // Record the robot's movement steps
bool back = false;   // Indicates whether the robot needs to backtrack

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
  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

// Function to move forward
void moveForward() {
  // Turn on motors to move forward
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, 252);
  analogWrite(enB, 255);
}

// Function to turn right
void turnRight() {
  // Turn on motors to turn right
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH); // Left motor forward
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH); // Right motor backward
  analogWrite(enA, 255);
  analogWrite(enB, 200);
}

// Function to turn left
void turnLeft() {
  // Turn on motors to turn left
  digitalWrite(in1, HIGH); // Left motor backward
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); // Right motor forward
  digitalWrite(in4, LOW);
  analogWrite(enA, 200);
  analogWrite(enB, 255);
}

// Function to stop the robot
void STOP() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void loop() {
  // Main code runs repeatedly
  int sensorValueLeft = analogRead(IR_left);
  int sensorValueMiddle = analogRead(IR_middle);
  int sensorValueRight = analogRead(IR_right);
  Serial.println(1);
  Serial.println(sensorValueLeft);
  delay(100);

  if (back) {
    // Backtracking logic (simplified)
    // For simplicity, stop the robot at this point
    STOP();
  }
  else if (sensorValueLeft > 100) {
    // Left sensor detects line
    path[index++] = LEFT; // Record the movement
    moveForward();
    delay(400);
    turnLeft();
    delay(200);
    starttime = millis();
    while (true) {
      int sensorValueMiddle = analogRead(IR_middle);
      Serial.println("turnleft");
      if (sensorValueMiddle > 100) {
        turnRight();
        delay(30);
        starttime = millis();
        break;
      }
    }

  } else if (sensorValueMiddle > 100) {
    // Middle sensor detects line
    path[index++] = FORWARD; // Record the movement
    moveForward();
    starttime = millis();

  } else if (sensorValueRight > 100) {
    // Right sensor detects line
    path[index++] = RIGHT; // Record the movement
    moveForward();
    delay(400);
    turnRight();
    delay(200);
    starttime = millis();
    while (true) {
      int sensorValueMiddle = analogRead(IR_middle);
      Serial.println("turnright");
      if (sensorValueMiddle > 100) {
        turnLeft();
        delay(40);
        starttime = millis();
        break;
      }
    }
  } else {
    // All sensors are low, possibly at a dead end
    Serial.println("Dead end detected");
    back = true;               // Set the backtracking flag
    path[index++] = BACK;      // Record the movement
    STOP();
    // Implement backtracking logic here if needed
  }
}
