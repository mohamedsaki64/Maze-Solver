#include <Arduino.h>
#include <limits.h>

// ------------------- Pin Definitions -------------------
// Motor A pins  - Left motor
const int motorIn1 = 7;     // IN1 pin (Motor A direction)
const int motorIn2 = 8;     // IN2 pin (Motor A direction)
const int motorEnableA = 9; // ENA pin (Motor A speed)

// Motor B pins - Right motor
const int motorIn3 = 10;    // IN3 pin (Motor B direction)
const int motorIn4 = 11;    // IN4 pin (Motor B direction)
const int motorEnableB = 5; // ENB pin (Motor B speed)

// Ultrasonic sensor pins
const int frontTrig = 3;  // Ultrasonic front trigger pin
const int frontEcho = 6;  // Ultrasonic front echo pin
const int leftTrig  = 4;  // Ultrasonic left trigger pin
const int leftEcho  = 2;  // Ultrasonic left echo pin
const int rightTrig = 13; // Ultrasonic right trigger pin
const int rightEcho = 12; // Ultrasonic right echo pin

// IR sensor pin
const int irSensorPin = A0; // For goal detection, if any

// ------------------- Variables for Distances -------------------
int frontDistance  = 0;
int leftDistance   = 0;
int rightDistance  = 0;

// ------------------- PID & Wall-Following -------------------
float Kp = 0.7;
float Ki = 0.0;
float Kd = 20;


float error = 0, lastError = 0, integralA = 0, integralB = 0, derivative = 0;
float desiredLDistance = 75;  // mm from left wall
float desiredRDistance = 97;  // mm from right wall
int baseSpeedA = 100;         // Motor A base speed
int baseSpeedB = 120;         // Motor B base speed
const int maxSpeed = 225;
const int minSpeed = 60;
const int errorThreshold = 3; // how close to target distance is "good"

// ------------------- Maze & DFS Variables -------------------
const int mazeSize = 9; 
int maze[mazeSize][mazeSize];               // Flood-fill cost values (unused here)
bool walls[mazeSize][mazeSize][4] = {false}; // [row][col][0:N,1:E,2:S,3:W]
bool visited[mazeSize][mazeSize] = {false};

int currentRow = 0; 
int currentCol = 0;
int currentDirection = 0;  // 0:N, 1:E, 2:S, 3:W

// For DFS path
const int MAX_PATH_LENGTH = 100;
int pathIndex = 0;
int path[MAX_PATH_LENGTH][2];

// Relative directions: 0=>North,1=>East,2=>South,3=>West
// This matches how we interpret "front, right, back, left."
int relativeDirections[4] = {0, 1, 2, 3}; 

// Goal detection
bool goalFound = false;
int goalRow = -1, goalCol = -1;

#define CELL_DISTANCE 80
#define SAFE_DISTANCE 50

// ------------------- Function Declarations -------------------
void setup();
void loop();
void readDistances();
int  getDistance(int trigPin, int echoPin);
void updateWalls();

void traverseMazeDFSIterative(int startRow, int startCol);
bool isGoalCell();

void moveTo(int newRow, int newCol);
void performWallFollowing();
void followLeftWall();
void followRightWall();
void followBothWalls();

void stopMotors();
void turnLeft();
void turnLeft2();
void turnLeft3();
void turnRight();
void turnRight2();
void turnRight3();
void turnAround();
void goStraight();
void diskbrke();
bool millisDelay(unsigned long duration, unsigned long &startTime);
void readfDistance();
void handleObstacle();
void updateRelativeDirections(int turn);

// ------------------- Setup -------------------
void setup() {
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);
  pinMode(motorEnableA, OUTPUT);

  pinMode(motorIn3, OUTPUT);
  pinMode(motorIn4, OUTPUT);
  pinMode(motorEnableB, OUTPUT);

  pinMode(frontTrig, OUTPUT);
  pinMode(frontEcho, INPUT);
  pinMode(leftTrig, OUTPUT);
  pinMode(leftEcho, INPUT);
  pinMode(rightTrig, OUTPUT);
  pinMode(rightEcho, INPUT);

  pinMode(irSensorPin, INPUT);

  Serial.begin(9600);

  // Motors forward by default
  // digitalWrite(motorIn1, HIGH);
  // digitalWrite(motorIn2, LOW);
  // digitalWrite(motorIn3, HIGH);
  // digitalWrite(motorIn4, LOW);

  // // Base motor speeds
  // analogWrite(motorEnableA, baseSpeedA);
  // analogWrite(motorEnableB, baseSpeedB);

  // Initialize visited or walls as needed
  for (int r = 0; r < mazeSize; r++) {
    for (int c = 0; c < mazeSize; c++) {
      visited[r][c] = false;
      for (int d = 0; d < 4; d++) {
        walls[r][c][d] = false;
      }
    }
  }

  
  updateWalls();
  delay(200);
}

// ------------------- Main Loop -------------------
void loop() {
  readDistances();
  // moveTo(1,0);
  // moveTo(1,1);
  // moveTo(2,1);
  // moveTo(2,0);
  // moveTo(3,0);
  // moveTo(3,1);
  // moveTo(3,2);
  // moveTo(3,3);
  // moveTo(2,3);
  // moveTo(2,4);
  // moveTo(3,4);
  // moveTo(4,4);
  // moveTo(4,3);

  // while (true) {
  //     stopMotors();
  //     delay(1000);
  //   }
  traverseMazeDFSIterative(0, 0);

  // If goal is found, stop
  if (goalFound) {
    digitalWrite(motorIn1, HIGH);
    digitalWrite(motorIn2, LOW);
    digitalWrite(motorIn3, HIGH);
    digitalWrite(motorIn4, LOW);

    for (int i = pathIndex - 2; i >= 0; i--) {
      int backR = path[i][0];
      int backC = path[i][1];
      moveTo(backR, backC);
    }

    // Freeze
    while (true) {
      stopMotors();
      delay(1000);
    }
  }
}


// void updateRelativeDirections(int turn) {
//     int temp[4];
//     for (int i = 0; i < 4; i++) {
//         temp[i] = (relativeDirections[i] + turn + 4) % 4; // Add turn to all directions
//     }
//     for (int i = 0; i < 4; i++) {
//         relativeDirections[i] = temp[i];
//     }

//     // Serial.print("Updated relative directions: ");
//     // for (int i = 0; i < 4; i++) {
//     //     Serial.print(relativeDirections[i]);
//     //     Serial.print(" ");
//     // }
//     // Serial.println();
// }


// ------------------- DFS Implementation -------------------
void traverseMazeDFSIterative(int startRow, int startCol) {
  pathIndex = 0;
  path[pathIndex][0] = startRow;
  path[pathIndex][1] = startCol;
  pathIndex++;
  visited[startRow][startCol] = true;
  int cellCount = 0; 
  const int rows = 9; // Replace 10 with the number of rows in your maze
  const int cols = 9; // Replace 10 with the number of columns in your maze


  while (pathIndex > 0) {
    int row = path[pathIndex - 1][0];
    int col = path[pathIndex - 1][1];

    updateWalls();


    // Check if IR sees the goal
    if (isGoalCell()) {
      goalRow = row;
      goalCol = col;
      goalFound = true;
      stopMotors();
      delay(100);
      return;
    }

    bool moved = false;
    int nextRow, nextCol;

    // front => 0 => North => (row - 1, col)
    if (!walls[row][col][relativeDirections[0]] && !visited[row + 1][col]) {
      nextRow = row + 1;
      nextCol = col;
      moveTo(nextRow, nextCol);
      moved = true;
    }
    // right => 1 => East => (row, col + 1)
    else if (!walls[row][col][relativeDirections[1]] && !visited[row][col + 1]) {
      nextRow = row;
      nextCol = col + 1;
      moveTo(nextRow, nextCol);
      moved = true;
    }
    // left => 3 => West => (row, col - 1)
    else if (!walls[row][col][relativeDirections[3]] && !visited[row][col - 1]) {
      nextRow = row;
      nextCol = col - 1;
      moveTo(nextRow, nextCol);
      moved = true;
    }

    // back => 2 => South => (row + 1, col)
    else if (!walls[row][col][relativeDirections[2]] && !visited[row - 1][col]) {
      nextRow = row - 1;
      nextCol = col;
      moveTo(nextRow, nextCol);
      moved = true;
    }
    
    
    if (moved) {
      visited[nextRow][nextCol] = true;
      path[pathIndex][0] = nextRow;
      path[pathIndex][1] = nextCol;
      pathIndex++;
    } else {
      // If no move is possible, backtrack
      pathIndex--;
      if (pathIndex > 0) {
        int backtrackRow = path[pathIndex - 1][0];
        int backtrackCol = path[pathIndex - 1][1];
        moveTo(backtrackRow, backtrackCol);
      }
    }
  }
}

// ------------------- Goal Check (IR Sensor) -------------------
bool isGoalCell() {
  int irValue = analogRead(irSensorPin);
  return (irValue < 650);
}


// ------------------- Update Walls Based on Sensors -------------------
void updateWalls() {
  stopMotors();
  delay(100);
  readDistances();

  // We assume 0=North, 1=East, 2=South, 3=West
  int frontDir = currentDirection;
  int rightDir = (currentDirection + 1) % 4;
  int leftDir  = (currentDirection + 3) % 4;

  // Tweak these to avoid over-blocking open corridors
  int frontThreshold = 100; 
  int sideThreshold  = 250; 

  // Mark the wall in front
  if (frontDistance < frontThreshold) {
    walls[currentRow][currentCol][frontDir] = true;
  }
  // Mark the wall on the right
  if (rightDistance < sideThreshold) {
    walls[currentRow][currentCol][rightDir] = true;
  }
  // Mark the wall on the left
  if (leftDistance < sideThreshold) {
    walls[currentRow][currentCol][leftDir] = true;
  }
}

// ------------------- Movement Logic -------------------
void moveTo(int newRow, int newCol) {
  // Determine absolute direction from (currentRow, currentCol) to (newRow, newCol)
  int targetDirection = -1;

  // 0 => North => row + 1
  if      (newRow > currentRow) targetDirection = 0; 
  // 2 => South => row - 1
  else if (newRow < currentRow) targetDirection = 2; 
  // 1 => East => col + 1
  else if (newCol > currentCol) targetDirection = 1; 
  // 3 => West => col - 1
  else if (newCol < currentCol) targetDirection = 3; 
  else return; // same cell?

  int relativeTarget = (targetDirection - currentDirection + 4) % 4;

  // Shorter move to avoid overshoot
  unsigned long stepDuration = 920; 

  if (relativeTarget == 0) {
    goStraight();
    navigate_without_oa(stepDuration);
  }
  else if (relativeTarget == 1) {
    if (frontDistance > 250){
      turnRight3();
      stopMotors();
      delay(300);
      goStraight2(); 
    }
    else{
      turnRight2();
      stopMotors();
      delay(300);
      goStraight(); 
    }

    // goStraight(); 
    // updateRelativeDirections(1);
    currentDirection = (currentDirection + 1) % 4;
    navigate_without_oa(stepDuration);
  }
  else if (relativeTarget == 2) {
    turnAround();
    stopMotors();
    delay(300);
    goStraight();
    // updateRelativeDirections(2);
    currentDirection = (currentDirection + 2) % 4;
    navigate_without_oa(stepDuration);
  }
  else if (relativeTarget == 3) {
    if (frontDistance > 250){
      turnLeft3();
    }
    else{
      turnLeft2();
    }
    stopMotors();
    delay(300);
    goStraight();
    // updateRelativeDirections(-1);
    currentDirection = (currentDirection + 3) % 4;
    navigate_without_oa(stepDuration);
  }

  currentRow = newRow;
  currentCol = newCol;

  stopMotors();
  delay(300);
  // updateWalls();
  // delay(100);
}

// ------------------- Navigate Without Obstacle Avoidance -------------------
bool millisDelay(unsigned long duration, unsigned long &startTime) {
  unsigned long currentMillis = millis();
  if (currentMillis - startTime >= duration) {
    startTime = currentMillis;
    return true;
  }
  return false;
}

void navigate_without_oa(unsigned long duration) {
  // Lower frontDistance threshold so it stops if close to a wall
  digitalWrite(motorIn1, HIGH);
  digitalWrite(motorIn2, LOW);
  digitalWrite(motorIn3, HIGH);
  digitalWrite(motorIn4, LOW);

  unsigned long startTime = millis();
  readfDistance();
 
  while (!millisDelay(duration, startTime) && frontDistance > 90) {
    readDistances();
    performWallFollowing();
    delay(15);
    }
  
  diskbrke();
}

// void navigate_without_oa(unsigned long duration) {
//   // 1) Turn both motors on (forward)
//   digitalWrite(motorIn1, HIGH);
//   digitalWrite(motorIn2, LOW);
//   digitalWrite(motorIn3, HIGH);
//   digitalWrite(motorIn4, LOW);

//   // 2) Start timing
//   unsigned long startTime = millis();

//   // 3) Keep going until:
//   //    A) 'duration' ms have passed (enough to move ~1 cell)
//   // OR B) frontDistance < some safety threshold
//   while (true) {
//     // read the front distance each iteration
//     readDistances();  // or readfDistance(), whichever updates frontDistance
//     if (frontDistance < 80) {
//       // Too close, stop early
//       break;
//     }
//     if (millisDelay(duration, startTime)) {
//       // Time is up (we traveled one cell's worth)
//       break;
//     }

//     // 4) Perform your wall following so you stay centered or aligned
//     performWallFollowing();

//     // 5) Small delay to avoid hammering sensors
//     delay(8);
//   }

// }
// ------------------- Wall-Following -------------------
void performWallFollowing() {
  bool leftWallDetected  = (leftDistance < 80);
  bool rightWallDetected = (rightDistance < 80);

  if (!leftWallDetected && !rightWallDetected) {
    // No walls => go straight
    digitalWrite(motorIn1, HIGH);
    digitalWrite(motorIn2, LOW);
    digitalWrite(motorIn3, HIGH);
    digitalWrite(motorIn4, LOW);
    analogWrite(motorEnableA, baseSpeedA);
    analogWrite(motorEnableB, baseSpeedB);
  }
  else if (leftWallDetected && !rightWallDetected) {
    followLeftWall();
  }
  else if (!leftWallDetected && rightWallDetected) {
    followRightWall();
  }
  else {
    // Both walls => pick one or blend
    followBothWalls();
    // followLeftWall();
  }
}

// ------------------- PD Control for Left Wall -------------------
void followLeftWall() {
  float leftError = (leftDistance - desiredLDistance);
  error = leftError*0.5;

  integralA += error;
  float correction = (Kp * error) + (Ki * integralA) + (Kd * (error - lastError));

  int motorSpeedA = baseSpeedA - correction;  
  int motorSpeedB = baseSpeedB + correction;  

  motorSpeedA = constrain(motorSpeedA, minSpeed, maxSpeed);
  motorSpeedB = constrain(motorSpeedB, minSpeed, maxSpeed);

  if (fabs(error) < errorThreshold) {
    integralA = 0;
    motorSpeedA = baseSpeedA;
    motorSpeedB = baseSpeedB;
  }

  analogWrite(motorEnableA, motorSpeedA);
  analogWrite(motorEnableB, motorSpeedB);
  lastError = error;
}

// ------------------- PD Control for Right Wall -------------------
void followRightWall() {
  float rightError = (desiredRDistance - rightDistance);
  error = rightError;
  integralA += error;
  float correction = (Kp * error) + (Ki * integralA) + (Kd * (error - lastError));

  int motorSpeedA = baseSpeedA - (correction);  
  int motorSpeedB = baseSpeedB + (correction);  

  motorSpeedA = constrain(motorSpeedA, minSpeed, maxSpeed);
  motorSpeedB = constrain(motorSpeedB, minSpeed, maxSpeed);

  if (fabs(error) < errorThreshold) {
    integralA = 0;
    motorSpeedA = baseSpeedA;
    motorSpeedB = baseSpeedB;
  }

  analogWrite(motorEnableA, motorSpeedA);
  analogWrite(motorEnableB, motorSpeedB);
  lastError = error;
}

// ------------------- PD Control for Both Walls -------------------
// void followBothWalls() {
//   float leftError  = leftDistance  - desiredLDistance;
//   float rightError = desiredRDistance - rightDistance;

//   error = rightError + (leftError * 0.5);

//   integralA += error;
//   float correction = (Kp * error) + (Ki * integralA) + (Kd * (error - lastError));

//   int motorSpeedA = baseSpeedA - correction;  
//   int motorSpeedB = baseSpeedB + correction;  

//   motorSpeedA = constrain(motorSpeedA, minSpeed, maxSpeed);
//   motorSpeedB = constrain(motorSpeedB, minSpeed, maxSpeed);

//   if (fabs(error) < errorThreshold) {
//     integralA = 0;
//     motorSpeedA = baseSpeedA;
//     motorSpeedB = baseSpeedB;
//   }

//   analogWrite(motorEnableA, motorSpeedA);
//   analogWrite(motorEnableB, motorSpeedB);
//   lastError = error;
// }

void followBothWalls() {
  // 1) Calculate how far off we are from each wall
  float leftErr  = leftDistance  - desiredLDistance;   // positive => too far from left wall
  float rightErr = rightDistance - desiredRDistance;   // positive => too far from right wall

  // 2) Combine them so that the robot tries to remain centered
  //    If centerError > 0 => drift to the right
  //    If centerError < 0 => drift to the left
  float centerError = leftErr - rightErr;

  // 3) Implement standard PID
  integralA += centerError;
  float derivative = (centerError - lastError);

  // 4) "correction" is how much we need to turn (positive => turn right, negative => turn left)
  float correction = (Kp * centerError) + (Ki * integralA) + (Kd * derivative);

  // 5) Compute new motor speeds (assuming A = left motor, B = right motor)
  int motorSpeedA = baseSpeedA - correction;  
  int motorSpeedB = baseSpeedB + correction;

  // 6) Constrain the speeds to avoid going below minSpeed or above maxSpeed
  motorSpeedA = constrain(motorSpeedA, minSpeed, maxSpeed);
  motorSpeedB = constrain(motorSpeedB, minSpeed, maxSpeed);

  // 7) If the absolute error is small, zero out the integral to reduce windup, and hold base speeds
  if (fabs(centerError) < errorThreshold) {
    integralA = 0;
    motorSpeedA = baseSpeedA;
    motorSpeedB = baseSpeedB;
  }

  // 8) Write to motors
  analogWrite(motorEnableA, motorSpeedA);
  analogWrite(motorEnableB, motorSpeedB);

  // 9) Store the current error as the last error
  lastError = centerError;
}

// ------------------- Basic Motions -------------------
void stopMotors() {
  analogWrite(motorEnableA, 0);
  analogWrite(motorEnableB, 0);
}

void goStraight() {
  digitalWrite(motorIn1, HIGH);
  digitalWrite(motorIn2, LOW);
  digitalWrite(motorIn3, HIGH);
  digitalWrite(motorIn4, LOW);
  analogWrite(motorEnableA, baseSpeedA);
  analogWrite(motorEnableB, baseSpeedB);
  delay(10);
}

void goStraight2() {
  digitalWrite(motorIn1, HIGH);
  digitalWrite(motorIn2, LOW);
  digitalWrite(motorIn3, HIGH);
  digitalWrite(motorIn4, LOW);
  analogWrite(motorEnableA, baseSpeedA);
  analogWrite(motorEnableB, baseSpeedB);
  delay(90);
}


void diskbrke() {
  digitalWrite(motorIn1, LOW);
  digitalWrite(motorIn2, HIGH);
  digitalWrite(motorIn3, LOW);
  digitalWrite(motorIn4, HIGH);
  analogWrite(motorEnableA, 80);
  analogWrite(motorEnableB, 90);
  delay(10);
}

void turnLeft() {
  digitalWrite(motorIn1, LOW);
  digitalWrite(motorIn2, HIGH);
  digitalWrite(motorIn3, HIGH);
  digitalWrite(motorIn4, LOW);
  analogWrite(motorEnableA, 115);
  analogWrite(motorEnableB, 120);
  delay(150);
  stopMotors();
  delay(100);
}

void turnLeft2() {
  // Turn left until front is clear > 220 mm
  digitalWrite(motorIn1, LOW);
  digitalWrite(motorIn2, HIGH);
  digitalWrite(motorIn3, HIGH);
  digitalWrite(motorIn4, LOW);
  while (frontDistance < 220) {
    readfDistance();
    analogWrite(motorEnableA, 115);
    analogWrite(motorEnableB, 120);
    delay(8);
  }
  stopMotors();
  delay(500);
  turnLeft();
}

void turnLeft3() {
  digitalWrite(motorIn1, LOW);
  digitalWrite(motorIn2, HIGH);
  digitalWrite(motorIn3, HIGH);
  digitalWrite(motorIn4, LOW);
  analogWrite(motorEnableA, 115);
  analogWrite(motorEnableB, 120);
  delay(320);
  stopMotors();
  delay(100);
}

void turnRight() {
  digitalWrite(motorIn1, HIGH);
  digitalWrite(motorIn2, LOW);
  digitalWrite(motorIn3, LOW);
  digitalWrite(motorIn4, HIGH);
  analogWrite(motorEnableA, 115);
  analogWrite(motorEnableB, 120);
  delay(145);
  stopMotors();
  delay(100);
}

void turnRight2() {
  // Turn right until front is clear > 220 mm
  digitalWrite(motorIn1, HIGH);
  digitalWrite(motorIn2, LOW);
  digitalWrite(motorIn3, LOW);
  digitalWrite(motorIn4, HIGH);
  while (frontDistance < 220) {
    readfDistance();
    analogWrite(motorEnableA, 115);
    analogWrite(motorEnableB, 120);
    delay(8);
  }
  stopMotors();
  delay(500);
  turnRight();
}

void turnRight3() {
  digitalWrite(motorIn1, HIGH);
  digitalWrite(motorIn2, LOW);
  digitalWrite(motorIn3, LOW);
  digitalWrite(motorIn4, HIGH);
  analogWrite(motorEnableA, 115);
  analogWrite(motorEnableB, 120);
  delay(420);
  stopMotors();
  delay(100);
}

void turnAround() {
  digitalWrite(motorIn1, HIGH);
  digitalWrite(motorIn2, LOW);
  digitalWrite(motorIn3, LOW);
  digitalWrite(motorIn4, HIGH);
  analogWrite(motorEnableA, 115);
  analogWrite(motorEnableB, 120);
  delay(700);
  stopMotors();
  delay(100);
}

// ------------------- Distance Reading -------------------
int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(8);
  digitalWrite(trigPin, LOW);
  unsigned long duration = pulseIn(echoPin, HIGH, 10000);
  // Convert to mm (speed of sound ~ 0.34 mm/µs)
  int dist = duration * 0.34 / 2;
  return dist;
}

void readfDistance() {
  frontDistance = getDistance(frontTrig, frontEcho);
  Serial.println(frontDistance);
  
}

void readDistances() {
  leftDistance   = getDistance(leftTrig, leftEcho);
  rightDistance  = getDistance(rightTrig, rightEcho);
  frontDistance  = getDistance(frontTrig, frontEcho);
}