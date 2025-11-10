#include <main.h>

bool verboseConsole = false;
bool verboseSensors = false;
bool followLastCommand = false;
bool remoteControl = true;

void setup() {
  Serial1.begin(9600);
  if (!remoteControl) Serial1.println("Starting....");
  Serial.begin(9600);
  Serial.println("Starting....");
  pinMode(In1A, OUTPUT);
  pinMode(In2A, OUTPUT);
  pinMode(EnM1A, OUTPUT);
  pinMode(In3A, OUTPUT);
  pinMode(In4A, OUTPUT);
  pinMode(EnM2A, OUTPUT);
  pinMode(In1B, OUTPUT);
  pinMode(In2B, OUTPUT);
  pinMode(EnM3B, OUTPUT);
  pinMode(In3B, OUTPUT);
  pinMode(In4B, OUTPUT);
  pinMode(EnM4B, OUTPUT);
  pinMode(LEDPin, OUTPUT);

  // setup ToF sensors
  if (true){
    Wire.begin();
    for (int i=0; i<4; i++){
      pinMode(tofPins[i], OUTPUT);
      digitalWrite(tofPins[i], LOW); // Disable all sensors
    }
    for (int i=0; i<4; i++){
      if (!remoteControl) Serial1.println("Initializing sensor " + String(i));
      pinMode(tofPins[i], INPUT);
      delay(100); // Wait for sensor to boot
      sensors[i].setTimeout(500);
      sensors[i].init();
      sensors[i].setMeasurementTimingBudget(20000);
      sensors[i].startContinuous();
      sensors[i].setAddress(0x30 + i);
    }
  }
  
  additionalSetup();
}

void additionalSetup(){
  if (true) return;
  delay(1000);
  centering();
  digitalWrite(LEDPin, HIGH);
}

void loop(){
  // while(true){pingSensors(); delay(1000);};
  // while(true){parallelSensorAdjustment(1); delay(1000);};
  // manualControl();
  // obstacleAvoidance();
  controlFromSerial();
}

String command = "";
int start = 0;
void controlFromSerial()
{
  if (Serial1.available())
  {
    command = Serial1.readString();
    // Serial1.println();
    if (!remoteControl) Serial1.println("Received command: " + command);
    start = command.indexOf('[');
    if (start == -1)
    {
      Serial1.println("Invalid command format.");
      return;
    }
    val = command.charAt(start + 1);
    lastCommandTime = millis();
  }
  else if (millis() - lastCommandTime > commandTimeout || !followLastCommand)
  {
      val = 0; 
  }

  // Execute commands based on received value
  if (val == 'f')
  {
    obstacleAvoidance();
  }
  else if (val == 'p')
  {
    pingToF();
    transmitToFData();
  }
  else if (val == 'u')
  {
    pingSensors();
    transmitSensorData();
  }
  else if (val == 'h')
  {
    halt();
  }
  else if (val == 'c')
  {
    centering();
  }
  else if (val == 'r')
  {
    val = command.charAt(start + 2); // get direction value
    if (val < '0' || val > '3'){
      Serial1.println("Invalid direction for 'r' command.");
      return;
    }
    changeFrontDirection(int(val));
  }
  else if (val == 'l'){
    // blink LED
    for (int i=0; i<10; i++){
      digitalWrite(LEDPin, HIGH);
      delay(200);
      digitalWrite(LEDPin, LOW);
      delay(200);
    }
  }
  else if (val == 'm')
  {
    manualControl();
  }
  else if (val == '<')
  {
    MOVELEFTWHENPOSSIBLE = true;
    MOVERIGHTWHENPOSSIBLE = false;
  }
  else if (val == '>')
  {
    MOVERIGHTWHENPOSSIBLE = true;
    MOVELEFTWHENPOSSIBLE = false;
  }
  delay(10);
  if (val != 0 && val != 'p')
  {
    delay(100);
    Serial1.println("[+]");
  }
}

void manualControl() 
{
  while (true){
    if (Serial1.available())
    {
      val = Serial1.read();
      if (val == 'w')
      {
          // drive forward
          forward(8000);
          lastDriveCommand = millis();
      }
      else if(val == 's')
      {
          // drive backward
          backwards();
          lastDriveCommand = millis();
      }
      else if(val == 'd')
      {
          // drive right
          right();
          lastDriveCommand = millis();
      }
      else if(val == 'a')
      {
          // drive left
          left();
          lastDriveCommand = millis();
      }
      else if(val == 'i')
      {
        frontLeft();
      }
      else if(val == 'o')
      {
        frontRight();
      }
      else if(val == 'k')
      {
        backLeft();
      }
      else if(val == 'l')
      {
        backRight();
      }
      else if(val == 'q')
      {
          // rotate Counter Clockwise
          rotateCCW();
          lastDriveCommand = millis();
      }
      else if(val == 'e')
      {
          // rotate Clockwise
          rotateCW();
          lastDriveCommand = millis();
      }
      else if(val == 'H')
      {
          // break
          halt();
      }
      else if(val == 'z')
      {
          // shift into low speed
          shift=0;
      }
      else if(val == 'x')
      {
          // shift into medium speed
          shift=4;
      }
      else if(val == 'c')
      {
          // shift into high speed
          shift=8;
      }
      else if (val == 'u')
      {
          pingSensors();
      }
      else if (val == 't')
      {
          pingToF();
      }
      else if (val == 'f')
      {
          obstacleAvoidance();
      }
      else if (val == 'm')
      {
          // exit manual control
          halt();
          Serial1.println("[+]");
          return;
      }
    }
    // Check for drive timeout
    if (millis() - lastDriveCommand > driveTimeout)
    {
        halt();
    }
  }
}

void obstacleAvoidance()
{
    // pingSensors(); // distances are in inches
    if (verboseConsole) Serial1.println("Pinging ToF sensors...");
    pingToF(); // distances are in mm

    if (MOVELEFTWHENPOSSIBLE){
      // try to move left if possible
      if (tofDistances[3] > 150){
        if (verboseConsole) Serial1.println("Path clear on the left, carefully moving straight and then rotating.");
        halt();
        forward();
        delay(500);
        halt();
        if (OMNIWHEELDRIVE){
          changeFrontDirection(3);
        }
        else {
          rotateCCW();
          delay(900);
        }
        MOVELEFTWHENPOSSIBLE = false;
      }
    }
    if (MOVERIGHTWHENPOSSIBLE){
      // try to move right if possible
      if (tofDistances[1] > 150){
        if (verboseConsole) Serial1.println("Path clear on the right, carefully moving straight and then rotating.");
        halt();
        forward();
        delay(500);
        halt();
        if (OMNIWHEELDRIVE){
          changeFrontDirection(1);
        }
        else {
          rotateCW();
          delay(900);
        }
        MOVERIGHTWHENPOSSIBLE = false;
      }
    }

    if (lastToFDistances[1] < 200 && abs(lastToFDistances[1] - tofDistances[1]) > 100){
      if (verboseConsole) Serial1.println("Significant change in right sensor distance.");
      // moving forward to avoid wall collision if wanting to move right
      halt();
      forward();
      delay(400);
      halt();
    }
    else if (lastToFDistances[3] < 200 && abs(lastToFDistances[3] - tofDistances[3]) > 100){
      if (verboseConsole) Serial1.println("Significant change in left sensor distance.");
      // moving forward to avoid wall collision if wanting to move left
      halt();
      forward();
      delay(400);
      halt();
    }

    // Deal with bad readings
    if (tofDistances[0] == 0)
    {
      halt();
      if (verboseConsole) Serial1.println("Bad reading on front ToF sensor, stopping rover.");
      return;
    }

    // Check for obstacles in front of the rover
    if (tofDistances[0] < 60)
    {
        // Obstacle detected in front, stop and back up
        if (verboseConsole) Serial1.println("Obstacle detected in front, backing up and rotating.");
        halt();
        delay(500);
        backwards();
        delay(100);
        halt();
        delay(500);
        // Rotate to avoid obstacle
        // if (remoteControl){
        //   if (verboseConsole) Serial1.println("Remote control active, not changing front direction.");
        // }
        if (OMNIWHEELDRIVE){
          if (tofDistances[1] < 150){
            frontDirection = (frontDirection + 3) % 4;
          }
          else {
            frontDirection = (frontDirection + 1) % 4;
          }
        }
        else if (tofDistances[1] < 150)
        {
            // Obstacle on the right, rotate ACW
            if (verboseConsole) Serial1.println("Obstacle detected on the right, rotating counter-clockwise.");
            rotateCCW();
            delay(900);
            halt();
        }
        else
        {
            // No obstacle on the right, rotate CW
            if (verboseConsole) Serial1.println("No obstacle on the right, rotating clockwise.");
            rotateCW();
            delay(900);
            halt();
        }
    }
    
    // Avoiding right wall collisions
    else if (tofDistances[1] < 60 && tofDistances[1] != 0)
    {
        // Obstacle too close on the right, veer left
        if (verboseConsole) Serial1.println("Obstacle too close on the right sensor: " + String(tofDistances[1]) + "mm, veering left.");
        halt();
        delay(100);
        left(1.1);
        delay(300);
        halt();
        parallelSensorAdjustment(1);
    }
    // Avoiding left wall collisions
    else if (tofDistances[3] < 60 && tofDistances[3] != 0)
    {
        if (verboseConsole) Serial1.println("Obstacle too close on the left sensor: " + String(tofDistances[3]) + "mm, veering right.");
        halt();
        delay(100);
        right(1.1);
        delay(300);
        halt();
        parallelSensorAdjustment(3);
    }
    // Hugging Left Wall
    else if (tofDistances[3]>90 && tofDistances[3] < 150)
    {
        if (verboseConsole) Serial1.println("Too far from left sensor: " + String(tofDistances[3]) + "mm, veering left.");
        halt();
        delay(100);
        left(1.1);
        delay(400);
        halt();
        parallelSensorAdjustment(3);
        // sensorChangeRotation(6, 100);
    }
    // Hugging Right Wall (only if no left wall detected)
    else if (tofDistances[1]>90 && tofDistances[1] < 150 && tofDistances[3] > 150)
    {
        if (verboseConsole) Serial1.println("Too far from right sensor: " + String(tofDistances[1]) + "mm, veering right.");
        halt();
        delay(100);
        right(1.1);
        delay(400);
        halt();
        parallelSensorAdjustment(1);
        // sensorChangeRotation(2, 100);
    }


    // Moving forward after adjustments
    if (verboseConsole) Serial1.println("Moving forward.");
    float speedDivisor = (tofDistances[0] < 100) ? 1.4 : 1.0; 
    forward(speedDivisor);
    delay(500);
    halt();
    delay(100);

}

// Assuming:
//         Forward
//       (M1     M2)
// Left               Right
//       (M3     M4)
//        Backwards

// Assuming:
// In1:HIGH and In2:LOW -> CCW

void forward(float speedDivisor = 1.0)
{
  switch (frontDirection)
  {
  case 0: // front
    frontLeft(speedDivisor);
    break;

  case 1: // right
    frontRight(speedDivisor);
    break;

  case 2: // back
    backRight(speedDivisor);
    break;

  case 3: // left
    backLeft(speedDivisor);
    break;

  default:
    break;
  }
  // frontLeft(sensorReading);

  // digitalWrite(In1A, HIGH); 
  // digitalWrite(In2A, LOW); 
  // analogWrite(EnM1A, speeds[0+shift]);

  // digitalWrite(In3A, LOW); 
  // digitalWrite(In4A, HIGH); 
  // analogWrite(EnM2A, speeds[1+shift]);

  // digitalWrite(In1B, HIGH); 
  // digitalWrite(In2B, LOW); 
  // analogWrite(EnM3B, speeds[2+shift]);

  // digitalWrite(In3B, LOW); 
  // digitalWrite(In4B, HIGH); 
  // analogWrite(EnM4B, speeds[3+shift]);

}
void backwards(float speedDivisor = 1.0)
{
  switch (frontDirection)
  {
  case 0: // front
    backRight(speedDivisor);
    break;
  case 1: // right
    backLeft(speedDivisor);
    break;
  case 2: // back
    frontLeft(speedDivisor);
    break;
  case 3: // left
    frontRight(speedDivisor);
    break;
  default:
    break;
  }
  // backRight();

  // digitalWrite(In1A, LOW); 
  // digitalWrite(In2A, HIGH); 
  // analogWrite(EnM1A, speeds[0+shift]);

  // digitalWrite(In3A, HIGH); 
  // digitalWrite(In4A, LOW); 
  // analogWrite(EnM2A, speeds[1+shift]);

  // digitalWrite(In1B, LOW); 
  // digitalWrite(In2B, HIGH); 
  // analogWrite(EnM3B, speeds[2+shift]);

  // digitalWrite(In3B, HIGH); 
  // digitalWrite(In4B, LOW); 
  // analogWrite(EnM4B, speeds[3+shift]);
}
void left(float speedDivisor = 1.0)
{
  switch (frontDirection)
  {
  case 0: // front
    backLeft(speedDivisor);
    break;
  case 1: // right
    frontLeft(speedDivisor);
    break;
  case 2: // back
    frontRight(speedDivisor);
    break;
  case 3: // left
    backRight(speedDivisor);
    break;
  default:
    break;
  }
  // backLeft();

  // digitalWrite(In1A, LOW); 
  // digitalWrite(In2A, HIGH); 
  // analogWrite(EnM1A, speeds[0+shift]);

  // digitalWrite(In3A, LOW); 
  // digitalWrite(In4A, HIGH); 
  // analogWrite(EnM2A, speeds[1+shift]);

  // digitalWrite(In1B, HIGH); 
  // digitalWrite(In2B, LOW); 
  // analogWrite(EnM3B, speeds[2+shift]);

  // digitalWrite(In3B, HIGH); 
  // digitalWrite(In4B, LOW); 
  // analogWrite(EnM4B, speeds[3+shift]);
}
void right(float speedDivisor = 1.0)
{
  switch (frontDirection)
  {
  case 0: // front
    frontRight(speedDivisor);
    break;
  case 1: // right
    backRight(speedDivisor);
    break;
  case 2: // back
    backLeft(speedDivisor);
    break;
  case 3: // left
    frontLeft(speedDivisor);
    break;
  default:
    break;
  }
  // frontRight();

  // digitalWrite(In1A, HIGH); 
  // digitalWrite(In2A, LOW); 
  // analogWrite(EnM1A, speeds[0+shift]);

  // digitalWrite(In3A, HIGH); 
  // digitalWrite(In4A, LOW); 
  // analogWrite(EnM2A, speeds[1+shift]);

  // digitalWrite(In1B, LOW); 
  // digitalWrite(In2B, HIGH); 
  // analogWrite(EnM3B, speeds[2+shift]);

  // digitalWrite(In3B, LOW); 
  // digitalWrite(In4B, HIGH); 
  // analogWrite(EnM4B, speeds[3+shift]);
}
void frontRight(float speedDivisor = 1.0)
{
  digitalWrite(In1A, HIGH); 
  digitalWrite(In2A, LOW); 
  analogWrite(EnM1A, speeds[0+shift]/speedDivisor);

  digitalWrite(In3A, LOW); 
  digitalWrite(In4A, LOW); 
  analogWrite(EnM2A, 0);

  digitalWrite(In1B, LOW); 
  digitalWrite(In2B, LOW); 
  analogWrite(EnM3B, 0);

  digitalWrite(In3B, LOW); 
  digitalWrite(In4B, HIGH); 
  analogWrite(EnM4B, speeds[3+shift]/speedDivisor);
}
void frontLeft(float speedDivisor = 1.0)
{
  digitalWrite(In1A, LOW); 
  digitalWrite(In2A, LOW); 
  analogWrite(EnM1A, 0);

  digitalWrite(In3A, LOW); 
  digitalWrite(In4A, HIGH); 
  analogWrite(EnM2A, speeds[1+shift]/speedDivisor);

  digitalWrite(In1B, HIGH); 
  digitalWrite(In2B, LOW); 
  analogWrite(EnM3B, speeds[2+shift]/speedDivisor);

  digitalWrite(In3B, LOW); 
  digitalWrite(In4B, LOW); 
  analogWrite(EnM4B, 0);
}
void backRight(float speedDivisor = 1.0)
{
  digitalWrite(In1A, LOW); 
  digitalWrite(In2A, LOW); 
  analogWrite(EnM1A, 0);

  digitalWrite(In3A, HIGH); 
  digitalWrite(In4A, LOW); 
  analogWrite(EnM2A, speeds[1+shift]/speedDivisor);

  digitalWrite(In1B, LOW); 
  digitalWrite(In2B, HIGH); 
  analogWrite(EnM3B, speeds[2+shift]/speedDivisor);

  digitalWrite(In3B, LOW); 
  digitalWrite(In4B, LOW); 
  analogWrite(EnM4B, 0);
}
void backLeft(float speedDivisor = 1.0)
{
  digitalWrite(In1A, LOW); 
  digitalWrite(In2A, HIGH); 
  analogWrite(EnM1A, speeds[0+shift]/speedDivisor);

  digitalWrite(In3A, LOW); 
  digitalWrite(In4A, LOW); 
  analogWrite(EnM2A, 0);

  digitalWrite(In1B, LOW); 
  digitalWrite(In2B, LOW); 
  analogWrite(EnM3B, 0);

  digitalWrite(In3B, HIGH); 
  digitalWrite(In4B, LOW); 
  analogWrite(EnM4B, speeds[3+shift]/speedDivisor);
}
void rotateCW(float speedDivisor = 1.0)
{
  digitalWrite(In1A, HIGH); 
  digitalWrite(In2A, LOW); 
  analogWrite(EnM1A, speeds[0]/speedDivisor);

  digitalWrite(In3A, HIGH); 
  digitalWrite(In4A, LOW); 
  analogWrite(EnM2A, speeds[1]/speedDivisor);

  digitalWrite(In1B, HIGH); 
  digitalWrite(In2B, LOW); 
  analogWrite(EnM3B, speeds[2]/speedDivisor);

  digitalWrite(In3B, HIGH); 
  digitalWrite(In4B, LOW); 
  analogWrite(EnM4B, speeds[3]/speedDivisor);
}
void rotateCCW(float speedDivisor = 1.0)
{
  digitalWrite(In1A, LOW); 
  digitalWrite(In2A, HIGH); 
  analogWrite(EnM1A, speeds[0]/speedDivisor);

  digitalWrite(In3A, LOW); 
  digitalWrite(In4A, HIGH); 
  analogWrite(EnM2A, speeds[1]/speedDivisor);

  digitalWrite(In1B, LOW); 
  digitalWrite(In2B, HIGH); 
  analogWrite(EnM3B, speeds[2]/speedDivisor);

  digitalWrite(In3B, LOW); 
  digitalWrite(In4B, HIGH); 
  analogWrite(EnM4B, speeds[3]/speedDivisor);
}
void halt()
{
  digitalWrite(In1A, HIGH); 
  digitalWrite(In2A, HIGH); 
  analogWrite(EnM1A, 255);

  digitalWrite(In3A, HIGH); 
  digitalWrite(In4A, HIGH); 
  analogWrite(EnM2A, 255);

  digitalWrite(In1B, HIGH); 
  digitalWrite(In2B, HIGH); 
  analogWrite(EnM3B, 255);

  digitalWrite(In3B, HIGH); 
  digitalWrite(In4B, HIGH); 
  analogWrite(EnM4B, 255);
}

void saveLastReadings(){
  for (int i = 0 ; i < 8 ; i++){
    lastSensorDistances[i] = sensorDistances[i];
  }
}

void pingSensors()
{
    pingTimes[0] = sonar1.ping();
    pingTimes[1] = sonar2.ping();
    pingTimes[2] = sonar3.ping();
    pingTimes[3] = sonar4.ping();
    pingTimes[4] = sonar5.ping();
    pingTimes[5] = sonar6.ping();
    pingTimes[6] = sonar7.ping();
    pingTimes[7] = sonar8.ping();

    saveLastReadings();
    sensorDistancesReal[0] = pingTimes[0] / 5.7; // convert to mm
    sensorDistancesReal[1] = pingTimes[1] / 5.7; // convert to mm
    sensorDistancesReal[2] = pingTimes[2] / 5.7; // convert to mm
    sensorDistancesReal[3] = pingTimes[3] / 5.7; // convert to mm
    sensorDistancesReal[4] = pingTimes[4] / 5.7; // convert to mm
    sensorDistancesReal[5] = pingTimes[5] / 5.7; // convert to mm
    sensorDistancesReal[6] = pingTimes[6] / 5.7; // convert to mm
    sensorDistancesReal[7] = pingTimes[7] / 5.7; // convert to mm

    for (int i = 0; i < 8; i++)
    {
        Serial.print(i); // Sensor number
        Serial.print(" - ");
        Serial.print(sensorDistances[i]);
        Serial.print(", ");
        delay(10); // delay of 10ms
    }
    Serial.println(".");
    for (int i = 0; i < 8; i++)
    {
        Serial.print(i); // Sensor number
        Serial.print(" - ");
        Serial.print(pingTimes[i]);
        Serial.print(", ");
        delay(10); // delay of 10ms
    }
    Serial.println(".");

    if (nicePrint)
    {
      Serial.print("            ");Serial.print(sensorDistances[0]);Serial.print("   ");Serial.print(sensorDistances[1]);Serial.println();
      Serial.print("    ");Serial.print(sensorDistances[7]);Serial.print("                  ");Serial.print(sensorDistances[2]);Serial.println();
      Serial.print("    ");Serial.print(sensorDistances[6]);Serial.print("                  ");Serial.print(sensorDistances[3]);Serial.println();
      Serial.print("            ");Serial.print(sensorDistances[5]);Serial.print("   ");Serial.print(sensorDistances[4]);Serial.println();
      Serial.println();
    }

    for (int i = 0 ; i < 8 ; i++){
      sensorDistances[i] = sensorDistancesReal[(i + frontDirection*2) % 8];
    }
}

void pingToF()
{
  saveLastReadings();
  for (int i=0; i<4; i++){
    tofDistancesReal[i] = sensors[i].readRangeContinuousMillimeters();
    if (sensors[i].timeoutOccurred()) {
      Serial1.print(" TIMEOUT");
      tofDistancesReal[i] = 8000;
    }
    if (verboseSensors) {
      Serial1.print(i);
      Serial1.print(": ");
      Serial1.print(tofDistancesReal[i]);
      Serial1.print(" mm, ");
    }
  }
  if (verboseSensors) Serial1.println();

  for (int i = 0 ; i < 4 ; i++){
    lastToFDistances[i] = tofDistances[i];
    tofDistances[i] = tofDistancesReal[(i + frontDirection) % 4];
  }
  if (OMNIWHEELDRIVE && verboseSensors){
    Serial1.print("Front: ");
    Serial1.print(frontDirection);
    Serial1.print("Front Distance: ");
    Serial1.print(tofDistances[0]);
  }
}

void centering(){
  if (verboseConsole) Serial1.println("Centering rover...");
  int lastMeasure1 = 8000;
  int lastMeasure2 = 8000;
  int lastMeasure3 = 8000;
  for (int count = 0; count < 40; count++){
    halt();
    rotateCW(1.3);
    delay(200);
    halt();
    delay(200);
    pingToF();
    lastMeasure3 = lastMeasure2;
    lastMeasure2 = lastMeasure1;
    lastMeasure1 = tofDistances[1];
    if (verboseConsole) Serial1.println("M1: " + String(lastMeasure1) + " M2: " + String(lastMeasure2) + " M3: " + String(lastMeasure3));
    if (tofDistances[0] > 300 and tofDistances[1] < 150){
      if (lastMeasure1 > lastMeasure2 && lastMeasure3 > lastMeasure2){
        halt();
        rotateCCW(1.3);
        delay(200);
        halt();
        return;
      }
    }
  }
  // If centering fails, rotate until two corners are detected (so we are facing a corner)
  // rotate 45 degrees so we are facing a corridor
  if (verboseConsole) Serial1.println("Centering with corners...");
  while(true){
    halt();
    rotateCW();
    delay(150);
    halt();
    delay(200);
    pingToF();
    if (tofDistances[0] < 200 && tofDistances[1] < 200 && (tofDistances[0] - tofDistances[1]) < 50){
      halt();
      rotateCW();
      delay(500);
      halt();
      return;
    }
  }
}

void sensorChangeRotation(int sensorIndex, unsigned long delayTime=100){
  // using lastSensorDistances and sensorDistances
  // check if they are close to a wall and rotate based on change (in the delay)

  if (lastSensorDistances[sensorIndex] < 150 && sensorDistances[sensorIndex] < 150){

    int change = sensorDistances[sensorIndex] - lastSensorDistances[sensorIndex];
    if (verboseSensors) Serial1.println("Sensor " + String(sensorIndex) + " change: " + String(change));

    if ((change < 0 && sensorIndex==2) || (change > 0 && sensorIndex==6)){
      // getting closer to wall, rotate CCW
      if (verboseSensors) Serial1.println("Rotating CCW to avoid wall.");
      halt();
      rotateCCW();
      delay(delayTime);
      halt();
    }

    else if ((change > 0 && sensorIndex==2) || (change < 0 && sensorIndex==6)){
      // getting away from wall, rotate CW
      if (verboseSensors) Serial1.println("Rotating CW to avoid wall.");
      halt();
      rotateCW();
      delay(delayTime);
      halt();
    }
  }

}

void parallelSensorAdjustment(int direction){
  // direction: 0 = front, 1 = right, 2 = back, 3 = left
  // Ultrasonic sensor orientations
  //       [0][1]
  //   [7]        [2]
  //   [6]        [3]
  //       [5][4]
  int sensor1, sensor2;
  if (direction == 0){
    sensor1 = 7;
    sensor2 = 2;
  }
  else if (direction == 1){
    sensor1 = 1;
    sensor2 = 4;
  }
  else if (direction == 2){
    sensor1 = 3;
    sensor2 = 6;
  }
  else if (direction == 3){
    sensor1 = 5;
    sensor2 = 0;
  }
  int diff = sensorDistances[sensor1] - sensorDistances[sensor2];
  if (verboseConsole) Serial1.println("Parallel adjustment diff: " + String(diff));
  if (abs(diff) > 0 && abs(diff) < 100 && tofDistances[direction] < 100){
    unsigned long delayTime = abs(diff) * 5; // adjust delay time based on difference
    if (diff > 0){
      // sensor1 is farther than sensor2, rotate CW
      if (verboseConsole) Serial1.println("Rotating CW for parallel adjustment.");
      halt();
      rotateCW(1.2);
      delay(delayTime);
      halt();
    }
    else{
      // sensor2 is farther than sensor1, rotate CCW
      if (verboseConsole) Serial1.println("Rotating CCW for parallel adjustment.");
      halt();
      rotateCCW(1.2);
      delay(delayTime);
      halt();
    }
    pingSensors(); // update readings after adjustment
    delay(100);
    if (abs(sensorDistances[sensor1] - sensorDistances[sensor2]) > 20 &&
        abs(sensorDistances[sensor1] - sensorDistances[sensor2]) < 100){
      if (verboseConsole) Serial1.println("Further parallel adjustment needed.");
      parallelSensorAdjustment(direction); // recursive call if still not parallel
    }
  }
}

void changeFrontDirection(int newFront){
  frontDirection = newFront % 4;
  if (verboseConsole) Serial1.println("Front direction changed to: " + String(frontDirection));
  pingSensors();
  delay(100);
}

void transmitToFData(){
  // Transmit sensor distances over Serial1 in a comma-separated format
  String buffer = "[";
  for (int i = 0; i < 4; i++){
    buffer += String(tofDistancesReal[i]);
    buffer += ",";
  }
  buffer += String(frontDirection);
  buffer += "]";
  Serial1.println(buffer);
}

void transmitSensorData(){
  // Transmit sensor distances over Serial1 in a comma-separated format
  String buffer = "[";
  for (int i = 0; i < 8; i++){
    buffer += String(sensorDistancesReal[i]);
    buffer += ",";
  }
  buffer += String(frontDirection);
  buffer += "]";
  Serial1.println(buffer);
}