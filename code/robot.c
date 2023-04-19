#include <Servo.h>

#include <QTRSensors.h>

#include <Stepper.h>

#include <robot474.h>

#define MIDPOINT 300
#define WWW 0
#define BBB 1
#define WBW 2
#define WBB 3
#define BBW 4
#define WWB 5
#define BWW 6
#define BWB 7
#define INCREMENT 1
#define SPEED 20
#define DELAY_TIME 200

#define WAIT 0
#define START 1
#define NEW 2
#define UNKNOWN 3
#define INTERSECTION 4

#define MANUAL 0
#define AUTOMATIC 1

#define EXPLORE 0
#define RESCUE 1

#define LEFT 48
#define RIGHT 49
#define FORWARD 50
#define REVERSE 51

#define LASTLINE 0
#define LASTINTERSECTION 1

#define REPORTINGINTERVAL 10000

Stepper stepper1(200, 5, 4);
Stepper stepper2(200, 7, 6);
Servo myservo;

QTRSensorsRC qtrrc((unsigned char[]) {
  8,
  9,
  10
}, 3);
unsigned int sensor_value[3];

int analogPinCurrent = 2;
int analogPinVoltage = 5;

int saveReadSensors = 0;
int ramp = 0;

unsigned char state = WAIT;
unsigned char mode = 0;
unsigned char phase = 0;

int last = 0;
boolean correct = false;

int timerCount = 0;

int stepCount = 0;
unsigned int totalStepCount = 0;

int correctionAttempts = 0;

unsigned long startT = 0;
unsigned long stopT = 0;

unsigned long stopWatchStartT = 0;
unsigned long stopWatchStopT = 0;

//------------------------------
//Map Variables
//------------------------------

MapNode nodeList[24];
unsigned char nodeTotal = 0;
MapNode * shortestPath[16];
unsigned char shortestPathTotal = 0;
MapNode * currentNode;
MapNode * endNode;

int Map[8][8];
Direction currentDirection = North;
Direction initialDirection = North;
unsigned char currentX = 0;
unsigned char currentY = 0;
unsigned char maxX = 8;
unsigned char maxY = 8;
boolean loopReverse = false;

void setup() {
  Serial.begin(9600);

  //ATmega on Arduino UNO is 16 MHz. Divide the clock by 1024 to get 6.4e-5s time intervals.
  //There are 15625 of these intervals in 1 second.
  //Source: http://www.engblaze.com/microcontroller-tutorial-avr-and-arduino-timer-interrupts/

  cli(); // disable global interrupts
  TCCR2A = 0; // set entire TCCR1A register to 0
  TCCR2B = 0; // same for TCCR1B
  // set compare match register to desired timer count:
  OCR2A = 250;
  // turn on CTC mode:
  TCCR2A |= (1 << WGM21);
  // Set CS10 and CS12 bits for 1024 prescaler:
  TCCR2B |= (1 << CS20);
  TCCR2B |= (1 << CS21);
  //TCCR2B |= (1 << CS22);
  // enable timer compare interrupt:
  TIMSK2 |= (1 << OCIE2A);
  // enable global interrupts:
  sei();

  stepper1.setSpeed(SPEED);
  stepper2.setSpeed(SPEED);
  myservo.attach(11); // attaches the servo on pin 9 to the servo object
  myservo.write(55);

  Serial.flush();
  delay(5000);
  Serial.println("Start Calibration");

  for (int i = 0; i < 308; i++) {
    stepper1.step(INCREMENT);
    stepper2.step(-INCREMENT);
    qtrrc.calibrate();
  }
  Serial.println();
  Serial.println("Done calibration");

  //Initialize Map
  currentNode = AddNode(0, 0, NULL);
  Map[0][0] = 5;
}

void loop() {
  saveReadSensors = ReadSensors();

  switch (state) {
  case WAIT:
    currentX = 0;
    currentY = 0;
    while (true) {
      char data[16];
      SerialReadLine(data);

      if (strcmp(data, "start") == 0) {
        state = START;
        currentDirection = initialDirection;
        if (currentDirection == North) Map[1][0] = 1;
        else Map[0][1] = 1;
        Serial.flush();
        totalStepCount = 0;
        ResetSpeed();
        StartTimer();
        break;
      } else if (strcmp(data, "explore") == 0) {
        phase = EXPLORE;
        state = START;
        currentDirection = initialDirection;
        if (currentDirection == North) Map[1][0] = 1;
        else Map[0][1] = 1;
        mode = AUTOMATIC;
        Serial.flush();
        totalStepCount = 0;
        ResetSpeed();
        StartTimer();
        break;
      } else if (strcmp(data, "rescue") == 0) {
        phase = RESCUE;
        state = START;
        mode = AUTOMATIC;
        currentDirection = initialDirection;
        shortestPathTotal--;
        Serial.flush();
        totalStepCount = 0;
        ResetSpeed();
        StartTimer();
        break;
      } else if (strcmp(data, "map") == 0) {
        LoadMap();
        LoadFromMap();
        DjikstrasAlgorithm();
        SendShortestPath();
      } else if (strcmp(data, "sd n") == 0) initialDirection = North;
      else if (strcmp(data, "sd e") == 0) initialDirection = East;
      else if (strcmp(data, "sm m") == 0) mode = MANUAL;
      else if (strcmp(data, "sm a") == 0) mode = AUTOMATIC;
    }
    break;

  case START:
    if (saveReadSensors == WWW) {
      Serial.println("New");
      state = NEW;
    } else GoForward(INCREMENT);
    break;

  case NEW:
    if (saveReadSensors == WBW) {
      GoForward(2);
      if (ReadSensors() == WBW) {
        switch (currentDirection) {
        case North:
          currentY++;
          break;
        case South:
          currentY--;
          break;
        case West:
          currentX--;
          break;
        case East:
          currentX++;
          break;
        }

        if (currentX == 0 && currentY == 0) {
          GoNumberSteps(180);
          StopTimer();
          Serial.print("Distance: ");
          Serial.println(totalStepCount * .09424778);
          if (phase == EXPLORE) {
            DjikstrasAlgorithm();
            SendShortestPath();
          }
          SendRobotLocation();
          state = WAIT;
        } else if (currentX == maxX - 1 && currentY == maxY - 1) {
          Serial.println("End");
          ResetSpeed();
          delay(DELAY_TIME);
          GoNumberSteps(-80);
          ResetSpeed();
          delay(DELAY_TIME);
          RotateCW(180);
          if (phase == RESCUE) {
            delay(DELAY_TIME);
            GoNumberSteps(-95);
            delay(DELAY_TIME);
            for (int pos = 55; pos < 115; pos += 1) {
              myservo.write(pos);
              delay(5);
            }
            ResetSpeed();
            delay(DELAY_TIME);
            GoNumberSteps(20);
          } else {
            UpdateMap();
            SendMap();
            SendRobotLocation();
            delay(DELAY_TIME);
            GoNumberSteps(-75);
            delay(DELAY_TIME);
            ResetSpeed();
          }
          currentDirection = (Direction)((currentDirection + 2) % 4);
          state = NEW;
        } else {
          Serial.println("Unknown");
          stepCount = 0;
          stepper1.step(1);
          state = UNKNOWN;
        }
      }
    } else if (saveReadSensors == BBB || saveReadSensors == WBB || saveReadSensors == BBW || saveReadSensors == BWB) {
      switch (currentDirection) {
      case North:
        if (currentY < maxY - 1) Map[currentX][currentY + 1] = 1;
        break;
      case South:
        if (currentY > 0) Map[currentX][currentY - 1] = 1;
        break;
      case West:
        if (currentX > 0) Map[currentX - 1][currentY] = 1;
        break;
      case East:
        if (currentX < maxX - 1) Map[currentX + 1][currentY] = 1;
        break;
      }

      ResetSpeed();
      delay(DELAY_TIME);
      GoNumberSteps(-154);
      if (last == LASTLINE) {
        Serial.println("Last tile was a line");
        ResetSpeed();
        delay(DELAY_TIME);
        RotateCW(180);
        FillMap();
        currentDirection = (Direction)((currentDirection + 2) % 4);
        stepCount = 219;
      } else if (last == LASTINTERSECTION) {
        Serial.println("Last tile was an intersection");
        if (!GetNextInstruction()) PrintError("Error 1");
      }
      //correct = false;
      state = INTERSECTION;
    } else GoForward(INCREMENT);
    break;

  case UNKNOWN:
    if (saveReadSensors == WWW && stepCount >= 276) {
      Serial.println("New");
      state = NEW;
      last = LASTLINE;
      //if (correct)
      stepper1.step(2);
      //correct = !correct;
    } else if (saveReadSensors == BBW || saveReadSensors == WBB) {
      ResetSpeed();
      if (correctionAttempts < 2) {
        delay(DELAY_TIME);
        Correction(saveReadSensors);
        if (correctionAttempts < 1) {
          Serial.println("Backing up");
          delay(DELAY_TIME);
          GoNumberSteps(-13);
          ResetSpeed();
        }

        delay(DELAY_TIME);
        correctionAttempts++;
      }
    } else if (saveReadSensors == BBB || saveReadSensors == BWB) {
      correctionAttempts = 0;
      GoNumberSteps(71);
      //correct = false;
      Serial.println("Intersection");
      if (GetNextInstruction()) state = INTERSECTION;
    } else GoForward(INCREMENT);
    break;

  case INTERSECTION:
    if (saveReadSensors == WWW && stepCount >= 276) {
      state = NEW;
      Serial.println("New");
      last = LASTINTERSECTION;
    } else GoForward(INCREMENT);
    break;
  }
}

void PrintError(char msg[]) {
  Serial.print(msg);
  Serial.print(" State ");
  Serial.print(state);
  Serial.print(" ");
  PrintSensors(saveReadSensors);
  delay(DELAY_TIME * 5);
}

boolean GetNextInstruction() {
  Direction prevDirection = currentDirection;
  unsigned char command;
  if (phase == EXPLORE) {
    UpdateMap();
    SendMap();
    SendRobotLocation();

    if (mode == AUTOMATIC) NextDirection();
    else {
      char data[16];
      while (Serial.available() == 0) delay(20);

      SerialReadLine(data);
      if (strcmp(data, "north") == 0) currentDirection = North;
      else if (strcmp(data, "east") == 0) currentDirection = East;
      else if (strcmp(data, "west") == 0) currentDirection = West;
      else if (strcmp(data, "south") == 0) currentDirection = South;
      Serial.flush();
    }
  } else if (phase == RESCUE) {
    NextDirectionRescue();
    SendRobotLocation();
  }
  if (currentDirection == North) Serial.println("North");
  else if (currentDirection == East) Serial.println("East");
  else if (currentDirection == South) Serial.println("South");
  else if (currentDirection == West) Serial.println("West");
  command = GetRotation(prevDirection);

  ResetSpeed();
  if (command == LEFT) {
    delay(DELAY_TIME);
    RotateCCW(90);
    delay(DELAY_TIME);
    GoNumberSteps(70);
    stepCount = 230;
  } else if (command == FORWARD) {
    stepCount = 219;
  } else if (command == RIGHT) {
    delay(DELAY_TIME);
    RotateCW(90);
    delay(DELAY_TIME);
    GoNumberSteps(70);
    stepCount = 230;
  } else if (command == REVERSE) {
    delay(DELAY_TIME);
    GoNumberSteps(-50);
    ResetSpeed();
    delay(DELAY_TIME);
    RotateCW(180);
    stepCount = 219;
  } else return false;
  return true;
}

unsigned char GetRotation(Direction prevDirection) {
  int diff = ((int) prevDirection - (int) currentDirection + 4) % 4;
  unsigned char r = 0;
  if (diff == 0) r = FORWARD;
  else if (diff == 1) r = LEFT;
  else if (diff == 2) r = REVERSE;
  else if (diff == 3) r = RIGHT;
  return r;
}

void Correction(int sensors) {
  sensors = ReadSensors();
  PrintSensors(sensors);
  if (sensors == WBW) {
    correctionAttempts = 0;
    return;
  }
  Serial.println("Correcting");
  int i = 0;

  while (sensors != BBB && sensors != BWB) {
    if (sensors == BBW || sensors == BWW) stepper2.step(INCREMENT);
    else if (sensors == WBB || sensors == WWB) stepper1.step(INCREMENT);
    else if (sensors == WBW || sensors == WWW) {
      correctionAttempts = 0;
      break;
    }
    sensors = ReadSensors();
    if (i++ % 10 == 0) PrintSensors(sensors);
  }
}

void RotateCCW(int degrees) {
  for (int i = 0; i < (154 * degrees) / (90); i++) {
    stepper1.step(-INCREMENT);
    stepper2.step(INCREMENT);
  }
}

void RotateCW(int degrees) {
  for (int i = 0; i < (154 * degrees) / (90); i++) {
    stepper1.step(INCREMENT);
    stepper2.step(-INCREMENT);
  }
}

void GoNumberSteps(int steps) {
  int i = 0;

  if (steps >= 0)
    for (; i < steps / INCREMENT; i++)
      GoForward(INCREMENT);
  else
    for (; i < -steps / INCREMENT; i++)
      GoBackward(INCREMENT);
}

int GoForward(int steps) {
  stepper1.step(steps);
  stepper2.step(steps);

  if (totalStepCount % 10 == 0 && ramp < 10) {
    ramp++;
    stepper1.setSpeed(SPEED + ramp);
    stepper2.setSpeed(SPEED + ramp);
  }

  stepCount += steps;
  totalStepCount = totalStepCount + steps;
}

int GoBackward(int steps) {
  stepper1.step(-steps);
  stepper2.step(-steps);

  if (totalStepCount % 10 == 0 && ramp < 10) {
    ramp++;
    stepper1.setSpeed(SPEED + ramp);
    stepper2.setSpeed(SPEED + ramp);
  }

  totalStepCount = totalStepCount + steps;
}

void ResetSpeed() {
  stepper1.setSpeed(SPEED);
  stepper2.setSpeed(SPEED);
  ramp = 0;
}

int ReadSensors() {
  qtrrc.readCalibrated(sensor_value);

  if ((sensor_value[0] <= MIDPOINT) && (sensor_value[1] <= MIDPOINT) && (sensor_value[2] <= MIDPOINT)) return WWW;
  else if ((sensor_value[0] > MIDPOINT) && (sensor_value[1] > MIDPOINT) && (sensor_value[2] > MIDPOINT)) return BBB;
  else if ((sensor_value[0] <= MIDPOINT) && (sensor_value[1] > MIDPOINT) && (sensor_value[2] <= MIDPOINT)) return WBW;
  else if ((sensor_value[0] > MIDPOINT) && (sensor_value[1] > MIDPOINT) && (sensor_value[2] <= MIDPOINT)) return BBW;
  else if ((sensor_value[0] <= MIDPOINT) && (sensor_value[1] > MIDPOINT) && (sensor_value[2] > MIDPOINT)) return WBB;
  else if ((sensor_value[0] > MIDPOINT) && (sensor_value[1] <= MIDPOINT) && (sensor_value[2] <= MIDPOINT)) return BWW;
  else if ((sensor_value[0] <= MIDPOINT) && (sensor_value[1] <= MIDPOINT) && (sensor_value[2] > MIDPOINT)) return WWB;
  else return BWB;
}

void PrintSensors(int sensors) {
  switch (sensors) {
  case WWW:
    Serial.println("WWW");
    break;
  case WBW:
    Serial.println("WBW");
    break;
  case BBB:
    Serial.println("BBB");
    break;
  case WBB:
    Serial.println("WBB");
    break;
  case BBW:
    Serial.println("BBW");
    break;
  case WWB:
    Serial.println("WWB");
    break;
  case BWW:
    Serial.println("BWW");
    break;
  case BWB:
    Serial.println("BWB");
    break;
  }
}

void SerialReadLine(char incoming[]) {
  int i = 0;
  while (i < 68) {
    while (Serial.available() == 0) delay(20);
    char c = Serial.read();
    if (c == '\n') break;
    incoming[i] = c;
    i++;
  }
  incoming[i] = NULL;
}

int StartTimer() {
  startT = millis();
  return 0;
}

int StopTimer() {
  stopT = millis();
  Serial.print("Time: ");
  Serial.println(stopT - startT);
  return 0;
}

int ResetStopWatch() {
  stopWatchStartT = millis();
  return 0;
}

int CheckStopWatch() {
  stopWatchStopT = millis();

  if ((stopWatchStopT - stopWatchStartT) >= REPORTINGINTERVAL) {
    ResetStopWatch();
    return 1;
  } else return 0;
}

ISR(TIMER2_COMPA_vect) {
  if (timerCount < 20000) timerCount++;

  if (timerCount == 20000 && state != WAIT) {
    float current = 0;
    float voltage = 0;
    current = analogRead(analogPinCurrent);
    current = current - 10;
    Serial.print("Current: ");
    Serial.print(((current * 5 / 1024) / 0.180) / 10);
    Serial.print(" ");
    Serial.print("Voltage: ");
    voltage = analogRead(analogPinVoltage);
    voltage = voltage * 5 * 2 / 1024;
    Serial.println(voltage);
    timerCount = 0;
  }
}

void SendMap() {
  Serial.print("map ");
  for (int j = 0; j < 8; j++)
    for (int i = 0; i < 8; i++)
      Serial.print(Map[i][j]);
  Serial.println();
}

void LoadMap() {
  char data[68];
  SerialReadLine(data);
  for (int j = 0, k = 0; j < 8; j++)
    for (int i = 0; i < 8; i++, k++)
      Map[i][j] = data[k] - 48;
}

void SendRobotLocation() {
  Serial.print("loc ");
  Serial.print(currentX);
  Serial.println(currentY);
}

void SendShortestPath() {
  Serial.print("path ");
  for (int i = shortestPathTotal - 1; i >= 0; i--) {
    MapNode * n = shortestPath[i];
    Serial.print(( * n).x);
    Serial.print(( * n).y);
  }
  Serial.println();
}

void LoadFromMap() {
  nodeTotal = 0;
  MapNode * lastNode = NULL;
  //Add Nodes and Vertical Edges
  for (int i = 0; i < 8; i++) {
    for (int j = 0; j < 8; j++) {
      if (Map[i][j] >= 4) {
        currentNode = AddNode(i, j, NULL);
        if (lastNode != NULL) AddEdge(lastNode, ( * currentNode).y - ( * lastNode).y);
        lastNode = currentNode;
        if (Map[i][j] == 6) endNode = currentNode;
      } else if (Map[i][j] <= 1) lastNode = NULL;
    }
    lastNode = NULL;
  }

  //Add Horizontal Edges
  for (int j = 0; j < 8; j++) {
    for (int i = 0; i < 8; i++) {
      if (Map[i][j] >= 4) {
        currentNode = FindNode(i, j);
        if (lastNode != NULL) AddEdge(lastNode, ( * currentNode).x - ( * lastNode).x);
        lastNode = currentNode;
      } else if (Map[i][j] <= 1) lastNode = NULL;
    }
    lastNode = NULL;
  }
  maxX = ( * endNode).x + 1;
  maxY = ( * endNode).y + 1;
}

//------------------------
// Exploration
//------------------------

MapNode * AddNode(int x, int y, MapNode * prevNode) {
  nodeList[nodeTotal].x = x;
  nodeList[nodeTotal].y = y;
  nodeList[nodeTotal].PrevNode = prevNode;
  nodeList[nodeTotal].Distance = 255;
  nodeList[nodeTotal].EdgeTotal = 0;
  nodeList[nodeTotal].Visited = false;
  for (int i = 0; i < 4; i++)
    nodeList[nodeTotal].Adjacent[i] = NULL;
  nodeTotal++;
  return & nodeList[nodeTotal - 1];
}

void AddEdge(MapNode * Node1, int Length) {
  ( * Node1).Adjacent[( * Node1).EdgeTotal] = currentNode;
  ( * Node1).Length[( * Node1).EdgeTotal] = Length;
  ( * Node1).EdgeTotal++;

  ( * currentNode).Adjacent[( * currentNode).EdgeTotal] = Node1;
  ( * currentNode).Length[( * currentNode).EdgeTotal] = Length;
  ( * currentNode).EdgeTotal++;
}

MapNode * FindNode(int x, int y) {
  int i = 0;
  for (; i < nodeTotal; i++)
    if (nodeList[i].x == x && nodeList[i].y == y) break;

  return & nodeList[i];
}

boolean EdgeExist(MapNode * n1, MapNode * n2) {
  int i = 0;
  for (; i < ( * n1).EdgeTotal; i++)
    if (( * n1).Adjacent[i] == n2) break;

  return (i < ( * n1).EdgeTotal);
}

Direction DirectionOfPrevNode(MapNode * node) {
  Direction d;
  MapNode * prevNode = ( * node).PrevNode;
  int diffX = ( * node).x - ( * prevNode).x;
  int diffY = ( * node).y - ( * prevNode).y;

  if (diffY == 0) {
    if (diffX < 0) d = East;
    else d = West;
  } else {
    if (diffY < 0) d = North;
    else d = South;
  }
  return d;
}

boolean UnexploredPathExists(MapNode * destination) {
  MapNode * trace = currentNode;
  while (trace != destination) {
    int x = ( * trace).x;
    int y = ( * trace).y;

    if (x > 0 && Map[x - 1][y] == 0) return true;
    if (x < maxX - 1 && Map[x + 1][y] == 0) return true;
    if (y > 0 && Map[x][y - 1] == 0) return true;
    if (y < maxY - 1 && Map[x][y + 1] == 0) return true;
    trace = ( * trace).PrevNode;
  }
  return false;
}

void UpdateMap() {
  int distance = abs(( * currentNode).x - currentX) + abs(( * currentNode).y - currentY);

  if (distance != 0) {
    boolean existingNode = false;
    if (Map[currentX][currentY] == 0) {
      //update Nodes
      MapNode * n = AddNode(currentX, currentY, currentNode);

      if (currentX == maxX - 1 && currentY == maxY - 1) {
        Map[currentX][currentY] = 6;
        if (currentDirection == North) Map[currentX - 1][currentY] = 1;
        else Map[currentX][currentY - 1] = 1;
        if (currentX < 7) Map[currentX + 1][currentY] = 1;
        if (currentY < 7) Map[currentX][currentY + 1] = 1;
        endNode = n;
      } else Map[currentX][currentY] = 4;
    } else existingNode = true;

    //update Map
    switch (currentDirection) {
    case (North):
      for (int i = distance - 1; i > 0; i--) {
        Map[currentX][currentY - i] = 2;
        if (currentX > 0 && Map[currentX - 1][currentY - distance] != 4 &&
          Map[currentX - 1][currentY - distance] != 0)
          Map[currentX - 1][currentY - i] = 1;
        if (currentX < 7 && Map[currentX + 1][currentY - distance] != 4 &&
          Map[currentX + 1][currentY - distance] != 0)
          Map[currentX + 1][currentY - i] = 1;
      }
      break;
    case (South):
      for (int i = distance - 1; i > 0; i--) {
        Map[currentX][currentY + i] = 2;
        if (currentX > 0 && Map[currentX - 1][currentY + distance] != 4 &&
          Map[currentX - 1][currentY + distance] != 0)
          Map[currentX - 1][currentY + i] = 1;
        if (currentX < 7 && Map[currentX + 1][currentY + distance] != 4 &&
          Map[currentX + 1][currentY + distance] != 0)
          Map[currentX + 1][currentY + i] = 1;
      }
      break;
    case (West):
      for (int i = distance - 1; i > 0; i--) {
        Map[currentX + i][currentY] = 3;
        if (currentY > 0 && Map[currentX + distance][currentY - 1] != 4 &&
          Map[currentX + distance][currentY - 1] != 0)
          Map[currentX + i][currentY - 1] = 1;
        if (currentY < 7 && Map[currentX + distance][currentY + 1] != 4 &&
          Map[currentX + distance][currentY + 1] != 0)
          Map[currentX + i][currentY + 1] = 1;
      }
      break;
    case (East):
      for (int i = distance - 1; i > 0; i--) {
        Map[currentX - i][currentY] = 3;
        if (currentY > 0 && Map[currentX - distance][currentY - 1] != 4 &&
          Map[currentX - distance][currentY - 1] != 0)
          Map[currentX - i][currentY - 1] = 1;
        if (currentY < 7 && Map[currentX - distance][currentY + 1] != 4 &&
          Map[currentX - distance][currentY + 1] != 0)
          Map[currentX - i][currentY + 1] = 1;
      }
      break;
    }

    //Update Edges
    MapNode * n = FindNode(currentX, currentY);
    if (!EdgeExist(currentNode, n)) {
      AddEdge(n, distance);

      //Detected a loop
      if (existingNode && UnexploredPathExists(n))
        loopReverse = true;
    }

    currentNode = n;
  }
}

void FillMap() {
  int distance = abs(( * currentNode).x - currentX) + abs(( * currentNode).y - currentY);
  switch (currentDirection) {
  case (North):
    for (int i = distance - 1; i >= 0; i--)
      Map[currentX][currentY - i] = 2;
    break;
  case (South):
    for (int i = distance - 1; i >= 0; i--)
      Map[currentX][currentY + i] = 2;
    break;
  case (West):
    for (int i = distance - 1; i >= 0; i--)
      Map[currentX + i][currentY] = 3;
    break;
  case (East):
    for (int i = distance - 1; i >= 0; i--)
      Map[currentX - i][currentY] = 3;
    break;
  }
}

void NextDirection() {
  if (loopReverse) //Reverses direction if unexplored node in loop
  {
    currentDirection = (Direction)((currentDirection + 2) % 4);
    loopReverse = false;
  } else {
    //Find next direction
    Direction p = DirectionOfPrevNode(currentNode);
    Direction d = (Direction)((currentDirection + 3) % 4);
    if (p == d) d = (Direction)((d + 1) % 4);
    bool flag = false;

    while (d != p && !flag) {
      switch (d) {
      case (North):
        if (currentY < maxY - 1 && Map[currentX][currentY + 1] == 0) flag = true;
        break;
      case (South):
        if (currentY > 0 && Map[currentX][currentY - 1] == 0) flag = true;
        break;
      case (West):
        if (currentX > 0 && Map[currentX - 1][currentY] == 0) flag = true;
        break;
      case (East):
        if (currentX < maxX - 1 && Map[currentX + 1][currentY] == 0) flag = true;
        break;
      }
      if (!flag) d = (Direction)((d + 1) % 4);
    }
    currentDirection = d;
  }
}

void NextDirectionRescue() {
  if (shortestPathTotal > 1) {
    currentNode = shortestPath[--shortestPathTotal];
    currentDirection = (Direction)((DirectionOfPrevNode(shortestPath[shortestPathTotal - 1]) + 2) % 4);
  } else {
    currentNode = FindNode(currentX, currentY);
    currentDirection = DirectionOfPrevNode(currentNode);
  }
}

//------------------------
// Djikstra's Algorithm
//------------------------

MapNode * NextShortestNode() {
  int i = 0;
  while (i < nodeTotal && nodeList[i].Visited) i++;
  if (i == nodeTotal) return NULL;

  //first unvisited node
  MapNode * smallest = & nodeList[i];
  for (; i < nodeTotal; i++) {
    if (!nodeList[i].Visited && nodeList[i].Distance < ( * smallest).Distance)
      smallest = & nodeList[i];
  }
  return smallest;
}

void DjikstrasAlgorithm() {
  //set origin at zero distance
  nodeList[0].Distance = 0;

  //calculate shortest path
  while (!( * endNode).Visited) {
    currentNode = NextShortestNode();
    for (int i = 0; i < ( * currentNode).EdgeTotal; i++) {
      if (!( * ( * currentNode).Adjacent[i]).Visited) {
        int distance = ( * currentNode).Distance + ( * currentNode).Length[i];
        if (distance < ( * ( * currentNode).Adjacent[i]).Distance) {
          ( * ( * currentNode).Adjacent[i]).Distance = distance;
          ( * ( * currentNode).Adjacent[i]).PrevNode = currentNode;
        }
      }
    }
    ( * currentNode).Visited = true;
  }

  //create shortest path stack
  shortestPath[0] = endNode;
  shortestPathTotal = 1;
  while (( * currentNode).PrevNode != NULL) {
    currentNode = ( * currentNode).PrevNode;
    shortestPath[shortestPathTotal] = currentNode;
    shortestPathTotal++;
  }
}
