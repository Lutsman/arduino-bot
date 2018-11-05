#include <SoftwareSerial.h>
#include <ObserverTower.h>

// esp rx-tx
const uint8_t VirtualRX = 2;
const uint8_t VirtualTX = 3;

SoftwareSerial EspSerial(VirtualRX, VirtualTX);

// start transmission symbols
char startS[] = "$!";

// motor variables
int motor_L1, motor_L2, input_L;
int motor_R1, motor_R2, input_R;

// motor pins
const int INPUT_R = 5;
const int MOTOR_R1 = 6;
const int MOTOR_R2 = 7;
const int MOTOR_L1 = 8;
const int MOTOR_L2 = 9;
const int INPUT_L = 10;

//direction
int direction = 5;
int newDirection = direction;

//speed
int speedL = 255;
int speedR = 255;
int newSpeedL = speedL;
int newSpeedR = speedR;

// speed toggler
bool speedToggler = 0;

//sonic and servo global variables

const int SERVO_POS_MIDDLE_FRONT = 80;
const int SERVO_POS_MIDDLE_BACK = 80;
const int SERVO_STEP = 10;
const int CYCLES_COUNT = 3;

//define servo pins
const uint8_t ServoFrontPin = 4;
const uint8_t ServoBackPin = 13;

//define sonic pins
// front
const uint8_t UltrasonicFrontTriggerPin = 'A0';
const uint8_t UltrasonicFrontEchoPin = 'A1';
//back
const uint8_t UltrasonicBackTriggerPin = 12;
const uint8_t UltrasonicBackEchoPin = 11;

//towers
ObserverTower towerFront(
    ServoFrontPin,
    UltrasonicFrontTriggerPin,
    UltrasonicFrontEchoPin,
    SERVO_POS_MIDDLE_FRONT,
    SERVO_STEP,
    CYCLES_COUNT);

ObserverTower towerBack(
    ServoBackPin,
    UltrasonicBackTriggerPin,
    UltrasonicBackEchoPin,
    SERVO_POS_MIDDLE_BACK,
    SERVO_STEP,
    CYCLES_COUNT);

/*Module Timers
        part of Arduino Mega Server project
            */

// Cycles
bool cycle1ms = false;
bool cycle5ms = false;
bool cycle10ms = false;
bool cycle15ms = false;
bool cycle20ms = false;
bool cycle30ms = false;
bool cycle40ms = false;
bool cycle50ms = false;
bool cycle100ms = false;
bool cycle200ms = false;
bool cycle500ms = false;
bool cycle1s = false;
bool cycle5s = false;
bool cycle20s = false;
bool cycle30s = false;
bool cycle1m = false;
bool cycle3m = false;
bool cycle5m = false;

unsigned long timer1ms;
unsigned long timer5ms;
unsigned long timer10ms;
unsigned long timer15ms;
unsigned long timer20ms;
unsigned long timer30ms;
unsigned long timer40ms;
unsigned long timer50ms;
unsigned long timer100ms;
unsigned long timer200ms;
unsigned long timer500ms;
unsigned long timeSec;
unsigned long timer1s;
unsigned long timer5s;
unsigned long timer20s;
unsigned long timer30s;
unsigned long timer1m;
unsigned long timer3m;
unsigned long timer5m;

void timersInit()
{
  unsigned long uptimeSec = millis() / 1000;
  timer1ms = uptimeSec;
  timer5ms = uptimeSec;
  timer10ms = uptimeSec;
  timer15ms = uptimeSec;
  timer20ms = uptimeSec;
  timer30ms = uptimeSec;
  timer40ms = uptimeSec;
  timer50ms = uptimeSec;
  timer100ms = uptimeSec;
  timer200ms = uptimeSec;
  timer500ms = uptimeSec;
  timer1s = uptimeSec;
  timer5s = uptimeSec;
  timer20s = uptimeSec;
  timer30s = uptimeSec;
  timer1m = uptimeSec;
  timer3m = uptimeSec;
  timer5m = uptimeSec;
}

void timersWorks()
{
  int timeMS = millis();
  int timeSec = timeMS / 1000;

  if (timeMS - timer1ms >= 1)
  {
    timer1ms = timeMS;
    cycle1ms = true;
  }
  if (timeMS - timer5ms >= 5)
  {
    timer5ms = timeMS;
    cycle5ms = true;
  }
  if (timeMS - timer10ms >= 10)
  {
    timer10ms = timeMS;
    cycle10ms = true;
  }
  if (timeMS - timer15ms >= 15)
  {
    timer15ms = timeMS;
    cycle15ms = true;
  }
  if (timeMS - timer20ms >= 20)
  {
    timer20ms = timeMS;
    cycle20ms = true;
  }
  if (timeMS - timer30ms >= 30)
  {
    timer30ms = timeMS;
    cycle30ms = true;
  }
  if (timeMS - timer40ms >= 40)
  {
    timer40ms = timeMS;
    cycle40ms = true;
  }
  if (timeMS - timer50ms >= 50)
  {
    timer50ms = timeMS;
    cycle50ms = true;
  }
  if (timeMS - timer100ms >= 100)
  {
    timer100ms = timeMS;
    cycle100ms = true;
  }
  if (timeMS - timer200ms >= 200)
  {
    timer200ms = timeMS;
    cycle200ms = true;
  }
  if (timeMS - timer500ms >= 500)
  {
    timer500ms = timeMS;
    cycle500ms = true;
  }
  if (timeSec - timer1s >= 1)
  {
    timer1s = timeSec;
    cycle1s = true;
  }
  if (timeSec - timer5s >= 5)
  {
    timer5s = timeSec;
    cycle5s = true;
  }
  if (timeSec - timer20s >= 20)
  {
    timer20s = timeSec;
    cycle20s = true;
  }
  if (timeSec - timer30s >= 30)
  {
    timer30s = timeSec;
    cycle30s = true;
  }
  if (timeSec - timer1m >= 60)
  {
    timer1m = timeSec;
    cycle1m = true;
  }
  if (timeSec - timer3m >= 180)
  {
    timer3m = timeSec;
    cycle3m = true;
  }
  if (timeSec - timer5m >= 300)
  {
    timer5m = timeSec;
    cycle5m = true;
  }
}

void eraseCycles()
{
  cycle1ms = false;
  cycle5ms = false;
  cycle10ms = false;
  cycle15ms = false;
  cycle20ms = false;
  cycle30ms = false;
  cycle40ms = false;
  cycle50ms = false;
  cycle100ms = false;
  cycle200ms = false;
  cycle500ms = false;
  cycle1s = false;
  cycle5s = false;
  cycle20s = false;
  cycle30s = false;
  cycle1m = false;
  cycle3m = false;
  cycle5m = false;
}

void setup()
{
  timersInit();
  setupSerial();
  setupChassi();
  setupTowers();
  Serial.println("Gate arduino start");
  delay(100);
}

void loop()
{
  getDataFromEsp();

  timersWorks();

  if (cycle100ms)
  {
    // Serial.print('Servo position');
    dataToSerial(towerFront.read(), 'Front tower');
    dataToSerial(towerBack.read(), 'Back tower');

    towerFront.lookAround();
    towerBack.lookAround();
    manageMovement();
  }

  eraseCycles();
}

void setupChassi()
{
  setupMotorSystem(MOTOR_L1, MOTOR_L2, MOTOR_R1, MOTOR_R2, INPUT_L, INPUT_R);
  setspeed(0, 0);
}

void setupSerial()
{
  Serial.begin(9600);
  EspSerial.begin(9600);
}

void setupTowers() {
  towerFront.init();
  towerBack.init();
}

void dataToSerial(towerData data, char name) {
  Serial.write(name);
  Serial.print('Servo position = ');
  Serial.println(data.servoData[0]);
  Serial.print('Distance = ');
  Serial.println(data.ultrasonicData[0]);
}

// connecting
void getDataFromEsp()
{
  if (!EspSerial.available())
    return;
  if (EspSerial.read() != startS[0])
    return;
  Serial.println("transmission stage 1");
  if (EspSerial.read() != startS[1])
    return;
  Serial.println("transmission stage 0");
  Serial.println("transmission start");

  int command = EspSerial.parseInt();
  Serial.print("command = ");
  Serial.println(command);

  switch (command)
  {
  case 1:
    int x = EspSerial.parseInt();
    int y = EspSerial.parseInt();
    newDirection = getDirection(x, y);
    break;
  case 2:
    int newSpeed = EspSerial.parseInt();
    newSpeedL = newSpeedR = newSpeed;
    break;
  case 3:
    speedToggler = EspSerial.parseInt();
    break;
  }
}

// movement
void manageMovement()
{
  bool isChaged = false;

  if (speedToggler && (newSpeedL != speedL || newSpeedR != speedR))
  {
    speedL = newSpeedL;
    speedR = newSpeedR;
    isChaged = true;
  }

  if (newDirection != direction)
  {
    direction = newDirection;
    isChaged = true;
  }

  if (isChaged)
  {
    move(direction, speedL, speedR);
  }
}

void move(int direction, int speedL, int speedR)
{
  _stop();
  setspeed(speedL, speedR);
  Serial.println(direction);

  switch (direction)
  {
  case 8:
    forward();
    Serial.println("forward");
    break;
  case 6:
    right();
    Serial.println("right");
    break;
  case 2:
    backward();
    Serial.println("backward");
    break;
  case 4:
    left();
    Serial.println("left");
    break;
  case 5:
    _stop();
    Serial.println("stop");
    break;
  default:
    _stop();
    Serial.println("stop");
    break;
  }
}

int getDirection(int x, int y)
{
  if (y > 900)
  {
    return 8;
  }

  if (y < 124)
  {
    return 2;
  }

  if (x > 900)
  {
    return 6;
  }

  if (x < 124)
  {
    return 4;
  }

  return 5;
}

void setupMotorSystem(int L1, int L2, int R1, int R2, int iL, int iR)
{
  // Заносятся в переменные номера контактов (пинов) Arduino.
  motor_L1 = L1;
  motor_L2 = L2;
  input_L = iL;
  // Для левых и правых моторов робота.
  motor_R1 = R1;
  motor_R2 = R2;
  input_R = iR;
  // Переводятся указанные порты в состояние вывода данных.
  pinMode(motor_R1, OUTPUT);
  pinMode(motor_R2, OUTPUT);
  pinMode(input_R, OUTPUT);

  pinMode(motor_L1, OUTPUT);
  pinMode(motor_L2, OUTPUT);
  pinMode(input_L, OUTPUT);
}

void setspeed(int LeftSpeed, int RightSpeed)
{
  // Задает ширину положительного фронта от 0 до 255.
  analogWrite(input_R, RightSpeed);
  analogWrite(input_L, LeftSpeed);
}

void forward()
{
  digitalWrite(motor_R1, 1);
  digitalWrite(motor_R2, 0);

  digitalWrite(motor_L1, 1);
  digitalWrite(motor_L2, 0);
}

void backward()
{
  digitalWrite(motor_R1, 0);
  digitalWrite(motor_R2, 1);

  digitalWrite(motor_L1, 0);
  digitalWrite(motor_L2, 1);
}

void right()
{
  digitalWrite(motor_R1, 0);
  digitalWrite(motor_R2, 1);

  digitalWrite(motor_L1, 1);
  digitalWrite(motor_L2, 0);
}

void left()
{
  digitalWrite(motor_R1, 1);
  digitalWrite(motor_R2, 0);

  digitalWrite(motor_L1, 0);
  digitalWrite(motor_L2, 1);
}

void _stop()
{
  digitalWrite(motor_R2, 0);
  digitalWrite(motor_R1, 0);

  digitalWrite(motor_L2, 0);
  digitalWrite(motor_L1, 0);
}