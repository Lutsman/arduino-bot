#include <SoftwareSerial.h>
#include <PWMServo.h>
// #include <timer-api.h>

// esp rx-tx
#define VirtualRX 10
#define VirtualTX 11

SoftwareSerial EspSerial(VirtualRX, VirtualTX);

// start transmission symbols
char startS[] = "$!";

// motor variables
int motor_L1, motor_L2, input_L;
int motor_R1, motor_R2, input_R;

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

PWMServo servoFrontEcho; // create servo object to control a servo
// twelve servo objects can be created on most boards

//define servo pins
#define ServoFrontPin 9

// servo variables
const int SERVO_FRONT_POS_MIDDLE = 80;
int servoFrontPosCurrent = SERVO_FRONT_POS_MIDDLE;
int servoFrontPosNext = NULL;

//define sonic pins
#define SonicFrontTriggerPin A0
#define SonicFrontEchorPin A1

// defines sonic variables
long sonicDuration;
int sonicDistance;
long inches, cm;

/*Module Timers
        part of Arduino Mega Server project
            */

// Cycles
bool cycle1ms = false;
bool cycle10ms = false;
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

unsigned long time1ms;
unsigned long timer10ms;
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
  time1ms = uptimeSec;
  timer10ms = uptimeSec;
  timer50ms = uptimeSec;
  timer100ms = uptimeSec;
  timer200ms = uptimeSec;
  timer500ms = uptimeSec;
  timeSec = uptimeSec;
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
  if (timeMS - timer10ms >= 10)
  {
    timer10ms = timeMS;
    cycle10ms = true;
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
  cycler10ms = false;
  cycler50ms = false;
  cycler100ms = false;
  cycler200ms = false;
  cycler500ms = false;
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
  setupTimer();
  setupSerial();
  setupChassi();
  setupServo();
  setupSonic();
  Serial.println("gate arduino start");
  Serial.println();
  delay(100);
}

void loop()
{
  getDataFromEsp();

  timersWorks();
    if (counterServo == 0) {
    checkDistance();
    lookAround();
    counterServo = PERIOD_20 - 1;
  } else {
    counterServo--;
    Serial.println(counterServo);
  }

  if (counterMovement == 0) {
    manageMovement();
    counterMovement = PERIOD_10 - 1;
  } else {
    counterMovement--;
  }

  eraseCycles();
}

// const int PERIOD_1 = 1;
// const int PERIOD_10 = 10;
// const int PERIOD_15 = 15;
// const int PERIOD_20 = 20;
// const int PERIOD_50 = 50;
// const int PERIOD_100 = 100;

// void timer_handle_interrupts(int timer) {
//   static long counterServo = PERIOD_20 - 1;
//   static long counterSonic = PERIOD_20 - 1;
//   static long counterMovement = PERIOD_10 - 1;

//   Serial.println("1");

//   // if (counterSonic == 0) {
//   //   checkDistance();
//   //   counterSonic = PERIOD_20 - 1;
//   // } else {
//   //   counterSonic--;
//   // }

//   // Serial.println(counterServo);

//   // if (counterServo == 0) {
//   //   checkDistance();
//   //   lookAround();
//   //   counterServo = PERIOD_20 - 1;
//   // } else {
//   //   counterServo--;
//   //   Serial.println(counterServo);
//   // }

//   // if (counterMovement == 0) {
//   //   manageMovement();
//   //   counterMovement = PERIOD_10 - 1;
//   // } else {
//   //   counterMovement--;
//   // }

//   // counterServo = executeOnTimer(counterServo, PERIOD_20, lookAround);
//   // counterMovement = executeOnTimer(counterMovement, PERIOD_10, manageMovement);
// }

int executeOnTimer(int timer, int timerPeriod, void (*callback)(void))
{
  Serial.write(timer);
  Serial.println();
  if (timer == 0)
  {
    Serial.write(1);
    Serial.println();
    callback();
    return timerPeriod - 1;
  }

  return timer--;
}

void setupChassi()
{
  setupMotorSystem(12, 7, 3, 4, 6, 5);
  setspeed(0, 0);
}

void setupSerial()
{
  Serial.begin(9600);
  EspSerial.begin(9600);
}

void setupSonic()
{
  pinMode(SonicFrontTriggerPin, OUTPUT); // Sets the SonicFrontTriggerPin as an Output
  pinMode(SonicFrontEchorPin, INPUT);    // Sets the SonicFrontEchorPin as an Input
}

void setupServo()
{
  servoFrontEcho.attach(ServoFrontPin);         // attaches the servo on pin 9 to the servo object
  servoFrontEcho.write(SERVO_FRONT_POS_MIDDLE); // tell servo to go to position in variable 'servoFrontPosCurrent'
}

void setupTimer()
{
  timer_init_ISR_100Hz(TIMER_DEFAULT);
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

void lookAround()
{
  static const int cyclesCount = 3;
  static const int servoStep = 10;
  static const int servoStartPos = 0;
  static const int servoEndPos = 160;
  static const int servoPosMiddle = SERVO_FRONT_POS_MIDDLE;
  static int currCycle = cyclesCount - 1;
  // static int heartBeat = 0;
  // static int servoFrontPosCurrent = SERVO_FRONT_POS_MIDDLE;
  // static int servoFrontPosNext = NULL;
  static const int WAITING_PERIOD_DEFAULT = 1000;
  static int waitingCounter = 0;

  if (!servoFrontPosNext)
  {
    if (servoFrontPosCurrent + servoStep <= servoEndPos)
    {
      servoFrontPosNext = servoFrontPosCurrent + servoStep;
    }
    else
    {
      servoFrontPosNext = servoFrontPosCurrent - servoStep;
    }
  }

  // checkDistance();

  if (waitingCounter)
  {
    waitingCounter--;
    return;
  }

  servoFrontEcho.write(servoFrontPosNext); // tell servo to go to position in variable 'servoFrontPosCurrent'
  servoFrontPosCurrent = servoFrontPosNext;

  if (currCycle % 2 == 0 && currCycle != 0)
  {
    if (servoFrontPosCurrent == servoEndPos)
    {
      currCycle--;
      servoFrontPosNext = servoFrontPosCurrent - servoStep;
      return;
    }

    servoFrontPosNext += servoStep;
    return;
  }

  if (currCycle % 2 == 1)
  {
    if (servoFrontPosCurrent == servoStartPos)
    {
      currCycle--;
      servoFrontPosNext = servoFrontPosCurrent + servoStep;
      return;
    }

    servoFrontPosNext -= servoStep;
    return;
  }

  if (currCycle == 0)
  {
    if (servoFrontPosCurrent == servoPosMiddle)
    {
      currCycle = cyclesCount - 1;
      waitingCounter = WAITING_PERIOD_DEFAULT - 1;
      servoFrontPosNext = NULL;
      return;
    }

    servoFrontPosNext += servoStep;
    return;
  }
}

void checkDistance()
{
  // Clears the SonicFrontTriggerPin
  digitalWrite(SonicFrontTriggerPin, 0);
  delayMicroseconds(2);
  // Sets the SonicFrontTriggerPin on 1 state for 10 micro seconds
  digitalWrite(SonicFrontTriggerPin, 1);
  delayMicroseconds(10);
  digitalWrite(SonicFrontTriggerPin, 0);
  // Reads the SonicFrontEchorPin, returns the sound wave travel time in microseconds
  sonicDuration = pulseIn(SonicFrontEchorPin, 1);

  // convert the time into a sonicDistance
  inches = microsecondsToInches(sonicDuration);
  cm = microsecondsToCentimeters(sonicDuration);

  Serial.print(servoFrontPosCurrent);
  Serial.print("position, ");
  Serial.print(inches);
  Serial.print("in, ");
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();
}

long microsecondsToInches(long microseconds)
{
  // According to Parallax's datasheet for the PING))), there are 73.746
  // microseconds per inch (i.e. sound travels at 1130 feet per second).
  // This gives the sonicDistance travelled by the ping, outbound and return,
  // so we divide by 2 to get the sonicDistance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the sonicDistance of the object we
  // take half of the sonicDistance travelled.
  return microseconds / 29 / 2;
}
