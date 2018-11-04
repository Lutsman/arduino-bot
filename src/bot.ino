#include <SoftwareSerial.h>
#include <Servo.h>
#include <Ultrasonic.h>
#include

// esp rx-tx
#define VirtualRX 2
#define VirtualTX 3

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

Servo servoFrontEcho; // create servo object to control a servo
Servo servoBackEcho;
// twelve servo objects can be created on most boards

//define servo pins
#define ServoFrontPin 4
#define ServoBackPin 13

// servo variables
const int SERVO_WAITING_PERIOD_DEFAULT = 500;
// front
const int SERVO_FRONT_POS_MIDDLE = 80;
int servoFrontPosCurrent = SERVO_FRONT_POS_MIDDLE;
int servoFrontPosNext = -1;
// back
const int SERVO_BACK_POS_MIDDLE = 80;
int servoBackPosCurrent = SERVO_BACK_POS_MIDDLE;
int servoBackPosNext = -1;


//define sonic pins
// front
#define SonicFrontTriggerPin A0
#define SonicFrontEchorPin A1
//back
#define SonicFrontTriggerPin 12
#define SonicFrontEchorPin 11

// defines sonic variables
long sonicDuration;
int sonicDistance;
long inches, cm;

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
  setupServo();
  setupSonic();
  Serial.println("Gate arduino start");
  delay(100);
}

void loop()
{
  getDataFromEsp();

  timersWorks();

  if (cycle10ms)
  {
    manageMovement();
    checkDistance();
    // lookAroundFront();
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

void setupSonic()
{
  pinMode(SonicFrontTriggerPin, OUTPUT); // Sets the SonicFrontTriggerPin as an Output
  pinMode(SonicFrontEchorPin, INPUT);    // Sets the SonicFrontEchorPin as an Input
}

void setupServo()
{
  servoFrontEcho.attach(ServoFrontPin);         // attaches the servo on pin 9 to the servo object
  servoFrontEcho.write(SERVO_FRONT_POS_MIDDLE); // tell servo to go to position in variable 'servoFrontPosCurrent'

  servoBackEcho.attach(ServoBackPin);
  servoBackEcho.write(SERVO_BACK_POS_MIDDLE);
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

void lookAroundFront(
  // Servo servo, 
  // Ultrasonic sonic, 
  // int servoPosMiddle, 
  // int servoPosStep, 
  // int servoPosCurrent, 
  // int servoPosNext
  )
{
  static const int cyclesCount = 3;
  static const int servoPosMiddle = SERVO_FRONT_POS_MIDDLE;
  static const int servoStep = 10;
  static const int servoPosStart = 0;
  static const int servoPosEnd = servoPosMiddle * 2;
  static int currCycle = cyclesCount - 1;
  static int waitingCounter = 0;

  if (waitingCounter)
  {
    waitingCounter--;
    // Serial.print("waitingCounter = ");
    // Serial.println(waitingCounter);
    return;
  }

  // Serial.println(servoFrontPosNext);
  if (servoFrontPosNext == -1)
  {
    if (servoFrontPosCurrent + servoStep <= servoPosEnd)
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

  // Serial.print("servoFrontPosNext end = ");
  // Serial.println(servoFrontPosNext);

  servoFrontEcho.write(servoFrontPosNext); // tell servo to go to position in variable 'servoFrontPosCurrent'
  servoFrontPosCurrent = servoFrontPosNext;

  if (currCycle % 2 == 0 && currCycle != 0)
  {
    // Serial.println("4et");
    if (servoFrontPosCurrent == servoPosEnd)
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
    // Serial.println("ne 4et");
    if (servoFrontPosCurrent == servoPosStart)
    {
      // Serial.println("ne 4et last step");
      currCycle--;
      servoFrontPosNext = servoFrontPosCurrent + servoStep;
      return;
    }

    servoFrontPosNext -= servoStep;
    return;
  }

  if (currCycle == 0)
  {
    // Serial.println("last 4et");
    if (servoFrontPosCurrent == servoPosMiddle)
    {
      currCycle = cyclesCount - 1;
      waitingCounter = SERVO_WAITING_PERIOD_DEFAULT - 1;
      servoFrontPosNext = -1;
      return;
    }

    servoFrontPosNext += servoStep;
    return;
  }
}

void lookAroundBack()
{
  static const int cyclesCount = 3;
  static const int servoPosMiddle = SERVO_BACK_POS_MIDDLE;
  static const int servoStep = 10;
  static const int servoPosStart = 0;
  static const int servoPosEnd = servoPosMiddle * 2;
  static int currCycle = cyclesCount - 1;
  static int waitingCounter = 0;

  if (waitingCounter)
  {
    waitingCounter--;
    // Serial.print("waitingCounter = ");
    // Serial.println(waitingCounter);
    return;
  }

  // Serial.println(servoBackPosNext);
  if (servoBackPosNext == -1)
  {
    if (servoBackPosCurrent + servoStep <= servoPosEnd)
    {
      servoBackPosNext = servoBackPosCurrent + servoStep;
    }
    else
    {
      servoBackPosNext = servoBackPosCurrent - servoStep;
    }
  }

  // checkDistance();

  if (waitingCounter)
  {
    waitingCounter--;
    return;
  }

  // Serial.print("servoBackPosNext end = ");
  // Serial.println(servoBackPosNext);

  servoBackEcho.write(servoBackPosNext); // tell servo to go to position in variable 'servoBackPosCurrent'
  servoBackPosCurrent = servoBackPosNext;

  if (currCycle % 2 == 0 && currCycle != 0)
  {
    // Serial.println("4et");
    if (servoBackPosCurrent == servoPosEnd)
    {
      currCycle--;
      servoBackPosNext = servoBackPosCurrent - servoStep;
      return;
    }

    servoBackPosNext += servoStep;
    return;
  }

  if (currCycle % 2 == 1)
  {
    // Serial.println("ne 4et");
    if (servoBackPosCurrent == servoPosStart)
    {
      // Serial.println("ne 4et last step");
      currCycle--;
      servoBackPosNext = servoBackPosCurrent + servoStep;
      return;
    }

    servoBackPosNext -= servoStep;
    return;
  }

  if (currCycle == 0)
  {
    // Serial.println("last 4et");
    if (servoBackPosCurrent == servoPosMiddle)
    {
      currCycle = cyclesCount - 1;
      waitingCounter = SERVO_WAITING_PERIOD_DEFAULT - 1;
      servoBackPosNext = -1;
      return;
    }

    servoBackPosNext += servoStep;
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
