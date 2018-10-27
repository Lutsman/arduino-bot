#include <SoftwareSerial.h>
#include <Servo.h>

// esp rx-tx
#define VirtualRX 10
#define VirtualTX 11

Servo servoFrontEcho;  // create servo object to control a servo
// twelve servo objects can be created on most boards

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
//define servo pins
#define ServoFrontPin 12

// servo variables
const int servoFrontPosMiddle = 80;
int servoFrontPosCurrent = servoFrontPosMiddle;    // variable to store the servo position

//define sonic pins
#define SonicFrontTriggerPin A0
#define SonicFrontEchorPin A1

// defines sonic variables
long sonicDuration;
int sonicDistance;
long inches, cm;
int counter;


void setup() {
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
  lookAround();
}

void setupChassi() {
  ssetupMotorSystem(9, 7, 3, 4, 6, 5);
  setspeed(0, 0);
}

void setupSerial() {
  Serial.begin(9600);
  EspSerial.begin(9600);
}

void setupSonic() {
  pinMode(SonicFrontTriggerPin, OUTPUT); // Sets the SonicFrontTriggerPin as an Output
  pinMode(SonicFrontEchorPin, INPUT); // Sets the SonicFrontEchorPin as an Input
}

void setupServo() {
  servoFrontEcho.attach(ServoFrontPin);  // attaches the servo on pin 9 to the servo object
  servoFrontEcho.write(servoFrontPosMiddle);              // tell servo to go to position in variable 'servoFrontPosCurrent'
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

  switch (command)
  {
  case 1:
    int x = EspSerial.parseInt();
    int y = EspSerial.parseInt();
    newDirection = getDirection(x, y);
    // if (newDirection == direction) break;
    // move(newDirection);
    break;
  case 2:
    int newSpeed = EspSerial.parseInt();
    newSpeedL = newSpeedR = newSpeed;
    // move(direction);
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

void ssetupMotorSystem(int L1, int L2, int R1, int R2, int iL, int iR)
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


// servo and sonic

void lookAround() {
    for (servoFrontPosCurrent = servoFrontPosMiddle; servoFrontPosCurrent <= 160; servoFrontPosCurrent += 10) { // goes from 0 degrees to 180 degrees
    checkDistance();
    // in steps of 1 degree
    servoFrontEcho.write(servoFrontPosCurrent);              // tell servo to go to position in variable 'servoFrontPosCurrent'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (servoFrontPosCurrent = 160; servoFrontPosCurrent >= 0; servoFrontPosCurrent -= 10) { // goes from 180 degrees to 0 degrees
    checkDistance();
    servoFrontEcho.write(servoFrontPosCurrent);              // tell servo to go to position in variable 'servoFrontPosCurrent'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (servoFrontPosCurrent = 0; servoFrontPosCurrent <= servoFrontPosMiddle; servoFrontPosCurrent += 10) { // goes from 180 degrees to 0 degrees
    checkDistance();
    servoFrontEcho.write(servoFrontPosCurrent);              // tell servo to go to position in variable 'servoFrontPosCurrent'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  
  counter = 50;
  while(counter) {
    checkDistance();
    delay(100);
    counter--;
  }
}

void checkDistance() {
    // Clears the SonicFrontTriggerPin
    digitalWrite(SonicFrontTriggerPin, LOW);
    delayMicroseconds(2);
    // Sets the SonicFrontTriggerPin on HIGH state for 10 micro seconds
    digitalWrite(SonicFrontTriggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(SonicFrontTriggerPin, LOW);
    // Reads the SonicFrontEchorPin, returns the sound wave travel time in microseconds
    sonicDuration = pulseIn(SonicFrontEchorPin, HIGH);

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

long microsecondsToInches(long microseconds) {
  // According to Parallax's datasheet for the PING))), there are 73.746
  // microseconds per inch (i.e. sound travels at 1130 feet per second).
  // This gives the sonicDistance travelled by the ping, outbound and return,
  // so we divide by 2 to get the sonicDistance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the sonicDistance of the object we
  // take half of the sonicDistance travelled.
  return microseconds / 29 / 2;
}
