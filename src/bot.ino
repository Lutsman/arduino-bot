#include <SoftwareSerial.h>

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

void setup()
{
  setup_motor_system(9, 7, 3, 4, 6, 5);
  setspeed(0, 0);

  Serial.begin(9600);
  EspSerial.begin(9600);

  Serial.println("gate arduino start");
  Serial.println("");
}
// Основная программа.
void loop()
{
  getDataFromEsp();
}

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

// Функция инициализации управления моторами.
void setup_motor_system(int L1, int L2, int R1, int R2, int iL, int iR)
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