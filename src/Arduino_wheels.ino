#include <SoftwareSerial.h>

// esp rx-tx
#define RX 10
#define TX 11

SoftwareSerial EspSerial(RX, TX);

// motor variables
int motor_L1, motor_L2, input_L;
int motor_R1, motor_R2, input_R;

// start symbols
char startS[] = "$!";
int direction = 5;
int speedL = 150;
int speedR = 150;
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
  while (EspSerial.available()) {
   if (EspSerial.read() == startS[0]) {
       Serial.println("transmission stage 1");
       if (EspSerial.read() == startS[1]) {
          Serial.println("transmission start");
          int command = EspSerial.parseInt();

          switch (command) {
            case 1:
              int x = EspSerial.parseInt();
              int y = EspSerial.parseInt();
              int newDirection = getDirection(x, y);

              if (newDirection == direction) break;

              move(newDirection);
              break;
            case 2:
              int newSpeed = EspSerial.parseInt();

              if (!speedToggler) break;

              speedL = speedR = newSpeed;
              move(direction);
              break;
            case 3:
              speedToggler = EspSerial.parseInt();
              break;
          }
       } 
   }
  }
}

void move(int newDirection) {
  _stop();
  setspeed();
  direction = newDirection;
  Serial.println(newDirection);

  switch(newDirection) {
    case 8:
      forward();
      Serial.println("forward");
      break;
    case 6:
      forward_right();
      Serial.println("forward_right");
      break;
    case 2:
      backward();
      Serial.println("backward");
      break;
    case 4:
      forward_left();
      Serial.println("forward_left");
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

int getDirection(int x, int y) {
  if (y > 900) {
    return 8;
  }

  if (y < 124) {
    return 2;
  }

  if (x > 900) {
    return 6;
  }

  if (x < 124) {
    return 4;
  }

  return 5;
}

// Функция инициализации управления моторами.
void setup_motor_system(int L1, int  L2, int  R1, int R2, int iL, int iR)
{
  // Заносятся в переменные номера контактов (пинов) Arduino.
  motor_L1=L1; motor_L2=L2; input_L=iL;
  // Для левых и правых моторов робота.
  motor_R1=R1; motor_R2=R2; input_R=iR;
  // Переводятся указанные порты в состояние вывода данных.
  pinMode(motor_L1, OUTPUT);
  pinMode(motor_L2, OUTPUT);
  pinMode(input_L, OUTPUT);
  pinMode(motor_R1, OUTPUT);
  pinMode(motor_R2, OUTPUT);
  pinMode(input_R, OUTPUT);
}
// Функция задает скорость двигателя.
void setspeed(int LeftSpeed, int RightSpeed)
{
  // Задает ширину положительного фронта от 0 до 255.
  analogWrite(input_L, LeftSpeed);
  analogWrite(input_R, RightSpeed);
  // Чем больше, тем интенсивнее работает мотор.
}
void setspeed()
{
  // Задает ширину положительного фронта от 0 до 255.
  analogWrite(input_L, speedL);
  analogWrite(input_R, speedR);
  // Чем больше, тем интенсивнее работает мотор.
}
// движение вперед.
void forward()
{
    // Если двигатель будет работать не в ту сторону,
    // поменять на нем контакты местами.
    digitalWrite(motor_L1, HIGH);
    digitalWrite(motor_L2,LOW);
    digitalWrite(motor_R1, HIGH);
    digitalWrite(motor_R2,LOW);
}
// Поворот налево с блокировкой левых колес.
void forward_left()
{
  // блокировка вращение левых колес.
    digitalWrite(motor_L1, LOW); 
    digitalWrite(motor_L2, LOW);
   // правые колеса вращаются.
    digitalWrite(motor_R1, HIGH);
    digitalWrite(motor_R2, LOW);
}
// Поворот направо с блокировкой правых колес.
void forward_right()
{
  // левые колеса вращаются.
    digitalWrite(motor_L1, HIGH); 
    digitalWrite(motor_L2, LOW);
    // блокировка вращение правых колес.
    digitalWrite(motor_R1, LOW);
    digitalWrite(motor_R2, LOW);
}
// Включаем движение назад.
void backward()
{
    // Смена направления вращения двигателей.
    digitalWrite(motor_L2, HIGH);
    digitalWrite(motor_L1, LOW);
    digitalWrite(motor_R2, HIGH);
    digitalWrite(motor_R1, LOW);
}

void backward_right()
{
    digitalWrite(motor_L2, LOW);
    digitalWrite(motor_L1, LOW);
    digitalWrite(motor_R2, HIGH);
    digitalWrite(motor_R1, LOW);
}

void backward_left()
{
    digitalWrite(motor_L2, HIGH);
    digitalWrite(motor_L1, LOW);
    digitalWrite(motor_R2, LOW);
    digitalWrite(motor_R1, LOW);
}

void _stop()
{
    // Блокировка всех колес.
    digitalWrite(motor_L2, LOW);
    digitalWrite(motor_L1, LOW);
    digitalWrite(motor_R2, LOW);
    digitalWrite(motor_R1, LOW);
}
