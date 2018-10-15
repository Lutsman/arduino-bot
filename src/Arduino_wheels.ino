#include <SoftwareSerial.h>

// esp rx-tx
#define RX 10
#define TX 11

SoftwareSerial Esp826Serial(RX, TX);

// motor variables
int motor_L1, motor_L2, input_L;
int motor_R1, motor_R2, input_R;



void setup()
{
  setup_motor_system(9, 7, 3, 4, 6, 5);
  setspeed(0, 0);

  Serial.begin(9600);
  Esp826Serial.begin(9600);

  Serial.println("gate arduino start");
  Serial.println("");
}
// Основная программа.
void loop()
{
  if (Esp826Serial.available()) {
    // char data[Esp826Serial.available()];
    // data = Esp826Serial.read();
    // Serial.write(Esp826Serial.read());
    // Serial.write(data);
    // Serial.println(Esp826Serial.readString());
    getDataFromSerial(Esp826Serial);
  }
}

void getDataFromSerial(SoftwareSerial serial) {
  char dataStart[] = "$$DATA_START$$";
  char dataEnd[] = "$$DATA_END$$";
  int start = -1;
  int end = -1;

  String dataStr = serial.readStringUntil('/n');
  // Serial.print("$$DATA_START$$");
  start = dataStr.indexOf(dataStart);
  
  if (start > -1) {
    end = dataStr.indexOf(dataEnd);

    dataStr = dataStr.substring(start + sizeof(dataStart), end);

    Serial.println(dataStr);
  }  
}

int * getValuesFromSerial(SoftwareSerial serial) {
  static int arr[5];

  for (int i = 0; i < 5; i++) {
    if (serial.available()) {
      arr[i] = serial.parseInt();
      if (serial.read() == '/n') break;
    } else {
      arr[i] = NULL;
    }
  }
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
    digitalWrite(motor_L1,LOW);
    digitalWrite(motor_R2, HIGH);
    digitalWrite(motor_R1,LOW);
}

void backward_right()
{
    digitalWrite(motor_L2, LOW);
    digitalWrite(motor_L1,LOW);
    digitalWrite(motor_R2, HIGH);
    digitalWrite(motor_R1,LOW);
}

void backward_left()
{
    digitalWrite(motor_L2, HIGH);
    digitalWrite(motor_L1,LOW);
    digitalWrite(motor_R2, LOW);
    digitalWrite(motor_R1,LOW);
}

void _stop()
{
    // Блокировка всех колес.
    digitalWrite(motor_L2, LOW);
    digitalWrite(motor_L1,LOW);
    digitalWrite(motor_R2, LOW);
    digitalWrite(motor_R1,LOW);
}
