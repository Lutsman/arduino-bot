#ifndef ObserverTower_h
#define ObserverTower_h

#include <Servo.h>
#include <Ultrasonic.h>

struct towerData
{
  int servoData[30];
  uint8_t ultrasonicData[30];
};

class ObserverTower
{
private:
  const int SERVO_WAITING_PERIOD_DEFAULT = 500;

  Servo servo;
  Ultrasonic ultrasonic;
  towerData observeData;

  uint8_t servoPin;
  int servoPosStart;
  int servoPosEnd;
  int servoPosMiddle;
  int servoPosNext;
  int servoPosCurrent;
  int servoStep;
  int ultrasonicDistance;
  int cyclesCount;
  int currCycle;
  int waitingCounter;
  bool isInited;

public:
  ObserverTower(
      uint8_t servoPinInit,
      uint8_t ultrasonicTrigPinInit,
      uint8_t ultrasonicEchoPinInit,
      int servoPosMiddleInit,
      int servoStepInit,
      int cyclesCountInit);
  void init();
  void lookAround();
  uint8_t getDistance();
  void logMeasurements();
  towerData read();
  void attach(uint8_t servoPinInit);
};

#endif
