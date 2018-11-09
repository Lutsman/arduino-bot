#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <ObserverTower.h>

ObserverTower::ObserverTower(
    uint8_t servoPinInit,
    uint8_t ultrasonicTrigPinInit,
    uint8_t ultrasonicEchoPinInit,
    int servoPosMiddleInit,
    int servoStepInit,
    int cyclesCountInit) : ultrasonic(ultrasonicTrigPinInit, ultrasonicEchoPinInit),
                           servoPosMiddle(servoPosMiddleInit),
                           servoStep(servoStepInit),
                           cyclesCount(cyclesCountInit),
                           currCycle(cyclesCountInit - 1),
                           servoPosStart(0),
                           servoPosEnd(servoPosMiddleInit * 2),
                           servoPosNext(-1),
                           waitingCounter(0),
                           ultrasonicDistance(0),
                           servoPin(servoPinInit),
                           isInited(false){};

void ObserverTower::attach(uint8_t servoPinInit)
{
  servo.attach(servoPin);
  servo.write(servoPosMiddle);
  servoPosCurrent = servoPosMiddle;
}

void ObserverTower::init()
{
  if (isInited)
    return;

  isInited = true;
  attach(servoPin);
}

void ObserverTower::lookAround()
{
  logMeasurements();

  // Serial.print('waiting counter');
  // Serial.println(waitingCounter);
  if (waitingCounter)
  {
    waitingCounter--;
    return;
  }

  if (servoPosNext == -1)
  {
    if (servoPosCurrent + servoStep <= servoPosEnd)
    {
      servoPosNext = servoPosCurrent + servoStep;
    }
    else
    {
      servoPosNext = servoPosCurrent - servoStep;
    }
  }

  // Serial.print('servo pos current');
  // Serial.println(servoPosCurrent);

  // Serial.print('servo pos next');
  // Serial.println(servoPosNext);

  servo.write(servoPosNext);
  servoPosCurrent = servoPosNext;

  // Serial.print('curr cycle');
  // Serial.println(currCycle);

  if (currCycle % 2 == 0 && currCycle != 0)
  {
    if (servoPosCurrent == servoPosEnd)
    {
      currCycle--;
      servoPosNext = servoPosCurrent - servoStep;
      return;
    }

    servoPosNext += servoStep;
    return;
  }

  if (currCycle % 2 == 1)
  {
    if (servoPosCurrent == servoPosStart)
    {
      currCycle--;
      servoPosNext = servoPosCurrent + servoStep;
      return;
    }

    servoPosNext -= servoStep;
    return;
  }

  if (currCycle == 0)
  {
    if (servoPosCurrent == servoPosMiddle)
    {
      currCycle = cyclesCount - 1;
      waitingCounter = SERVO_WAITING_PERIOD_DEFAULT - 1;
      servoPosNext = -1;
      return;
    }

    servoPosNext += servoStep;
    return;
  }
}

uint8_t ObserverTower::getDistance()
{
  return ultrasonic.read();
}

void ObserverTower::logMeasurements()
{
  uint8_t distance = getDistance();
  // Serial.println(servoPosCurrent);
  // Serial.println(distance);
  // Serial.println();
  observeData.servoData[0] = servoPosCurrent;
  observeData.ultrasonicData[0] = getDistance();
}

towerData ObserverTower::read()
{
  // logMeasurements();
  return observeData;
}
