#include <Servo.h>
#include <Ultrasonic.h>
#include <ObserverTower.h>
#include <Arduino.h>

// const int ObserverTower::servoPosEnd = servoPosMiddle * 2;

void ObserverTower::lookAround()
{

  if (waitingCounter)
  {
    waitingCounter--;
    // Serial.print("waitingCounter = ");
    // Serial.println(waitingCounter);
    return;
  }

  // Serial.println(servoPosNext);
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

  // checkDistance();

  if (waitingCounter)
  {
    waitingCounter--;
    return;
  }

  // Serial.print("servoPosNext end = ");
  // Serial.println(servoPosNext);

  ultrasonicDistance = ultrasonic.read();
  servo.write(servoPosNext); // tell servo to go to position in variable 'servoPosCurrent'
  servoPosCurrent = servoPosNext;

  if (currCycle % 2 == 0 && currCycle != 0)
  {
    // Serial.println("4et");
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
    // Serial.println("ne 4et");
    if (servoPosCurrent == servoPosStart)
    {
      // Serial.println("ne 4et last step");
      currCycle--;
      servoPosNext = servoPosCurrent + servoStep;
      return;
    }

    servoPosNext -= servoStep;
    return;
  }

  if (currCycle == 0)
  {
    // Serial.println("last 4et");
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
