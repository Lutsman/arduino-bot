#include <ObserverTower.h>

// const int ObserverTower::servoPosEnd = servoPosMiddle * 2;

void ObserverTower::init(
    Servo servoInit,
    Ultrasonic ultrasonicInit,
    HardwareSerial &serialInit,
    int servoPosMiddleInit,
    int servoStepInit,
    int cyclesCountInit)
{
  servo = servoInit;
  ultrasonic = ultrasonicInit;
  serial = serialInit;
  servoPosMiddle = servoPosMiddleInit;
  servoStep = servoStepInit;
  cyclesCount = cyclesCountInit;
  currCycle = cyclesCountInit - 1;
  servoPosStart = 0;
  servoPosEnd = servoPosMiddle * 2;
  waitingCounter = 0;
  ultrasonicDistance = 0;

  servo.write(servoPosMiddle);
  servoPosCurrent = servoPosMiddle;
};

void ObserverTower::lookAround()
{
  serial.println('look around');
  if (waitingCounter)
  {
    waitingCounter--;
    // serial.print("waitingCounter = ");
    // serial.println(waitingCounter);
    return;
  }

  // serial.println(servoPosNext);
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

  // serial.print("servoPosNext end = ");
  // serial.println(servoPosNext);

  // ultrasonicDistance = ultrasonic.read();
  write();
  // serial.println(servoPosNext);
  servo.write(servoPosNext); // tell servo to go to position in variable 'servoPosCurrent'
  servoPosCurrent = servoPosNext;

  if (currCycle % 2 == 0 && currCycle != 0)
  {
    // serial.println("4et");
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
    // serial.println("ne 4et");
    if (servoPosCurrent == servoPosStart)
    {
      // serial.println("ne 4et last step");
      currCycle--;
      servoPosNext = servoPosCurrent + servoStep;
      return;
    }

    servoPosNext -= servoStep;
    return;
  }

  if (currCycle == 0)
  {
    // serial.println("last 4et");
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

int ObserverTower::read()
{
  return ultrasonicDistance;
}

void ObserverTower::write()
{
  ultrasonicDistance = ultrasonic.read();

  serial.print(servoPosCurrent);
  serial.print("position, ");
  serial.print(ultrasonicDistance);
  serial.print("cm");
  serial.println();
};
