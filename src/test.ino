void lookAroundBack()
{
  static const int cyclesCount = 3;
  static const int servoPosMiddle = SERVO_BACK_POS_MIDDLE;
  static const int servoStep = 10;
  static const int servoPosStart = 0;
  static const int servoPosEnd = servoPosMiddle * 2;
  static int currCycle = cyclesCount - 1;
  static const int WAITING_PERIOD_DEFAULT = 500;
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
      waitingCounter = WAITING_PERIOD_DEFAULT - 1;
      servoBackPosNext = -1;
      return;
    }

    servoBackPosNext += servoStep;
    return;
  }
}