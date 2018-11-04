struct towerData {
    int servoData[30];
    int ultrasonicData[30];
};

class ObserverTower
{
  private:
    Servo servo;
    Ultrasonic ultrasonic;

    const int SERVO_WAITING_PERIOD_DEFAULT = 500;
    int servoPosStart;
    int servoPosEnd;
    int servoPosMiddle;
    int servoPosNext;
    int servoPosCurrent;
    int servoStep;
    int cyclesCount;
    int currCycle; // = cyclesCount - 1;
    int waitingCounter;
    int ultrasonicDistance;

  public:
    ObserverTower(
        Servo servoInit,
        Ultrasonic ultrasonicInit,
        int servoPosMiddleInit,
        int servoStepInit,
        int cyclesCountInit):
        servo(servoInit),
        ultrasonic(ultrasonicInit),
        servoPosMiddle(servoPosMiddleInit),
        servoStep(servoStepInit),
        cyclesCount(cyclesCountInit),
        currCycle(cyclesCountInit - 1),
        servoPosStart(0),
        waitingCounter(0),
        ultrasonicDistance(0)
        {
            servoPosEnd = servoPosMiddle * 2;
            servo.write(servoPosMiddle);
            servoPosCurrent = servoPosMiddle;
        }

    void lookAround();
    int read();
};
