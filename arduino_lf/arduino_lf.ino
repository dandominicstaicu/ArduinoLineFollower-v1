double error;
double lastError;
double errorDiff;
double errorInt;

#define KP 17.00
#define KD 600.00
#define KI 0.00
#define baseSpeed 80.00
#define nearLineModifier 0.50

/*
best 1
KP 6
KD 500 
KI 0.1
base speed 25

best 2 
#define KP 6.0
#define KD 560
#define KI 0.1
#define baseSpeed 25.0
#define nearLineModifier 1.0


best 3 
#define KP 3.00
#define KD 580.0
#define KI 0.0
#define baseSpeed 25.0
#define nearLineModifier 1.0

19s
best 4
#define KP 6.00
#define KD 580.00
#define KI 0.00
#define baseSpeed 30.00
#define nearLineModifier 0.5

16.50s

best 5
#define KP 6.00
#define KD 580.00
#define KI 0.00
#define baseSpeed 30.00
#define nearLineModifier 0.5

15.05s
best 6 - extensie 
#define KP 6.00
#define KD 580.00
#define KI 0.00
#define baseSpeed 30.00
#define nearLineModifier 0.5

#define KP 17.00
#define KD 600.00
#define KI 0.00
#define baseSpeed 70.00
#define nearLineModifier 0.50
10.6s

///prototype
#define KP 31.00
#define KD 810.00
#define KI 0.00
#define baseSpeed 100.00
#define nearLineModifier 0.60

best of best 
#define KP 17.00
#define KD 600.00
#define KI 0.00
#define baseSpeed 80.00
#define nearLineModifier 0.50

10.3s
*/

struct motor
{

  int motorControllerForwardPIN;
  int motorControllerBackwardPIN;
  int motorControllerSpeedPIN;
  int internalPower;
  int mappedPower;
  bool internalOrientation = 0;

  void setOrientation(bool orientation)
  {
    internalOrientation = orientation;
  }

  void setPins(int forwardPIN, int backwardPIN, int speedPWMPIN)
  {
    motorControllerForwardPIN = forwardPIN;
    motorControllerBackwardPIN = backwardPIN;
    motorControllerSpeedPIN = speedPWMPIN;
    pinMode(motorControllerForwardPIN, OUTPUT);
    pinMode(motorControllerBackwardPIN, OUTPUT);
    pinMode(motorControllerSpeedPIN, OUTPUT);
  }

  void setPower(int power)
  {
    power = constrain(power, -100, 100);
    mappedPower = power;
    if (internalOrientation)
      power *= -1;
    internalPower = map(abs(power), 0, 100, 0, 255);
    if (power < 0)
    {
      digitalWrite(motorControllerForwardPIN, LOW);
      digitalWrite(motorControllerBackwardPIN, HIGH);
    }
    else
    {
      digitalWrite(motorControllerBackwardPIN, LOW);
      digitalWrite(motorControllerForwardPIN, HIGH);
    }
    analogWrite(motorControllerSpeedPIN, internalPower);
  }
};

motor Right;
motor Left;

struct lineSensor
{
  bool leftValue;
  bool leftCenterValue;
  bool centerValue;
  bool rightCenterValue;
  bool rightValue;

  double globalLineValue = 0;
  String lineOutput;
  int lastGlobalLineValue = 0;
  int leftSensorPIN;
  int leftCenterSensorPIN;
  int centerSensorPIN;
  int rightCenterSensorPIN;
  int rightSensorPIN;

  void setPins(int left, int leftCenter, int center, int rightCenter, int right)
  {
    leftSensorPIN = left;
    leftCenterSensorPIN = leftCenter;
    centerSensorPIN = center;
    rightCenterSensorPIN = rightCenter;
    rightSensorPIN = right;
    pinMode(leftSensorPIN, INPUT);
    pinMode(leftCenterSensorPIN, INPUT);
    pinMode(centerSensorPIN, INPUT);
    pinMode(rightCenterSensorPIN, INPUT);
    pinMode(rightSensorPIN, INPUT);
  }

   void LineValueAnalyzer()
  {
     if (!noDetection())
      lastGlobalLineValue = globalLineValue;
    leftValue = !digitalRead(leftSensorPIN);
    leftCenterValue = !digitalRead(leftCenterSensorPIN);
    centerValue = !digitalRead(centerSensorPIN);
    rightCenterValue = !digitalRead(rightCenterSensorPIN);
    rightValue = !digitalRead(rightSensorPIN);
    lineOutput = (String)leftValue + (String)leftCenterValue + (String)centerValue + (String)rightCenterValue + (String)rightValue;
    //Serial.println(lineOutput);
    if (lineOutput == "10000")
    {
      globalLineValue = -6;//-5
    }
    else if (lineOutput == "11000")
    {
      globalLineValue = -5;//-3
    }
    else if (lineOutput == "01000")
    {
      globalLineValue = -2*nearLineModifier;
    }
    else if (lineOutput == "01100")
    {
      globalLineValue = -1*nearLineModifier;
    }
    else if (lineOutput == "00100")
    {
      globalLineValue = 0;
    }
    else if (lineOutput == "00110")
    {
      globalLineValue = 1*nearLineModifier;
    }
    else if (lineOutput == "00010")
    {
      globalLineValue = 2*nearLineModifier;
    }
    else if (lineOutput == "00011")
    {
      globalLineValue = 5;//3
    }
    else if (lineOutput == "00001")
    {
      globalLineValue = 6;//5
    }
    else if (lineOutput == "11100" || lineOutput == "11110"||lineOutput=="10100"||lineOutput=="10010")
    {
      globalLineValue = -6;
    }
    else if (lineOutput == "00111" || lineOutput == "01111"||lineOutput=="00101"||lineOutput=="01001")
    {
      globalLineValue = 6;
    }
    else if (lineOutput == "11111" || lineOutput == "01110"  || lineOutput=="01010")
    {
      globalLineValue = lastGlobalLineValue;
    }

   
  } 

  /* void LineValueAnalyzer(){
    leftValue = digitalRead(leftSensorPIN);
    leftCenterValue = digitalRead(leftCenterSensorPIN);
    centerValue = digitalRead(centerSensorPIN);
    rightCenterValue = digitalRead(rightCenterSensorPIN);
    rightValue = digitalRead(rightSensorPIN);
    globalLineValue = (double)((3)*leftValue+(1)*leftCenterValue+(-1)*rightCenterValue+(-3)*rightValue);
    if(!noDetection())
    lastGlobalLineValue = globalLineValue;
  } 
 */
  bool noDetection()
  {
    leftValue = digitalRead(leftSensorPIN);
    leftCenterValue = digitalRead(leftCenterSensorPIN);
    centerValue = digitalRead(centerSensorPIN);
    rightCenterValue = digitalRead(rightCenterSensorPIN);
    rightValue = digitalRead(rightSensorPIN);
    if (leftValue == 1 && leftCenterValue == 1 && centerValue == 1 && rightCenterValue == 1 && rightValue == 1)
      return 1;
    else
      return 0;
  }

  int returnLineValue()
  {
    if (noDetection())
    {
      Serial.println(lastGlobalLineValue);
      return lastGlobalLineValue;
    }
    else
    {
      Serial.println(globalLineValue);
      return globalLineValue;
    }
  }

  void SerialLineAnalyzer(String command)
  {
    LineValueAnalyzer();

    if (command == "every sensor")
    {
      Serial.print(leftValue);
      Serial.print(" ");
      Serial.print(leftCenterValue);
      Serial.print(" ");
      Serial.print(centerValue);
      Serial.print(" ");
      Serial.print(rightCenterValue);
      Serial.print(" ");
      Serial.print(rightValue);
      Serial.print("\n");
    }
    if (command == "all")
    {
      Serial.print(leftValue);
      Serial.print(" ");
      Serial.print(leftCenterValue);
      Serial.print(" ");
      Serial.print(centerValue);
      Serial.print(" ");
      Serial.print(rightCenterValue);
      Serial.print(" ");
      Serial.print(rightValue);
      Serial.print("       ");
      Serial.print(globalLineValue);
      Serial.print(" ");
    }
    if (command == "value")
    {
      Serial.print(globalLineValue);
      Serial.print("\n");
    }
  }
};

lineSensor lineSensorModule;

void setup()
{
  Serial.begin(9600);
  Right.setOrientation(0);
  Left.setOrientation(1);
  Right.setPins(11, 7, 9);
  Left.setPins(12, 13, 10);
  lineSensorModule.setPins(2, 3, 4, 5, 6);
}

double output;

double PID(int input)
{

  error = error * 0.4 + input * 0.6;
  errorDiff = error - lastError;
  errorInt = constrain(error + errorInt, -20, 20);
  output = KP * error + KD * errorDiff + KI * errorInt;
  lastError = error;

  return output;
}

void loop()
{
  lineSensorModule.LineValueAnalyzer();
  //Serial.println("LEFT   Internal Power : "+(String)Left.internalPower+"  Power:"+(String)Left.mappedPower +
  //               "     RIGHT   Internal Power : "+(String)Right.internalPower+"  Power:"+(String)Right.mappedPower);
  Left.setPower(baseSpeed - PID(lineSensorModule.returnLineValue()));
  Right.setPower(baseSpeed + PID(lineSensorModule.returnLineValue()));
}
