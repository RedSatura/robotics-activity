#define leftSensor 2
#define centerSensor 3
#define rightSensor 4

//lights
#define redLight 12
#define blueLight 13

//right motor
#define ENA 11
#define IN1 10
#define IN2 9

//left motor
#define ENB 6
#define IN3 8
#define IN4 7

int speed = 255;
int turnSpeed1 = 255;
int turnSpeed2 = 50;
int currentState = 0;

enum carState {STOPPED, FORWARD, TURNING_LEFT, TURNING_RIGHT};

carState state = FORWARD;

void setup() {
  Serial.begin(9600);

  pinMode(leftSensor, INPUT);
  pinMode(centerSensor, INPUT);
  pinMode(rightSensor, INPUT);

  //motor
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {
  manageCarState();
}

void goLeft() {
  analogWrite(ENA, turnSpeed1);
  analogWrite(ENB, turnSpeed2);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void goRight() {
  analogWrite(ENA, turnSpeed2);
  analogWrite(ENB, turnSpeed1);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void goForward() {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopCar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void manageCarState() {
  bool leftSensorActive = digitalRead(leftSensor);
  bool centerSensorActive = digitalRead(centerSensor);
  bool rightSensorActive = digitalRead(rightSensor);

  if (!leftSensorActive && !centerSensorActive && !rightSensorActive) {
    switch (state) {
      case(FORWARD):
        goForward();
        break;
      case(TURNING_LEFT):
        goLeft();
        break;
      case(TURNING_RIGHT):
        goRight();
        break;
      default:
        stopCar();
    }
  } else if (!leftSensorActive && !centerSensorActive && rightSensorActive) {
    state = TURNING_RIGHT;
    goRight();
  } else if (!leftSensorActive && centerSensorActive && !rightSensorActive) {
    state = FORWARD;
    goForward();
  } else if (!leftSensorActive && centerSensorActive && rightSensorActive) {
    state = TURNING_RIGHT;
    goRight();
  } else if (leftSensorActive && !centerSensorActive && !rightSensorActive) {
    state = TURNING_LEFT;
    goLeft();
  } else if (leftSensorActive && !centerSensorActive && rightSensorActive) {
    state = FORWARD;
    goForward();
  } else if (leftSensorActive && centerSensorActive && !rightSensorActive) {
    state = TURNING_LEFT;
    goLeft();
  } else if (leftSensorActive && centerSensorActive && rightSensorActive) {
    state = STOPPED;
    stopCar();
  }
}