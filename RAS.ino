#include <DualMAX14870MotorShield.h>
#include <QTRSensors.h>
#include <NewPing.h>
#include <PS2X_lib.h>

#define Black 700
#define White 300

#define echo 32
#define trig 30
#define echo1 24
#define trig1 22
#define echo2 28
#define trig2 26
#define echo3 34
#define trig3 36
#define echo4 38
#define trig4 40
// tout terrain
bool sens = false;
char etat = '0';
const byte dataPin = 2;
const byte commandPin = 3;
const byte attentionPin = 4;
const byte clockPin = 5;
PS2X ps2x;
int error1 = 0;
byte type = 0;
byte vibrate = 0;

float Kp = 0.05;
float Kd = 0.01;
int NUM_SENSORS = 8;
int TIMEOUT = 2500;

float MaxSpeed = 400;
float MaxSpeed1 = 400;
float BaseSpeed = 200;
float MinSpeed1 = 0;
float MinSpeed = 0;

NewPing UltraSon(trig, echo, 30);
NewPing UltraSon1(trig1, echo1, 30);
NewPing UltraSon2(trig2, echo2, 30);

unsigned int distance;
unsigned int distance1;
unsigned int distance2;
unsigned int distance3;
unsigned int distance4;
unsigned int Ddistance = 20;
unsigned int mesureDist();



#define LED 53

byte f = 0;

QTRSensorsRC qtrrc((unsigned char[]){
                     A2, A3, A4, A5, A6, A7, A8, A9 },
                   NUM_SENSORS, TIMEOUT, QTR_NO_EMITTER_PIN);


unsigned int SensorValues[8];
unsigned int sensors[8];

long timer;
long timer1;
long timer2;
bool controller = false;
DualMAX14870MotorShield motors;

void setup() {
  Serial.begin(9600);

  ps2x.config_gamepad(clockPin, commandPin, attentionPin, dataPin, true, true);
  //capteur avant
  pinMode(trig, OUTPUT);
  digitalWrite(trig, LOW);
  pinMode(echo, INPUT);

  //capteur gauche1
  pinMode(trig1, OUTPUT);
  digitalWrite(trig1, LOW);
  pinMode(echo1, INPUT);

  //capteur gauche2
  pinMode(trig2, OUTPUT);
  digitalWrite(trig2, LOW);
  pinMode(echo2, INPUT);

  //capteur arriere1
  pinMode(trig3, OUTPUT);
  digitalWrite(trig3, LOW);
  pinMode(echo3, INPUT);

  //capteur arriere2
  pinMode(trig4, OUTPUT);
  digitalWrite(trig4, LOW);
  pinMode(echo4, INPUT);

  pinMode(52, INPUT_PULLUP);
  pinMode(53, OUTPUT);

  for (int i = 0; i < 140; i++) {
    qtrrc.calibrate();
    delay(20);
  }


  while (!digitalRead(52)) {
    digitalWrite(LED, HIGH);
  }
  digitalWrite(LED, LOW);
  wait(100);
}

void loop() {
  while (controller == false) {
    
    distance = calculeDistancef();
    distance1 = calculeDistanceg1();
    distance2 = calculeDistanceg2();
    distance3 = calculeDistancea1();
    distance4 = calculeDistancea2();

    int position = qtrrc.readLine(sensors);

    int error, lastError;

    error = -position + 3500;
    int motorSpeed = Kp * error + Kd * (error - lastError);
    lastError = error;

    int rightMotorSpeed = BaseSpeed + motorSpeed;
    int leftMotorSpeed = BaseSpeed - motorSpeed;

    set_motors(rightMotorSpeed, leftMotorSpeed);
    // distance frontale
    if ( distance <= 20 && distance >= 5)
    {
      BaseSpeed = BaseSpeed - ((20 - distance) * 13);
    }
    if ( distance < 5 )
    {
      BaseSpeed = 0;
      wait(500);
    }
      // black
      if (sensors[1] > Black && sensors[2] > Black && sensors[3] > Black && sensors[4] > Black && sensors[5] > Black
          && sensors[6] > Black && sensors[7] > Black) {
        f++;
        avance(350, 100);
      }
    if (ps2x.Button(PSB_R1)) {
      controller = true;
    }
  }
  while (controller == true) {
    distance = calculeDistancef();
    distance1 = calculeDistanceg1();
    distance2 = calculeDistanceg2();
    distance3 = calculeDistancea1();
    distance4 = calculeDistancea2();

    ps2x.read_gamepad(false, vibrate);

    if (ps2x.Button(PSB_L1)) {
      controller = false;
    }

    if (ps2x.Button(PSB_L2)) {
      recule1(100);
    }
    if (ps2x.Button(PSB_R2)) {
      avance1(100);
    }
    if (ps2x.Button(PSB_PAD_RIGHT)) {
      droite1(100);
    }
    if (ps2x.Button(PSB_PAD_LEFT)) {
      gauche1(100);
    }
  }
}
void set_motors(int leftMotorSpeed, int rightMotorSpeed) {
  if (leftMotorSpeed > MaxSpeed) leftMotorSpeed = MaxSpeed1;
  if (rightMotorSpeed > MaxSpeed) rightMotorSpeed = MaxSpeed1;
  if (leftMotorSpeed < MinSpeed) leftMotorSpeed = MinSpeed1;    // keep motor above 0
  if (rightMotorSpeed < MinSpeed) rightMotorSpeed = MinSpeed1;  // keep motor speed above 0
  motors.setM1Speed(rightMotorSpeed);
  motors.setM2Speed(leftMotorSpeed);
}
void avance(unsigned int vitesse, int temps) {

  motors.setM1Speed(vitesse);
  motors.setM2Speed(vitesse);

  delay(temps);
}

void wait(int temps) {
  motors.setM1Speed(0);
  motors.setM2Speed(0);
  delay(temps);
}
void recule(unsigned int vitesse, int temps) {

  motors.setM1Speed(-vitesse);
  motors.setM2Speed(-vitesse);

  delay(temps);
}
void gauche(unsigned int vitesse, int temps) {


  motors.setM1Speed(vitesse);
  motors.setM2Speed(-vitesse);

  delay(temps);
}
void avance1(unsigned int vitesse) {

  motors.setM1Speed(vitesse);
  motors.setM2Speed(vitesse);
}

void recule1(unsigned int vitesse) {

  motors.setM1Speed(-vitesse);
  motors.setM2Speed(-vitesse);
}
void gauche1(unsigned int vitesse) {


  motors.setM1Speed(vitesse);
  motors.setM2Speed(-vitesse);
}
void droite1(unsigned int vitesse) {

  motors.setM1Speed(-vitesse);
  motors.setM2Speed(vitesse);
}
void gauchea(unsigned int vitesse1, int temps1, unsigned int vitesse, int temps) {

  motors.setM1Speed(vitesse1);
  motors.setM2Speed(vitesse1);
  delay(temps1);
  wait(50);
  motors.setM1Speed(vitesse);
  motors.setM2Speed(0);

  delay(temps);
}
void droite(unsigned int vitesse, int temps) {

  motors.setM1Speed(-vitesse);
  motors.setM2Speed(vitesse);

  delay(temps);
}
void droitea(unsigned int vitesse1, int temps1, unsigned int vitesse, int temps) {
  motors.setM1Speed(vitesse1);
  motors.setM2Speed(vitesse1);
  delay(temps1);
  wait(50);
  motors.setM1Speed(0);
  motors.setM2Speed(vitesse);

  delay(temps);
}
unsigned int calculeDistancef() {
  int distanceA1 = UltraSon1.ping_cm();
  delay(20);
  int distanceA2 = UltraSon1.ping_cm();
  if (abs(distanceA1 - distanceA2) <= 2) {
    return distanceA1 == 0 ? 30 : distanceA1;
  } else {
    delay(20);
    int distanceA3 = UltraSon1.ping_cm();
    distanceA3 == 0 ? 30 : distanceA3;
    if (abs(distanceA3 - distanceA1) <= abs(distanceA3 - distanceA2)) {
      return distanceA1 == 0 ? 30 : distanceA1;
    } else {
      return distanceA2 == 0 ? 30 : distanceA2;
    }
  }
}
unsigned int calculeDistanceg1() {
  int distanceA1 = UltraSon1.ping_cm();
  delay(20);
  int distanceA2 = UltraSon1.ping_cm();
  if (abs(distanceA1 - distanceA2) <= 2) {
    return distanceA1 == 0 ? 30 : distanceA1;
  } else {
    delay(20);
    int distanceA3 = UltraSon1.ping_cm();
    distanceA3 == 0 ? 30 : distanceA3;
    if (abs(distanceA3 - distanceA1) <= abs(distanceA3 - distanceA2)) {
      return distanceA1 == 0 ? 30 : distanceA1;
    } else {
      return distanceA2 == 0 ? 30 : distanceA2;
    }
  }
}
unsigned int calculeDistanceg2() {
  int distanceA1 = UltraSon1.ping_cm();
  delay(20);
  int distanceA2 = UltraSon1.ping_cm();
  if (abs(distanceA1 - distanceA2) <= 2) {
    return distanceA1 == 0 ? 30 : distanceA1;
  } else {
    delay(20);
    int distanceA3 = UltraSon1.ping_cm();
    distanceA3 == 0 ? 30 : distanceA3;
    if (abs(distanceA3 - distanceA1) <= abs(distanceA3 - distanceA2)) {
      return distanceA1 == 0 ? 30 : distanceA1;
    } else {
      return distanceA2 == 0 ? 30 : distanceA2;
    }
  }
}
unsigned int calculeDistancea1() {
  int distanceA1 = UltraSon1.ping_cm();
  delay(20);
  int distanceA2 = UltraSon1.ping_cm();
  if (abs(distanceA1 - distanceA2) <= 2) {
    return distanceA1 == 0 ? 30 : distanceA1;
  } else {
    delay(20);
    int distanceA3 = UltraSon1.ping_cm();
    distanceA3 == 0 ? 30 : distanceA3;
    if (abs(distanceA3 - distanceA1) <= abs(distanceA3 - distanceA2)) {
      return distanceA1 == 0 ? 30 : distanceA1;
    } else {
      return distanceA2 == 0 ? 30 : distanceA2;
    }
  }
}
unsigned int calculeDistancea2() {
  int distanceA1 = UltraSon1.ping_cm();
  delay(20);
  int distanceA2 = UltraSon1.ping_cm();
  if (abs(distanceA1 - distanceA2) <= 2) {
    return distanceA1 == 0 ? 30 : distanceA1;
  } else {
    delay(20);
    int distanceA3 = UltraSon1.ping_cm();
    distanceA3 == 0 ? 30 : distanceA3;
    if (abs(distanceA3 - distanceA1) <= abs(distanceA3 - distanceA2)) {
      return distanceA1 == 0 ? 30 : distanceA1;
    } else {
      return distanceA2 == 0 ? 30 : distanceA2;
    }
  }
}