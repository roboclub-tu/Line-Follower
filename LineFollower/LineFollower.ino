#include <QTRSensors.h>

QTRSensors qtr;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

//motors
int E1 = 10;
int M1 = 12;
int E2 = 11;
int M2 = 13;

int SumLeft = 0;
int SumRight = 0;
int SumDifference = 0;
int Last = 0;

void setLeftMotor(int value){
   analogWrite(E1,value);
}

void setRightMotor(int value){
  analogWrite(E2, value);
}

void configureSensors() {
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]) {
    A0, A1, A2, A3, A4
  }, SensorCount);
  delay(500);
}

void calibrationMode() {
  pinMode(LED_BUILTIN, OUTPUT);

  // turn on Arduino's LED to indicate we are in calibration mode
  digitalWrite(LED_BUILTIN, HIGH);

  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  // turn off Arduino's LED to indicate we are through with calibration
  digitalWrite(LED_BUILTIN, LOW);
}

void setup() {
  // configure the sensors
  configureSensors();

  //begin calibration
  calibrationMode();
  
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
}

void loop() {
  uint16_t position = qtr.readLineBlack(sensorValues);

  SumLeft =(sensorValues[0] + sensorValues[1] + sensorValues[2]);
  SumRight = (sensorValues[2] + sensorValues[3] + sensorValues[4]);
  SumDifference = (SumLeft - SumRight);
  Serial.print(SumLeft);
  Serial.print(SumRight);
  Serial.print(SumDifference);

  if(abs(SumDifference) < 700){
    setLeftMotor(175);
    setRightMotor(175);
    Serial.print("Forward");
  }
}
