#include <QTRSensors.h>


QTRSensors qtr;
const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];

//motors
int E1 = 10;
int M1 = 12;
int E2 = 11;
int M2 = 13;

// Motor speed when traveling straight
int M1Speed = 175;
int M2Speed = 175;

int maxSpeed = 255;

// Sensors sum used for stopping
int sumSensors = 0;

// Variables for PID calculation
float Kp = 0.07;
float Ki = 0.0008;
float Kd = 0.6;
int P;
int I;
int D;
int lastError = 0;

//Methods for setting speed of individual wheels
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


void printCalibrationValues() {
  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  delay(1000);
}


//This is the algorithm used for calculation
// Refer here : https://create.arduino.cc/projecthub/anova9347/line-follower-robot-with-pid-controller-cdedbd
void PID_control() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = 2000 - position;

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P*Kp + I*Ki + D*Kd;
  
  int motorspeeda = (M1Speed + motorspeed);
  int motorspeedb = (M2Speed - motorspeed);
  
  if (motorspeeda > maxSpeed) {
    motorspeeda = maxSpeed;
  }
  if (motorspeedb > maxSpeed) {
    motorspeedb = maxSpeed;
  }
  if (motorspeeda < 0) {
    motorspeeda = 0;
  }
  if (motorspeedb < 0) {
    motorspeedb = 0;
  } 
  Serial.print("M1 Speed: ");
  Serial.print(motorspeeda);
  Serial.print(" M2 Speed:");
  Serial.print(motorspeedb);
  
  setLeftMotor(motorspeeda);
  setRightMotor(motorspeedb);
}

void setup() {

  // configure the sensors
  configureSensors();

  //begin calibration
  calibrationMode();

  //output values to terminal
  printCalibrationValues();

  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
}


void loop() {
  uint16_t position = qtr.readLineBlack(sensorValues);

  PID_control();

  // print the sensor values where values are in range 0 - 1000
  sumSensors = 0;
  for (uint8_t i = 0; i < SensorCount; i++) {
    sumSensors += sensorValues[i];
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }

  if(sumSensors < 50) {
    setLeftMotor(0);
    setRightMotor(0);
  }
   
  Serial.print(" SumSensors: ");
  Serial.print(sumSensors);
  Serial.print(" Position: ");
  Serial.println(position);

}
