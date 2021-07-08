#include "GyverPID.h"
#include "GParsingStream.h"

GyverPID regulatorL(0.05, 3, 0.05, 1);
GyverPID regulatorR(0.05, 3, 0.05, 1);

#define PARSE_AMOUNT 2
int intData[PARSE_AMOUNT];

const byte sensorPinL = 2;
const byte sensorPinR = 4;

const byte leftForwardPin = 9;
const byte leftBackwardPin = 10;
const byte rightForwardPin = 5;
const byte rightBackwardPin = 6;

uint32_t timerSpeedL;
uint32_t timerSpeedR;
byte valuePrevL;
byte valuePrevR;
uint16_t countPrevL = 0;
uint16_t countPrevR = 0;
uint16_t countL = 0;
uint16_t countR = 0;

int numberOfPeriods = 5;
uint32_t timerResetL;
uint32_t timerResetR;
uint32_t periodReset = 20;

uint32_t timerTransmit;
uint32_t periodTransmit = 10;

int maxSpeed = 400;
int maxPwm = 250;

byte valueCurrentL;
byte valueCurrentR;

float frequencyL;
float frequencyR;
float frequencyFilteredL;
float frequencyFilteredR;

int pwmLF = 0;
int pwmLB = 0;
int pwmRF = 0;
int pwmRB = 0;

int directionL = 0;
int directionR = 0;

int setL = 0;
int setR = 0;

int measuredL = 0;
int measuredR = 0;

int controlL = 0;
int controlR = 0;

int pwmTreashsholdL = 70;
int pwmTreashsholdR = 70;

// медианный фильтр на 3 значения со своим буфером для левого энкодера
float medianFreqL(float newVal) {
  static float buf[3];
  static byte count = 0;
  buf[count] = newVal;
  if (++count >= 3) count = 0;
  float a = buf[0];
  float b = buf[1];
  float c = buf[2];
  float middle;
  if ((a <= b) && (a <= c)) {
    middle = (b <= c) ? b : c;
  } else {
    if ((b <= a) && (b <= c)) {
      middle = (a <= c) ? a : c;
    }
    else {
      middle = (a <= b) ? a : b;
    }
  }
  return middle;
}

// медианный фильтр на 3 значения со своим буфером для правого энкодера
float medianFreqR(float newVal) {
  static float buf[3];
  static byte count = 0;
  buf[count] = newVal;
  if (++count >= 3) count = 0;
  float a = buf[0];
  float b = buf[1];
  float c = buf[2];
  float middle;
  if ((a <= b) && (a <= c)) {
    middle = (b <= c) ? b : c;
  } else {
    if ((b <= a) && (b <= c)) {
      middle = (a <= c) ? a : c;
    }
    else {
      middle = (a <= b) ? a : b;
    }
  }
  return middle;
}

void setup() {
  pinMode(sensorPinL,INPUT);
  pinMode(sensorPinR,INPUT);
  pinMode(leftForwardPin, OUTPUT);
  pinMode(leftBackwardPin, OUTPUT);
  pinMode(rightForwardPin, OUTPUT);
  pinMode(rightBackwardPin, OUTPUT);
  valuePrevL = digitalRead(sensorPinL);
  valuePrevR = digitalRead(sensorPinR);
  Serial.begin(115200);
  timerSpeedL = micros();
  timerSpeedR = micros();
  timerTransmit = millis();
  timerResetL = millis();
  timerResetR = millis();
  regulatorL.setLimits(-maxPwm, maxPwm); 
  regulatorR.setLimits(-maxPwm, maxPwm); 
}

void loop() {
  
  parsingStream((int*)&intData);
  if (dataReady()) {
    setL = intData[0];
    setR = intData[1];
  }

  regulatorL.setpoint = setL;
  regulatorR.setpoint = setR;

  if (pwmLF > 0) {
    directionL = 1;
  } else if (pwmLB > 0) {
    directionL = -1;
  } else {
    directionL = 0;
  }

  if (pwmRF > 0) {
    directionR = 1;
  } else if (pwmRB > 0) {
    directionR = -1;
  } else {
    directionR = 0;
  }
  
  valueCurrentL = digitalRead(sensorPinL);
  if (valueCurrentL != valuePrevL) {
    timerResetL = millis();
    valuePrevL = valueCurrentL;
    if (valueCurrentL == HIGH) {
      countL++;
      // TODO: add global signed counters or ...?
      if (countL - countPrevL == numberOfPeriods) {
        countPrevL = countL;
        frequencyL = directionL * float(1000000) * numberOfPeriods / (micros() - timerSpeedL);
        frequencyFilteredL = medianFreqL(frequencyL);
        timerSpeedL = micros();
      }
    }
  }

  if ((millis() - timerResetL) > periodReset) {
    frequencyL = 0;
    frequencyFilteredL = 0;
  }

  valueCurrentR = digitalRead(sensorPinR);
  if (valueCurrentR != valuePrevR) {
    timerResetR = millis();
    valuePrevR = valueCurrentR;
    if (valueCurrentR == HIGH) {
      countR++;
      if (countR - countPrevR == numberOfPeriods) {
        countPrevR = countR;
        frequencyR = directionR * float(1000000) * numberOfPeriods / (micros() - timerSpeedR);
        frequencyFilteredR = medianFreqR(frequencyR);
        timerSpeedR = micros();

      }
    }
  }

  if ((millis() - timerResetR) > periodReset) {
    frequencyR = 0;
    frequencyFilteredR = 0;
  }
  
  measuredL = map(frequencyFilteredL, -maxSpeed, maxSpeed, -maxPwm, maxPwm);
  measuredR = map(frequencyFilteredR, -maxSpeed, maxSpeed, -maxPwm, maxPwm);

  regulatorL.input = measuredL;
  regulatorR.input = measuredR;

  controlL = regulatorL.getResultTimer();
  controlR = regulatorR.getResultTimer();

  if (controlL > pwmTreashsholdL) {
    pwmLF = controlL;
    pwmLB = 0;
  } else  if (controlL < -pwmTreashsholdL) {
    pwmLF = 0;
    pwmLB = -controlL;
  } else {
    pwmLF = 0;
    pwmLB = 0;
  }

  if (controlR > pwmTreashsholdR) {
    pwmRF = controlR;
    pwmRB = 0;
  } else  if (controlR < -pwmTreashsholdR) {
    pwmRF = 0;
    pwmRB = -controlR;
  } else {
    pwmRF = 0;
    pwmRB = 0;
  }
  
  analogWrite(leftForwardPin, pwmLF);
  analogWrite(leftBackwardPin, pwmLB);
  analogWrite(rightForwardPin, pwmRF);
  analogWrite(rightBackwardPin, pwmRB);

//  msg = String(measuredL) + ' ' + String(measuredR);

//  Serial.print(measuredL);
//  Serial.print(' ');
  Serial.println(String(measuredL) + ' ' + String(measuredR));
  
//  Serial.print(' ');
//  Serial.println(valueCurrentL * 100);
 
}
