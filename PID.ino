#include <Servo.h>
#include <PID_v1.h>
#include <Adafruit_VL53L1X.h>
double Setpoint, Input, Output;
// Definizione pin e oggetti
#define IRQ_PIN 2
#define XSHUT_PIN 3
double Ki, Kp, Kd;
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);
float error;
const int servoPin = 9;
const int setposition = A0;  // Collega il potenziometro al pin analogico A0
const int set_p = A1;  // Collega il potenziometro al pin analogico A1
const int set_i = A2;  // Collega il potenziometro al pin analogico A2
const int set_d = A3;  // Collega il potenziometro al pin analogico A3
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
double valPosition, valSet_P, valSet_D, valSet_I;
Servo myservo;  // create servo object to control a servo
void setup() {
  pinMode(set_p, INPUT);
  pinMode(set_i, INPUT);
  pinMode(set_d, INPUT);
  pinMode(servoPin, OUTPUT);
  myservo.attach(servoPin);
  myservo.write(90); 
  Serial.begin(115200);
  while (!Serial) delay(10);
  Wire.begin();
  if (! vl53.begin(0x29, &Wire)) {
    Serial.print(F("Error on init of VL sensor: "));
    Serial.println(vl53.vl_status);
    while (1)       delay(10);
  }

  if (! vl53.startRanging()) {
    Serial.print(F("Couldn't start ranging: "));
    Serial.println(vl53.vl_status);
    while (1)       delay(10);
  }
  vl53.setTimingBudget(50);
  myPID.SetOutputLimits(20, 100);
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  valPosition = analogRead(setposition);     //     sensore set point
  valPosition = map(valPosition, 0, 1023, 0, 660);     // scale it for use with the servo (value between 0 and 180)
  //valPosition= 400;
  valSet_P=analogRead(set_p);
  valSet_P=map(valSet_P, 0, 1023, 0, 10);     // scale it for use with the servo (value between 0 and 180)
  //valSet_I=analogRead(set_i);
  valSet_I = (analogRead(set_i) - 0.0) / (1023.0 - 0.0) * (1.0 - 0.0) + 0.0;
  //valSet_I=map(valSet_I, 0, 1023, 0, 0.5);     // scale it for use with the servo (value between 0 and 180)
  //valSet_D=analogRead(set_d);
  //valSet_D=map(valSet_D, 0, 1023, 0, 6);     // scale it for use with the servo (value between 0 and 180)
  valSet_D = (analogRead(set_d) - 0.0) / (1023.0 - 0.0) * (4.0 - 0.0) + 0.0;
  int16_t distance;
  // Stampa i valori su Serial per il plotter con timestamp
  if (vl53.dataReady()) {
    // new measurement for the taking!
    distance = vl53.distance();
    if (distance == -1) {
      // something went wrong!
      Serial.print(F("Couldn't get distance: "));
      Serial.println(vl53.vl_status);
      return;
    }
    Serial.print(F("Distance: "));
    Serial.print(distance);
    Serial.println(" mm");

    // data is read out, time for another reading!
    vl53.clearInterrupt();
  }
  Setpoint=valPosition;
  Input=distance;
  Kp=valSet_P;
  Kd=valSet_D;
  Ki=valSet_I;
  myPID.SetTunings(Kp, Ki, Kd);
  error=Setpoint-Input;
  myPID.Compute();
  /*if(Output==0) {
    for (pos = 0; pos <= Output; pos += 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15 ms for the servo to reach the position
    }
  } else {
    for (pos = 0; pos <= Output; pos += 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15 ms for the servo to reach the position
    }
  }*/
  myservo.write(Output);
  Serial.print("Timestamp:");
  Serial.print(millis());
  Serial.print(",");
  Serial.print("Distance:");
  Serial.print(Input);
  Serial.print(",");
  Serial.print("output:");
  Serial.print(Output);
  Serial.print(",");
  Serial.print("Kp:");
  Serial.print(Kp);
  Serial.print(",");
  Serial.print("Ki:");
  Serial.print(Ki, 4);
  Serial.print(",");
  Serial.print("Kd:");
  Serial.print(Kd, 4);
  Serial.print(",");
  Serial.print("Setpoint:");
  Serial.println(Setpoint);
  Serial.print("Errore: ");
  Serial.print(error);
  
  //delay(100);
}


