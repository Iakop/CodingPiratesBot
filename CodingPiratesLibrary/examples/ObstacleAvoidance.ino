#include <CodingPirates.h>
#include <NewPing.h>

IRsensor frontR(FR);
IRsensor frontL(FL);
IRsensor backR(BR);
IRsensor backL(BL);

NewPing fSonar(U1_TRIG, U1_ECHO, 400);
NewPing lSonar(U2_TRIG, U2_ECHO, 400);
NewPing rSonar(U3_TRIG, U3_ECHO, 400);

unsigned int f_cm, l_cm, r_cm;

void setup() {
  motorInit();
  Serial.begin(9600);
}

void loop() {
  f_cm = fSonar.ping_cm();
  l_cm = lSonar.ping_cm();
  r_cm = rSonar.ping_cm();
  Serial.print("F = "); Serial.println(f_cm);
  Serial.print("L = "); Serial.println(l_cm);
  Serial.print("R = "); Serial.println(r_cm);

  if(f_cm <= 15 && f_cm != 0){
    Serial.println(" ----- F Trig'd ----- ");
    if(l_cm > r_cm){
      backward(100);
      delay(1000);
      turnLeft(120);
      delay(1000);
    }
    else{
      backward(100);
      delay(1000);
      turnRight(120);
      delay(1000);
    }
  }
  else if(l_cm <= 20 && l_cm != 0){
    motorSpeeds(150,0);
    Serial.println(" ----- L Trig'd ----- ");
  }
  else if(r_cm <= 20 && r_cm != 0){
    motorSpeeds(0,150);
    Serial.println(" ----- R Trig'd ----- ");
  }
  else{
    motorSpeeds(150,150);
  }
}
