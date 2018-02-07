#include <NewPing.h>
#include <CodingPirates.h>

NewPing midt(U1_TRIG, U1_ECHO, 400);
NewPing hoe(U2_TRIG, U2_ECHO, 400);
NewPing ven(U3_TRIG, U3_ECHO, 400);

IRsensor forHoe(FR);
IRsensor forVen(FL);
IRsensor bagHoe(BR);
IRsensor bagVen(BL);

void setup() {
  pinMode(FR, INPUT);
  motorInit();
}

void loop() {
  float afstand = midt.ping_cm();
  
  if(afstand < 20 && afstand != 0){
      motorSpeeds(255, 255);
  }
  
  if(forHoe.read() == 0){
      motorSpeeds(75, 75);
  } else {
      motorSpeeds(-75, -100);
      delay(500);
      motorSpeeds(-100, -75);
      delay(500);
  }

}
