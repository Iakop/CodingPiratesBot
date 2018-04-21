// Author: Jacob Bechmann Pedersen
// 2018-04-11
// GNU GPL v2

// Purpose:
// To give our BetterBoi a model, by which to reaact to the opposing robots.
// The flowchart, that illustrates the FSM is as follows:
//                         _____
//                        |     |
//                        |Start|
//                        |_____|
//                           |
//                      OnButtonPress
//                           |
//                        ___v__
//                       |      |
//                       | Idle |
//                       |______|
//                           |
//                    On5SecondsPassed
//   ______                  |
//  |  L/R |<--OnRight--|____v___
//  | Turn |<--OnLeft---|        |
//  |______|- --OnFind--| Search |<----
//           |          |________|     |
//        ___v___            |         |
//       |       |         OnEdge   OnSafe
//       | Chase |           |         |
//       |_______|       ____v___      |
//           |          |        |     |
//         OnEdge------>| BackUp |-----     
//                      |________|
//
// Method:
// In order to keep track of the states in the FSM, two enums are generated:
// One for states and one for events.
// 
// The sensors are updated internally in the states, so irrelevant sensor input is ignored on a statewise basis.
// Whenever a sensor meets certain set requirements, the event fires, and the state is updated accordingly.
//
// Then the code specific to run in that state runs, until the requirements are met for an event to set
// The machine out of the state.

// Let's first include all the relevant libraries:
#include <NewPing.h> // For the ultrasonic sensors
#include <CodingPirates.h> // For the Coding Pirates motor


// Ultrasound sensor _______________________________________________________________________________________

// Let's define all the important pins:
#define TRIGGER_PIN U1_TRIG
#define ECHO_PIN U1_ECHO

// And for the left:
#define TRIGGER_PIN_LEFT U3_TRIG
#define ECHO_PIN_LEFT U3_ECHO
// And right:
#define TRIGGER_PIN_RIGHT U2_TRIG
#define ECHO_PIN_RIGHT U2_ECHO

// The maximum distance for the ultrasonic sensor
#define MAX_DISTANCE 100
// And the threshold for closeness:
#define SONAR_THRESHOLD 50

// Then we instantiate a new object of type NewPing:
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
// And two more for left and right sensing:
NewPing sonarLeft(TRIGGER_PIN_LEFT, ECHO_PIN_LEFT, MAX_DISTANCE);
NewPing sonarRight(TRIGGER_PIN_RIGHT, ECHO_PIN_RIGHT, MAX_DISTANCE);

// A variable to hold the read values:
int pingRead = 0;
int pingReadLeft = 0;
int pingReadRight = 0;

// And the update function:
int sonarUpdate(void){
  pingRead = sonar.ping_median(3) / US_ROUNDTRIP_CM; // The calculation done in floating point arithmetic.
  Serial.print("pingRead: ");
  Serial.println(pingRead);
  return pingRead;
}
// Plus left/right variants:
int sonarUpdateLeft(void){
  pingReadLeft = sonarLeft.ping_median(3) / US_ROUNDTRIP_CM; // The calculation done in floating point arithmetic.
  Serial.print("pingReadLeft: ");
  Serial.println(pingReadLeft);
  return pingReadLeft;
}
int sonarUpdateRight(void){
  pingReadRight = sonarRight.ping_median(3) / US_ROUNDTRIP_CM; // The calculation done in floating point arithmetic.
  Serial.print("pingReadRight: ");
  Serial.println(pingReadRight);
  return pingReadRight;
}

// IR Reflection Sensors _____________________________________________________________________________________________

// enum to make the front, back left, and back right IR triggers explicit:
enum IRTrigger { front_trigd, back_L_trigd, back_R_trigd };

enum IRTrigger trigger;

// Firstly, some important pins:
IRsensor front(FR);
IRsensor backL(BR);
IRsensor backR(BL);
#define IR_THRESH 512  // The value that analogRead must exceed to trigger a response.

void IRInit(void){
  pinMode(FR, INPUT);
  pinMode(BL, INPUT);
  pinMode(BR, INPUT);
}

// And the update function:
int IRUpdate(void){
  if(front.read() == false){
    return 0;
  }
  else if(backL.read() == false){
    return 1;
  }
  else if(backR.read() == false){
    return 2;
  }
  return -1;
}


// Motors ____________________________________________________________________________________________

// Motor pins are initialized by the CodingPirates library using motorInit().

// Constants to set the motor speeds:
#define REVERSE_SPEED     255 
#define TURN_SPEED        200
#define FORWARD_SPEED     255
#define SEARCH_SPEED      200
// Duration of backing, and turn:
#define REVERSE_DURATION  250 // ms
#define TURN_DURATION     500 // ms

// The time to wait in the beginning.
#define IDLETIME 3000

// State Machine ____________________________________________________________________________________________

// First, the states and events are enumerated:
enum States { Start, Idle, Search, Chase, BackUp, TurnLeft, TurnRight, FindBorder };
enum Events { None, OnButtonPress, On5SecondsPassed, OnFind, OnEdge, OnSafe, OnLost, OnLeft, OnRight, OnBorderFind };

// Then the variables to hold the states themselves are defined:
static enum States zumoState = Start;
static enum Events zumoEvent = None;

// Prototypes of the state functiosn so they can be appended after setup() and loop()
void start(void);
void idle(void);
void search(void);
void chase(void);
void backUp(void);

// And finally the functions to handle the states:

// This function handles the event based state transitions:
void updateState(void){
  switch(zumoEvent){
    case None:
      break;
    case OnButtonPress:
      if(zumoState == Start){
        zumoState = Idle;
      }
      break;
    case On5SecondsPassed:
      if(zumoState == Idle){
        zumoState = FindBorder;
      }
      break;
    case OnFind:
      if(zumoState == Search){
        zumoState = Chase;
      }
      if(zumoState == TurnLeft){
        zumoState = Chase;
      }
      if(zumoState == TurnRight){
        zumoState = Chase;
      }
      break;
    case OnEdge:
      if(zumoState == Search){
        zumoState = BackUp;
      }
      if(zumoState == Chase){
        zumoState = BackUp;
      }
      if(zumoState == TurnLeft){
        zumoState = BackUp;
      }
      if(zumoState == TurnRight){
        zumoState = BackUp;
      }
      break;
    case OnSafe:
      if(zumoState == BackUp){
        zumoState = Search;
      }
      break;
    case OnLost:
      if(zumoState == Chase){
        zumoState = Search;
      }
      break;
    case OnLeft:
      if(zumoState == Search){
        zumoState = TurnLeft;
      }
      break;
    case OnRight:
      if(zumoState == Search){
        zumoState = TurnRight;
      }
      break;
    case OnBorderFind:
      if(zumoState == FindBorder){
        zumoState = Search;
      }
      break;
    default:
      break;
  }
}

// This loops, and checks all the sensors, to update the events, and thus change states:
void updateEvent(void){
  
}

// This maps the states to the proper state-exclusives functionality:
void doState(void){
  switch (zumoState){
    case Start:
      start();
      break;
    case Idle:
      idle();
      break;
    case Search:
      search();
      break;
    case Chase:
      chase();
      break;
    case BackUp:
      backUp();
      break;
    case TurnLeft:
      turnLeft();
      break;
    case TurnRight:
      turnRight();
      break;
    case FindBorder:
      findBorder();
      break;
    default:
      break;
  }
}
// Timers ____________________________________________________________________________________________________

// Some of the states require timers, and some also require similar mechanisms to function properly:

// The five seconds wait in the idle state needs a flag to keep track of start, and firing:
bool startIdleTimer = false;
bool stopIdleTimer = false;
unsigned long long int idleTimer;

// Handles the idleTimer.
bool updateIdle(void){
  if (startIdleTimer && millis() >= idleTimer + IDLETIME && !stopIdleTimer){
    stopIdleTimer = true;
    return true;
  }
  else{
    return false;
  }
}

// Both idle and search need a timer to track their actions properly.
unsigned long long int otteTalsTimer;

// init() function ___________________________________________________________________________________________

// Initializes the components of the robot in the beginning of the setup() function.
void initialize(){
  IRInit();
  motorInit();
  zumoState = Start;
  zumoEvent = None;
  idleTimer = millis();
  otteTalsTimer = millis();
}

// setup() and loop() ________________________________________________________________________________________

void setup(){
  initialize();
  Serial.begin(115200);
  /*Serial.print("Event: ");
  Serial.println(zumoEvent);
  Serial.print("State: ");
  Serial.println(zumoState);*/
}

void loop(){
  updateEvent();
  /*Serial.print("Event: ");
  Serial.println(zumoEvent);*/
  updateState();
  /*Serial.print("State: ");
  Serial.println(zumoState);*/
  doState();
}

// The state functions _______________________________________________________________________________________

void start(void){
  zumoEvent = OnButtonPress;
}

void idle(void){
  // Starts the idle timer once, to count down the time.
  if(!stopIdleTimer && !startIdleTimer){
    idleTimer = millis();
    startIdleTimer = true;
  }
  // When both stopIdleTimer and startIdletimer turn true, the time has gone by,
  // and the updateIdle() function fires the new event.
  if(updateIdle()){
    zumoEvent = On5SecondsPassed;
    return;
  }
}

void search(void){
  // Set the speeds counter each other for left and right, to spin:
  /*if(otteTal() == -1){
      zumoEvent = OnEdge;
      return; // Instantly return, to not waste any time.
  }*/
  
  trigger = IRUpdate();
  // Update IR for edge detection, to fire OnEdge as fast as possible.
  if(trigger > -1){
    zumoEvent = OnEdge;
    return; // Instantly return, to not waste any time.
  }
  
  motorSpeeds(SEARCH_SPEED/2, SEARCH_SPEED);
  /*
  unsigned long long int millisRead = millis();  
  if (millisRead > otteTalsTimer && millisRead < otteTalsTimer + 1000){
    motors.setSpeeds(400 , 125);
  }
  else if (millisRead > otteTalsTimer + 1000 && millisRead < otteTalsTimer + 4000){
    motors.setSpeeds(125 , 400);
  }
  else if (millisRead > otteTalsTimer + 4000 && millisRead < otteTalsTimer + 6000){
    motors.setSpeeds(400 , 125);
  }
  else{
    otteTalsTimer = millis();
  }*/

  int sonarRead = sonarUpdate();
  int sonarReadLeft = sonarUpdateLeft();
  int sonarReadRight = sonarUpdateRight();
  // If no edge is seen, check for the opponent:
  if(sonarRead < SONAR_THRESHOLD && sonarRead > 0){
    motorSpeeds(0, 0);
    delay(50);
    zumoEvent = OnFind;
    return;
  }
  // Check for the opponent on left:
  else if(sonarReadLeft < SONAR_THRESHOLD && sonarReadLeft > 0){
    zumoEvent = OnLeft;
    return;
  } 
  // Check for the opponent on Right:
  else if(sonarReadRight < SONAR_THRESHOLD && sonarReadRight > 0){
    zumoEvent = OnRight;
    return;
  }
}

void chase(void){
  motorSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  
  trigger = IRUpdate();
  // Update IR for edge detection, to fire OnEdge as fast as possible.
  if(trigger > -1){
    zumoEvent = OnEdge;
    return; // Instantly return, to not waste any time.
  }
}

void backUp(void){

  if(trigger == front_trigd){
    motorSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motorSpeeds(-TURN_SPEED, TURN_SPEED);
    delay(TURN_DURATION);
  }
  if(trigger == back_L_trigd){
    motorSpeeds(FORWARD_SPEED, FORWARD_SPEED/2);
    delay(REVERSE_DURATION);
  }
  if(trigger == back_R_trigd){
    motorSpeeds(FORWARD_SPEED/2, FORWARD_SPEED);
    delay(REVERSE_DURATION);
  }
  
  zumoEvent = OnSafe;
}

void turnLeft(void){
  motorSpeeds(-TURN_SPEED, TURN_SPEED);
  // check for the opponent:
  
  trigger = IRUpdate();
  // Update IR for edge detection, to fire OnEdge as fast as possible.
  if(trigger > -1){
    zumoEvent = OnEdge;
    return; // Instantly return, to not waste any time.
  }
  
  int sonarRead = sonarUpdate();
  // If no edge is seen, check for the opponent:
  if(sonarRead < SONAR_THRESHOLD && sonarRead > 0){
    motorSpeeds(0, 0);
    delay(50);
    zumoEvent = OnFind;
    return;
  }
}

void turnRight(void){
  motorSpeeds(TURN_SPEED, -TURN_SPEED);
  // check for the opponent:
  
  trigger = IRUpdate();
  // Update IR for edge detection, to fire OnEdge as fast as possible.
  if(trigger > -1){
    zumoEvent = OnEdge;
    return; // Instantly return, to not waste any time.
  }

  int sonarRead = sonarUpdate();
  // If no edge is seen, check for the opponent:
  if(sonarRead < SONAR_THRESHOLD && sonarRead > 0){
    motorSpeeds(0, 0);
    delay(50);
    zumoEvent = OnFind;
    return;
  }
}

void findBorder(void){
  motorSpeeds(250, 250);
  while(IRUpdate() <= -1);
  motorSpeeds(-50, -50);
  delay(50);
  motorSpeeds(255, -255);
  delay(1000);
  zumoEvent = OnBorderFind;
}

int otteTal(void){
  trigger = IRUpdate();
  // Update IR for edge detection, to fire OnEdge as fast as possible.
  if(trigger > -1){
    zumoEvent = OnEdge;
    return; // Instantly return, to not waste any time.
  }

  unsigned long long int millisRead = millis();  
  if (millisRead > otteTalsTimer && millisRead < otteTalsTimer + 1000){
    motorSpeeds(255 , 200);
  }
  else if (millisRead > otteTalsTimer + 1000 && millisRead < otteTalsTimer + 4000){
    motorSpeeds(200 , 255);
  }
  else if (millisRead > otteTalsTimer + 4000 && millisRead < otteTalsTimer + 6000){
    motorSpeeds(255 , 200);
  }
  else{
    otteTalsTimer = millis();
  }
  return 0;
}

