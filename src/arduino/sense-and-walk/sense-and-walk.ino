#include "Pulse.h"
#include <Timers.h>
#include <Servo.h> 
 
// servo stuff
Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
int pos = 0;    // variable to store the servo position 

// timing values
#define SPOT_REVERSE 1000
#define ACTIVE_BRAKE 100
#define FWD_LEFT 800
#define WAIT_BALL 3000
#define ROTATE 50
#define STOP 10
#define BACK_UP 30

// tape sensor constants
#define LEFTPIN A0
#define RIGHTPIN A1

#define NVAL 3     // TODO: moving average filter not getting new inputs from tape
#define LTHRLO 750
#define LTHRHI 900
#define RTHRLO 750
#define RTHRHI 900

#define TAPE false
#define NOTAPE true

// motor constants
#define INPUT_HIGH 1023
#define OUTPUT_HIGH 255
#define PERIOD 10000

#define ENABLEL 5
#define ENABLER 6
#define DIRECTIONR 11
#define DIRECTIONL 10

#define ML_HIGH 175        // TODO: mod this until bot goes straight
#define MR_HIGH 185        // TODO: mod this until bot goes straight
#define ML_REV 190
#define MR_REV 190

#define ML_VEER_FAST 195   // TODO: mod this until bot veers enough
#define MR_VEER_FAST 195   // TODO: mod this until bot veers enough
#define ML_VEER_SLOW 0   // TODO: mod this until bot veers enough
#define MR_VEER_SLOW 0   // TODO: mod this until bot veers enough
#define ML_STOP 0
#define MR_STOP 0
#define SPOT_FAST 255
#define SPOT_SLOW 120

#define FWD_ML LOW
#define FWD_MR HIGH
#define REV_ML HIGH
#define REV_MR LOW

#define MANUAL false
#define AUTO true

#define START_AUTO 'g'
#define STOP 's'
#define BACK 'b'
#define LEFT 'l'
#define RIGHT 'r'
#define FWD 'f'

#define MAIN_TIMER 0
#define ONE_SEC 1000
#define TIME_INTERVAL ONE_SEC
#define BUMP 1

// bump sensor constants
#define BUMPPINRL 12
#define BUMPPINRR 13
#define BUMPPINFR 4
#define BUMPPINFL 3

// servo constants
#define RELEASE_SHOT 0
#define READY_SHOT 180
#define SERVO_IN 9

// tape sensor module variables
static int lvals[NVAL];
static int ltot = 0;
static int lavg = 0;
static int lidx = 0;
static int rvals[NVAL];
static int rtot = 0;
static int ravg = 0;
static int ridx = 0;

static bool lout = false;
static bool rout = false;

// motor module variables
static char mvmt = 's';
static char input = 's';
static bool mode = AUTO;

static bool midTapeFound = false;
static bool lowerWallHit = false;
static bool brbFound = false;
static bool lowerWallReversed = false;
static bool brbReversed = false;

static bool findingMidtape = true;
static bool findingBRB = false;
static bool facingBasket = false;
static bool onTape = false;

static bool bumped = false;
static char prev_mvmt = 's';

static bool spotLefting = false;
static bool spotReversing = false;
static bool ballsReceived = false;
static bool firstLefted = false;

static bool firstBacked = false;
static bool firstReversed = false;
static bool shotMade = false;

// bumper module variables
static bool flbump = false;
static bool frbump = false;
static bool rlbump = false;
static bool rrbump = false;
static int nbump = 0;

// function prototypes
void tape_sensor_init();
void motor_init();
void tape_sensor();
void motor();
void check_emergency();
void follow_tape();
void dev_test();
void find_mid_tape();
void get_on_tape();
void move_forward();
void move_back();
void veer_left();
void veer_right();
void stop();
void spot_reverse();
void bump_sensor();
void bump_sensor_init();
void active_brake();
void fwd_left();
void get_on_tape();

// main
void setup() {
  Serial.begin(9600);
  tape_sensor_init();
  motor_init();
  bump_sensor_init();
  servo_init();
}

void tape_sensor_init() {
  pinMode(LEFTPIN, INPUT);
  pinMode(RIGHTPIN, INPUT);
  for (int iVal = 0; iVal < NVAL; iVal++) {
    lvals[iVal] = 0;
    rvals[iVal] = 0;
  }
}

void motor_init() {
  pinMode(ENABLEL, OUTPUT);
  pinMode(ENABLER, OUTPUT);
  pinMode(DIRECTIONL, OUTPUT);
  pinMode(DIRECTIONR, OUTPUT);
}

void bump_sensor_init() {
  pinMode(BUMPPINRL, INPUT);
  pinMode(BUMPPINRR, INPUT);
  pinMode(BUMPPINFR, INPUT);
  pinMode(BUMPPINFL, INPUT);
}

void servo_init() {
  myservo.attach(SERVO_IN);
  myservo.write(READY_SHOT); 
}

void loop() {
  bump_sensor();
  tape_sensor();
//  follow_tape(); // remove later after testing
  motor();
  dev_test();
}

void bump_sensor() {
  rlbump = digitalRead(BUMPPINRL);
  rrbump = digitalRead(BUMPPINRR);
  frbump = digitalRead(BUMPPINFR);
  flbump = digitalRead(BUMPPINFL);
}

void tape_sensor() {
  // left tape sensor
  ltot -= lvals[lidx];
  lvals[lidx] = analogRead(LEFTPIN);
  ltot += lvals[lidx];
  lidx++;
  lidx = lidx % NVAL;
  lavg = ltot / NVAL;

  // right tape sensor
  rtot -= rvals[ridx];
  rvals[ridx] = analogRead(RIGHTPIN);
  rtot += rvals[ridx];
  ridx++;
  ridx = ridx % NVAL;
  ravg = rtot / NVAL;

  // convert to 0-1 output (false = tape, true = notape)
  if (lavg < LTHRLO && lout == NOTAPE) lout = TAPE;
  if (ravg < RTHRLO && rout == NOTAPE) rout = TAPE;
  if (lavg > LTHRHI && lout == TAPE) lout = NOTAPE;
  if (ravg > RTHRHI && rout == TAPE) rout = NOTAPE;

  delayMicroseconds(1); // for moving average stability
}

void motor() {
  check_emergency();
  if (mode == AUTO) {
    if (findingMidtape) 
      find_mid_tape();
    else if (ballsReceived)
      get_on_tape();
    else if (onTape) {
      follow_tape();
      if (flbump == BUMP || frbump == BUMP) {
        stop();
        jump_shot();
        shotMade = true;
        onTape = false;
        spotReversing = true;
        firstReversed = false;
      }
    } else if (shotMade) {
      // spot reverse and get back onto tape
      if (spotReversing && !firstReversed) {
        spot_reverse();
        delay(SPOT_REVERSE);    
        firstReversed = true;
      } else if (spotReversing && firstReversed) {
        spot_reverse();
        if (lout == TAPE || rout == TAPE) {
          spotReversing = false;
        }
      } else {
        // back up a little bit more
        move_back();
        delay(BACK_UP);
        
        // get ready for tape following
        shotMade = false;
        ballsReceived = false;
        onTape = false;
        findingMidtape = true;

        // redefine parameters to use previous code
        lowerWallHit = true;
        midTapeFound = true;
        spotLefting = false;
        bumped = false;
      }
      
    }
  }
}

void jump_shot() {
  // for(pos = 0; pos < 180; pos += 1) {  // goes from 0 degrees to 180 degrees
  //   myservo.write(pos);              // tell servo to go to position in variable 'pos' 
  //   delay(15);                       // waits 15ms for the servo to reach the position 
  // }
  
  // for(pos = 80; pos>=1; pos-=1) {   // goes from 120 degrees to 0 degrees 
  //   myservo.write(pos);              // tell servo to go to position in variable 'pos' 
  //   delay(15);                       // waits 15ms for the servo to reach the position 
  // } 
  myservo.write(RELEASE_SHOT);
  delay(200);
  // Reset to all the way back
  myservo.write(READY_SHOT); // Put servo all the say back
}

void get_on_tape() {
  // back up a little bit
  if (!spotReversing && !firstBacked) {
    move_back();
    delay(BACK_UP);
    spotReversing = true;
    firstBacked = true;
  }
  
  // spot reverse until it hits a tape
  if (spotReversing && !firstReversed) {
    spot_reverse();
    delay(SPOT_REVERSE);    
    firstReversed = true;
  } else if (spotReversing && firstReversed) {
    spot_reverse();
    if (lout == TAPE || rout == TAPE) {
      spotReversing = false;
    }
  } else {
    // back up a little bit more
    move_back();
    delay(BACK_UP);
    
    // get ready for tape following
    ballsReceived = false;
    onTape = true;
  }
}

void find_mid_tape() { // assumes bot is oriented to "lower wall"
  if (!lowerWallHit) {
    if (flbump == BUMP || frbump == BUMP) {
      lowerWallHit = true;
    } else {
      move_forward();
    }
  } else if (lowerWallHit && !midTapeFound) {
    if (lout == TAPE || rout == TAPE) {
      move_back();
      delay(BACK_UP);
      spotLefting = true;
      midTapeFound = true;
    } else {
      move_back();
    }
  } else if (spotLefting) {
    fwd_left();
    if (!firstLefted) 
      delay(FWD_LEFT);
      firstLefted = true;
    if (lout == TAPE || rout == TAPE) {
      spotLefting = false;
    }
  } else if (lowerWallHit && midTapeFound && !bumped) {
    if (flbump == BUMP || frbump == BUMP) {
      bumped = true;
    } else {
      follow_tape();
    }
  } else if (bumped) {
    nbump++;
    if (nbump <= 3) {
      stop();
      delay(WAIT_BALL);
      move_back();
      delay(500);
      bumped = false;
      if (nbump == 3) 
        bumped = true;
    } else {
      findingMidtape = false;
      nbump = 0;
      ballsReceived = true;
    }
  } else {
    Serial.println("Problem with finding mid tape section..."); 
  }
}

void fwd_left() {
  digitalWrite(DIRECTIONL, FWD_ML);
  digitalWrite(DIRECTIONR, FWD_MR);
  analogWrite(ENABLEL, SPOT_SLOW);
  analogWrite(ENABLER, SPOT_FAST);
}

void spot_reverse() { // on the spot reverse
  digitalWrite(DIRECTIONL, FWD_ML);
  digitalWrite(DIRECTIONR, REV_MR);
  analogWrite(ENABLEL, ML_HIGH);
  analogWrite(ENABLER, MR_HIGH);
}

void check_emergency() {
  if (Serial.available()) {
    input = Serial.read();
    if (input == START_AUTO) {
      mode = AUTO;
    } else { // emergency override
      mvmt = STOP;
      mode = MANUAL;
    }
  }
}

void move_forward() {
  digitalWrite(DIRECTIONL, FWD_ML);
  digitalWrite(DIRECTIONR, FWD_MR);
  analogWrite(ENABLEL, ML_HIGH);
  analogWrite(ENABLER, MR_HIGH);
}

void move_back() {
  digitalWrite(DIRECTIONL, REV_ML);
  digitalWrite(DIRECTIONR, REV_MR);
  analogWrite(ENABLEL, ML_HIGH);
  analogWrite(ENABLER, MR_HIGH);
}

void veer_left() {
  digitalWrite(DIRECTIONL, FWD_ML);
  digitalWrite(DIRECTIONR, FWD_MR);
  analogWrite(ENABLEL, ML_VEER_SLOW);
  analogWrite(ENABLER, MR_VEER_FAST);
  delay(ROTATE);
  stop();
}

void veer_right() {
  digitalWrite(DIRECTIONL, FWD_ML);
  digitalWrite(DIRECTIONR, FWD_MR);
  analogWrite(ENABLEL, ML_VEER_FAST);
  analogWrite(ENABLER, MR_VEER_SLOW);
  delay(ROTATE);
  stop();
}

void stop() {
  digitalWrite(DIRECTIONL, FWD_ML);
  digitalWrite(DIRECTIONR, FWD_MR);
  analogWrite(ENABLEL, ML_STOP);
  analogWrite(ENABLER, MR_STOP);
  delayMicroseconds(STOP);
}

void active_brake() {
  digitalWrite(DIRECTIONL, REV_ML);
  digitalWrite(DIRECTIONR, REV_MR);
  analogWrite(ENABLEL, ML_REV);
  analogWrite(ENABLER, MR_REV);
  delay(ACTIVE_BRAKE);
}

void follow_tape() {
  // auto tape-following logic
  prev_mvmt = mvmt;
  if (flbump == BUMP || frbump == BUMP) {
    mvmt = STOP;
  } else {
    if (lout == NOTAPE && rout == NOTAPE || 
        lout == TAPE && rout == TAPE)
      mvmt = FWD;
    else if (lout == TAPE && rout == NOTAPE) 
      mvmt = LEFT;
    else if (lout == NOTAPE && rout == TAPE) 
      mvmt = RIGHT;
    else
      mvmt = STOP;
  }
  
  if (prev_mvmt == FWD && mvmt != FWD)
      active_brake();
  
  // execute movement
  switch (mvmt) {
    case FWD:
      move_forward();
      break;
    case BACK:
      move_back();
      break;
    case LEFT: // veer left
      veer_left();
      break;
    case RIGHT: // veer right
      veer_right();
      break;
    case STOP:
      stop();
      break;
    default: // emergency stop when error encountered
      Serial.println("Which SOB led us here?");
      stop();
  }
}

void dev_test() {
  Serial.print("FL bump: ");
  Serial.print(flbump);
  Serial.print(", ");
  Serial.print("FR bump: ");
  Serial.print(frbump);
  
  Serial.print(" | ");
  
  Serial.print("L tape: ");
  Serial.print(lout);
  Serial.print(": ");
  Serial.print(lavg);
  Serial.print(" | ");
  Serial.print("R tape: ");
  Serial.print(rout);
  Serial.print(": ");
  Serial.println(ravg);
}
