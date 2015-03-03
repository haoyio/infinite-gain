#include "Pulse.h"
#include <Timers.h>

// timing values
#define SPOT_REVERSE 1000
#define ACTIVE_BRAKE 100
#define SPOT_LEFT 1000
#define WAIT_BALL 6000
#define ROTATE 100
#define STOP 100

// tape sensor constants
#define LEFTPIN A0
#define RIGHTPIN A1

#define NVAL 3     // TODO: moving average filter not getting new inputs from tape
#define LTHRLO 800
#define LTHRHI 1000
#define RTHRLO 800
#define RTHRHI 1000

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

#define ML_HIGH 185        // TODO: mod this until bot goes straight
#define MR_HIGH 195        // TODO: mod this until bot goes straight
#define ML_REV 190
#define MR_REV 190

#define ML_VEER_FAST 170   // TODO: mod this until bot veers enough
#define MR_VEER_FAST 170   // TODO: mod this until bot veers enough
#define ML_VEER_SLOW 0   // TODO: mod this until bot veers enough
#define MR_VEER_SLOW 0   // TODO: mod this until bot veers enough
#define ML_STOP 0
#define MR_STOP 0
#define SPOT_FAST 190
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

// bump sensor variables
#define BUMPPINRL 12
#define BUMPPINRR 13
#define BUMPPINFR 4
#define BUMPPINFL 3

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

static bool spotReverse = false;

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
void find_BRB();
void bump_sensor();
void bump_sensor_init();
void active_brake();
void fwd_left();

// main
void setup() {
  Serial.begin(9600);
  tape_sensor_init();
  motor_init();
  bump_sensor_init();
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
    if (findingMidtape) find_mid_tape();
    else if (onTape) follow_tape();
    else Serial.println("Which SOB led us here?");
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
      TMRArd_InitTimer(MAIN_TIMER, SPOT_LEFT);
      midTapeFound = true;
    } else {
      move_back();
    }
  } else if (!(unsigned char)TMRArd_IsTimerExpired(MAIN_TIMER)) {
    fwd_left();
  } else if (lowerWallHit && midTapeFound && !bumped) {
    if (flbump == BUMP || frbump == BUMP) {
      bumped = true;
    } else {
      follow_tape();
    }
  } else if (bumped) {
    nbump++;
    if (nbump < 3) {
      stop();
      delay(WAIT_BALL);
      move_back();
      delay(500);
      bumped = false;
    } else {
      stop();
      Serial.println("Stopping the robot");
//      findingMidtape = false;
//      nbump = 0;
//      spot_reverse();
//      onTape = true;
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

void find_BRB() {
  if (brbFound) {
    if ((unsigned char)TMRArd_IsTimerExpired(MAIN_TIMER)) { 
      if (brbReversed) {
        findingBRB = false;
        facingBasket = true;
      } else {
        //TODO: find some way to determine whether you've reversed 
        //      and on tape
        spot_reverse();
      }
    }
  } else {
    follow_tape();
    if (flbump == BUMP || frbump == BUMP) {
      brbFound = true;
      TMRArd_InitTimer(MAIN_TIMER, TIME_INTERVAL);
    }
  }
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
