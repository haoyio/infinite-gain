#include "Pulse.h"
#include <Timers.h>

// tape sensor constants
#define LEFTPIN A0
#define RIGHTPIN A1

#define NVAL 10
#define LTHRLO 400
#define LTHRHI 750
#define RTHRLO 550
#define RTHRHI 850

#define TAPE false
#define NOTAPE true

// motor constants
#define INPUT_HIGH 1023
#define OUTPUT_HIGH 255
#define PERIOD 10000

#define ENABLEL 5
#define ENABLER 6
#define DIRECTIONR 9
#define DIRECTIONL 10

#define ML_HIGH 255  //TODO: mod this until bot goes straight
#define MR_HIGH 255  //TODO: mod this until bot goes straight
#define ML_VEER 220  //TODO: mod this until bot veers enough
#define MR_VEER 220  //TODO: mod this until bot veers enough
#define ML_STOP 0
#define MR_STOP 0

#define FWD_ML HIGH
#define FWD_MR LOW
#define REV_ML LOW
#define REV_MR HIGH

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
#define BUMP HIGH

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

// bumper module variables
static bool flbump = false;
static bool frbump = false;

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

// main
void setup() {
  Serial.begin(9600);
  tape_sensor_init();
  motor_init();
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

void loop() {
  tape_sensor();
  motor();
  dev_test();
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

  delay(1); // for moving average stability
}

void motor() {
  check_emergency();
  if (mode == AUTO) {
    if (findingMidtape) find_mid_tape();
    else if (findingBRB) find_BRB();
    else if (onTape) follow_tape();
    else Serial.println("Which SOB led us here?");
  }
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

void find_mid_tape() { // assumes bot is oriented to "lower wall"
  if (midTapeFound && lowerWallHit) {
    veer_right();
    // if () { //TODO: find some way to determine whether bot is on tape
    //   findingMidtape = false;
    //   findingBRB = true;
    // }
  } else if (midTapeFound && !lowerWallHit) {
    veer_left();
    // if () { //TODO: find some way to determine whether bot is on tape
    //   findingMidtape = false;
    //   findingBRB = true;
    // }
  } else if (lowerWallHit) {
    if (lowerWallReversed) {
      move_forward();
    } else {
      //TODO: find some way to determine whether you've reversed and
      //      facing midtape (roughly)
      spot_reverse();
    }
  } else {
    move_forward();
    if (lout == TAPE || rout == TAPE) midTapeFound = true;
    if (flbump == BUMP || frbump == BUMP) lowerWallHit = true;
  }
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
  analogWrite(ENABLEL, ML_VEER);
  analogWrite(ENABLER, MR_HIGH);
}

void veer_right() {
  digitalWrite(DIRECTIONL, FWD_ML);
  digitalWrite(DIRECTIONR, FWD_MR);
  analogWrite(ENABLEL, ML_HIGH);
  analogWrite(ENABLER, MR_VEER);
}

void stop() {
  digitalWrite(DIRECTIONL, FWD_ML);
  digitalWrite(DIRECTIONR, FWD_MR);
  analogWrite(ENABLEL, ML_STOP);
  analogWrite(ENABLER, MR_STOP);
}

void follow_tape() {
  // auto tape-following logic
  if (flbump == BUMP || frbump == BUMP) {
    mvmt = STOP;
  } else {
    if (lout == NOTAPE && rout == NOTAPE) mvmt = FWD;
    else if (lout == TAPE && rout == NOTAPE) mvmt = RIGHT;
    else if (lout == NOTAPE && rout == TAPE) mvmt = LEFT;
    else mvmt = STOP;
  }

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
  Serial.print(lout);
  Serial.print(": ");
  Serial.print(lavg);
  Serial.print(" | ");
  Serial.print(rout);
  Serial.print(": ");
  Serial.println(ravg);
}