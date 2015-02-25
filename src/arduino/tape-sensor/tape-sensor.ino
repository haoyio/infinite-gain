// tape sensor with moving average filter

#define LEFTPIN A0
#define CENTERPIN A1
#define RIGHTPIN A2

#define NVAL 10
#define LTHRLO 400
#define LTHRHI 750
#define CTHRLO 300
#define CTHRHI 500
#define RTHRLO 550
#define RTHRHI 850

static int lvals[NVAL];
static int ltot = 0;
static int lavg = 0;
static int lidx = 0;
static int cvals[NVAL];
static int ctot = 0;
static int cavg = 0;
static int cidx = 0;
static int rvals[NVAL];
static int rtot = 0;
static int ravg = 0;
static int ridx = 0;

static bool lout = false;
static bool cout = false;
static bool rout = false;

void setup() {
  Serial.begin(9600);
  pinMode(LEFTPIN, INPUT);
  pinMode(CENTERPIN, INPUT);
  pinMode(RIGHTPIN, INPUT);
  for (int iVal = 0; iVal < NVAL; iVal++) {
    lvals[iVal] = 0;
    cvals[iVal] = 0;
    rvals[iVal] = 0;
  }
}

void loop() {
  // left tape sensor
  ltot -= lvals[lidx];
  lvals[lidx] = analogRead(LEFTPIN);
  ltot += lvals[lidx];
  lidx++;
  lidx = lidx % NVAL;
  lavg = ltot / NVAL;
  
  // center tape sensor
  ctot -= cvals[cidx];
  cvals[cidx] = analogRead(CENTERPIN);
  ctot += cvals[cidx];
  cidx++;
  cidx = cidx % NVAL;
  cavg = ctot / NVAL;
  
  // right tape sensor
  rtot -= rvals[ridx];
  rvals[ridx] = analogRead(RIGHTPIN);
  rtot += rvals[ridx];
  ridx++;
  ridx = ridx % NVAL;
  ravg = rtot / NVAL;
  
  // convert to 0-1 output
  if (lavg < LTHRLO && lout == true) lout = false;
  if (cavg < CTHRLO && cout == true) cout = false;
  if (ravg < RTHRLO && rout == true) rout = false;
  if (lavg > LTHRHI && lout == false) lout = true;
  if (cavg > CTHRHI && cout == false) cout = true;
  if (ravg > RTHRHI && rout == false) rout = true;
  
  delay(1); // for stability
  
  Serial.print(lavg);
  Serial.print(": ");
  Serial.println(lout);
//  Serial.print(cavg);
//  Serial.print(": ");
//  Serial.println(cout);
//  Serial.print(ravg);
//  Serial.print(": ");
//  Serial.println(rout);
}
