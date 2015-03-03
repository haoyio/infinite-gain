// tape sensor with moving average filter

#define LEFTPIN A0
#define RIGHTPIN A1

#define NVAL 10
#define LTHRLO 500
#define LTHRHI 900
#define RTHRLO 500
#define RTHRHI 900

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

void setup() {
  Serial.begin(9600);
  pinMode(LEFTPIN, INPUT);
  pinMode(RIGHTPIN, INPUT);
  for (int iVal = 0; iVal < NVAL; iVal++) {
    lvals[iVal] = 0;
    rvals[iVal] = 0;
  }
  
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
}

void loop() {
  
  digitalWrite(5, 0);
  digitalWrite(6, 0);
  
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
  
  // convert to 0-1 output
  if (lavg < LTHRLO && lout == true) lout = false;
  if (ravg < RTHRLO && rout == true) rout = false;
  if (lavg > LTHRHI && lout == false) lout = true;
  if (ravg > RTHRHI && rout == false) rout = true;
  
//  delay(1);
  delayMicroseconds(1); // for stability
  
  Serial.print(lavg);
  Serial.print(": ");
  Serial.print(lout);
  Serial.print(" | ");
  Serial.print(ravg);
  Serial.print(": ");
  Serial.println(rout);
}
