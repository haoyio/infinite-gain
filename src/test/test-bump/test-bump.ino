#define BUMPPIN1 A5
#define BUMPPIN2 8

void setup(){
  Serial.begin(9600);
  pinMode(BUMPPIN1, INPUT);
  pinMode(BUMPPIN2, INPUT);
}

void loop() {
  // Bump pin
  int bumpPin1 = digitalRead(BUMPPIN1);
  Serial.print("Bump Pin: ");
  Serial.println(bumpPin1); 
  
//  int bumpPin2 = digitalRead(BUMPPIN2);
//  Serial.print("Bump Pin: ");
//  Serial.println(bumpPin2); 
}
