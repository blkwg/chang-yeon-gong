#define PIN7 7

void setup() {
  pinMode(PIN7, OUTPUT);
  digitalWrite(PIN7, LOW);
  delay(1000);
  int counter = 0;  
  while(counter < 5) {
    digitalWrite(PIN7, HIGH);
    delay(200);
    digitalWrite(PIN7, LOW);
    counter++;
    delay(200);
  }
}

void loop() {
  digitalWrite(PIN7, HIGH);
}

