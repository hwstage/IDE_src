#define PIN7 7

void setup() {
  pinMode(PIN7, OUTPUT);
}

void loop() {
  int cnt = 0;
  digitalWrite(PIN7, LOW);
  delay(1000);
  while(cnt<5) {
  
    digitalWrite(PIN7, HIGH);
    delay(100);
    digitalWrite(PIN7, LOW);
    delay(100);

  cnt += 1;
  }

  while(1) {
    digitalWrite(PIN7, HIGH);}
}
