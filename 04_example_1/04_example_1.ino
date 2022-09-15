#define PIN_LED 13

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_LED, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  // digitalWrite(PIN_LED, 0);
  digitalWrite(PIN_LED, 1);
}
