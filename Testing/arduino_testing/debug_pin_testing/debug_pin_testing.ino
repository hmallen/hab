const int debugStatePin = A2;
const int debugHeaterPin = A1;

void setup() {
  pinMode(debugStatePin, INPUT_PULLUP);
  pinMode(debugHeaterPin, INPUT_PULLUP);

  Serial.begin(115200);
}

void loop() {
  bool state = digitalRead(debugStatePin);
  bool heater = digitalRead(debugHeaterPin);

  Serial.print(state);
  Serial.print("/");
  Serial.println(heater);

  delay(500);
}
