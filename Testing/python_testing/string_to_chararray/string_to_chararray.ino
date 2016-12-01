void setup() {
  Serial.begin(115200);
}

void loop() {
  int txCount = 1;
  String txCountString = String(txCount);
  char txCountChar[6];
  txCountString.toCharArray(txCountChar, 6);
  
  Serial.println(txCount);
  Serial.println(txCountString);
  Serial.println(txCountChar);
  
  delay(2500);
  
  txCount = 1000;
  txCountString = String(txCount);
  //txCountChar[6];
  txCountString.toCharArray(txCountChar, 6);
  
  Serial.println(txCount);
  Serial.println(txCountString);
  Serial.println(txCountChar);
  
  delay(2500);
  
  txCount = 10000;
  txCountString = String(txCount);
  //txCountChar[6];
  txCountString.toCharArray(txCountChar, 6);
  
  Serial.println(txCount);
  Serial.println(txCountString);
  Serial.println(txCountChar);

  while (true) {
    ;
  }
}

