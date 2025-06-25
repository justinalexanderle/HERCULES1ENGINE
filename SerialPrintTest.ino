void setup() {
  Serial.begin(9600);
  while (!Serial);
}

float ch[8] = {0, 10, 20, 30, 40, 50, 60, 70};

void loop() {
  for (int i = 0; i < 8; i++) {
    ch[i] += (i + 1) * 0.1;
    if (ch[i] > 100) ch[i] = 0;
  }

  for (int i = 0; i < 8; i++) {
    Serial.print(ch[i], 2);
    if (i < 7) Serial.print(",");
  }
  Serial.println();
  delay(100);
}