int minA0 = 1023, maxA0 = 0;
int minA2 = 1023, maxA2 = 0;
unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  int a0 = analogRead(A0);
  int a2 = analogRead(A2);

  if (a0 < minA0) minA0 = a0;
  if (a0 > maxA0) maxA0 = a0;
  if (a2 < minA2) minA2 = a2;
  if (a2 > maxA2) maxA2 = a2;

  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    Serial.print("A0=");
    Serial.print(a0);
    Serial.print("  MIN=");
    Serial.print(minA0);
    Serial.print("  MAX=");
    Serial.print(maxA0);
    Serial.print("    A2=");
    Serial.print(a2);
    Serial.print("  MIN=");
    Serial.print(minA2);
    Serial.print("  MAX=");
    Serial.println(maxA2);
  }
}
