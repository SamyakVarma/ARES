const int botXPin = A0;
const int botYPin = A1;
const int turretXPin = A2;
const int turretYPin = A3;

const int botSwitchPin = 13;
const int turretSwitchPin = 12;

unsigned long lastConnectedSend = 0;
const unsigned long connectedInterval = 1000; // 1 second

void setup() {
  Serial.begin(115200);
  pinMode(botSwitchPin, INPUT_PULLUP);
  pinMode(turretSwitchPin, INPUT_PULLUP);
}

void loop() {
  unsigned long currentMillis = millis();

  // Read analog joystick positions
  int botX = analogRead(botXPin);
  int botY = analogRead(botYPin);
  int turretX = analogRead(turretXPin);
  int turretY = analogRead(turretYPin);

  // Read button states (inverted logic due to pull-up)
  bool botPressed = digitalRead(botSwitchPin) == LOW;
  bool turretPressed = digitalRead(turretSwitchPin) == LOW;

  // Heartbeat message
  if (currentMillis - lastConnectedSend > connectedInterval) {
    Serial.println("JOYSTICK_CONNECTED");
    lastConnectedSend = currentMillis;
  }

  // Scale analog values to -1.0 to +1.0
  float linear_x = (botY - 512) / 512.0;   // forward/backward
  float angular_z = (botX - 512) / 512.0;  // turning

  float aim_x = (turretX - 512) / 512.0;
  float aim_y = (turretY - 512) / 512.0;

  // Send JSON string to Flask
  Serial.print("{");
  Serial.print("\"linear_x\":"); Serial.print(linear_x, 3);
  Serial.print(",\"angular_z\":"); Serial.print(angular_z, 3);
  Serial.print(",\"aim_x\":"); Serial.print(aim_x, 3);
  Serial.print(",\"aim_y\":"); Serial.print(aim_y, 3);
  Serial.print(",\"fire\":"); Serial.print(turretPressed ? "true" : "false");
  Serial.println("}");

  delay(50); // ~20Hz
}
