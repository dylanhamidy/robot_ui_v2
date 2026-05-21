// Pin Assignments
const int stepPin = 10;   // PUL+ on DM542
const int dirPin = 12;    // DIR+ on DM542
const int enPin = 11;     // ENA+ on DM542

bool enabled = true;
String serialBuffer = "";

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);

  // 1. Enable the driver (t1: ENA must be ahead of DIR by at least 5us)
  // Note: DM542 ENA is often left unconnected (default ENABLED),
  // but if used, logic depends on NPN/PNP wiring.
  digitalWrite(enPin, LOW);
  delayMicroseconds(10);
  // Set Direction (CW)
  digitalWrite(dirPin, LOW);

  // 2. Lead time (t2: DIR must be ahead of PUL effective edge by 5us)
  delayMicroseconds(10);

  Serial.begin(9600);
}

int maxSpeed = 5;
int minSpeed = 60000;
int speedDelay = 50;

void processCommand(String cmd) {
  if (cmd == "ENABLE") {
    // t1: ENA ahead of DIR by >= 5us
    digitalWrite(enPin, LOW);
    delayMicroseconds(10);
    enabled = true;
  } else if (cmd == "DISABLE") {
    digitalWrite(enPin, HIGH);
    delayMicroseconds(10);
    enabled = false;
  } else if (cmd == "DIR:CW") {
    // t2: DIR must settle before next PUL edge
    digitalWrite(dirPin, LOW);
    delayMicroseconds(10);
  } else if (cmd == "DIR:CCW") {
    digitalWrite(dirPin, HIGH);
    delayMicroseconds(10);
  } else if (cmd.startsWith("SPEED:")) {
    int val = cmd.substring(6).toInt();
    if (val >= 3) speedDelay = val;  // enforce >= 2.5us per DM542A spec
  }
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        processCommand(serialBuffer);
        serialBuffer = "";
      }
    } else {
      serialBuffer += c;
    }
  }
}

void loop() {
  handleSerial();

  if (!enabled) return;

  for(int x = 0; x < 200; x++) {
    // 3. Pulse High (t3: Pulse width not less than 2.5us)
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(speedDelay); // Standard speed; must be >= 2.5us

    // 4. Pulse Low (t4: Low level width not less than 2.5us)
    digitalWrite(stepPin, LOW);
    delayMicroseconds(speedDelay);
  }

  handleSerial();
}