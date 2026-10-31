#include <Arduino.h>

// --- L298N PIN MAPPING (Verified Working Layout) ---
const int IN1 = 21;
const int IN2 = 22;
const int IN3 = 25;
const int IN4 = 26;

// Bipolar full-step sequence for smooth H-bridge motion
const int stepSequence[4][4] = {
  {1, 0, 1, 0},
  {0, 1, 1, 0},
  {0, 1, 0, 1},
  {1, 0, 0, 1}
};

int currentStep = 0;
long currentMotorSteps = 0;
long targetMotorSteps = 0;

// Conversion factor: radians to motor steps (Based on 3:1 gear ratio & 200 steps/rev motor)
const float STEPS_PER_RADIAN = 95.55; 

// Status feedback timing (10 Hz reporting to main.py)
unsigned long lastStatusTime = 0;
const unsigned long STATUS_INTERVAL_MS = 100;

// Serial parsing buffer for binary protocol from main.py
byte rxBuffer[16];
int rxIndex = 0;

void setup() {
  Serial.begin(115200);

  // Initialize motor control pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Initialize Hall sensor pin (GPIO 32 for dart present)
  pinMode(32, INPUT_PULLUP);
}

void stepMotor(bool clockwise) {
  currentStep = clockwise ? (currentStep + 1) % 4 : (currentStep - 1 + 4) % 4;
  digitalWrite(IN1, stepSequence[currentStep][0]);
  digitalWrite(IN2, stepSequence[currentStep][1]);
  digitalWrite(IN3, stepSequence[currentStep][2]);
  digitalWrite(IN4, stepSequence[currentStep][3]);
}

void releaseCoils() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void loop() {
  // 1. Read binary packets sent from main.py (Protocol header: 0xA5)
  while (Serial.available() > 0) {
    byte b = Serial.read();

    if (rxIndex == 0) {
      if (b == 0xA5) {
        rxBuffer[rxIndex++] = b;
      }
    } else {
      rxBuffer[rxIndex++] = b;
      // Expected packet length: 0xA5 (1) + body (8) + checksum (1) = 10 bytes
      if (rxIndex >= 10) {
        // Validate checksum (XOR of body bytes 1 through 8)
        byte checksum = 0;
        for (int i = 1; i <= 8; i++) {
          checksum ^= rxBuffer[i];
        }

        if (checksum == rxBuffer[9]) {
          uint8_t type = rxBuffer[1];
          if (type == 0x01) {
            int16_t pan_mrad;
            int16_t tilt_mrad;
            memcpy(&pan_mrad, &rxBuffer[4], sizeof(int16_t));
            memcpy(&tilt_mrad, &rxBuffer[6], sizeof(int16_t));
            uint8_t flags = rxBuffer[8];

            bool valid = (flags & 0x01);

            if (valid) {
              // Convert target pan angle from milliradians to motor steps
              float pan_rad = pan_mrad / 1000.0;
              targetMotorSteps = (long)(pan_rad * STEPS_PER_RADIAN);
            }
          }
        }
        rxIndex = 0; // Reset buffer
      }
    }
  }

  // 2. Smooth non-blocking step movement towards target position
  if (currentMotorSteps != targetMotorSteps) {
    bool dir = (targetMotorSteps > currentMotorSteps);
    stepMotor(dir);
    if (dir) currentMotorSteps++;
    else currentMotorSteps--;
    delay(5); 
  } else {
    releaseCoils(); // Power down coils when idle to prevent heating
  }

  // 3. Send telemetry status packet back to main.py at ~10 Hz (Protocol header: 0x5A)[cite: 9]
  unsigned long now = millis();
  if (now - lastStatusTime >= STATUS_INTERVAL_MS) {
    lastStatusTime = now;

    // Check Hall sensor state (GPIO 32 LOW = dart present)
    bool dartPresent = (digitalRead(32) == LOW);

    uint8_t statusFlags = 0;
    if (dartPresent) statusFlags |= 0x01; // Bit 0: Dart Present
    statusFlags |= 0x02;                  // Bit 1: Sensor OK

    byte statusType = 0x02;
    byte checksum = statusType ^ statusFlags;

    Serial.write(0x5A);
    Serial.write(statusType);
    Serial.write(statusFlags);
    Serial.write(checksum);
  }
}