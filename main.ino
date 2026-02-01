/*
 * Line Following Robot with Obstacle Avoidance
 * Arduino Uno R4
 *
 * ALGORITHM: Edge-following along RIGHT side of line
 * - Robot stays on WHITE, to the RIGHT of the black line
 * - On BLACK (hit line): turn LEFT (back to white on right)
 * - On WHITE (normal): curve RIGHT (toward line on our left)
 * - On OBSTACLE: detour around, then curve back to line
 *
 * PAUSE/RUN: Press RESET to toggle between paused and running
 * Robot must start on the BLACK line for auto-calibration
 */

#include <EEPROM.h>

// ==================== PIN DEFINITIONS ====================
// (Keep these the same)

// Motor A (Left motor)
const int IN1 = 8;
const int IN2 = 9;
const int ENA = 6;   // PWM

// Motor B (Right motor)
const int IN3 = 10;
const int IN4 = 11;
const int ENB = 5;   // PWM

// TCS3200 Color Sensor
const int CS_S0 = A2;
const int CS_S1 = A3;
const int CS_S2 = A4;
const int CS_S3 = A5;
const int CS_OUT = 12;

// IR Obstacle Sensor
const int IR_PIN = 2;

// ==================== TUNING PARAMETERS ====================
// Adjust these to tune the robot's behavior

// Motor speeds (0-255)
const int BASE_SPEED = 80;        // Base forward speed
const int TURN_REDUCTION = 30;    // How much to slow inner wheel when ON line (gentle)
const int RETURN_TURN_REDUCTION = 45; // Sharper turn when OFF line (returning to it)
const int RIGHT_MOTOR_BOOST = 10; // Compensation for weaker right motor

// Color detection
const int BLACK_TOLERANCE = 20;   // +/- from calibrated value counts as black

// Obstacle avoidance
const int DETOUR_PIVOT_MS = 400;  // How long to pivot right when obstacle seen
const int DETOUR_FORWARD_MS = 300; // How long to go forward after pivot
const int PIVOT_SPEED = 90;       // Speed during pivot turns

// Sharp turn handling
const unsigned long WHITE_ESCALATE_MS = 500; // If on white this long, escalate to pivot

// ==================== STATE ====================

bool isPaused = true;
int calibratedBlack = 0;
bool isCalibrated = false;
unsigned long whiteStartTime = 0;  // When we first went on white
bool wasOnBlack = true;            // Track transitions

// ==================== SETUP ====================

void setup() {
  Serial.begin(9600);
  delay(200);

  // Initialize all pins
  initMotors();
  initColorSensor();
  initIRSensor();

  // Ensure motors are stopped
  stopMotors();

  // Handle EEPROM pause/run toggle
  handlePauseToggle();

  // Print status
  Serial.println("====================");
  Serial.println("LINE FOLLOWER v2.0");
  Serial.println("====================");

  if (isPaused) {
    Serial.println("");
    Serial.println("STATUS: PAUSED");
    Serial.println("Press RESET to RUN");
    Serial.println("");
    return;
  }

  Serial.println("");
  Serial.println("STATUS: RUNNING");
  Serial.println("Press RESET to PAUSE");
  Serial.println("");

  // Auto-calibrate (robot must be on black line!)
  Serial.println("Place robot on BLACK LINE!");
  delay(1000);
  autoCalibrate();

  if (!isCalibrated) {
    Serial.println("Calibration FAILED!");
    isPaused = true;
    return;
  }

  Serial.println("");
  Serial.println("Starting in 2 seconds...");
  delay(2000);
  Serial.println("GO!");
}

// ==================== MAIN LOOP ====================

void loop() {
  // If paused or not calibrated, do nothing
  if (isPaused || !isCalibrated) {
    stopMotors();
    delay(100);
    return;
  }

  // ===== DEBUG MODE =====
  // Uncomment ONE of these lines to test components:
  // testSensor(); return;  // Just print sensor values (motors off)
  // testMotors(); return;  // Test motor directions

  // PRIORITY 1: Check for obstacle
  if (isObstacleDetected()) {
    handleObstacle();
    return;
  }

  // PRIORITY 2: Follow the line
  followLine();
}

// Test sensor without motors - move robot by hand to see values
void testSensor() {
  stopMotors();
  int val = readColorSensor();
  Serial.print("Sensor: ");
  Serial.print(val);
  Serial.print(" | Cal: ");
  Serial.print(calibratedBlack);
  Serial.print(" | ");
  Serial.println(isBlack(val) ? "BLACK" : "WHITE");
  delay(200);
}

// Test motors - should go: forward, left, right, stop
void testMotors() {
  Serial.println("Forward...");
  driveForward();
  delay(1000);

  Serial.println("Curve Left...");
  curveLeft();
  delay(1000);

  Serial.println("Curve Right...");
  curveRight();
  delay(1000);

  Serial.println("Stop.");
  stopMotors();
  delay(2000);
}

// ==================== LINE FOLLOWING ====================

void followLine() {
  int sensorValue = readColorSensor();
  bool onBlack = isBlack(sensorValue);

  // Track how long we've been on white (for sharp turn handling)
  if (onBlack) {
    wasOnBlack = true;
    whiteStartTime = 0;  // Reset white timer
  } else {
    if (wasOnBlack) {
      // Just transitioned to white
      whiteStartTime = millis();
      wasOnBlack = false;
    }
  }

  // Calculate time on white
  unsigned long timeOnWhite = 0;
  if (!onBlack && whiteStartTime > 0) {
    timeOnWhite = millis() - whiteStartTime;
  }

  // Debug output
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    Serial.print("Val:");
    Serial.print(sensorValue);
    Serial.print(" Cal:");
    Serial.print(calibratedBlack);
    Serial.print("±");
    Serial.print(BLACK_TOLERANCE);
    Serial.print(" -> ");
    if (onBlack) {
      Serial.println("BLACK(turn-L)");
    } else if (timeOnWhite > WHITE_ESCALATE_MS) {
      Serial.println("WHITE(PIVOT-R!)");
    } else {
      Serial.println("WHITE(curve-R)");
    }
    lastPrint = millis();
  }

  if (onBlack) {
    // Hit the line - turn LEFT (away from line, back to white on right)
    pivotRight();
  } else {
    // On white - turn toward the line (which is on our right)
    if (timeOnWhite > WHITE_ESCALATE_MS) {
      // Been on white too long (sharp turn?) - aggressive pivot to find line
      pivotLeft();
    } else {
      // Normal curve toward line
      curveLeft();
    }
  }
}

// ==================== OBSTACLE HANDLING ====================

void handleObstacle() {
  Serial.println("");
  Serial.println("*** OBSTACLE DETECTED ***");
  Serial.println("Taking detour...");

  // Stop
  stopMotors();
  delay(100);

  // Pivot to go around obstacle (cross over line temporarily)
  pivotRight();
  delay(DETOUR_PIVOT_MS);

  // Go forward to clear the obstacle area
  driveForward();
  delay(DETOUR_FORWARD_MS);

  // Resume normal line following
  // Robot will be on white, line is now to the right
  // curveLeft() will bring it back to the line
  Serial.println("Resuming line following...");
  Serial.println("");
}

// ==================== MOTOR FUNCTIONS ====================

void initMotors() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void driveForward() {
  // Both motors forward, same speed
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, BASE_SPEED);
  analogWrite(ENB, BASE_SPEED + RIGHT_MOTOR_BOOST);
}

void curveLeft() {
  // Both motors forward, left MUCH slower (sharp return to line)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, BASE_SPEED - RETURN_TURN_REDUCTION);
  analogWrite(ENB, BASE_SPEED + RIGHT_MOTOR_BOOST);
}

void curveRight() {
  // Both motors forward, right MUCH slower (sharp return to line)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, BASE_SPEED);
  analogWrite(ENB, BASE_SPEED - RETURN_TURN_REDUCTION + RIGHT_MOTOR_BOOST);
}

void sharpLeft() {
  // Left wheel STOPPED, right wheel forward (sharp but not spinning)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, BASE_SPEED + RIGHT_MOTOR_BOOST);
}

void pivotLeft() {
  // Left motor backward, right motor forward (spin in place)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, PIVOT_SPEED);
  analogWrite(ENB, PIVOT_SPEED + RIGHT_MOTOR_BOOST);
}

void pivotRight() {
  // Left motor forward, right motor backward (spin in place)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, PIVOT_SPEED);
  analogWrite(ENB, PIVOT_SPEED + RIGHT_MOTOR_BOOST);
}

// ==================== COLOR SENSOR ====================

void initColorSensor() {
  pinMode(CS_S0, OUTPUT);
  pinMode(CS_S1, OUTPUT);
  pinMode(CS_S2, OUTPUT);
  pinMode(CS_S3, OUTPUT);
  pinMode(CS_OUT, INPUT);

  // Set frequency scaling to 2% (most stable)
  digitalWrite(CS_S0, LOW);
  digitalWrite(CS_S1, HIGH);
}

int readColorSensor() {
  // Use clear channel (no color filter) for brightness
  digitalWrite(CS_S2, HIGH);
  digitalWrite(CS_S3, LOW);

  // Delay for filter to settle (needs ~10ms)
  delay(10);

  // Read pulse width (higher = darker)
  return pulseIn(CS_OUT, LOW, 50000);
}

bool isBlack(int value) {
  // If sensor returned 0 (timeout/error), treat as white (not on line)
  if (value == 0) {
    return false;
  }

  // Check if value is within tolerance of calibrated black
  int diff = abs(value - calibratedBlack);
  return (diff <= BLACK_TOLERANCE);
}

void autoCalibrate() {
  Serial.println("Calibrating...");

  long total = 0;
  int validCount = 0;

  // Take 10 samples
  for (int i = 0; i < 10; i++) {
    int reading = readColorSensor();
    if (reading > 0) {
      total += reading;
      validCount++;
      Serial.print("  Sample ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.println(reading);
    }
    delay(50);
  }

  if (validCount > 0) {
    calibratedBlack = total / validCount;
    isCalibrated = true;

    Serial.println("");
    Serial.print("Calibrated BLACK = ");
    Serial.println(calibratedBlack);
    Serial.print("Detection range: ");
    Serial.print(calibratedBlack - BLACK_TOLERANCE);
    Serial.print(" to ");
    Serial.println(calibratedBlack + BLACK_TOLERANCE);
  } else {
    Serial.println("ERROR: No valid sensor readings!");
    isCalibrated = false;
  }
}

// ==================== IR OBSTACLE SENSOR ====================

void initIRSensor() {
  pinMode(IR_PIN, INPUT);
}

bool isObstacleDetected() {
  // Most IR sensors: LOW = obstacle detected
  return (digitalRead(IR_PIN) == LOW);
}

// ==================== EEPROM PAUSE/RUN ====================

void handlePauseToggle() {
  // Read current state from EEPROM
  byte stored = EEPROM.read(0);

  // Toggle: 0 -> paused (write 1), anything else -> running (write 0)
  if (stored == 0) {
    isPaused = true;
    EEPROM.update(0, 1);  // update() only writes if different (saves EEPROM life)
  } else {
    isPaused = false;
    EEPROM.update(0, 0);
  }

  // Small delay to ensure write completes
  delay(10);
}
