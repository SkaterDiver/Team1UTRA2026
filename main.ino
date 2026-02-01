/*
 * Line Following Robot with Obstacle Avoidance
 * Arduino Uno R4
 *
 * ALGORITHM: Edge-following along RIGHT side of line
 * - Robot stays on WHITE, to the RIGHT of the trackable line
 * - Tracks BLACK line (calibrated) AND optional secondary RGB color
 * - On LINE (black or secondary color): turn LEFT (back to white on right)
 * - On WHITE (off line): curve RIGHT (toward line on our left)
 * - On OBSTACLE: detour around, then curve back to line
 *
 * SECONDARY COLOR: Set SECONDARY_COLOR to track additional colors:
 *   0 = disabled, 1 = RED, 2 = GREEN, 3 = BLUE
 *   Color is detected when it's significantly more dominant than others
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
const int BASE_SPEED = 70;        // Base forward speed
const int TURN_REDUCTION = 30;    // How much to slow inner wheel when ON line (gentle)
const int RETURN_TURN_REDUCTION = 25; // Curve sharpness when OFF line (returning to it)
const int RIGHT_MOTOR_BOOST = 10; // Compensation for weaker right motor
const int LEFT_MOTOR_REDUCTION = 10; // Reduce left motor power (robot drifts right)

// Color detection
const int BLACK_TOLERANCE = 20;   // +/- from calibrated value counts as black

// Secondary color tracking (treats this color same as black)
// Set to 0=NONE, 1=RED, 2=GREEN, 3=BLUE
const int SECONDARY_COLOR = 1;    // 1 = RED (set to 0 to disable)
const float COLOR_DOMINANCE = 1.2; // Secondary color must be this many times stronger (lower = more sensitive)

// Obstacle avoidance
const int DETOUR_PIVOT_MS = 400;  // How long to pivot right when obstacle seen
const int DETOUR_FORWARD_MS = 300; // How long to go forward after pivot
const int PIVOT_SPEED = 100;      // Speed during pivot turns

// Sharp turn handling
const unsigned long WHITE_ESCALATE_MS = 3000; // If off line this long, escalate to pivot (3 sec)
const int MIN_PIVOT_MS = 30;                  // Minimum time to pivot when hitting line

// ==================== STATE ====================

bool isPaused = true;
int calibratedBlack = 0;
bool isCalibrated = false;
unsigned long whiteStartTime = 0;  // When we first went on white
bool wasOnBlack = true;            // Track transitions

// Action recording for reversal (circular buffer)
const int ACTION_BUFFER_SIZE = 60;  // Store ~3 seconds of actions (at ~20 actions/sec)
struct Action {
  byte type;           // 0=none, 1=pivotRight, 2=curveLeft
  unsigned long time;  // When action started
};
Action actionBuffer[ACTION_BUFFER_SIZE];
int actionIndex = 0;
bool isReversing = false;
int reverseIndex = 0;

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

  // Initialize action buffer
  for (int i = 0; i < ACTION_BUFFER_SIZE; i++) {
    actionBuffer[i].type = 0;
    actionBuffer[i].time = 0;
  }

  // Handle EEPROM pause/run toggle
  handlePauseToggle();

  // Print status
  Serial.println("====================");
  Serial.println("LINE FOLLOWER v3.0");
  Serial.println("+ Secondary Color");
  Serial.println("====================");

  // Print secondary color config
  if (SECONDARY_COLOR > 0) {
    Serial.print("Secondary color: ");
    switch (SECONDARY_COLOR) {
      case 1: Serial.println("RED"); break;
      case 2: Serial.println("GREEN"); break;
      case 3: Serial.println("BLUE"); break;
    }
  }

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

  int clearVal = readColorSensor();
  int red = readRedChannel();
  int green = readGreenChannel();
  int blue = readBlueChannel();

  Serial.print("Clear:");
  Serial.print(clearVal);
  Serial.print(" R:");
  Serial.print(red);
  Serial.print(" G:");
  Serial.print(green);
  Serial.print(" B:");
  Serial.print(blue);

  Serial.print(" | ");
  if (isBlack(clearVal)) {
    Serial.print("BLACK");
  } else if (SECONDARY_COLOR > 0 && isSecondaryColorDetected()) {
    Serial.print("SEC-COLOR");
  } else {
    Serial.print("WHITE");
  }

  Serial.print(" | OnLine:");
  Serial.println(isOnLine() ? "YES" : "NO");

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
  // If we're in reversal mode, execute that instead
  if (isReversing) {
    executeReversal();
    return;
  }

  // Check if on trackable line (black OR secondary color)
  bool onLine = isOnLine();

  // Track how long we've been off the line (for sharp turn handling)
  if (onLine) {
    wasOnBlack = true;
    whiteStartTime = 0;  // Reset white timer
    isReversing = false; // Cancel any reversal
  } else {
    if (wasOnBlack) {
      // Just transitioned off the line
      whiteStartTime = millis();
      wasOnBlack = false;
    }
  }

  // Calculate time off line
  unsigned long timeOffLine = 0;
  if (!onLine && whiteStartTime > 0) {
    timeOffLine = millis() - whiteStartTime;
  }

  // Debug output
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    Serial.print("OnLine:");
    Serial.print(onLine ? "YES" : "NO");
    Serial.print(" -> ");
    if (onLine) {
      Serial.println("LINE(turn-L)");
    } else if (timeOffLine > WHITE_ESCALATE_MS) {
      Serial.println("OFF(REVERSE!)");
    } else {
      Serial.println("OFF(curve-R)");
    }
    lastPrint = millis();
  }

  if (onLine) {
    // Hit the line - turn LEFT (away from line, back to white on right)
    recordAction(1);  // Record pivotRight action
    pivotRight();
    delay(MIN_PIVOT_MS);
  } else {
    // Off line - turn toward the line (which is on our left)
    if (timeOffLine > WHITE_ESCALATE_MS) {
      // Been off line too long - start reversal sequence
      Serial.println("*** STARTING REVERSAL ***");
      startReversal();
    } else {
      // Normal curve toward line
      recordAction(2);  // Record curveLeft action
      curveLeft();
    }
  }
}

// Record an action to the circular buffer
void recordAction(byte actionType) {
  actionBuffer[actionIndex].type = actionType;
  actionBuffer[actionIndex].time = millis();
  actionIndex = (actionIndex + 1) % ACTION_BUFFER_SIZE;
}

// Start the reversal sequence
void startReversal() {
  isReversing = true;
  reverseIndex = (actionIndex - 1 + ACTION_BUFFER_SIZE) % ACTION_BUFFER_SIZE;

  // Find how many actions to reverse (last 3 seconds worth)
  unsigned long cutoffTime = millis() - WHITE_ESCALATE_MS;
  int count = 0;
  int idx = reverseIndex;
  while (count < ACTION_BUFFER_SIZE) {
    if (actionBuffer[idx].time < cutoffTime || actionBuffer[idx].type == 0) {
      break;
    }
    count++;
    idx = (idx - 1 + ACTION_BUFFER_SIZE) % ACTION_BUFFER_SIZE;
  }

  Serial.print("Reversing ");
  Serial.print(count);
  Serial.println(" actions");
}

// Execute one step of the reversal
void executeReversal() {
  unsigned long cutoffTime = millis() - WHITE_ESCALATE_MS - 1000; // Extra buffer

  // Check if this action is recent enough
  if (actionBuffer[reverseIndex].time < cutoffTime || actionBuffer[reverseIndex].type == 0) {
    // Done reversing
    Serial.println("*** REVERSAL COMPLETE ***");
    isReversing = false;
    whiteStartTime = millis();  // Reset timer
    return;
  }

  // Execute OPPOSITE action
  byte originalAction = actionBuffer[reverseIndex].type;
  if (originalAction == 1) {
    // Original was pivotRight, do pivotLeft
    pivotLeft();
    delay(MIN_PIVOT_MS);
  } else if (originalAction == 2) {
    // Original was curveLeft, do curveRight (backup)
    driveBackward();
    delay(30);
  }

  // Move to previous action
  reverseIndex = (reverseIndex - 1 + ACTION_BUFFER_SIZE) % ACTION_BUFFER_SIZE;

  // Check if we found the line during reversal
  if (isOnLine()) {
    Serial.println("*** FOUND LINE DURING REVERSAL ***");
    isReversing = false;
  }
}

// ==================== OBSTACLE HANDLING ====================

void handleObstacle() {
  Serial.println("");
  Serial.println("*** OBSTACLE DETECTED ***");
  Serial.println("Detouring RIGHT (off track)...");

  // Stop
  stopMotors();
  delay(100);

  // Turn RIGHT physically (away from track, into open space)
  // pivotLeft() produces physical RIGHT turn due to motor wiring
  pivotLeft();
  delay(DETOUR_PIVOT_MS);

  // Go forward to clear the obstacle area
  driveForward();
  delay(DETOUR_FORWARD_MS);

  // Turn back LEFT toward track
  pivotRight();  // pivotRight() produces physical LEFT turn
  delay(DETOUR_PIVOT_MS / 2);  // Half turn back toward track

  // Resume normal line following
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
  // Both motors forward (left reduced to prevent right drift)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, BASE_SPEED - LEFT_MOTOR_REDUCTION);
  analogWrite(ENB, BASE_SPEED + RIGHT_MOTOR_BOOST);
}

void driveBackward() {
  // Both motors backward (left reduced to prevent drift)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, BASE_SPEED - LEFT_MOTOR_REDUCTION);
  analogWrite(ENB, BASE_SPEED + RIGHT_MOTOR_BOOST);
}

void curveLeft() {
  // Both motors forward, left slower (curve toward line)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, BASE_SPEED - RETURN_TURN_REDUCTION - LEFT_MOTOR_REDUCTION);
  analogWrite(ENB, BASE_SPEED + RIGHT_MOTOR_BOOST);
}

void curveRight() {
  // Both motors forward, right slower
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, BASE_SPEED - LEFT_MOTOR_REDUCTION);
  analogWrite(ENB, BASE_SPEED - RETURN_TURN_REDUCTION + RIGHT_MOTOR_BOOST);
}

void sharpLeft() {
  // Left wheel STOPPED, right wheel forward
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
  analogWrite(ENA, PIVOT_SPEED - LEFT_MOTOR_REDUCTION);
  analogWrite(ENB, PIVOT_SPEED + RIGHT_MOTOR_BOOST);
}

void pivotRight() {
  // Left motor forward, right motor backward (spin in place)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, PIVOT_SPEED - LEFT_MOTOR_REDUCTION);
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

  // Delay for filter to settle
  delay(5);

  // Read pulse width (higher = darker)
  return pulseIn(CS_OUT, LOW, 30000);
}

// Read individual color channels (lower value = more of that color)
// Reduced delays for faster response
int readRedChannel() {
  digitalWrite(CS_S2, LOW);
  digitalWrite(CS_S3, LOW);
  delay(2);
  return pulseIn(CS_OUT, LOW, 20000);
}

int readGreenChannel() {
  digitalWrite(CS_S2, HIGH);
  digitalWrite(CS_S3, HIGH);
  delay(2);
  return pulseIn(CS_OUT, LOW, 20000);
}

int readBlueChannel() {
  digitalWrite(CS_S2, LOW);
  digitalWrite(CS_S3, HIGH);
  delay(2);
  return pulseIn(CS_OUT, LOW, 20000);
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

// Check if the secondary color is detected
// Returns true if the configured color is significantly more dominant than others
bool isSecondaryColorDetected() {
  if (SECONDARY_COLOR == 0) {
    return false;  // Secondary color tracking disabled
  }

  int red = readRedChannel();
  int green = readGreenChannel();
  int blue = readBlueChannel();

  // Handle sensor errors
  if (red == 0 || green == 0 || blue == 0) {
    return false;
  }

  // Debug: print RGB values occasionally
  static unsigned long lastRGBPrint = 0;
  if (millis() - lastRGBPrint > 500) {
    Serial.print("RGB: R=");
    Serial.print(red);
    Serial.print(" G=");
    Serial.print(green);
    Serial.print(" B=");
    Serial.println(blue);
    lastRGBPrint = millis();
  }

  // For TCS3200: LOWER pulse width = MORE of that color
  // Check if target color value is significantly lower than others
  // Using multiplication instead of division for clearer logic

  bool detected = false;

  switch (SECONDARY_COLOR) {
    case 1:  // RED
      // Red value must be much lower than green and blue
      // red * dominance < green AND red * dominance < blue
      detected = (red * COLOR_DOMINANCE < green) && (red * COLOR_DOMINANCE < blue);
      break;

    case 2:  // GREEN
      // Green value must be much lower than red and blue
      detected = (green * COLOR_DOMINANCE < red) && (green * COLOR_DOMINANCE < blue);
      break;

    case 3:  // BLUE
      // Blue value must be much lower than red and green
      detected = (blue * COLOR_DOMINANCE < red) && (blue * COLOR_DOMINANCE < green);
      break;
  }

  if (detected) {
    Serial.println("*** SECONDARY COLOR DETECTED ***");
  }

  return detected;
}

// Combined check: is the sensor on a trackable line (black OR secondary color)
bool isOnLine() {
  int clearValue = readColorSensor();

  // First check for black (fast check)
  if (isBlack(clearValue)) {
    return true;
  }

  // Then check for secondary color if enabled
  if (SECONDARY_COLOR > 0 && isSecondaryColorDetected()) {
    return true;
  }

  return false;
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
