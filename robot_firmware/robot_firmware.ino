/*
  robot_firmware_elegoo_v4_ros.ino

  ROS2-compatible firmware for ELEGOO Smart Robot Car V4.0
  =========================================================
  
  This sketch runs on the Arduino UNO inside the robot car.
  It talks to a Raspberry Pi (or any computer) via USB Serial.
  
  WHAT IT DOES:
  - Receives velocity commands from the Pi: "V,<linear>,<angular>"
  - Converts those into left/right motor speeds
  - Reads the ultrasonic sensor and sends distance back: "R,<distance>"
  - Has a safety timeout: stops motors if Pi stops talking
  
  HARDWARE NOTE - CRITICAL FOR V4.0:
  This car uses the TB6612FNG motor driver (not the old L298N).
  ELEGOO changed the pinout from V3.0 to V4.0!
  
  The V4.0 uses a simplified control scheme:
  - Only ONE direction pin per motor (not two)
  - STBY (standby) is on pin 3 (not 12)
  - Right motor: PWM=5, Direction=7
  - Left motor: PWM=6, Direction=8
  
  WIRING:
  - Arduino UNO is stacked on the ELEGOO motor shield
  - USB cable connects Arduino to Raspberry Pi
  - Remove Bluetooth module when using USB (they share pins)
  - Power: 7.4V Li-ion battery powers motors + Arduino
  
  SERIAL PROTOCOL:
  Pi → Arduino: "V,0.20,0.00\n"  (move forward at 0.2 m/s)
  Pi → Arduino: "V,0.00,0.50\n"  (spin in place)
  Arduino → Pi: "R,0.45\n"       (obstacle at 45cm)
  
  Created: 2024
  Hardware: ELEGOO Smart Robot Car V4.0 with TB6612FNG
*/


#include <Arduino.h>


/*
  ------------------------------------------------------
  PIN DEFINITIONS - CORRECT FOR V4.0 TB6612FNG
  ------------------------------------------------------
  
  WARNING: These pins are different from V3.0!
  I learned this the hard way when the robot wouldn't move.
  
  The TB6612FNG has a STBY (standby) pin that MUST be HIGH
  for the chip to work. On the V4.0 shield, this is pin 3.
  
  Each motor uses:
  - 1 PWM pin for speed control (0-255)
  - 1 direction pin (HIGH=forward, LOW=backward)
  
  The V4.0 shield handles the second direction pin internally,
  so we only control one per motor.
*/

// Standby pin - MUST be HIGH or motors won't work at all!
const int STBY = 3;

// Right motor (Motor A on the board)
const int PWMA = 5;      // Speed control - PWM pin
const int AIN_1 = 7;     // Direction: HIGH=forward, LOW=backward

// Left motor (Motor B on the board)  
const int PWMB = 6;      // Speed control - PWM pin
const int BIN_1 = 8;     // Direction: HIGH=forward, LOW=backward

// Ultrasonic sensor (HC-SR04)
const int TRIG = 13;     // Trigger pin - we send the pulse
const int ECHO = 12;     // Echo pin - we read the response


/*
  ------------------------------------------------------
  ROBOT PHYSICS PARAMETERS
  ------------------------------------------------------
  
  These numbers convert ROS velocity commands (meters/second)
  into PWM values (0-255) that the motors understand.
  
  You'll probably need to tune these for your specific robot.
  Every car is slightly different due to wheel friction,
  battery voltage, motor variations, etc.
*/

// Distance between left and right wheels (in meters)
// I measured this with a ruler - it's about 14cm
const float WHEEL_BASE_M = 0.14;

// Conversion: meters/second → PWM value
// If your robot is too slow, increase this.
// If it's too fast or jerky, decrease this.
// Start with 400 and adjust based on testing.
const float SPEED_TO_PWM = 400.0;

// Minimum PWM to actually make the wheels turn
// Below this, the motors just buzz and don't move
// (Static friction is a thing with DC motors)
const int MIN_PWM_TO_MOVE = 50;

// Safety timeout in milliseconds
// If we don't hear from the Pi for 500ms, we stop
// This prevents the robot from driving off if WiFi drops
const unsigned long CMD_TIMEOUT_MS = 500;


/*
  ------------------------------------------------------
  GLOBAL VARIABLES
  ------------------------------------------------------
  These store the current state of the robot
*/

// When did we last get a command from the Pi?
unsigned long lastCmdMs = 0;

// When did we last send ultrasonic data?
unsigned long lastRangeMs = 0;

// What speed does the Pi want us to go?
// Linear = forward/backward speed (m/s)
// Angular = rotation speed (rad/s)
float targetLinear = 0.0;
float targetAngular = 0.0;

// Buffer for reading serial commands character by character
String line;


/*
  ------------------------------------------------------
  MOTOR CONTROL FUNCTIONS
  ------------------------------------------------------
*/

/*
  enableMotors()
  --------------
  The TB6612FNG has a standby mode to save power.
  We must set STBY HIGH before any motor will work.
  Call this once in setup().
*/
void enableMotors() {
  digitalWrite(STBY, HIGH);
}

/*
  disableMotors()
  ---------------
  Puts the motor driver in standby mode.
  Not really used, but good to have for completeness.
*/
void disableMotors() {
  digitalWrite(STBY, LOW);
}

/*
  wheelSpeedToPwm()
  -----------------
  Converts a speed in meters/second to a PWM value (0-255).
  
  Also handles:
  - Minimum PWM threshold (so motors actually move)
  - Clamping to valid range (-255 to 255)
  
  Example: 0.1 m/s → 40 PWM (with SPEED_TO_PWM = 400)
*/
int wheelSpeedToPwm(float v_mps) {
  // Convert speed to PWM using our scaling factor
  int pwm = (int)(v_mps * SPEED_TO_PWM);
  
  // If the PWM is too small (but not zero), boost it to minimum
  // This overcomes static friction in the motors
  if (pwm > 0 && pwm < MIN_PWM_TO_MOVE) {
    pwm = MIN_PWM_TO_MOVE;
  }
  if (pwm < 0 && pwm > -MIN_PWM_TO_MOVE) {
    pwm = -MIN_PWM_TO_MOVE;
  }
  
  // Make sure we don't exceed Arduino's PWM limits
  pwm = constrain(pwm, -255, 255);
  
  return pwm;
}

/*
  setRightMotor()
  ---------------
  Controls the right wheel.
  
  pwm > 0: forward
  pwm < 0: backward  
  pwm = 0: stop (coast)
  
  On the V4.0, direction is simple:
  - AIN_1 HIGH = forward
  - AIN_1 LOW = backward
*/
void setRightMotor(int pwm) {
  if (pwm == 0) {
    // Stop - just turn off PWM
    analogWrite(PWMA, 0);
    return;
  }
  
  if (pwm > 0) {
    // Forward
    digitalWrite(AIN_1, HIGH);
    analogWrite(PWMA, pwm);
  } else {
    // Backward
    digitalWrite(AIN_1, LOW);
    analogWrite(PWMA, -pwm);  // -pwm because pwm is negative
  }
}

/*
  setLeftMotor()
  --------------
  Controls the left wheel.
  
  Same logic as right motor, but mirrored because
  the left motor is mounted facing the other way.
  
  Wait, actually - on my robot, both motors have the same
  "forward" direction when wired correctly. But sometimes
  you need to flip one if the robot spins instead of going straight.
  
  If your robot spins in place when it should go straight,
  swap the HIGH/LOW in one of these functions.
*/
void setLeftMotor(int pwm) {
  if (pwm == 0) {
    analogWrite(PWMB, 0);
    return;
  }
  
  if (pwm > 0) {
    // Forward
    digitalWrite(BIN_1, HIGH);
    analogWrite(PWMB, pwm);
  } else {
    // Backward
    digitalWrite(BIN_1, LOW);
    analogWrite(PWMB, -pwm);
  }
}

/*
  stopMotors()
  ------------
  Emergency stop - sets both motors to zero immediately.
  Used for safety timeout and initialization.
*/
void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}


/*
  ------------------------------------------------------
  ULTRASONIC SENSOR FUNCTIONS
  ------------------------------------------------------
*/

/*
  readUltrasonicMeters()
  ----------------------
  Measures distance using the HC-SR04 ultrasonic sensor.
  
  How it works:
  1. Send a 10 microsecond pulse on TRIG pin
  2. Sensor sends ultrasonic burst and listens for echo
  3. Measure how long until ECHO pin goes HIGH
  4. Convert time to distance using speed of sound
  
  Returns distance in meters, or -1.0 if no echo detected
  (meaning no obstacle in range, or sensor error)
  
  The timeout of 25000 microseconds means we stop waiting
  after about 4 meters - no need to wait for distant echoes.
*/
float readUltrasonicMeters() {
  // Clear the trigger pin
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  
  // Send the trigger pulse
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  
  // Read the echo duration (time until sound returns)
  // 25000 us timeout = ~4 meter max range
  long duration = pulseIn(ECHO, HIGH, 25000);
  
  // If duration is 0, the pulse timed out - no obstacle detected
  if (duration == 0) {
    return -1.0;
  }
  
  // Calculate distance in centimeters
  // Speed of sound is ~343 m/s, or 0.0343 cm/us
  // Divide by 2 because sound travels there AND back
  float cm = duration * 0.0343 / 2.0;
  
  // Convert to meters for ROS
  return cm / 100.0;
}


/*
  ------------------------------------------------------
  SERIAL COMMAND HANDLING
  ------------------------------------------------------
*/

/*
  parseVelocityLine()
  -------------------
  Parses velocity commands from the Raspberry Pi.
  
  Expected format: "V,<linear_speed>,<angular_speed>"
  
  Examples:
    "V,0.20,0.00"  -> move forward at 0.2 m/s
    "V,0.00,1.00"  -> spin counter-clockwise at 1 rad/s
    "V,-0.10,0.50" -> back up while turning
  
  Returns true if parsing succeeded, false if format was wrong.
*/
bool parseVelocityLine(const String& s, float &lin, float &ang) {
  // Check minimum length and command prefix
  if (s.length() < 2) return false;
  if (s.charAt(0) != 'V') return false;
  
  // Find the two commas
  int firstComma = s.indexOf(',');
  int secondComma = s.indexOf(',', firstComma + 1);
  
  // Must have exactly two commas
  if (firstComma < 0 || secondComma < 0) return false;
  
  // Extract the two numbers and convert to float
  lin = s.substring(firstComma + 1, secondComma).toFloat();
  ang = s.substring(secondComma + 1).toFloat();
  
  return true;
}


/*
  ------------------------------------------------------
  DIFFERENTIAL DRIVE KINEMATICS
  ------------------------------------------------------
*/

/*
  driveFromTwist()
  ----------------
  Converts linear + angular velocity into left/right wheel speeds.
  
  This is the classic differential drive equation:
  
    left_speed  = linear - (angular * wheel_base / 2)
    right_speed = linear + (angular * wheel_base / 2)
  
  Think of it this way:
  - To go straight: both wheels same speed
  - To turn right: left wheel faster, right wheel slower (or reverse)
  - To spin in place: wheels opposite directions
  
  The wheel_base is the distance between wheels. A wider robot
  needs more speed difference to turn the same amount.
*/
void driveFromTwist(float lin, float ang) {
  // Calculate individual wheel speeds in m/s
  float vLeft  = lin - (ang * WHEEL_BASE_M / 2.0);
  float vRight = lin + (ang * WHEEL_BASE_M / 2.0);
  
  // Convert to PWM and send to motors
  int pwmLeft  = wheelSpeedToPwm(vLeft);
  int pwmRight = wheelSpeedToPwm(vRight);
  
  setLeftMotor(pwmLeft);
  setRightMotor(pwmRight);
}


/*
  ------------------------------------------------------
  ARDUINO SETUP
  ------------------------------------------------------
*/

void setup() {
  // Start serial communication with the Raspberry Pi
  // 115200 baud is fast and reliable over USB
  Serial.begin(115200);
  
  // Configure motor control pins as outputs
  pinMode(STBY, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN_1, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN_1, OUTPUT);
  
  // Configure ultrasonic sensor pins
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  
  // CRITICAL: Enable the motor driver!
  // Without this, STAY is LOW and motors won't work
  enableMotors();
  
  // Make sure motors are stopped at startup
  stopMotors();
  
  // Record that we've started (for timeout logic)
  lastCmdMs = millis();
  
  // Optional: send ready message to Pi
  // Serial.println("Robot ready");
}


/*
  ------------------------------------------------------
  MAIN LOOP
  ------------------------------------------------------
*/

void loop() {
  
  /*
    PART 1: READ COMMANDS FROM RASPBERRY PI
    ---------------------------------------
    We read characters one at a time, building up a line
    until we see a newline character (\n).
    
    This is non-blocking - if no data available, we just
    continue to the next part of the loop.
  */
  while (Serial.available()) {
    char ch = (char)Serial.read();
    
    if (ch == '\n') {
      // End of line - process the command
      line.trim();  // Remove whitespace
      
      float lin, ang;
      if (parseVelocityLine(line, lin, ang)) {
        // Valid velocity command received!
        targetLinear = lin;
        targetAngular = ang;
        
        // Record when we got this command (for timeout)
        lastCmdMs = millis();
      }
      
      // Clear the buffer for next command
      line = "";
    } else {
      // Add character to our buffer
      line += ch;
    }
  }
  
  
  /*
    PART 2: SAFETY TIMEOUT
    ----------------------
    If we haven't received a command from the Pi recently,
    something might be wrong (Pi crashed, WiFi dropped, etc.)
    
    For safety, we stop the motors rather than continuing
    with old velocity commands.
    
    This is like a dead man's switch on a train.
  */
  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    // Too long since last command - stop!
    stopMotors();
  } else {
    // All good - drive at requested speed
    driveFromTwist(targetLinear, targetAngular);
  }
  
  
  /*
    PART 3: SEND SENSOR DATA BACK TO PI
    -----------------------------------
    We read the ultrasonic sensor and send the distance
    back to the Raspberry Pi.
    
    Format: "R,<distance_in_meters>"
    Example: "R,0.45" means 45cm to obstacle
    
    We do this about 10 times per second (every 100ms).
    That's plenty fast for obstacle avoidance.
  */
  if (millis() - lastRangeMs > 100) {
    lastRangeMs = millis();
    
    float distance = readUltrasonicMeters();
    
    // Only send valid readings (ignore -1.0 errors)
    if (distance > 0.0) {
      Serial.print("R,");
      Serial.println(distance, 2);  // 2 decimal places
    }
  }
  
  /*
    That's it! The loop runs hundreds of times per second,
    constantly checking for commands, updating motors, and
    sending sensor data.
    
    The actual timing is handled by millis() checks rather
    than delay(), so we never block and miss serial data.
  */
  
}  // End of loop()