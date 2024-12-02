#include <Servo.h>
#include <AFMotor.h>

// Pin Definitions
#define Echo A0
#define Trig A1
#define MotorServoPin 10
#define TouchSensorPin 2
#define Speed 170
#define ServoMidpoint 103
#define LedManualMode 13
#define LedAutoMode 9

// Variables
char value;
int distance;
int LeftDistance;
int RightDistance;
int ManualMode = 1;
int AutoMode = 0;

// Servo and Motor Setup
Servo servo;
AF_DCMotor M1(1);
AF_DCMotor M2(2);
AF_DCMotor M3(3);
AF_DCMotor M4(4);

void setup() {
    Serial.begin(9600);
    
    // Pin Modes
    pinMode(Trig, OUTPUT);
    pinMode(Echo, INPUT);
    pinMode(TouchSensorPin, INPUT);
    pinMode(LedManualMode, OUTPUT);
    pinMode(LedAutoMode, OUTPUT);
    
    // Attach Servo
    servo.attach(MotorServoPin);

    // Interrupt for Touch Sensor
    attachInterrupt(digitalPinToInterrupt(TouchSensorPin), ToggleMode, FALLING);

    // Set Motor Speeds
    M1.setSpeed(Speed);
    M2.setSpeed(Speed);
    M3.setSpeed(Speed);
    M4.setSpeed(Speed);
}

// Function to Move Forward
void moveForward() {
    M1.run(FORWARD);
    M2.run(FORWARD);
    M3.run(FORWARD);
    M4.run(FORWARD);
}

// Function to Move Backward
void moveBackward() {
    M1.run(BACKWARD);
    M2.run(BACKWARD);
    M3.run(BACKWARD);
    M4.run(BACKWARD);
}

// Function to Turn Right
void turnRight() {
    M1.run(BACKWARD);
    M2.run(BACKWARD);
    M3.run(FORWARD);
    M4.run(FORWARD);
}

// Function to Turn Left
void turnLeft() {
    M1.run(FORWARD);
    M2.run(FORWARD);
    M3.run(BACKWARD);
    M4.run(BACKWARD);
}

// Function to Stop
void stopMovement() {
    M1.run(RELEASE);
    M2.run(RELEASE);
    M3.run(RELEASE);
    M4.run(RELEASE);
}

// Function for Ultrasonic Sensor
int getDistance() {
    digitalWrite(Trig, LOW);
    delayMicroseconds(4);
    digitalWrite(Trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(Trig, LOW);
    long duration = pulseIn(Echo, HIGH);
    int distance = duration * 0.034 / 2; // Convert to cm
    return distance;
}

// Function to Look Right
int lookRight() {
    servo.write(20); // Rotate Servo to the Right
    delay(800);
    int distance = getDistance();
    return distance;
}

// Function to Look Left
int lookLeft() {
    servo.write(180); // Rotate Servo to the Left
    delay(800);
    int distance = getDistance();
    return distance;
}

// Function for Manual Mode
void manualMode() {
    if (Serial.available() > 0) {
        value = Serial.read();
        Serial.println(value);
    }

    if (value == 'F') {
        moveForward();
    } else if (value == 'L') {
        turnLeft();
    } else if (value == 'B') {
        moveBackward();
    } else if (value == 'R') {
        turnRight();
    } else if (value == 'S') {
        stopMovement();
    }
}

// Function for Auto Mode
void autoMode() {
    distance = getDistance();

    if (distance > 10 && distance < 50) {
        moveForward();
        delay(200);
        stopMovement();
    } else if (distance <= 10) {
        stopMovement();
        delay(100);

        LeftDistance = lookLeft();
        servo.write(ServoMidpoint);
        delay(800);

        servo.write(ServoMidpoint);

        if (LeftDistance > RightDistance) {
            turnLeft();
            delay(500);
        } else {
            turnRight();
            delay(500);
        }
        stopMovement();
        delay(200);
    } else {
        moveForward();
        delay(100);
    }
}

// Interrupt to Toggle Modes
void ToggleMode() {
    ManualMode = !ManualMode;
    AutoMode = !AutoMode;
}

// Main Loop
void loop() {
    if (ManualMode) {
        digitalWrite(LedManualMode, HIGH);
        digitalWrite(LedAutoMode, LOW);
        manualMode();
    } else if (AutoMode) {
        digitalWrite(LedManualMode, LOW);
        digitalWrite(LedAutoMode, HIGH);
        autoMode();
    }
}
