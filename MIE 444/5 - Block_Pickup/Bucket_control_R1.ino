#include <Servo.h>

Servo Servo1; // First servo for main functionality (bucket)
Servo Servo2; // Second servo for sweeper
int servoPin1 = 8; // Digital pin for first servo (bucket)
int servoPin2 = 9; // Digital pin for second servo (sweeper)

bool sweeping = false; // Flag to track if sweeping is active
int sweepStartAngle = 180; // Start angle for sweeper
int sweepAngle = 180; // Current sweep angle
int sweepDirection = -1; // 1 for increasing, -1 for decreasing

void setup() {
    Serial.begin(9600);  // Initialize serial communication
}

void loop() {
    // Handle sweeper motion if sweeping is active
    if (sweeping) {
        if (!Servo2.attached()) {
            Servo2.attach(servoPin2); // Attach Servo2 if not attached
        }
        Servo2.write(sweepAngle);
        sweepAngle += sweepDirection * 5; // Adjust step size for speed

        if ((sweepDirection == -1 && sweepAngle <= 0) ||  // Stop at 0° for LS
            (sweepDirection == 1 && sweepAngle >= 180)) { // Stop at 180° for RS
            sweeping = false;
            Servo2.write(sweepAngle); // Set final position
            Servo2.detach(); // Detach Servo2 to save power
        }

        delay(30); // Adjust delay for sweeping speed
    }

    // Read serial commands
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');  // Read the incoming command

        if (command == "L") {  // Move first servo to 0 degrees
            if (!Servo1.attached()) Servo1.attach(servoPin1); // Attach Servo1 if not attached
            for (int pos = 270; pos >= 0; pos--) {
                Servo1.write(pos);
                delay(5);  // Adjust the delay to control speed
                sweeping = true;
                sweepStartAngle = 0; // Set starting position for sweeper
                sweepAngle = sweepStartAngle; // Reset sweep angle to start
                sweepDirection = 1; // Sweep upward to 180°
            }
            Servo1.detach(); // Detach Servo1 to save power
        } else if (command == "R") {  // Move sweeper first, then move bucket
            sweeping = true;
            sweepStartAngle = 180;  // Set starting position for the sweeper (start at 180°)
            sweepAngle = sweepStartAngle; // Reset sweep angle to start
            sweepDirection = -1; // Sweep downward to 0°

            if (!Servo2.attached()) {
                Servo2.attach(servoPin2); // Attach the sweeper servo if not already attached
            }

            // Move the sweeper from 180° to 0° (downward)
            for (int pos = 180; pos >= 0; pos--) {
                Servo2.write(pos);  // Rotate the sweeper servo
                delay(15);  // Adjust delay for sweeping speed (slower speed)
            }

            // Now move the bucket after the sweeper has completed its motion
            sweeping = false; // Stop the sweeper once it is done moving

            if (!Servo1.attached()) {
                Servo1.attach(servoPin1); // Attach the bucket servo if not already attached
            }

            // Move the bucket from 0° to 270° (upward)
            for (int pos = 0; pos <= 270; pos++) {
                Servo1.write(pos);  // Rotate the bucket servo
                delay(15);  // Adjust delay for bucket speed (slower speed)
            }

            // After both movements are complete, detach the servos to save power
            Servo2.detach();  // Detach sweeper servo
            Servo1.detach();  // Detach bucket servo
        } else if (command == "STOP") {  // Stop sweeping
            sweeping = false;
            if (Servo2.attached()) {
                Servo2.detach(); // Detach Servo2 to save power
            }
        }
    }
}
