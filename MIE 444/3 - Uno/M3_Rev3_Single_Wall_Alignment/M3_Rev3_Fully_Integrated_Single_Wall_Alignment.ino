#define QUEUE_MAX_SIZE 100

// Queue structure
struct Queue {
    String data[QUEUE_MAX_SIZE]; // Array to hold queue elements
    int front;                   // Index of the front element
    int rear;                    // Index of the rear element
    int count;                   // Number of elements in the queue
};

// Initialize an empty queue
Queue commandQueue = { {}, 0, 0, 0 };

// Function to add an element to the queue
bool enqueue(Queue &q, String element) {
    if (q.count == QUEUE_MAX_SIZE) {
        Serial.println("Queue is full!");
        return false;
    }
    q.data[q.rear] = element;
    q.rear = (q.rear + 1) % QUEUE_MAX_SIZE;
    q.count++;
    return true;
}

// Function to remove an element from the queue
String dequeue(Queue &q) {
    if (q.count == 0) {
        Serial.println("Queue is empty!");
        return "";
    }
    String element = q.data[q.front];
    q.front = (q.front + 1) % QUEUE_MAX_SIZE;
    q.count--;
    return element;
}

// Function to check if the queue is empty
bool isQueueEmpty(Queue &q) {
    return q.count == 0;
}

// Flags and timing variables
bool isExecutingCommand = false;
unsigned long commandStartTime = 0;
unsigned long driveDuration = 0;
//String receivedData = "";

// Motor Pin Definitions
int enA = 5, in1 = 9, in2 = 8; // RIGHT
int enB = 10, in3 = 11, in4 = 12;

// Sensor data placeholders
float leftDistance = 0;
float rightDistance = 0;
float Min_Distance = 0;

const float alignmentThreshold = 0; // Alignment error threshold in inches, prev. 0.5 
const float kp_alignment = 6;       // PID tuning parameter for alignment
const float kd_alignment = 0.1;
const float ki_alignment = 0;
// PWM limits for motors
const int FMIN_PWM_RIGHT = 140, FMAX_PWM_RIGHT = 255;
const int FMIN_PWM_LEFT = 190, FMAX_PWM_LEFT = 255; // befpre it was 170 and 200

float targetAngle = 0;

float threshold = 1.0;               // Rotation threshold in degrees for when to stop
float Single_Wall_Threshold = 2.3;
float Sign_Indicator = 0;

#include <Wire.h>
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

#include <Servo.h>

Servo Servo1; // First servo for main functionality (bucket)
Servo Servo2; // Second servo for sweeper
int servoPin1 = 3; // Digital pin for first servo (bucket)
int servoPin2 = 6; // Digital pin for second servo (sweeper)

bool sweeping = false; // Flag to track if sweeping is active
int sweepStartAngle = 180; // Start angle for sweeper
int sweepAngle = 180; // Current sweep angle
int sweepDirection = -1; // 1 for increasing, -1 for decreasing
String Bucket_Action = "";
String Current_Step = "";

// Debugging flag
#define DEBUG 0
#if DEBUG
    #define DEBUG_PRINT(x) Serial.println(x)
#else
    #define DEBUG_PRINT(x)
#endif

void setup() {
    // Motor pin setup
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

    // Serial communication setup
    Serial.begin(115200); // Communication with Mega
}

void loop() {
    // Check for incoming data from the Mega
    if (Serial.available()) {
        String receivedData = Serial.readStringUntil('\n');
        if (receivedData.startsWith("L:")) {
            // Process ultrasonic data continuously
            // processSensorData(receivedData);
            processAndSendFeedback(receivedData);
        } else {
            // Add drive commands to the queue
            if (!enqueue(commandQueue, receivedData)) {
                DEBUG_PRINT("Failed to enqueue command!");
            }
        }
    }

    // Execute commands from the queue if no command is active
    if (!isExecutingCommand && !isQueueEmpty(commandQueue)) {
        String nextCommand = dequeue(commandQueue);
        executeCommand(nextCommand);
    }

    // Continue executing the current command if active
    if (isExecutingCommand) {
      if (Current_Step == "w0") {
        executeDriveForward();
      }
      
      else if (Current_Step == "r0") {
        executeDriveRotate();
      }

      else if (Current_Step == "bp") {
        executeBucketMotion();
      }

    // // Periodically send feedback to Mega
    // sendFeedbackToMega();
}}

// NEW ARDUINO UNO AND DRIVE: 
void processAndSendFeedback(String data) {
    // Parse incoming data
    // ARDUINO MEGA INPUT AT A FREQUENCY OF 50MS "L:XX.XX,R:XX.XX"
    int leftIndex = data.indexOf("L:");
    int rightIndex = data.indexOf("R:");

    if (leftIndex != -1 && rightIndex != -1) {
        leftDistance = data.substring(leftIndex + 2, rightIndex).toFloat();
        rightDistance = data.substring(rightIndex + 2).toFloat();

        // Debug: Log parsed data
        DEBUG_PRINT("Received Data: " + data);
        DEBUG_PRINT("Parsed Left Distance: " + String(leftDistance) + ", Right Distance: " + String(rightDistance));

        // Send feedback immediately
        String feedback = "Left:" + String(leftDistance) + ", Right:" + String(rightDistance);
        Serial.println("<" + feedback + ">");

        // Debug: Log feedback
        DEBUG_PRINT("Feedback Sent: " + feedback);
    } else {
        // Debug: Log invalid data
        DEBUG_PRINT("Invalid Data Received: " + data);
    }
}

// Function to start executing a drive command
void executeCommand(String cmd) {
    isExecutingCommand = true;
    commandStartTime = millis();

    if (cmd.startsWith("w0:")) {
        float distance = cmd.substring(3).toFloat();
        driveDuration = (distance / 10.0) * 1000;
        Current_Step = "w0";
    } 
    //// ADD other if statements for R0, B0, S0

    else if (cmd.startsWith("r0:")) {
        float targetAngle = cmd.substring(3).toFloat();
        Current_Step = "r0";
    }

    else if (cmd.startsWith("bp:")) {
      float action = cmd.substring(3).toFloat();
      Current_Step = "bp";
      if (action == 0)  {
        Bucket_Action = "L";
      }
      else if (action == 1) {
        Bucket_Action = "R";
    }
    }
    else {
        Serial.println("Invalid Command");
        isExecutingCommand = false;
    }
}

static float previousAlignmentError = 0;
static float previousLeftAlignmentError = 0;
static float previousRightAlignmentError = 0;
static unsigned long lastUpdateTime = 0;

int pwmRight = 0;
int pwmLeft = 0;
int alignmentIntegral = 0;
int LeftalignmentIntegral = 0;
int RightalignmentIntegral = 0;
void executeDriveForward() {
    if (millis() - commandStartTime < driveDuration) {
        Min_Distance = min(leftDistance, rightDistance);
        float alignmentError = Min_Distance - Single_Wall_Threshold;
        // float LeftalignmentError = leftDistance - Single_Wall_Threshold;
        // float RightalignmentError = rightDistance - Single_Wall_Threshold;
        // Serial.println("< Alignment Error: " + String(alignmentError) + ">");
        unsigned long currentTime = millis();
        float deltaTime = (currentTime - lastUpdateTime) / 1000.0; // Convert to seconds

        // Calculate derivative term
        float alignmentDerivative = 0;
        // float LeftalignmentDerivative = 0;
        // float RightalignmentDerivative = 0;
        if (deltaTime > 0) {
            alignmentDerivative = (alignmentError - previousAlignmentError) / deltaTime;
            // LeftalignmentDerivative = (LeftalignmentError - previousLeftAlignmentError) / deltaTime;
            // RightalignmentDerivative = (RightalignmentError - previousRightAlignmentError) / deltaTime;
        }

        // Calculate integral term (accumulating error over time)
        alignmentIntegral += alignmentError * deltaTime;
        // LeftalignmentIntegral += LeftalignmentError * deltaTime;
        // RightalignmentIntegral += RightalignmentError * deltaTime;

        // Calculate alignment correction
        float alignmentCorrection = kp_alignment * alignmentError
                                     + kd_alignment * alignmentDerivative
                                     + ki_alignment * alignmentIntegral;

        // float LeftalignmentCorrection = kp_alignment * LeftalignmentError
        //                              + kd_alignment * LeftalignmentDerivative
        //                              + ki_alignment * LeftalignmentIntegral;

        // float RightalignmentCorrection = kp_alignment * RightalignmentError
        //                              + kd_alignment * RightalignmentDerivative
        //                              + ki_alignment * RightalignmentIntegral;

        // Wall-following logic
        if (leftDistance - rightDistance < 0) {
            // Following left wall
            if (alignmentError > 0) { // Veer left
                pwmRight = constrain(((FMAX_PWM_RIGHT + FMIN_PWM_RIGHT) / 2) + alignmentCorrection, FMIN_PWM_RIGHT, FMAX_PWM_RIGHT);
                pwmLeft = constrain(((FMAX_PWM_LEFT + FMIN_PWM_LEFT) / 2) - alignmentCorrection, FMIN_PWM_LEFT, FMAX_PWM_LEFT);
            } else if (alignmentError < 0) { // Veer right
                pwmRight = constrain(((FMAX_PWM_RIGHT + FMIN_PWM_RIGHT) / 2) + alignmentCorrection, FMIN_PWM_RIGHT, FMAX_PWM_RIGHT);
                pwmLeft = constrain(((FMAX_PWM_LEFT + FMIN_PWM_LEFT) / 2) - alignmentCorrection, FMIN_PWM_LEFT, FMAX_PWM_LEFT);
            }

            // if (LeftalignmentError > 0) { // Veer left
            //     pwmRight = constrain(((FMAX_PWM_RIGHT + FMIN_PWM_RIGHT) / 2) + LeftalignmentCorrection, FMIN_PWM_RIGHT, FMAX_PWM_RIGHT);
            //     pwmLeft = constrain(((FMAX_PWM_LEFT + FMIN_PWM_LEFT) / 2) - LeftalignmentCorrection, FMIN_PWM_LEFT, FMAX_PWM_LEFT);
            // } else if (LeftalignmentError < 0) { // Veer right
            //     pwmRight = constrain(((FMAX_PWM_RIGHT + FMIN_PWM_RIGHT) / 2) + LeftalignmentCorrection, FMIN_PWM_RIGHT, FMAX_PWM_RIGHT);
            //     pwmLeft = constrain(((FMAX_PWM_LEFT + FMIN_PWM_LEFT) / 2) - LeftalignmentCorrection, FMIN_PWM_LEFT, FMAX_PWM_LEFT);
            // }

        } else {
            // Following right wall
            if (alignmentError > 0) { // Veer right
                pwmRight = constrain(((FMAX_PWM_RIGHT + FMIN_PWM_RIGHT) / 2) - alignmentCorrection, FMIN_PWM_RIGHT, FMAX_PWM_RIGHT);
                pwmLeft = constrain(((FMAX_PWM_LEFT + FMIN_PWM_LEFT) / 2) + alignmentCorrection, FMIN_PWM_LEFT, FMAX_PWM_LEFT);
            } else if (alignmentError < 0) { // Veer left
                pwmRight = constrain(((FMAX_PWM_RIGHT + FMIN_PWM_RIGHT) / 2) + alignmentCorrection, FMIN_PWM_RIGHT, FMAX_PWM_RIGHT);
                pwmLeft = constrain(((FMAX_PWM_LEFT + FMIN_PWM_LEFT) / 2) - alignmentCorrection, FMIN_PWM_LEFT, FMAX_PWM_LEFT);
            }

            // if (RightalignmentError > 0) { // Veer right
            //     pwmRight = constrain(((FMAX_PWM_RIGHT + FMIN_PWM_RIGHT) / 2) - RightalignmentCorrection, FMIN_PWM_RIGHT, FMAX_PWM_RIGHT);
            //     pwmLeft = constrain(((FMAX_PWM_LEFT + FMIN_PWM_LEFT) / 2) + RightalignmentCorrection, FMIN_PWM_LEFT, FMAX_PWM_LEFT);
            // } else if (RightalignmentError < 0) { // Veer left
            //     pwmRight = constrain(((FMAX_PWM_RIGHT + FMIN_PWM_RIGHT) / 2) + RightalignmentCorrection, FMIN_PWM_RIGHT, FMAX_PWM_RIGHT);
            //     pwmLeft = constrain(((FMAX_PWM_LEFT + FMIN_PWM_LEFT) / 2) - RightalignmentCorrection, FMIN_PWM_LEFT, FMAX_PWM_LEFT);
            // }
            
        }

        // Drive motors
        driveMotors(pwmRight, pwmLeft);

        // Update for next iteration
        previousAlignmentError = alignmentError;
        // previousLeftAlignmentError = LeftalignmentError;
        // previousRightAlignmentError = RightalignmentError;
        lastUpdateTime = currentTime;
    } else {
        stopMotors();
        isExecutingCommand = false;
    }
}

void executeDriveRotate() {
    const float Kp_left = 1.05;  // Proportional gain for left motor // best so far was 1.08, 220 left max 
    const float Kp_right = 1.0; // Proportional gain for right motor
    const int MIN_PWM_RIGHT = 160;
    const int MAX_PWM_RIGHT = 200;
    const int MIN_PWM_LEFT = 180;
    const int MAX_PWM_LEFT = 220;
    const float threshold = 0.5; // Error threshold for stopping rotation

    // Reset the gyroscope baseline
    readGyroData();
    float initialRoll = roll;
    float targetRoll = initialRoll + targetAngle;

    // Rotate the robot
    while (abs(targetRoll - roll) > threshold) {
        readGyroData(); // Continuously update roll
        float error = targetRoll - roll;

        // Calculate PWM using proportional control
        int PWM_right = constrain(Kp_right * abs(error), MIN_PWM_RIGHT, MAX_PWM_RIGHT);
        int PWM_left = constrain(Kp_left * abs(error), MIN_PWM_LEFT, MAX_PWM_LEFT);

        int direction = (error > 0) ? 1 : -1; // Determine CW or CCW rotation
        digitalWrite(in1, direction > 0 ? LOW : HIGH);
        digitalWrite(in2, direction > 0 ? HIGH : LOW);
        analogWrite(enA, PWM_right);
        digitalWrite(in3, direction > 0 ? HIGH : LOW);
        digitalWrite(in4, direction > 0 ? LOW : HIGH);
        analogWrite(enB, PWM_left);

        //delay(10); // Allow time for sensor readings
    }

  // Stop the motors
  stopMotors();
  isExecutingCommand = false;

}

void readGyroData() {
  // === Read accelerometer and gyroscope data === //
  // Code to read MPU6050 sensor and update roll, pitch, yaw

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;

  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 2.34;

  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000;

  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;

  GyroX = GyroX + 0.30;
  gyroAngleX = gyroAngleX + GyroX * elapsedTime;
  roll = 0.95 * gyroAngleX + 0.05 * accAngleX;
}

// Function to drive motors with specified PWM
void driveMotors(int pwmRight, int pwmLeft) {
    analogWrite(enA, pwmRight);
    analogWrite(enB, pwmLeft);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
}

// Function to stop all motors
void stopMotors() {
    analogWrite(enA, 0);
    analogWrite(enB, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
}

void executeBucketMotion() {
  if (Bucket_Action == "L") {  // Move first servo to 0 degrees
    if (!Servo1.attached()) Servo1.attach(servoPin1); // Attach Servo1 if not attached
    for (int pos = 270; pos >= 0; pos--) {
        Servo1.write(pos);
        delay(5);  // Adjust the delay to control speed
        // sweeping = true;
        // sweepStartAngle = 0; // Set starting position for sweeper
        // sweepAngle = sweepStartAngle; // Reset sweep angle to start
        // sweepDirection = 1; // Sweep upward to 180°
    }
    Servo1.detach(); // Detach Servo1 to save power
    if (!Servo2.attached()) Servo2.attach(servoPin2); // Attach Servo2 if not attached
    // Move the sweeper from 180° to 0° (downward)
    for (int pos = 0; pos <= 180; pos++) {
        Servo2.write(pos);  // Rotate the sweeper servo
        delay(15);  // Adjust delay for sweeping speed (slower speed)
    }
    Servo2.detach(); // Detach Servo2 to save power
} 
else if (Bucket_Action == "R") {  // Move sweeper first, then move bucket
    if (!Servo2.attached()) {
        Servo2.attach(servoPin2); // Attach the sweeper servo if not already attached
    }
    // Move the sweeper from 180° to 0° (downward)
    for (int pos = 180; pos >= 0; pos--) {
        Servo2.write(pos);  // Rotate the sweeper servo
        delay(15);  // Adjust delay for sweeping speed (slower speed)
    }

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
} else if (Bucket_Action == "STOP") {  // Stop sweeping
    if (Servo2.attached()) {
        Servo2.detach(); // Detach Servo2 to save power
    }
}
isExecutingCommand = false;
}
