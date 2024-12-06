/*
command_response_test

This Arduino sketch serves as an example to help students understand command data
parsing, response construction, and packetization. It will take any command received
over the serial connection, split out the command ID and data values, and parse the
data value as a floating point number. It will then respond to the command by echoing
back the data value. It can also toggle the built-in LED with a command "ld".
*/

/* Declarations and Constants */
String packet;
String responseString;
double DIFFERENCE = 0; // value to increment numerical data by before responding
bool DEBUG = false;    // If not debugging, set this to false to suppress debug messages

// Motor 1 (FR) Pin Definitions
int enA = 5;           // Motor speed control PWM pin
int in1 = 9;           // Motor direction pin 1
int in2 = 8;           // Motor direction pin 2

// Motor 2 (FL) Pin Definitions
int enB = 10;           // Motor speed control PWM pin
int in3 = 11;           // Motor direction pin 1
int in4 = 12;           // Motor direction pin 2



// Constants
const float wheelDiameter = 60 / 25.4;  // Diameter of the wheel in inches
const float Turn_Dia = 185 / 25.4; // Distance between both wheels in inches
const float PPR = 700; // Encoder's Pulse Per Revolution

// Variables
volatile long motCount = 0;          // Encoder A count (Right Wheel)
volatile long motCount1 = 0;         // Encoder B count (Left Wheel)
bool moveForward = false;            // Flag to trigger forward movement
bool moveSide = false;               // Flag to trigger side movement
bool rotate = false;                 // Flag to trigger rotation
float targetDistance = 0;            // Variable to store the distance to move in inches
float targetAngle = 0;               // Variable to store the rotation angle to move in degrees
float threshold = 1.0;               // Rotation threshold in degrees for when to stop
#include <Wire.h>
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;


// Hardware Serial Definitions
char FRAMESTART = '[';
char FRAMEEND = ']';
int MAX_PACKET_LENGTH = 143; // equivalent to 16 8-byte commands of format "xx:#####", with 15 delimiting commas between them
int TIMEOUT = 250;           // Hardware Serial timeout in milliseconds
int BAUDRATE = 9600;

/* Create a debug message */
void debugMessage(String msg) {
  if (DEBUG) {
    Serial.println(msg);
  }
}

/* Serial receive function */
String receiveSerial() {
  String frontmatter = "";
  String msg = "";
  char front_char = 0;
  char msg_char = 0;
  unsigned long start_time = 0;

  if (Serial.available()) {
    start_time = millis();
    while (millis() < start_time + TIMEOUT) {
      if (Serial.available()) {
        front_char = Serial.read();
        if (front_char == FRAMESTART) {
          msg += front_char;
          break;
        } else {
          frontmatter += front_char;
        }
      }
    }
    if (frontmatter.length() > 0) {
      debugMessage("Prior to FRAMESTART, received: " + frontmatter);
    }

    while (millis() < start_time + TIMEOUT) {
      if (Serial.available()) {
        msg_char = Serial.read();

        if (msg_char == FRAMESTART) {
          debugMessage("A new framestart character was received, dumping: " + msg);
          msg = "";
        }

        msg += msg_char;
        if (msg_char == FRAMEEND) {
          break;
        }
      }
    }

    if (msg.length() < 1) {
      debugMessage("Timed out without receiving any message.");
    } else if (msg_char != FRAMEEND) {
      debugMessage("Timed out while receiving a message.");
    } else {
      debugMessage("Depacketizing received message:" + msg);
      return depacketize(msg);
    }
  }

  return "";
}

/* Remove packet framing information */
String depacketize(String msg) {
  if (msg.length() > 1 && msg[0] == FRAMESTART) {
    if (msg[msg.length()-1] == FRAMEEND) {
      return msg.substring(1, msg.length()-1);
    }
  }
  debugMessage("Missing valid packet framing characters, instead detected these: " + String(msg[0]) + ',' + String(msg[msg.length()-1]));
  return "";
}

/* Add packet framing information */
String packetize(String msg) {
  return FRAMESTART + msg + FRAMEEND;
}

/* Handle the received packet */
String parsePacket(String pkt) {
  int cmdStartIndex = 0;
  String responseString = "";
  for (int ct = 0; ct < pkt.length(); ct++) {
    if (pkt[ct] == ',') {
      responseString += parseCmd(pkt.substring(cmdStartIndex, ct)) + ',';
      cmdStartIndex = ct + 1;
    }
  }
  responseString += parseCmd(pkt.substring(cmdStartIndex));
  debugMessage("Response String is: " + responseString);
  return responseString;
}

/* Handle the received commands (including driving forward by a variable distance) */
String parseCmd(String cmdString) {
  debugMessage("Parsing command: " + cmdString);

  String cmdID = cmdString.substring(0, min(2, cmdString.length()));
  double data = 0;
  if (cmdString.length() >= 4) {
    data = cmdString.substring(3).toDouble();
  }

  debugMessage("Command ID is: " + cmdID);
  debugMessage("The parsed data string is:" + String(data));

  // if (cmdID == "ld") {
  //   bool led_state = digitalRead(LED_BUILTIN);
  //   digitalWrite(LED_BUILTIN, !led_state);
  //   return cmdID + ':' + (!led_state ? "True" : "False");
  // }

  if (cmdID == "w0") {
    targetDistance = data;
    moveForward = true;
    return cmdID + ":Moving " + String(data) + " inches";
  }

  if (cmdID == "r0") {
    targetAngle = data;
    rotate = true;
    return cmdID + ":Rotating " + String(data) + " degrees";
  }

  return cmdID + ':' + String(data + DIFFERENCE);
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

/* Original Function to drive the motor forward by a specified distance in inches - TIME BASED NO FEEDBACK*/
// void driveForward(float inches) {
//   // // Read the initial roll value and set it as the baseline           ((0.6237 * x) - 0.2772)
//   // readGyroData(); // Get the current roll reading         
//   // float initialRoll = roll;
//   // float Desired_Angle = 0;
//   // float targetRoll = initialRoll + Desired_Angle;
//   // float error = targetRoll - roll;
//   int enA_PWM = 207;
//   int enB_PWM = 232;
//   // float drive_duration = (3000);     //165 
//   // float kp = 1.1;
//   unsigned long start_time = millis();
//   readGyroData(); // Get the current roll reading
//   delay(10);         
//   float initialRoll = roll;
//   while (millis() - start_time < drive_duration)  {
//       // Read the initial roll value and set it as the baseline           ((0.6237 * x) - 0.2772)
//     readGyroData(); // Get the current roll reading

//     delay(10);      
//     float currentRoll = roll;
//     float error = currentRoll - initialRoll;
//     // int correction = kp * error;
//     if (inches > 0) {  // Moving forward
//       digitalWrite(in1, HIGH);
//       digitalWrite(in2, LOW);
//       analogWrite(enA, enA_PWM);
//       digitalWrite(in3, HIGH);
//       digitalWrite(in4, LOW);
//       analogWrite(enB, enB_PWM);
//       // if (error > 1)  {
//       //   digitalWrite(in1, HIGH);
//       //   digitalWrite(in2, LOW);
//       //   analogWrite(enA, enA_PWM + correction);
//       //   digitalWrite(in3, HIGH);
//       //   digitalWrite(in4, LOW);
//       //   analogWrite(enB, enB_PWM);

//       // }
      

//       // else if (error < -1) {
//       //   digitalWrite(in1, HIGH);
//       //   digitalWrite(in2, LOW);
//       //   analogWrite(enA, enA_PWM - correction); // 
//       //   digitalWrite(in3, HIGH);
//       //   digitalWrite(in4, LOW);
//       //   analogWrite(enB, enB_PWM);
//       // }

//       // else {
//       // digitalWrite(in1, HIGH);
//       // digitalWrite(in2, LOW);
//       // analogWrite(enA, enA_PWM);
//       // digitalWrite(in3, HIGH);
//       // digitalWrite(in4, LOW);
//       // analogWrite(enB, enB_PWM);
//       // }

//       }

//     if (inches < 0) {  // Moving backward
//       digitalWrite(in1, LOW);
//       digitalWrite(in2, HIGH);
//       analogWrite(enA, enA_PWM);
//       digitalWrite(in3, LOW);
//       digitalWrite(in4, HIGH);
//       analogWrite(enB, enB_PWM);
//     }}

//   analogWrite(enA, 0);
//   digitalWrite(in1, HIGH);
//   digitalWrite(in2, HIGH);
//   analogWrite(enB, 0);
//   digitalWrite(in3, HIGH);
//   digitalWrite(in4, HIGH);
// }
// TEST 1 - WITH PID 
// void driveForward(float inches) {
//   // PID tuning parameters
//   float kp = 1.1;  // Proportional gain
//   float ki = 0.00; // Integral gain
//   float kd = 0.0;  // Derivative gain

//   // PID variables
//   float previousError = 0;
//   float integral = 0;
//   float dt = 0.01; // Time step in seconds

//   // Read the initial roll value
//   readGyroData(); // Get the current roll reading
//   float initialRoll = roll;
//   // int enA_PWM = 207; // Baseline PWM for Motor A
//   // int enB_PWM = 232; // Baseline PWM for Motor B
//   unsigned long start_time = millis();
//   float drive_duration = 10000;
//   kp_f_left = 1.0
//   kp_f_right = 1.0 

//   const int FMIN_PWM_RIGHT = 120;
//   const int FMAX_PWM_RIGHT = 210;
//   const int FMIN_PWM_LEFT = 140;
//   const int FMAX_PWM_LEFT = 210;
//   while (millis() - start_time < drive_duration) {
//       readGyroData(); // Continuously update the roll reading
//       float currentRoll = roll;
//       float error = currentRoll - initialRoll;

//       // PID calculations
//       integral += error * dt;                     // Accumulate the error
//       float derivative = (error - previousError) / dt; // Rate of change of error
//       float correction_left = (kp_f_left * error) + (ki * integral) + (kd * derivative);
//       float correction_right = (kp_f_right * error) + (ki * integral) + (kd * derivative);
//       previousError = error;

//       // Adjust motor speeds
//       // int adjusted_enA_PWM = enA_PWM - correction; // Reduce speed for correction
//       // int adjusted_enB_PWM = enB_PWM + correction; // Increase speed for correction

//       // Clamp PWM values to a valid range (0 to 255)
//       // adjusted_enA_PWM = constrain(adjusted_enA_PWM, , 255);
//       // adjusted_enB_PWM = constrain(adjusted_enB_PWM, 0, 255);
//           // Calculate PWM using proportional control
//       adjust_FPWM_right = constrain(correction_right, FMIN_PWM_RIGHT, FMAX_PWM_RIGHT);
//       adjust_FPWM_left = constrain(correction_left, FMIN_PWM_LEFT, FMAX_PWM_LEFT);

//       // Drive the motors
//       if (inches > 0) { // Moving forward
//           digitalWrite(in1, HIGH);
//           digitalWrite(in2, LOW);
//           analogWrite(enA, adjust_FPWM_right);
//           digitalWrite(in3, HIGH);
//           digitalWrite(in4, LOW);
//           analogWrite(enB, adjust_FPWM_left);
//       } else if (inches < 0) { // Moving backward
//           digitalWrite(in1, LOW);
//           digitalWrite(in2, HIGH);
//           analogWrite(enA, adjust_FPWM_right);
//           digitalWrite(in3, LOW);
//           digitalWrite(in4, HIGH);
//           analogWrite(enB, adjust_FPWM_left);
//       }
//       // delay(dt * 1000); // Delay to match time step
//   }


// // Stop motors after the loop exits
// analogWrite(enA, 0);
// digitalWrite(in1, HIGH);
// digitalWrite(in2, HIGH);
// analogWrite(enB, 0);
// digitalWrite(in3, HIGH);
// digitalWrite(in4, HIGH);
// }
// TEST 2 - PID but only P control of each wheel 
// can try 1 instead of  2 and increase left PWM by 5 
void driveForward(float inches) {
    // Constants and PID tuning parameters
    float kp_f_left = 1.25;  // Proportional gain for left drive
    float kp_f_right = 1.0; // Proportional gain for right drive
    const float threshold = 2.0; // Maximum allowable angular error in degrees
    float ki = 0.0;         // Integral gain (not used)
    float kd = 0.0;         // Derivative gain (not used)
    // Modelled equation = d = 9.71 * t + 1.02 
    // PID variables
    float previousError = 0;
    float integral = 0;

    // PWM limits
    const int FMIN_PWM_RIGHT = 130;
    const int FMAX_PWM_RIGHT = 220;
    const int FMIN_PWM_LEFT = 140;
    const int FMAX_PWM_LEFT = 220;

    // Calculate drive duration based on distance and estimated speed
    // const float speedInchesPerSecond = 3.0; // Estimated speed in inches per second
    // float drive_duration = inches / speedInchesPerSecond * 1000; // Convert to milliseconds
    float drive_duration = ((inches) / 9.71) *1000; 

    // Read the initial roll value
    readGyroData(); // Get the current roll reading
    float initialRoll = roll;

    unsigned long start_time = millis();

    while (millis() - start_time < drive_duration) {
        // Read gyroscope data
        readGyroData();
        float currentRoll = roll;
        float error = currentRoll - initialRoll;
        delay(10);
        // Check if error exceeds the threshold
        if (abs(error) > threshold) {
            // Apply corrections dynamically
            float correction_left = kp_f_left * error;
            float correction_right = kp_f_right * error;

            // Calculate adjusted PWM values
            int adjust_FPWM_right = constrain(FMIN_PWM_RIGHT - correction_right, FMIN_PWM_RIGHT, FMAX_PWM_RIGHT);
            int adjust_FPWM_left = constrain(FMAX_PWM_LEFT + correction_left, FMIN_PWM_LEFT, FMAX_PWM_LEFT);

            // Drive the motors with corrections
            digitalWrite(in1, HIGH);
            digitalWrite(in2, LOW);
            analogWrite(enA, adjust_FPWM_right);
            digitalWrite(in3, HIGH);
            digitalWrite(in4, LOW);
            analogWrite(enB, adjust_FPWM_left);
        } else {
            // If within threshold, drive motors at baseline PWM
            analogWrite(enA, FMAX_PWM_RIGHT);
            analogWrite(enB, FMAX_PWM_LEFT);
            digitalWrite(in1, HIGH);
            digitalWrite(in2, LOW);
            digitalWrite(in3, HIGH);
            digitalWrite(in4, LOW);
        }

        delay(10); // Small delay to stabilize loop timing
    }

    // Stop the motors after reaching the target distance
    analogWrite(enA, 0);
    analogWrite(enB, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
}
// Test 3 - PID with active PWM direct amplification - not effective at all (tried kp left 0.1 higher and changing PWM min values by 20 degrees)
	// void driveForward(float inches) {
	//     // Proportional gain for left and right motors
	//     float Kp_right = 1.0;
	//     float Kp_left = 1.1;
	//     // Gyroscope threshold for acceptable alignment
	//     const float threshold = 2.0; // Degrees
	//     // PWM limits
	//     const int MIN_PWM_RIGHT = 120; // Added 20
	//     const int MAX_PWM_RIGHT = 210;
	//     const int MIN_PWM_LEFT = 160;
	//     const int MAX_PWM_LEFT = 210;
	//     // Calculate drive duration based on distance and estimated speed
	//     const float speedInchesPerSecond = 3.0; // Estimated speed in inches per second
	//     float drive_duration = inches / speedInchesPerSecond * 1000; // Convert to milliseconds
	//     // Read the initial roll value
	//     readGyroData(); // Get the current roll reading
	//     float initialRoll = roll;
	//     unsigned long start_time = millis();
	//     while (millis() - start_time < drive_duration) {
	//         // Read gyroscope data
	//         readGyroData();
  //         delay(10);
	//         float currentRoll = roll;
	//         float error = currentRoll - initialRoll;
	//         // Apply threshold to avoid unnecessary small corrections
	//         if (abs(error) < threshold) {
	//             error = 0;
	//         }
	//         // Calculate PWM using proportional control
	//         int PWM_right = constrain(Kp_right * abs(error), MIN_PWM_RIGHT, MAX_PWM_RIGHT);
	//         int PWM_left = constrain(Kp_left * abs(error), MIN_PWM_LEFT, MAX_PWM_LEFT);
	//         // Drive motors with calculated PWM values
	//         digitalWrite(in1, HIGH);
	//         digitalWrite(in2, LOW);
	//         analogWrite(enA, PWM_right); // Adjusted speed for right motor
	//         digitalWrite(in3, HIGH);
	//         digitalWrite(in4, LOW);
	//         analogWrite(enB, PWM_left);  // Adjusted speed for left motor
	//         delay(10); // Small delay for stability
	//     }
	//     // Stop the motors after reaching the target distance
	//     analogWrite(enA, 0);
	//     analogWrite(enB, 0);
	//     digitalWrite(in1, LOW);
	//     digitalWrite(in2, LOW);
	//     digitalWrite(in3, LOW);
	//     digitalWrite(in4, LOW);
	// }

// Original Drive Rotate - works but overshoots and also has random double rotation bug sometimes 
// void driveRotate(float targetAngle) {
//     int enA_PWM = 110; // RM
//     int enB_PWM = 135; // LM 
//   // Read the initial roll value and set it as the baseline           ((0.6237 * x) - 0.2772)
//   readGyroData(); // Get the current roll reading         
//   float initialRoll = roll;
//   float targetRoll = initialRoll + targetAngle;

//   while (abs(roll - targetRoll) > threshold) { // Loop until within threshold
//     readGyroData(); // Continuously update roll from the MPU6050

//     float error = targetRoll - roll;


//     // Set motor speed based on the direction to rotate
//     if (error > 0) { // Rotate CW
//       digitalWrite(in1, LOW);
//       digitalWrite(in2, HIGH);
//       analogWrite(enA, enA_PWM);
//       digitalWrite(in3, HIGH);
//       digitalWrite(in4, LOW);
//       analogWrite(enB, enB_PWM);
//     } else { // Rotate CCW
//       digitalWrite(in1, HIGH);
//       digitalWrite(in2, LOW);
//       analogWrite(enA, enA_PWM);
//       digitalWrite(in3, LOW);
//       digitalWrite(in4, HIGH);
//       analogWrite(enB, enB_PWM);
//     }

//     // Small delay to allow sensor readings to update
//     delay(10);
//   }

//   // Stop the motors once target angle is within threshold
//   analogWrite(enA, 100);
//   digitalWrite(in1, HIGH);
//   digitalWrite(in2, HIGH);
//   analogWrite(enB, 100);
//   digitalWrite(in3, HIGH);
//   digitalWrite(in4, HIGH);
// }

// TEST 1 CLEANER LOGIC - Still sees dragging of one motor but is stable 
// void driveRotate(float targetAngle) {
//     float Kp = 1.1; // Proportional gain constant (tune this experimentally)
//     float threshold = 1.0; // Error threshold for stopping the rotation

//     // Reset the gyroscope baseline
//     readGyroData();
//     float initialRoll = roll;
//     float targetRoll = initialRoll + targetAngle;

//     // Rotate the robot
//     while (abs(targetRoll - roll) > threshold) {
//         readGyroData(); // Continuously update roll
//         float error = targetRoll - roll;

//         // Calculate proportional motor speed
//         int PWM = abs(Kp * error); // Proportional term
//         PWM_right = constrain(PWM, 120, 200); // Clamp PWM between minimum (60) and maximum (255)
//         PWM_left = constrain(PWM, 140, 200); // Clamp PWM between minimum (60) and maximum (255)
//         int direction = (error > 0) ? 1 : -1; // Determine CW or CCW rotation
//         digitalWrite(in1, direction > 0 ? LOW : HIGH);
//         digitalWrite(in2, direction > 0 ? HIGH : LOW);
//         analogWrite(enA, PWM_right);
//         digitalWrite(in3, direction > 0 ? HIGH : LOW);
//         digitalWrite(in4, direction > 0 ? LOW : HIGH);
//         analogWrite(enB, PWM_left);

//         delay(10); // Allow time for sensor readings
//     }

//     // Stop the motors
//     analogWrite(enA, 0);
//     digitalWrite(in1, LOW);
//     digitalWrite(in2, LOW);
//     analogWrite(enB, 0);
//     digitalWrite(in3, LOW);
//     digitalWrite(in4, LOW);
// }

// TEST 2 IMPROVE DOUBLE P CONTROL AND DOUBLE PWM RANGE CONTROL - Deal with inherent motor inconsistencies 
void driveRotate(float targetAngle) {
    const float Kp_left = 1.1;  // Proportional gain for left motor
    const float Kp_right = 1.0; // Proportional gain for right motor
    const int MIN_PWM_RIGHT = 120;
    const int MAX_PWM_RIGHT = 210;
    const int MIN_PWM_LEFT = 140;
    const int MAX_PWM_LEFT = 210;
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

        delay(10); // Allow time for sensor readings
    }

    // Stop the motors
    analogWrite(enA, 0);
    analogWrite(enB, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
}

void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    delay(5);
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
  Serial.println("");
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  Serial.begin(BAUDRATE);
  //pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  calculate_IMU_error();
  delay(20);
}

void loop() {
  if (Serial.available()) {
    packet = receiveSerial();
    if (packet.length() > 0) {
      responseString = parsePacket(packet);
      Serial.println(packetize(responseString));
    }
  }
  if (moveForward) {
    driveForward(targetDistance);
    moveForward = false;
  }

  if (rotate) {
    driveRotate(targetAngle);
    rotate = false;
  }

}
