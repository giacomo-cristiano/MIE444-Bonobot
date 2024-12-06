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
bool STOP = false;


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

  if (cmdID == "xx")  {
    STOP = true;
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

void driveForward(float inches) {
    // Constants and PID tuning parameters
    STOP = false;
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
      // if (Serial.available()) {
      //   packet = receiveSerial();
      //   if (packet.length() > 0) {
      //       parsePacket(packet);
      //       if (STOP == true) {
      //         // Stop the motors after reaching the target distance
      //         analogWrite(enA, 0);
      //         analogWrite(enB, 0);
      //         digitalWrite(in1, LOW);
      //         digitalWrite(in2, LOW);
      //         digitalWrite(in3, LOW);
      //         digitalWrite(in4, LOW);
      //         break;
      //       }
      //   }}
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


// TEST 2 IMPROVE DOUBLE P CONTROL AND DOUBLE PWM RANGE CONTROL - Deal with inherent motor inconsistencies 
void driveRotate(float targetAngle) {
    const float Kp_left = 1.1;  // Proportional gain for left motor
    const float Kp_right = 1.0; // Proportional gain for right motor
    const int MIN_PWM_RIGHT = 120;
    const int MAX_PWM_RIGHT = 210;
    const int MIN_PWM_LEFT = 160;
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
