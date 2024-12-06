#include <RPLidar.h>
#include <SoftwareSerial.h>
#include <NewPing.h>

// LiDAR and Bluetooth communication
RPLidar lidar;
const int bluetoothRxPin = 10; // RX ON ATHE ARDUINO IS TX FROM THE HC05
const int bluetoothTxPin = 11; // TX ON THE ARDUINO IS RX FROM THE HC05
SoftwareSerial bluetooth(bluetoothRxPin, bluetoothTxPin);

//Ultrasonic sensor pins and setup
// const int numSensors = 6;  // Total number of ultrasonic sensors
// const int triggerPins[numSensors] = {48, 52, 44, 42, 50, 46};
// const int echoPins[numSensors] = {49, 53, 45, 43, 51, 47};
 const int numSensors = 5;  // Total number of ultrasonic sensors
const int triggerPins[numSensors] = {48, 52, 44, 50, 46};
// 48 and 49 are FRONT SENSOR 
// 52 and 53 are RIGHT SENSOR 
// 44 and 45 are LEFT SENSOR 
// 50 and 51 are FRONT RIGHT DIAGONAL SENSOR (30 - 45 degree angled)
// 46 and 47 are FRONT RIGHT DIAGONAL SENSOR (30 - 45 degree angled)

const int echoPins[numSensors] = {49, 53, 45, 51, 47};
const int MAX_DISTANCE_CM = 508; // Maximum distance in cm
// String sensorData[numSensors];  // Store ultrasonic sensor data as strings

NewPing sonars[numSensors] = {
    NewPing(triggerPins[0], echoPins[0], MAX_DISTANCE_CM), 
    NewPing(triggerPins[1], echoPins[1], MAX_DISTANCE_CM), 
    NewPing(triggerPins[2], echoPins[2], MAX_DISTANCE_CM),
    NewPing(triggerPins[3], echoPins[3], MAX_DISTANCE_CM),
    NewPing(triggerPins[4], echoPins[4], MAX_DISTANCE_CM)
};

// LiDAR state variables
int fullRotations = 0;
bool rotationStarted = false;
float distances[36];
bool hasData[36];
const float tolerance = 2.5;
const int maxRotations = 20;

// Timing variables for non-blocking logic
unsigned long lastSensorPollTime = 0;
const unsigned long sensorPollInterval = 50; // Poll sensors every 50 ms

void setup() {
    // Initialize communication
    // lidar.begin(Serial2);
    Serial.begin(115200); // For debugging
    Serial1.begin(115200);  // For Uno communication
    bluetooth.begin(9600); // For Python communication

    // Initialize ultrasonic sensor pins
    for (int i = 0; i < numSensors; i++) {
        pinMode(triggerPins[i], OUTPUT);
        pinMode(echoPins[i], INPUT);
    }

    // Initialize LiDAR
    // initializeLidar();

    // Serial.println("Mega setup complete. Waiting for commands...");
}

void loop() {
    // // 1. Periodically poll ultrasonic sensors
    if (millis() - lastSensorPollTime >= sensorPollInterval) {
        lastSensorPollTime = millis();
        gatherSensorData();
        sendUltrasonicDataToUno();
    }
    //debugging
        // gatherSensorData();
    // 2. Check for incoming Bluetooth commands
    if (bluetooth.available()) {
        String command = bluetooth.readStringUntil('\n');
        command.trim();
        handleBluetoothCommand(command);
    }

    // 3. Handle incoming commands from Uno or Python
    if (Serial1.available()) {
        String response = Serial1.readStringUntil('\n'); // Read the complete response
        handleUnoResponse(response); // Pass the response to the function
    }
}

// Function to handle Bluetooth commands from Python
void handleBluetoothCommand(String command) {
    if (command == "[LD]") {
        // Handle LiDAR data collection
        bluetooth.println("ACK");
        Serial.println("Command received: [LD]");
        // collectLidarData();
    } else if (command == "[UD]") {
        // Handle ultrasonic data collection
        bluetooth.println("ACK");
        Serial.println("Command received: [UD]");
        gatherSensorData();
        sendUltrasonicDataToPython();
    } else {
        // Forward other commands to the Uno
        Serial1.println(command);
        bluetooth.println("ACK");
    }
}

// Function to gather ultrasonic sensor data - OLD 
// void gatherSensorData() {
//     String sensorData[numSensors];
//     unsigned long startTime = millis(); // Start the timer

//     for (int i = 0; i < numSensors; i++) {
//         float totalDistance = 0;
//         const int numReadings = 3; // Average over 3 readings
//         int validReadings = 0;

//         for (int j = 0; j < numReadings; j++) {
//             float distance = sonars[i].ping_cm();
//             float distanceInches = distance / 2.54; // Convert to inches
//             if (distanceInches > 0) {
//                 totalDistance += distanceInches;
//                 validReadings++;
//             }
//             delay(10);
//         }

//         if (validReadings > 0) {
//             sensorData[i] = String(totalDistance / validReadings); // Average distance
//         } else {
//             sensorData[i] = "200"; // Default for no valid readings
//         }
//     }

//     unsigned long endTime = millis(); // End the timer
//     unsigned long elapsedTime = endTime - startTime; // Calculate elapsed time
//     Serial.print("Time taken to gather sensor data: ");
//     Serial.print(elapsedTime);
//     Serial.println(" ms");
//     for (int i = 0; i < 6; i++) {
//         Serial.print("Sensor ");
//         Serial.print(i);
//         Serial.print(": ");
//         Serial.println(sensorData[i]);
//     }
// }
// NEW IMPROVED gather UT DATA 

// Reduced delays: delay(10) replaced with a minimal delayMicroseconds(100).
// Fewer readings: Reduced numReadings from 3 to 2, halving the inner loop time.
// Efficient storage: Using float instead of String reduces memory overhead.

// Global sensor data array
float sensorData[numSensors];

void gatherSensorData() {
    for (int i = 0; i < numSensors; i++) {
        float totalDistance = 0;
        int validReadings = 0;

        for (int j = 0; j < 2; j++) { // Reduced to 2 readings
            float distance = sonars[i].ping_cm() / 2.54; // Convert to inches
            if (distance > 0) {
                totalDistance += distance;
                validReadings++;
            }
            delayMicroseconds(50); // Reduced delay
        }

        sensorData[i] = validReadings > 0 ? totalDistance / validReadings : -1.0; // Use float
    }
}

    // unsigned long elapsedTime = millis() - startTime; // Calculate elapsed time
    // Serial.print("Time taken: ");
    // Serial.print(elapsedTime);
    // Serial.println(" ms");

    // for (int i = 0; i < numSensors; i++) {
    //     Serial.print("Sensor ");
    //     Serial.print(i);
    //     Serial.print(": ");
    //     Serial.println(sensorData[i]);
    // }


// Function to send ultrasonic data to Uno
void sendUltrasonicDataToUno() {
    String message = "L:" + String(sensorData[2]) + ",R:" + String(sensorData[1]);
    Serial1.println(message);
}

// // Function to send ultrasonic data to Python
void sendUltrasonicDataToPython() {
    for (int i = 0; i < numSensors; i++) {
        bluetooth.print(sensorData[i]);
        if (i < numSensors - 1) {
            bluetooth.print(",");
        }
    }
    bluetooth.println();
}

// Function to handle responses from the Uno
// void handleUnoResponse(String response) {
//     Serial.println("Uno Response: " + response); // For debugging or further action
// }
// void handleUnoResponse() {
//     if (Serial1.available()) {
//         String response = Serial1.readStringUntil(">");
//         response = response.substring(response.indexOf("<") + 1);
//         Serial.println("Uno Response: " + response);
//     }
// }

void handleUnoResponse(String response) {
    response = response.substring(response.indexOf("<") + 1); // Extract the data between < and >
    Serial.println("Uno Response: " + response); // Print the formatted response
}

// Function to initialize LiDAR
void initializeLidar() {
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
        lidar.startScan();
        Serial.println("LiDAR initialized.");
    } else {
        Serial.println("Failed to initialize LiDAR.");
    }
}

// Function to collect LiDAR data
void collectLidarData() {
    while (fullRotations < maxRotations) {
        if (IS_OK(lidar.waitPoint())) {
            float distance = lidar.getCurrentPoint().distance;
            float angle = lidar.getCurrentPoint().angle;
            byte quality = lidar.getCurrentPoint().quality;

            // Detect full rotations based on angle
            if (!rotationStarted && angle < 10) {
                rotationStarted = true;
                fullRotations++;
                if (fullRotations >= maxRotations) {
                    sendLidarDataToPython();
                    resetLidarData();
                    break;
                }
            } else if (rotationStarted && angle > 350) {
                rotationStarted = false;
            }

            // Store LiDAR data
            for (int i = 0; i < 36; i++) {
                float targetAngle = i * 10.0;
                if (quality > 10 && distance < 2500 && distance >= 100 &&
                    angle >= (targetAngle - tolerance) && angle <= (targetAngle + tolerance)) {
                    distances[i] = distance / 25.4; // Convert to inches
                    hasData[i] = true;
                    break;
                }
            }
        }
        delay(10); // Prevent excessive CPU usage
    }
}

// Function to send LiDAR data to Python
void sendLidarDataToPython() {
    for (int i = 0; i < 36; i++) {
        bluetooth.print(hasData[i] ? distances[i] : -1);
        if (i < 35) {
            bluetooth.print(",");
        }
    }
    bluetooth.println();
}

// Function to reset LiDAR data
void resetLidarData() {
    fullRotations = 0;
    for (int i = 0; i < 36; i++) {
        distances[i] = -1;
        hasData[i] = false;
    }
}



