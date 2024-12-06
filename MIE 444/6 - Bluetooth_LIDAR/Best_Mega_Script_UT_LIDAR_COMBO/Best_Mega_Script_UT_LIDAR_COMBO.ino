#include <RPLidar.h>
#include <SoftwareSerial.h>
#include <NewPing.h>

RPLidar lidar;
const int bluetoothRxPin = 10;
const int bluetoothTxPin = 11;
SoftwareSerial bluetooth(bluetoothRxPin, bluetoothTxPin);

int fullRotations = 0;
bool rotationStarted = false;

float distances[36];
bool hasData[36];
const float tolerance = 2.5;
const int maxRotations = 20;

const int numSensors = 6;
const int triggerPins[numSensors] = {48, 52, 44, 42, 50, 46};
const int echoPins[numSensors] = {49, 53, 45, 43, 51, 47};
const int MAX_DISTANCE_CM = 508;
String sensorData[numSensors];

NewPing sonars[numSensors] = {
    NewPing(triggerPins[0], echoPins[0], MAX_DISTANCE_CM),
    NewPing(triggerPins[1], echoPins[1], MAX_DISTANCE_CM),
    NewPing(triggerPins[2], echoPins[2], MAX_DISTANCE_CM),
    NewPing(triggerPins[3], echoPins[3], MAX_DISTANCE_CM),
    NewPing(triggerPins[4], echoPins[4], MAX_DISTANCE_CM),
    NewPing(triggerPins[5], echoPins[5], MAX_DISTANCE_CM)
};

void setup() {
  lidar.begin(Serial2);
  Serial.begin(115200);
  bluetooth.begin(9600);

  Serial.println("LIDAR and Ultrasonic setup complete. Waiting for command...");
  bluetooth.println("Bluetooth connection started. Waiting for command...");

  rplidar_response_device_info_t info;
  if (IS_OK(lidar.getDeviceInfo(info, 100))) {
    lidar.startScan();
    delay(1000);
  } else {
    Serial.println("Failed to detect RPLIDAR device.");
    bluetooth.println("Failed to detect RPLIDAR device.");
  }

  resetData();
}

void loop() {
  // Check for incoming commands from Bluetooth and process them immediately
  if (bluetooth.available()) {
    String command = bluetooth.readStringUntil('\n');
    command.trim();
    if (command == "[LD]") {
      bluetooth.println("ACK");  // Send acknowledgment for LIDAR command
      Serial.println("Command received: [LD]");
      collectLidarData();
    } else if (command == "[UD]") {
      bluetooth.println("ACK");  // Send acknowledgment for Ultrasonic command
      Serial.println("Command received: [UD]");
      gatherSensorData();
      sendUltrasonicData();
    }

    // Clear the Bluetooth buffer to ensure it's ready for the next command
    while (bluetooth.available()) {
      bluetooth.read();
    }
  }
}

void collectLidarData() {
  while (fullRotations < maxRotations) {
    if (IS_OK(lidar.waitPoint())) {
      float distance = lidar.getCurrentPoint().distance;
      float angle = lidar.getCurrentPoint().angle;
      byte quality = lidar.getCurrentPoint().quality;

      if (!rotationStarted && angle < 10) {
        rotationStarted = true;
        fullRotations++;

        if (fullRotations >= maxRotations) {
          sendData(); // Send LIDAR data over Bluetooth
          resetData();
          break;
        }
      } else if (rotationStarted && angle > 350) {
        rotationStarted = false;
      }

      for (int i = 0; i < 36; i++) {
        float targetAngle = i * 10.0;
        if (quality > 10 && distance < 2500 && distance >= 100 &&
            angle >= (targetAngle - tolerance) && angle <= (targetAngle + tolerance)) {
          distances[i] = distance;
          hasData[i] = true;
          break;
        }
      }
      delayMicroseconds(200);
    }
  }
}

void sendData() {
  for (int i = 0; i < 36; i++) {
    if (hasData[i]) {
      bluetooth.print(distances[i]);
    } else {
      bluetooth.print(-1);
    }
    if (i < 35) {
      bluetooth.print(",");
    }
  }
  bluetooth.println();
}

void resetData() {
  fullRotations = 0;
  for (int i = 0; i < 36; i++) {
    distances[i] = -1;
    hasData[i] = false;
  }
}

void gatherSensorData() {
  for (int i = 0; i < numSensors; i++) {
    float totalDistance = 0;
    const int numReadings = 3;
    int validReadings = 0;

    for (int j = 0; j < numReadings; j++) {
      float distance = sonars[i].ping_cm();
      float distance_in = distance / 2.54;

      if (distance_in > 0) {
        totalDistance += distance_in;
        validReadings++;
      }
      delay(20);
    }

    if (validReadings > 0) {
      float averageDistance = totalDistance / validReadings;
      sensorData[i] = String(averageDistance);
    } else {
      sensorData[i] = "200";
    }
  }
}

void sendUltrasonicData() {
  for (int i = 0; i < numSensors; i++) {
    bluetooth.print(sensorData[i]);
    if (i < numSensors - 1) {
      bluetooth.print(",");
    }
  }
  bluetooth.println();
}


