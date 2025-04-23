# Computer Vision Driven Robotic Arm

## 📋 Project Overview

This project implements a sophisticated system that allows a robotic arm to mimic human hand movements in real-time. By integrating IoT technology with artificial intelligence, the system captures and processes hand gestures through computer vision techniques and translates them into precise robotic arm movements.



# Hand Gesture Recognition

## 🌟 Key Features

- **Real-time Hand Tracking**: Detects and tracks hand movements using advanced computer vision algorithms
- **Finger Position Recognition**: Identifies individual finger positions and states (open/closed)
- **Orientation Detection**: Determines hand rotation for 3-axis movement control
- **Gesture-to-Motion Translation**: Converts detected hand gestures into corresponding robotic arm commands
- **High Precision Control**: Provides accurate finger-by-finger control of the robotic arm

## 🛠️ Technologies & Dependencies

### Core Libraries
- **MediaPipe**: For hand landmark detection and tracking
- **OpenCV (cv2)**: For image processing and camera input handling


## 💻 Implementation Details

### 🏗️ Architecture Overview
The system is built using a modular approach with two main components:
1. **Hand Detection Module**: Processes video input and extracts hand landmarks
2. **Main Processing System**: Translates landmark data into robotic control signals

### 🖐️ Hand Detection Module

The `handDetector` class serves as the foundation for hand tracking and landmark detection:

```python
class handDetector():
    def __init__(self, mode=False, maxHands=2, detectionCon=0.5, trackCon=0.5):
        self.mode = mode
        self.maxHands = maxHands
        self.detectionCon = detectionCon
        self.trackCon = trackCon
        
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands, 
                                        self.detectionCon, self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils
        
    def findHands(self, img, draw=True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)
        
        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(img, handLms, 
                                              self.mpHands.HAND_CONNECTIONS)
        return img
    
    def findPosition(self, img, handNo=0, draw=True):
        lmList = []
        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handNo]
            for id, lm in enumerate(myHand.landmark):
                h, w, c = img.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                lmList.append([id, cx, cy])
                if draw:
                    cv2.circle(img, (cx, cy), 7, (255, 0, 0), cv2.FILLED)
        return lmList
```


### 🔄 Main Processing System

The main system performs several key functions:

1. **Video Capture and Processing**:
```python
cap = cv2.VideoCapture(0)
detector = handDetector(detectionCon=0.75)

while True:
    success, img = cap.read()
    img = detector.findHands(img)
    lmList = detector.findPosition(img)
    
    if len(lmList) != 0:
        # Process landmark data
        # ...
```

2. **Finger State Detection**:
```python
# Process each finger
fingerOpenness = []

# Thumb processing (horizontal distance)
if lmList:
    x_thumb, y_thumb = lmList[4][1], lmList[4][2]  # Thumb tip
    x_base, y_base = lmList[13][1], lmList[13][2]  # Ring finger base
    
    thumb_dist = abs(x_thumb - x_base)
    thumb_status = 1 if thumb_dist > thumb_threshold else 0
    fingerOpenness.append(thumb_status)
    
    # Process remaining fingers using Euclidean distance
    for finger_tip, finger_base in [(8,5), (12,9), (16,13), (20,17)]:
        tip_x, tip_y = lmList[finger_tip][1], lmList[finger_tip][2]
        base_x, base_y = lmList[finger_base][1], lmList[finger_base][2]
        
        distance = math.sqrt((tip_x - base_x)**2 + (tip_y - base_y)**2)
        finger_status = 1 if distance > finger_thresholds[finger_tip//4] else 0
        fingerOpenness.append(finger_status)
```

3. **Rotation Detection**:
```python
# Hand rotation detection
x_pinky_base, y_pinky_base = lmList[17][1], lmList[17][2]
x_wrist, y_wrist = lmList[0][1], lmList[0][2]

horizontal_dist = x_pinky_base - x_wrist

if horizontal_dist < rotation_thresholds[0]:
    rotation = 0
elif horizontal_dist < rotation_thresholds[1]:
    rotation = 90
else:
    rotation = 180
```

4. **Special Thumb Control**:
```python
# Special thumb control using thumb tip and index base
x_thumb_tip, y_thumb_tip = lmList[4][1], lmList[4][2]
x_index_base, y_index_base = lmList[5][1], lmList[5][2]

if x_thumb_tip < x_index_base:  # Thumb passes index base
    fingerOpenness[0] = 0  # Closed
else:
    fingerOpenness[0] = 1  # Open
```


## 🖐️ Hand Landmark Reference

MediaPipe provides 21 landmarks for each detected hand:

![hand landmarks](hand%20landmarks.jpg)


## 📊 Data Processing

### 📏 Distance Calculation
- **Thumb**: Horizontal distance between thumb tip (landmark 4) and ring finger base (landmark 13)
- **Other Fingers**: Euclidean distance between each finger's tip and its corresponding base
- **Hand Rotation**: Horizontal distance between pinky base (landmark 17) and wrist (landmark 0)


### ⚖️ Normalization
To accommodate different hand sizes and distances from the camera:
1. Track maximum distances for each measurement
2. Normalize current readings against recorded maximums
3. Apply thresholds to determine binary states (open/closed, rotation angles)


## 📡 Communication Protocol

The system communicates with the robotic arm using a structured data format:

```
[thumb_state]|[index_state]|[middle_state]|[ring_state]|[pinky_state]|[rotation_angle]
```

Example: `1|0|1|1|0|90` represents:
- Thumb open
- Index closed
- Middle open
- Ring open
- Pinky closed
- 90° rotation


# WebSocket Client


The **WSClient** module provides a simple, auto-reconnecting WebSocket client to communicate with an ESP8266 microcontroller. It maintains a persistent connection, retries on failure, and throttles message sends to prevent flooding.

### 📥 Imports & Configuration

```python
import websocket
import threading
import time
from config import WS_IP
```

- **websocket**: Provides WebSocket client functionality.
- **threading**: Enables background threads for non-blocking operations.
- **time**: Used for tracking and throttling send intervals.
- **config.WS_IP**: The WebSocket server URL (e.g., on your ESP8266).

---

### 🏗️ WSClient Class

```python
class WSClient:
    def __init__(self):
        self.ws = websocket.WebSocket()
        self.last_sent_time = 0
        self.connected = False

        # Start the connection in a background thread
        threading.Thread(target=self.connect_ws, daemon=True).start()
```

- **ws**: An instance of `websocket.WebSocket` to manage the socket.
- **last_sent_time**: Timestamp of the last successful send, for throttling.
- **connected**: Flag to indicate if the connection is active.
- **Background Thread**: Automatically attempts to connect without blocking the main thread.

---

### 🔌 Establish WebSocket Connection

```python
    def connect_ws(self):
        while not self.connected:
            try:
                self.ws.connect(WS_IP)
                self.connected = True
                print("[✓] Connected to ESP8266 WebSocket")
            except Exception as e:
                print(f"[!] WebSocket retrying... {e}")
                time.sleep(2)
```

- **Loop Until Connected**: Keeps retrying every 2 seconds until a connection is established.
- **Error Handling**: Catches exceptions and logs them, then waits before retrying.
- **Success Feedback**: Logs a confirmation on successful connection.

---

### 📤 send Method

```python
    def send(self, finger_data):

        if time.time() - self.last_sent_time >= 0.5:
            try:
                self.ws.send(str(finger_data))
                print(f"[→] Sent to ESP: {finger_data}")
                self.last_sent_time = time.time()
            except Exception as e:
                print(f"[!] WebSocket send failed: {e}")
                self.connected = False
                threading.Thread(target=self.connect_ws, daemon=True).start()
```

- **Throttling**: Ensures at least 0.5 seconds between sends to avoid overwhelming the server.
- **Send Payload**: Converts `finger_data` to string (or JSON) and sends it.
- **Error Recovery**: On failure, resets connection flag and spawns a reconnect thread.





# ESP8266 WebSocket Communication

implementimg WebSocket communication using an ESP8266 microcontroller to receive data from a client device (such as a mobile phone or computer) over a network and forward it to an Arduino board.


## ⚙️ Functionality

The ESP8266 performs the following functions:
- Connects to a specified WiFi network
- Establishes a WebSocket server on a designated port
- Listens for incoming WebSocket messages
- Parses received data (comma-separated values)
- Forwards the processed data to an Arduino via serial communication


## 💻 Code Structure Explanation

### Libraries and Configuration

```cpp
#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>
const char* ssid = "OPPO A55";  
const char* password = "12345678";
WebSocketsServer webSocket(81);
double data[7]; 
```

This section:
- Includes the necessary libraries for WiFi and WebSocket functionality
- Defines the WiFi credentials (SSID and password)
- Initializes the WebSocket server on port 81
- Creates an array to store up to 7 double-precision values


### Setup Function

```cpp
void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
     
    Serial.println(WiFi.localIP());
}
```

The setup function:
1. Initializes serial communication at 115200 baud rate
2. Connects to the WiFi network using the provided credentials
3. Starts the WebSocket server
4. Registers the event handler function for WebSocket events
5. Outputs the ESP8266's IP address to the Serial monitor


### WebSocket Event Handler

```cpp
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    if (type == WStype_TEXT) {
        String dataString = String((char*)payload);
        int dataIndex = 0;
        int startIdx = 0;
        int commaIdx;
        while (dataIndex < 7 && (commaIdx = dataString.indexOf(',', startIdx)) != -1) {
            String valueStr = dataString.substring(startIdx, commaIdx);
            data[dataIndex] = valueStr.toFloat();
            startIdx = commaIdx + 1;
            dataIndex++;
        }
        // Handle the last value
        if (dataIndex < 7 && startIdx < dataString.length()) {
            data[dataIndex] = dataString.substring(startIdx).toFloat();
            dataIndex++;
        }
        // Print the values in required format
        for (int i = 0; i < dataIndex; i++) {
            Serial.print(data[i], 2);  // print with 2 decimal places
            if (i < dataIndex - 1) {
                Serial.print("|");
            }
        }
        Serial.print(":\n");
    }
}
```

This function:
1. Triggers when a WebSocket message is received
2. Verifies the message is text
3. Converts the message to a String
4. Parses the comma-separated values and stores them as floats in the data array
5. Formats and sends the data to Serial:
   - Each value is printed with 2 decimal places
   - Values are separated by pipe characters (|)
   - The data string is terminated with a colon (:) and a newline


### Main Loop

```cpp
void loop() {
    webSocket.loop();
}
```

The loop function maintains the WebSocket connection by repeatedly calling `webSocket.loop()`.


## 📝 Instructions

1. Update the WiFi credentials (`ssid` and `password`) to match your network
2. Upload this code to your ESP8266 board
3. Open the Serial Monitor to view the ESP8266's IP address
4. Connect your client device to the same WiFi network
5. Establish a WebSocket connection to the ESP8266 using its IP address and port 81
6. Send comma-separated numeric values via the WebSocket connection
7. The ESP8266 will parse these values and forward them to the Arduino



# Controlling servo motors by Arduino Uno based on data received via serial communication

## 💻 Software Implementation

### Libraries Required
```cpp
#include <Servo.h>
```

### Data Structure

```cpp
struct Hand {
  Servo finger;       // Servo object
  bool done[10];      // Movement phase flags (10 intervals)
};
```

- Define a custom `Hand` struct to represent each servo-controlled finger.
- The `done` array ensures that each movement phase is executed only once.



### Serial Communication
- **Baud Rate**: 115200
- **Data Format**: `value1|value2|value3|value4|value5|value6|value7`
  - Each value controls the angle of a corresponding servo

### Setup Function
```cpp
void setup() {
  // Attach each servo to corresponding pin
  thumb.finger.attach(3);
  index.finger.attach(5);
  middle.finger.attach(6);
  ring.finger.attach(9);
  pinky.finger.attach(10);
  thumb2.finger.attach(11);
  qo3.finger.attach(12);
  
  // Initialize serial communication
  Serial.begin(115200);
  
  // Set initial positions
  thumb.finger.write(0);      // Thumb: 0°
  index.finger.write(180);    // Index: 180°
  middle.finger.write(180);   // Middle: 180°
  ring.finger.write(180);     // Ring: 180°
  pinky.finger.write(180);    // Pinky: 180°
  qo3.finger.write(90);       // Qo3: 90°
  thumb2.finger.write(180);   // Thumb2: 180°
}
```

- Initializes the servo motors and sets their default positions

### Main Loop

```cpp
void loop() {
  if (Serial.available() > 0) {
    // Parse input format: value1|value2|value3|value4|value5|value6|value7
    String data = Serial.readStringUntil('\n');
    
    // Process values for each servo
    int values[7];
    int index = 0;
    int lastIndex = 0;
    
    // Parse pipe-separated values
    for (int i = 0; i < data.length(); i++) {
      if (data.charAt(i) == '|' || i == data.length() - 1) {
        values[index] = data.substring(lastIndex, i).toInt();
        lastIndex = i + 1;
        index++;
      }
    }
    
    // Apply values to servos with any necessary transformations
    thumb.finger.write(180 - values[0]);  // Reversed
    index.finger.write(values[1]);
    middle.finger.write(values[2]);
    ring.finger.write(values[3]);
    pinky.finger.write(values[4]);
    qo3.finger.write(values[5]);
    thumb2.finger.write((values[6] / 2) + 90);  // Adjusted
  }
}
```

The main loop constantly checks for serial input and updates servo positions accordingly

#### Advanced movement control
```cpp
void servoRotation(double read, Hand hand) {
  // Divide input into 10 intervals for advanced movement control
  if (read >= 0.0 && read < 0.1 && !hand.done[0]) {
    hand.finger.write(0);
    doneFun(hand.done, 0);
  }
  else if (read >= 0.1 && read < 0.2 && !hand.done[1]) {
    hand.finger.write(20);
    doneFun(hand.done, 1);
  }
  // ...continued for all intervals
}
```

- Provides smooth, staged servo movement using 10 value intervals
- This method prevents the servo from repeating the same motion within the same range



#### Phase tracking
```cpp
void doneFun(bool done[], int num) {
  // Reset all flags except the current one
  for (int i = 0; i < 10; i++) {
    done[i] = false;
  }
  done[num] = true;
}
```

- Manages the phase-tracking array to ensure only one interval is active at a time







