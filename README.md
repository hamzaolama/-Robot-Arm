# Computer Vision Driven Robotic Arm

## 📋 Project Overview

This project implements a sophisticated system that allows a robotic arm to mimic human hand movements in real-time. By integrating IoT technology with artificial intelligence, the system captures and processes hand gestures through computer vision techniques and translates them into precise robotic arm movements.

![Robotic Arm Banner](/api/placeholder/800/200)

# HandGestureRecognition

##  Key Features

- **Real-time Hand Tracking**: Detects and tracks hand movements using advanced computer vision algorithms
- **Finger Position Recognition**: Identifies individual finger positions and states (open/closed)
- **Orientation Detection**: Determines hand rotation for 3-axis movement control
- **Gesture-to-Motion Translation**: Converts detected hand gestures into corresponding robotic arm commands
- **High Precision Control**: Provides accurate finger-by-finger control of the robotic arm

##  Technologies & Dependencies

### Core Libraries
- **MediaPipe**: For hand landmark detection and tracking
- **OpenCV (cv2)**: For image processing and camera input handling


##  Implementation Details

### Architecture Overview
The system is built using a modular approach with two main components:
1. **Hand Detection Module**: Processes video input and extracts hand landmarks
2. **Main Processing System**: Translates landmark data into robotic control signals

### Hand Detection Module

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


### Main Processing System

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


##  Hand Landmark Reference

MediaPipe provides 21 landmarks for each detected hand:

![Hand Landmarks](/api/placeholder/500/300)


##  Data Processing

### Distance Calculation
- **Thumb**: Horizontal distance between thumb tip (landmark 4) and ring finger base (landmark 13)
- **Other Fingers**: Euclidean distance between each finger's tip and its corresponding base
- **Hand Rotation**: Horizontal distance between pinky base (landmark 17) and wrist (landmark 0)


### Normalization
To accommodate different hand sizes and distances from the camera:
1. Track maximum distances for each measurement
2. Normalize current readings against recorded maximums
3. Apply thresholds to determine binary states (open/closed, rotation angles)


##  Communication Protocol

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





# ESP8266 WebSocket Communication

implementimg WebSocket communication using an ESP8266 microcontroller to receive data from a client device (such as a mobile phone or computer) over a network and forward it to an Arduino board.


## Functionality

The ESP8266 performs the following functions:
- Connects to a specified WiFi network
- Establishes a WebSocket server on a designated port
- Listens for incoming WebSocket messages
- Parses received data (comma-separated values)
- Forwards the processed data to an Arduino via serial communication


## Code Structure Explanation

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


## Usage Instructions

1. Update the WiFi credentials (`ssid` and `password`) to match your network
2. Upload this code to your ESP8266 board
3. Open the Serial Monitor to view the ESP8266's IP address
4. Connect your client device to the same WiFi network
5. Establish a WebSocket connection to the ESP8266 using its IP address and port 81
6. Send comma-separated numeric values via the WebSocket connection
7. The ESP8266 will parse these values and forward them to the Arduino





