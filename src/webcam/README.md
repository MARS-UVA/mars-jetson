# webcam (Node)
Detects a connected non-Realsense webcam, then polls it on a timer. Reduces resolution of each frame and converts to grayscale, then sends the finished frame to the control station over TCP port 2026.

Code: `webcam/src/webcam.cpp`

## Publishers
None

## Subscriptions
None

## Usage
`ros2 run webcam webcam`
