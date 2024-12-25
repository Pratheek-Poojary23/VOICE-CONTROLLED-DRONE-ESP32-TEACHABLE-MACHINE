# VOICE-CONTROLLED-DRONE-ESP32-TEACHABLE-MACHINE
This project integrates ESP32 with Teachable Machine for a voice-controlled drone. The system supports dual-mode operation (voice and manual) and provides real-time responsiveness using WebSocket communication.
## Description:
"Voice-controlled drone project integrating ESP32 for hardware control and a voice recognition model trained on Teachable Machine. The ESP32 code is available in the Voice_controlled_code directory. For connecting the Teachable Machine HTML interface with ESP32, refer to the esp32-teachable-machine-led-websocket repository. Replace the my_model folder in that repository with the folder containing your trained voice recognition model from Teachable Machine."
## System Architecture Overview
## Components
The following components are required for building the voice-controlled drone:

ESP32 Microcontroller: Dual-core processor with Wi-Fi and Bluetooth capabilities for control and communication.
MPU6050: 3-axis accelerometer and gyroscope for orientation and stability.
BLDC Motors (4): Brushless DC motors for drone propulsion.
Electronic Speed Controllers (ESC) (4): To regulate the speed of BLDC motors.
F450 Drone Frame: Durable quadcopter frame with a 450mm wheelbase.
Propellers (4): 8x4.5 dimensions for lift generation.
Li-Po Battery: 5200mAh, 11.1V for powering the drone.
Fly Sky FS-i6 Transmitter and Receiver: For manual control of the drone.
Additional Accessories: LED, Jumper wires, connectors, resistors, and a universal board.
## Circuit Diagram
## Assembly Diagram
### Below is the assembly diagram of the drone:
Assembly Notes <br/>
  1.Place the MPU6050 at the center of the drone frame for accurate orientation sensing.
  2.Ensure balanced placement of the Li-Po battery under the frame to maintain the center of gravity.

## Predefined Commands
Below are the predefined commands and their corresponding PWM values for controlling the drone. These values can be edited to suit user preferences. The listed command names are the labeled names used in Teachable Machine for voice recognition, and they can be edited based on the labels you train your model with.

These commands should be edited in the code to match the labels you define in Teachable Machine.

| Command     | Pitch PWM | Roll PWM | Throttle PWM | Yaw PWM |
|-------------|-----------|----------|--------------|---------|
| Turn on     | 1500      | 1500     | 1200         | 1500    |
| Switch off  | 1500      | 1500     | 1000         | 1500    |
| Hover       | 1500      | 1500     | 1600         | 1500    |
| High        | 1500      | 1500     | 1690         | 1500    |
| Low         | 1500      | 1500     | 1300         | 1500    |
| Forward     | 1625      | 1500     | 1600         | 1500    |
| Backward    | 1330      | 1500     | 1600         | 1500    |
| Right       | 1500      | 1625     | 1600         | 1500    |
| Left        | 1500      | 1330     | 1600         | 1500    |
| Clock       | 1500      | 1500     | 1600         | 1600    |
| Counter     | 1500      | 1500     | 1600         | 1400    |
