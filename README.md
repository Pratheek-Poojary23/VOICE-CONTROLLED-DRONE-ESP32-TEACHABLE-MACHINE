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
Below is the assembly diagram of the drone:
Assembly Notes
1.Place the MPU6050 at the center of the drone frame for accurate orientation sensing.
