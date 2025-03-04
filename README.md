# VOICE-CONTROLLED-DRONE-ESP32-TEACHABLE-MACHINE
This project integrates ESP32 with Teachable Machine for a voice-controlled drone. The system supports dual-mode operation (voice and manual) and provides real-time responsiveness using WebSocket communication.

---

## Description:
"Voice-controlled drone project integrating ESP32 for hardware control and a voice recognition model trained on Teachable Machine. The ESP32 code is available in the Voice_controlled_code directory. For connecting the Teachable Machine HTML interface with ESP32, refer to the esp32-teachable-machine-led-websocket repository. Replace the my_model folder in that repository with the folder containing your trained voice recognition model from Teachable Machine."

---

## System Architecture Overview
<img src="https://github.com/Pratheek-Poojary23/VOICE-CONTROLLED-DRONE-ESP32-TEACHABLE-MACHINE/blob/main/Images/architecture.png" height="300" width="1700">

---

## Components
The following components are required for building the voice-controlled drone:

1. ESP32 Microcontroller: Dual-core processor with Wi-Fi and Bluetooth capabilities for control and communication.<br/>
2. MPU6050: 3-axis accelerometer and gyroscope for orientation and stability.<br/>
3. BLDC Motors (4): Brushless DC motors for drone propulsion.<br/>
4. Electronic Speed Controllers (ESC) (4): To regulate the speed of BLDC motors.<br/>
5. F450 Drone Frame: Durable quadcopter frame with a 450mm wheelbase.<br/>
6. Propellers (4): 8x4.5 dimensions for lift generation.<br/>
7. Li-Po Battery: 5200mAh, 11.1V for powering the drone.<br/>
8. Fly Sky FS-i6 Transmitter and Receiver: For manual control of the drone.<br/>
9. Additional Accessories: LED, Jumper wires, connectors, resistors, and a universal board.<br/>

---

## Circuit Diagram
<img src="https://github.com/Pratheek-Poojary23/VOICE-CONTROLLED-DRONE-ESP32-TEACHABLE-MACHINE/blob/main/Images/circuit.png" height="700" width="600">

---

## Assembly Diagram
Below is the assembly diagram of the drone:

<img src="https://github.com/Pratheek-Poojary23/VOICE-CONTROLLED-DRONE-ESP32-TEACHABLE-MACHINE/blob/main/Images/assembly.png" height="500" width="700">

Assembly Notes <br/>
* Place the MPU6050 at the center of the drone frame for accurate orientation sensing. <br/>
<img src="https://github.com/Pratheek-Poojary23/VOICE-CONTROLLED-DRONE-ESP32-TEACHABLE-MACHINE/blob/main/Images/IMU_placement.png" height="200" width="500">
<img src="https://github.com/Pratheek-Poojary23/VOICE-CONTROLLED-DRONE-ESP32-TEACHABLE-MACHINE/blob/main/Images/Esp_placement.png" height="200" width="500">

* Ensure balanced placement of the Li-Po battery under the frame to maintain the center of gravity.<br/>

---

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

### Notes:
* Pitch PWM controls the forward/backward tilt of the drone. <br/>
* Roll PWM controls the left/right tilt. <br/>
* Throttle PWM controls the altitude of the drone. <br/>
* Yaw PWM controls the rotation of the drone. <br/>
  
These values are preconfigured for smooth and responsive control. However, you can adjust these values according to your drone's specific requirements and hardware capabilities.

The commands listed here correspond to the labels you assign to your voice commands in Teachable Machine. You should edit these commands in the code to match the labels you have trained your model with, providing flexibility in customizing the voice control commands for your drone.

---

## PID Tuning
In addition to editing the predefined commands, you will also need to edit the PID values in the code. PID values (Proportional, Integral, and Derivative) are crucial for maintaining stable flight. These values can vary significantly from one drone to another due to differences in weight, motor power, frame design, and other factors.

PID values must be tuned using the trial-and-error method to achieve optimal performance for your specific drone. Here’s how you can approach it:

* Proportional (P): Adjusts the response to the error. A higher value makes the system more responsive but can cause overshooting. <br/>
* Integral (I): Accounts for accumulated past errors. It helps eliminate steady-state error but can lead to oscillations if too high. <br/>
* Derivative (D): Predicts future errors based on the rate of change. It helps smooth out oscillations and reduces overshoot.<br/>

### To tune the PID values:

  * Start with low values and gradually increase them until you achieve stable flight.<br/>
  * Test with your drone and observe the behavior. Fine-tune the values based on the results.<br/>
  * The trial-and-error method requires patience, but it ensures the best possible control for your drone.<br/>
## Special Note on Pitch and Roll PID:
Since the drone is symmetric, the PID values for pitch and roll are almost identical. You can start by using the same PID values for both axes and adjust them accordingly based on your testing. This symmetry helps simplify the tuning process as both axes will behave similarly under the same control settings.

You can find and edit the PID values in the ESP32 code within the Voice_controlled_code directory. Be sure to experiment with different values to find the optimal balance for your drone’s performance.

---



