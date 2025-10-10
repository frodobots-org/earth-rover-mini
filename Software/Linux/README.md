# Earth Rover Mini+ Host Development

## Build Example
```
git clone --recursive https://github.com/SIGRobotics-UIUC/earth-rover-mini-OpenSource.git
cd earth-rover-mini-OpenSource/Software/Linux
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake ..
make
```

## Connect to Robot
- Install ADB on your development platform
- Search and connect the wireless network <b>frodobot_xxx</b>. The default password is <b>12345678</b>.
- Once the connection is established. Connect to your robot with adb
```bash
adb connect 192.168.11.1:5555
adb shell
```


# Host Control Instructions

This section provides instructions on how to control the robot from a host computer for development and testing purposes.

---

## TCP Control Mechanism Demo w/ Move

The robot has been configured such that it is possible to send commands via TCP and Python. To do this, first navigate to the **/data** folder inside the robot shell and then run the **tcp_bridge** executable. This sets a TCP receiver connection on the robot side so it is ready to receive the packets sent from external code. You should see some sort of confirmation message that this worked.

Next, go to the **/src/Examples** folder and run the **move.py** script. This is some basic code that mirrors **move.cpp** but instead in Python. You should see the rover move if you execute this part right. 

I'll now explain how some of this works internally. The script **uart_cp.py** was created as an API for the C structs defined in **ucp.h**. By translating the alignments and the types correctly, can call these Python classes in the same way the C structs were earlier. You can see that the two move files have very similar structure because of this. Another thing that's important is to ensure that the right IP address and port are being used, or else the communication between computer and robot will not happen. If you wish to change these values or you aren't happy with the way the robot handles the connection, you must modify the **bridge.c** file, cmake and make, and then **adb push** the resultant **tcp_bridge** executable into the right folder of the robot, and then run the executable again from the robot side.

*Problems/Notes*
This is all reliant on the IP address and port being constant, which makes it vulnerable to being hacked. Idk what to do about this.\nI tested this out on my dual-boot computer with x86 processors. Not sure how well this would work on ARM chip computers.

## Control Mechanism

The robot is controlled via a **serial interface** using the custom **UART Control Protocol (UCP)** defined in `applications/ucp.h`.  
By sending formatted data packets to the robot's `uart3` port, you can:

- Control movement  
- Trigger calibration routines  
- Receive telemetry  

---

## Serial Port Settings

- **Baud Rate:** 115200  
- **Data Bits:** 8  
- **Parity:** None  
- **Stop Bits:** 1  

---

## Teleoperation (Manual Control)

To manually control the robot, send **`UCP_MOTOR_CTL` (ID: 0x02)** packets.  
These packets contain the desired **linear** and **angular velocity**.

---

## Control Packet Structure (`ucp_ctl_cmd_t`)

The command packet is composed of a **header**, a **payload**, and a **CRC checksum**.  
The **motor control payload** is defined as follows:

| Field     | Type      | Description                                                                 |
|-----------|-----------|-----------------------------------------------------------------------------|
| `hd`      | `ucp_hd_t`| UCP Header (contains length, ID=0x02, index)                                |
| `speed`   | `int16_t` | Desired linear velocity. Positive = forward, negative = backward. Range: -100 to 100. |
| `angular` | `int16_t` | Desired angular velocity. Positive = right, negative = left. Range: -100 to 100. |
| `front_led` | `int16_t` | Controls the front LEDs (not fully implemented).                          |
| `back_led`  | `int16_t` | Controls the back LEDs (not fully implemented).                           |
| `...`       | `...`     | Reserved fields.                                                          |

---

## Getting Started: A Simple Teleop Example

Look to `src/examples/move.cpp`

# Camera Example

## Getting Started: Dual Camera Streaming Over RTSP

Look to `src/Examples/sample_demo_dual_camera.c`
#### Run the example code on the Earth Rover Mini
- Build Examples
- Push `sample_demo_dual_camera` to device via ADB
```
adb push sample_demo_dual_camera /tmp/
```
- Run example
```
adb shell /tmp/sample_demo_dual_camera -s 0 -W 1920 -H 1080 -w 720 -h 576 -f 30 -r 0 -s 1 -W 1920 -H 1080 -w 720 -h 576 -f 30 -r 0 -n 1 -b 1
```
#### Capture video from the Earth Rover Mini camera on the computer
- Connect with Earth Rover Mini with same local network.
- Use Python Opencv
```python
import cv2
'''
rtsp://<Earth Rover Mini IP>/live/0  Front camera main-stream
rtsp://<Earth Rover Mini IP>/live/1  Front camera sub-stream
rtsp://<Earth Rover Mini IP>/live/2  Rear camera main-stream
rtsp://<Earth Rover Mini IP>/live/3  Rear camera sub-stream
'''
cap = cv2.VideoCapture("rtsp://<Earth Rover Mini IP>/live/0")
if not cap.isOpened():
    print("Failed to open RTSP stream")
    exit()
while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to read frame")
        break
    cv2.imshow("RTSP Stream", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
```
