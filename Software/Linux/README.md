# Earth Rover Mini+ Host Development

## Build Example
```
cd earth-rover-mini/Software/Linux
git submodule update --init
mkdir build
cd build
cmake ..
make
```

## Connect to Robot
- Install ADB on your development platform
- Search and connect the wireless network <b>frodobot_xxx</b>. The default password is <b>12345678</b>.
- Once the connection is established. Connect to your robot with adb
```bash
adb connect 192.168.11.1
adb push hello /data/
adb shell
./hello
```

# Host Control Instructions

This section provides instructions on how to control the robot from a host computer for development and testing purposes.

---

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

The following C code provides a basic example of how to send control commands from a Linux-based host computer:

```c
// Example: Sending a UCP_MOTOR_CTL packet
// (Implementation details go here)
