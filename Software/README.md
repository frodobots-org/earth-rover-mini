# Specification
|          | Specification |
|---------|---------|
| **Host** | RV1106G3 |
| **RAM (DDR)** | 256MB |
| **Flash (Nand)** | 128MB |
| **Operating System (Host)** | Linux |
| **Client** | STM32F407 |
| **Operating System (Client)** | RT-Thread |
| **Battery Type** | 18650 |
| **Battery Capacity** | 6000mAh |

# Block Diagram
![Image](https://github.com/user-attachments/assets/bb8ac540-4493-40bc-91dd-f2f63005b83f)

# Core Architecture

The firmware is built on a **multi-threaded architecture**, where dedicated tasks handle specific subsystems.  
This concurrent design ensures that critical operations like **motor control** and **sensor processing** are not blocked by lower-priority tasks like communication.

The main threads are orchestrated in `applications/main.c`, which acts as the system's entry point.

## Key Threads

- **Main Thread (`main.c`)**  
  - Initializes all hardware peripherals and application threads.  
  - Enters a loop that manages system-level state (e.g., controlling LEDs for charging, running, error).

- **IMU Thread (`imu.c`)**  
  - Manages the Inertial Measurement Unit (MPU6050 + Magnetometer).  
  - Reads sensor data, performs sensor fusion, and handles calibration.

- **State Thread (`state.c`)**  
  - Handles robot power and operational logic.  
  - Monitors battery voltage/current, manages power sequences, and tracks system state.

- **Motor Thread (`motor.c`)**  
  - Translates desired speed/steering into PWM signals for motor drivers.

- **UART Threads (`uart_mutex.c`)**  
  - One thread listens for incoming commands.  
  - Another sends telemetry back to the host.

- **Control Loop Timer (`hwtimer.c`)**  
  - Provides a high-precision tick for the closed-loop motion control system.  
  - Runs PID calculations at a fixed rate.

---

# Important Files and Interactions

The system is divided into key modules that interact to achieve the robot’s functionality.

## 1. System Entry and State Management

- **`applications/main.c`**  
  - Initializes hardware, creates RTOS threads, and starts the scheduler.  
  - `main()`: Manages RGB status LEDs (charging, running, low battery).  

- **`applications/state.c`**  
  - Handles robot power state and operational mode.  
  - `state_thread_entry()`: Monitors battery (INA226), charger connection, and updates `robot_state`.  
  - `robot_power_on()` / `robot_power_off()`: Control physical power supply.  
  - `key_isr()` & `key_detect_task()`: Detect button presses and long-press events.

---

## 2. Motion Control and Closed-Loop Feedback

The motion control system integrates the **hardware timer**, **PID controller**, and **motor driver**.

- **`applications/hwtimer.c`**  
  - Provides timing for the PID loop.  
  - `timeout_cb()`: Executes every tick (e.g., 10 ms), computes motor RPM, and updates PID outputs.

- **`applications/pid.c`**  
  - Implements PID controllers for speed and heading.  
  - `get_Inc_pid_result()`: Incremental PID for speed (RPM adjustment).  
  - `get_heading_pid_result()`: PID for heading correction (yaw).

- **`applications/motor.c`**  
  - Interfaces control logic with physical motors.  
  - `motor_thread_entry()`: Main loop applying PID outputs.  
  - `motor_get_duty()`: Converts speed/steer commands to duty cycles.  
  - `motor_move_forward()`, `motor_move_backward()`, etc.: Handle motor direction and PWM updates.

---

## 3. Sensor Fusion and Orientation

The IMU subsystem provides stable orientation estimates using the MPU6050 and magnetometer.

- **`applications/imu.c`**  
  - `imu_thread_entry()`: Reads sensors, manages calibration, runs sensor fusion.  
  - `IMUupdate()` / `IMUupdate_dmp()`: Compute pitch, roll, and yaw.  
  - `MPU_6050_Calibrate()` / `QMC5883L_Calibrate()`: Handle calibration routines.

- **`applications/MPU6050_DMP/`**  
  - Contains InvenSense MPU6050 motion driver.  
  - `inv_mpu_dmp_motion_driver.c`: Loads/configures DMP firmware.  
  - `dmp_read_fifo()`: Reads quaternion data from FIFO.

---

## 4. Communication Protocol

External communication uses a **custom UART protocol (UCP)**.

- **`applications/ucp.h`**  
  - Defines UART Control Protocol (UCP).  
  - Contains command IDs (e.g., `UCP_MOTOR_CTL`, `UCP_RPM_REPORT`) and packet structures.

- **`applications/uart_mutex.c`**  
  - Implements thread-safe UART communication using mutexes.  
  - `uart_thread_entry()`: Receives and parses incoming data packets (CRC check, command handling).  
  - `uart_send_thread_entry()`: Periodically sends telemetry (RPM, battery, IMU orientation).  

---


# Development
- **Host**: 🖥 [Learn more here](./Software/Linux)
- **Client**: ⚙ [Learn more here](http://example.com/stm32f407)
