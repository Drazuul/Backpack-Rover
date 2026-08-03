# Backpack Rover

A line-following rover that can drive itself or be teleoperated with an Xbox controller built on a three-tier control stack: bare-metal ARM firmware for motor control, an OpenMV camera for real-time line detection, and a Raspberry Pi bridging controller input and vision data together.

The purpose of this project was to develop a "Backpackable" robot that can be used to test control algorithms in outdoor environments. Although this demonstration uses an embedded camera for fast computer vision, the goal is to swap out the top portion of the robot with other sensors or actuators (LiDAR, GPS, IMU, arms, claws, etc.) for fast controls development in a wide variety of applications.

<img src="https://github.com/Drazuul/Backpack-Rover/blob/main/images/robot.jpg" class="center">
<img src="https://github.com/Drazuul/Backpack-Rover/blob/main/images/CAD_model.png" class="center">

---

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Hardware Overview](#hardware-overview)
- [Bare Metal Firmware](#bare-metal-firmware)
- [Computer Vision](#computer-vision)
- [Controller Input](#controller-input)
- [Robot Construction](#robot-construction)

---

## Overview

The Backpack Rover is a 4-motor rover that fuses two control modes:

- **Autonomous line-following** using an OpenMV camera that detects a black line via grayscale blob detection and reports a steering angle.
- **Manual teleoperation** via a Bluetooth Xbox controller, read on a Raspberry Pi and relayed over serial.

A Raspberry Pi acts as the coordination layer: it reads the controller, listens for the vision system's line-angle data, and forwards drive/turn commands over UART to a TM4C123 microcontroller, which handles low-level PWM motor control on a hardware timer interrupt.

## System Architecture

```

```

## Hardware Overview

- **TM4C123GH6PM (Tiva C Launchpad)** - motor controller, running bare-metal C
- **OpenMV Cam** - grayscale camera module for line detection
- **Raspberry Pi** - controller input + serial bridge between Pi, camera, and MCU
- **Xbox Wireless Controller** - manual drive input (Bluetooth)
- **4x DC motors** - driven in forward/reverse pairs via PWM (mixed for differential steering)

## Bare Metal Firmware

<img src="https://github.com/Drazuul/Backpack-Rover/blob/main/images/Redboard.jpg" class="center">

The TM4C123 firmware (`robogobo.c`) runs two interrupt-driven loops:

- **UART0 RX ISR** - a small state machine (`WAIT_HEADER -> WAIT_CMD -> WAIT_HI -> WAIT_LO`) parses incoming serial commands in the format `M` + `D`/`T` + high byte + low byte, updating global `forward` and `turn` values.
- **Timer0 ISR (100 Hz)** - mixes `forward` and `turn` into independent PWM duty cycles for the left and right motor pairs, clamping to the PWM module's max compare value (1023) to avoid faulting, then drives each motor forward or reverse accordingly.

Motor PWM is generated on four PWM generator blocks at a 1024-count load value, with GPIO pins mapped across ports B, C, and E.

## Computer Vision

<img src="https://github.com/Drazuul/Backpack-Rover/blob/main/images/line_angle_detect2.png" class="center">

The OpenMV script (`computer_vision.py`) performs weighted line tracking:

- The image is scanned in three horizontal regions of interest (ROIs), each weighted differently - closer-to-robot regions count more toward the final result.
- In each ROI, the largest grayscale blob is found and its centroid recorded.
- Centroids are combined into a single weighted position, then converted into a deflection angle using an arctangent function so the response gets stronger the further the rover strays from the line.
- The resulting angle is packed and sent to the Raspberry Pi over USB serial for use in the drive loop.

## Controller Input

`controller_input.py` runs on the Raspberry Pi and handles:

- **Joystick polling** via `pygame`, reading left-stick Y (forward/back) and right-stick X (turn) axes with a deadzone threshold.
- **Dead-man's switch** - a held button must be active for any drive input to register.
- **Mode arbitration** - a second button toggles between manual turning and the camera-driven autonomous turn angle.
- **Rumble feedback** on startup/shutdown to confirm connection state.
- **Serial forwarding** - packs and writes drive/turn values to the TM4C123 in the `MD`/`MT` protocol the firmware expects.
- A held menu button acts as a kill switch for the control loop.

## Robot Construction

<img src="https://github.com/Drazuul/Backpack-Rover/blob/main/images/under_the_hood.jpg" class="center">
