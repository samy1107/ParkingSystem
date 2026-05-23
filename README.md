# ParkingSystem
-An automatic garage parking system with saftey features
# Advanced Embedded Systems – Smart Gate Controller

## Overview
A real-time smart gate control system developed using the TM4C123 microcontroller and FreeRTOS. The project demonstrates embedded systems concepts including task scheduling, GPIO interfacing, synchronization, and state-machine-based control.

## Features
- Automatic and manual gate control
- Obstacle detection and automatic reversal
- Limit switch handling
- Concurrent RTOS tasks
- RGB LED status indication

## Tools & Technologies
- Embedded C
- FreeRTOS
- ARM Cortex-M4
- Keil uVision
- TM4C123GH6PM

## RTOS Concepts Used
- Tasks
- Queues
- Mutexes
- Event-driven scheduling

## Project Structure
```text
main.c          -> Main application logic
/inc            -> Header files
/RTE            -> Device/system files
