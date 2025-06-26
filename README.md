# 🚁 Drone Control System
This project integrates firmware development, sensor fusion, motor control for building, simulating, and controlling a custom quadcopter drone.

## 📌 Objectives
  - Control 4 brushless motors for a drone using a PID controller
  - Wireless communication using NRF24L01 (SPI protocol) 
  - Deliver comple 3D designs and PCB schematics

## 🧠 System Design
In this DIY project , we are building a wireless flight controller for a quadcopter using the NRF24L01 module.
![]()  
Initially , out goal was to use a single Arduino Nano to handle everything(wireless communication , sensor reading , and motor control using a PID algorithm.

However , we soon realized that a single microcontroller couldn't meet all the demands of real-time data processing and precise control due to its limited computing power. 
To address this limitation , we decided to use 2 Arduino Nano. One microcontroller is dedicated to handling wireless communication via the NRF24L01 module. It processes incoming data and generates control pulses. These pulses are then sent to the main flight controller , which reads the signal and translates them into control outputs for the PID-based flight control system
