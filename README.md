# 🚁 Drone Control System
This project integrates firmware development, sensor fusion, motor control for building, simulating, and controlling a custom quadcopter drone.

![thumbnail](https://raw.githubusercontent.com/hoanganhdo207/DIY_Drone_Project/main/images/thumbnail.png) 
## 📌 Objectives
  - Control 4 brushless motors for a drone using a PID controller
  - Wireless communication using NRF24L01 (SPI protocol) 
  - Deliver comple 3D designs and PCB schematics

## 🧠 System Design
In this DIY project , we are building a wireless flight controller for a quadcopter using the NRF24L01 module.

![thumbnail](https://raw.githubusercontent.com/hoanganhdo207/DIY_Drone_Project/main/images/images(1).png) 

Initially , out goal was to use a single Arduino Nano board to handle everything(wireless communication , MPU sensor reading , and motor control using a PID algorithm).

![thumbnail](https://raw.githubusercontent.com/hoanganhdo207/DIY_Drone_Project/main/images/arduino.jpg) 

However , we soon realized that a single microcontroller couldn't meet all the demands of real-time data processing and precise control due to its limited computing power. 
To address this limitation , we decided to use 2 Arduino Nano. One microcontroller is dedicated to handling wireless communication via the NRF24L01 module. It processes incoming data and generates control pulses. These pulses are then sent to the main flight controller , which reads the signal and translates them into control outputs for the PID-based flight control system

![thumbnail](https://raw.githubusercontent.com/hoanganhdo207/DIY_Drone_Project/main/images/schematic.png)

## 🛠️ Technologies Used
| Component        | Tool / Technology         |
|------------------|---------------------------|
| Microcontroller  | Arduino Nano*3            |
| Sensor           | MPU6050                   |
| Mechanical frame | Frame wheel F450          |
| IDE              | Arduino IDE               |
| Communication    | Wireless 2.4 GHz          |
| CAD Modeling     | SolidWorks                |
| PCB Design       | Altium Designer           |

## 📁 Project Structure

| Folder/File         | Description                                                                 |
|---------------------|-----------------------------------------------------------------------------|
| `Hardware/`          | Contains all mechanical and electrical design files.                       |
| └── `CAD/`           | 3D models (STEP/STL) for 3D printing and simulation.                       |
| └── `Schematics/`    | Circuit diagrams and wiring documents.                                     |
| `Software/`          | Source code for firmware                                                   |
| └── `Main control/`  | Main source code for flight control.                                       |
| └── `Transmitter/`   | Signal transmission by remote control.                                     |
| └── `Receiver   /`   | Receiver data and convert to control signal.                               |
| `images/`            | Screenshots and images for documentation and README.                       |
| `.gitignore`         | List of files/folders ignored by Git.                                      |
| `README.md`          | Project overview (this file).                                              |

## 📄 License
This project is licensed under the [MIT License](LICENSE).  
You are free to use, modify, and distribute this software in both personal and commercial projects, as long as the original license is included.

## 📬 Contact
For questions or collaboration:
  - Name: Đỗ Hoàng Anh
  - Email: hoanganhdo207@gmail.com
  - GitHub: [hoanganhdo207](https://github.com/hoanganhdo207)

