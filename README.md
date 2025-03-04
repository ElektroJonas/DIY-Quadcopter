# Quadcopter Built from Scratch

This project contains all the materials used in building a quadcopter drone. It was part of a project course in electrical engineering and embedded systems at Uppsala University, where we spent a portion of the semester designing and constructing a quadcopter from scratch using off-the-shelf components and some interesting control algorithms! 🚀

### Project Demonstration and Summary
- A fun summary of the project's development can be found here: [YouTube Summary](https://youtu.be/-V470g73WhM?si=RAoppi26RXBP144A)  
- A demonstration of the drone can be found here: [YouTube Demonstration](https://youtu.be/1-YmHmeVQKA?si=Rilc_he6dhYzZJqS)  

### Final Result
Below is a figure of the completed drone and its custom-built remote control:

| Drone | Remote |  
| --- | --- |  
| ![](https://github.com/ElektroJonas/DIY-Quadcopter/blob/main/Pictures/IMG_3623.jpg) | ![](https://github.com/ElektroJonas/DIY-Quadcopter/blob/main/Pictures/IMG_3621.jpg) |

### Control System  
The control system consists of two main components: **Attitude Control** and **Altitude Control**.  

- **Attitude Control** (angle relative to the ground plane) is implemented using a cascaded PID controller with a Kalman filter and an internal rate control loop. It relies on a low-cost MEMS sensor (gyroscope/accelerometer) for angle estimation.  
- **Altitude Control** is managed using a pressure sensor in combination with the MEMS sensor for attitude estimation, supplemented by a Zero Velocity Update (ZUPT) algorithm for improved stability and accuracy.   

The control system was implemented on an ESP32 microcontroller, utilizing the ESP-NOW protocol for wireless communication.

### Full Project Report
A detailed description of this project, including development insights, sources, and useful references for anyone interested in building their own drone, can be found in **Quadcopter_final_report.pdf**.

### Acknowledgments  
We would like to express our gratitude to [CarbonAeronautics](https://github.com/CarbonAeronautics) for providing valuable resources that greatly aided the early development of this project.  

A special thanks to **Uppsala University** for sponsoring the project and providing the opportunity to bring this idea to life!
