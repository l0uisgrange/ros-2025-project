# Embedded systems

This is our semester project for the _Embedded systems and robotics_ course at EPFL from [Prof. Francesco Mondada](https://people.epfl.ch/102717?lang=en). We had to control an E-Puck2 using its real-time OS, C++, and its different sensors and actuators. The robot had to find the highest point of a plane, go there, and bounce on the edge like a tennis ball.

<img width="400" alt="IMG_1777" src="https://github.com/user-attachments/assets/cd18121a-8b8f-41a6-a729-f7c7b20f7dfe" />

## ⚙️ Quickstart

1. Download the course-specific compiled VSCode version
2. Build the project using `make` command
3. Run the executable

## 🗃️ Project structure

```
.
├── src
│   ├── chconf.h
│   ├── detection.c          # Wall detection
│   ├── detection.h
│   ├── halconf.h
│   ├── main.c               # Process loop
│   ├── main.h
│   ├── makefile
│   ├── mcuconf.h
│   ├── travel.c             # Accelerometer and travelling
│   └── travel.h
├── .gitignore
└── README.md
```
