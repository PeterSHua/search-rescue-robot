## search-rescue-robot
The goal was to create a robot that could assist in disaster relief efforts by quickly and accurately navigating through disaster zones. To that end, the robot needed to be able to navigate a maze, retrieve an object, and find the shortest path back to the origin.

[video](https://youtu.be/GNuyD7eQRL8)

## Features
The project involved designing and building a robot with the following functionalities:
- Movement: The robot could accurately move in the desired path for the desired distance using stepper motors.
- Panel identification: IR sensors were used to differentiate between types of panels in the maze.
- Current, voltage, and power measurements: A current sensor and voltage divider circuit were used to detect power consumption and voltage as a function of time.
- Chassis: The chassis was made of lightweight stainless steel featuring a hinged cover for easy access to components.
- Shortest path calculation: Dijkstra's algorithm was used to detect the shortest path from the earthquake victim to the origin.
- Wireless communication: XBEE modules were used to send and receive data between Arduino and laptop.
- Mission control GUI: A GUI was created to map the disaster maze in real-time as the robot traversed its path, command the robot to initiate manual or automatic mode, report additional information from robot to mission control.

## Schematic
![schematic](https://github.com/PeterSHua/search-rescue-robot/blob/main/img/schematic.png?raw=true)

## PCB
![pcb](https://github.com/PeterSHua/search-rescue-robot/blob/main/img/pcb.png?raw=true)

## GUI
![gui](https://github.com/PeterSHua/search-rescue-robot/blob/main/img/gui.png?raw=true)
