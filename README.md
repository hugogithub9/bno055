This project explores the BNO055, a 9-axis sensor that includes an accelerometer, magnetometer, and gyroscope, with built-in fusion algorithms to provide quaternion and Euler angle data.

-------

Project Structure


Several examples are included:

calibrate: Based on the example provided in the official documentation. It initializes the sensor and reads both quaternion and Euler angle data.

quaternion: A custom version where some functions were rewritten due to compatibility issues. It includes:

Real-time reading of register values,

Additional debug output to better understand sensor behavior.

-------

Current Issues


Some register readings do not behave as expected on the ESP32-C6 board.

A specific issue with register access remains unresolved, which prevents proper real-time data acquisition.

An issue has been opened on GitHub in the official driver repository, but no response has been received so far.

I have created a separate project that isolates and demonstrates this issue more clearly, to help with debugging and to potentially assist others facing the same problem.
