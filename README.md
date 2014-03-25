10DOF
=====
This is my homemade 10-DOF built up around a 33FJ64MC202 16-bits dsPIC microcontroller.
Further more is uses a ITG3200 gyro, BMA180 accelerometer, BMP085 pressure sensor and a HMC5883L 3-axis magnetic sensor.
All the filtering is done by the dsPIC. I'm using a kalman filter for the angles and a FIR filter for the altitude. 
Normally it communicates over a 5MHz SPI interface with data ready and SS. (RS232 for debugging)
Tilt compensation for the magnetic sensor works fine now too. 

This unit works perfect in my quadrocopter, where i originally designed it for. But you can use it for a lot of other applications as well. 

See the 10DOF in action here: https://www.youtube.com/watch?v=DtUdlfKq6_U
