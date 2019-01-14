# SimulinkOdrive

Simulink SystemObject for controlling odrive by Raspberry PI Simulink target

# Usage

1) Clone this repository
1) Add it into matlab path
1) Insert MATLAB System block into your simulink model and select ODrive
1) Set up everything in block mask

# Used time in your model
All measurings are done with original ODrive firmware, which comunicates on 115200 bauds. Increasing this speed shoud lower theese times.
If there are only reference inputs time used by this block on raspberry Pi3 connected by serial port is insignificant (0.1ms).

Enabling any output will add time for reading it:

Approx:
- Axis estimated position 5.6-5.7 ms
- Axis estimated velocity 5.9-6.0 ms
- Axis measured current 6.7-6.9 ms
- Axis error 3.6-3.7 ms
All prepared outputs took 17.2-18.3ms

You can use block timing options to compare your results 

