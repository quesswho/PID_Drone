# PID DRONE
This software is used to stabilize and control a drone. The drone uses the MPU-6050 sensor as the primary source of information such as orientation, accelerations and angular velocities.
To retrieve the sensor values a [library](https://github.com/ElectronicCats/mpu6050) by Electronic Cats is used.
This library provides user access to the onboard DMP(Digital Motion Processor). The DMP produces a quaternion useful for calculating the respective motor values.

![Final Drone](/pics/drone.jpg)

## Installation
1. Firstly, you need [processing](https://processing.org/).
2. Clone the repository `git clone https://github.com/quesswho/PID_Drone.git`
3. Get the [Game Control Plus](http://lagers.org.uk/gamecontrol/) library installed through Processing by Sketch->Import Library->Add Library
4. If using Arduino IDE go Tools->Library Manager, then search for mpu6050 and install the library by Electronic Cats (Should be option 3)
5. Also get the library RF24 TMRh20
6. Setup a circuit with an arduino and a mpu6050 imu. Example:
![Arduino and Mpu6050](/pics/imusetup.jpg)