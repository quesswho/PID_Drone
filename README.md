# PID DRONE
This software is used to stabilize and control a drone. The drone uses the MPU-6050 sensor as the primary source of information such as orientation, accelerations and angular velocities.
To retrieve the sensor values a [library](https://github.com/ElectronicCats/mpu6050) by Electronic Cats is used.
The average user normally only has access to raw accelerometer and gyroscope values which are stored in their respective registers documentet on the official register map for the MPU-6000 and MPU-6050.
This library provides user access to the onboard DMP(Digital Motion Processor) which is capable of generating a quaternion orientation.

## Installation
1. Clone the repository `git clone https://github.com/quesswho/PID_Drone.git`
2. If using Arduino IDE go Tools->Library Manager Then search for mpu6050 and install the library by Electronic Cats(Should be option 3)
3. Setup a circuit with an arduino and a mpu6050 imu. Example:
![Arduino and Mpu6050](/pics/imusetup.jpg)
