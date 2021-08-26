#include <MPU6050_6Axis_MotionApps_V6_12.h>
#include <Wire.h>

#include <EEPROM.h> // Read/Write calibration offsets on the onboard eeprom

MPU6050 mpu;

uint8_t fifoBuffer[64];
Quaternion quat;
VectorInt16 accel;
VectorInt16 gyro;

void setup() 
{
 	Wire.begin();
  	//Wire.setClock(400000);
	Serial.begin(115200);

	mpu.initialize();

	while (Serial.available() && Serial.read()); // empty buffer

	const int dmpStatus = mpu.dmpInitialize();

	if(dmpStatus==0) {

		Serial.println("Successfully initialized dmp!");

		if(EEPROM.read(0) == 1) // Check whether offsets are written or not
		{
			calibrateRead();
		} else {
			calibrateWrite();
		}
		mpu.setDMPEnabled(true);
	}
}

void loop() 
{
	if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
		mpu.dmpGetQuaternion(&quat, fifoBuffer);
    	mpu.dmpGetAccel(&accel, fifoBuffer);
		mpu.dmpGetGyro(&gyro, fifoBuffer);

		Serial.print("Quat =(");
		Serial.print(quat.w);
		Serial.print(", ");
		Serial.print(quat.y);
		Serial.print(", ");
		Serial.print(quat.z);
		Serial.print(", ");
		Serial.print(quat.x);
		Serial.println(")");

		const float igu = 250.0f / pow(2, 15); // From units to °/s. Also inverse it because multiplication is faster than division

		Serial.print("Gyro =(");
		Serial.print(gyro.x * igu);
		Serial.print(", ");
		Serial.print(gyro.y * igu);
		Serial.print(", ");
		Serial.print(gyro.z * igu);
		Serial.println(")");

		const float iau = 1.0f / pow(2, 14);

		Serial.print("Accel =(");
		Serial.print(accel.x * iau);
		Serial.print(", ");
		Serial.print(accel.y * iau);
		Serial.print(", ");
		Serial.print(accel.z * iau);
		Serial.println(")");
	}
}

void calibrateRead() {

	auto readEEPROM = [](int index) -> int16_t
	{
		int16_t value;
		EEPROM.get(index, value);
		return value;
	};

	mpu.setXAccelOffset(readEEPROM(1));
	mpu.setYAccelOffset(readEEPROM(3));
	mpu.setZAccelOffset(readEEPROM(5));

	mpu.setXGyroOffset(readEEPROM(7));
	mpu.setYGyroOffset(readEEPROM(9));
	mpu.setZGyroOffset(readEEPROM(11));
	mpu.PrintActiveOffsets();
}

void calibrateWrite() {
	Serial.println("Calculating calibration offsets, please let the sensor rest.");
	mpu.CalibrateAccel(6);
   	mpu.CalibrateGyro(6);
	mpu.PrintActiveOffsets(); // Print offsets for debugging reasons
	if(EEPROM.length() > 0) {
		EEPROM.put(1, mpu.getXAccelOffset());
		EEPROM.put(3, mpu.getYAccelOffset());
		EEPROM.put(5, mpu.getZAccelOffset());

		EEPROM.put(7, mpu.getXGyroOffset());
		EEPROM.put(9, mpu.getYGyroOffset());
		EEPROM.put(11, mpu.getZGyroOffset());
		
		EEPROM.update(0, 1); // Indicate that the offsets are present
	} else {
		Serial.println("You do not have a functional EEPROM. Offsets will not be written, consider using a chip with an EEPROM");
	}
}