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
  	Wire.setClock(400000);
	Serial.begin(115200);

	mpu.initialize();

	while (Serial.available() && Serial.read()); // empty buffer

	const int dmpStatus = mpu.dmpInitialize();
	mpu.setXGyroOffset(115);
	mpu.setYGyroOffset(-58);
	mpu.setZGyroOffset(-16);
	mpu.setXAccelOffset(-2028);
	mpu.setYAccelOffset(-1374);
	mpu.setZAccelOffset(766);

	if(dmpStatus==0) {
		Serial.println("Successfully initialized dmp!");
		//mpu.CalibrateAccel(6);
    	//mpu.CalibrateGyro(6);
		CalibrateWrite();
		//mpu.PrintActiveOffsets();
		mpu.setDMPEnabled(true);
	}
}

void loop() 
{
	if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
		mpu.dmpGetQuaternion(&quat, fifoBuffer);
    	mpu.dmpGetAccel(&accel, fifoBuffer);
		mpu.dmpGetGyro(&gyro, fifoBuffer);

		/*Serial.print("Quat =(");
		Serial.print(quat.w);
		Serial.print(", ");
		Serial.print(quat.y);
		Serial.print(", ");
		Serial.print(quat.z);
		Serial.print(", ");
		Serial.print(quat.x);
		Serial.println(")");

		Serial.print("Gyro =(");
		Serial.print(gyro.x);
		Serial.print(", ");
		Serial.print(gyro.y);
		Serial.print(", ");
		Serial.print(gyro.z);
		Serial.println(")");
*/
		Serial.print("Accel =(");
		Serial.print(accel.x / 16384.0f);
		Serial.print(", ");
		Serial.print(accel.y / 16384.0f);
		Serial.print(", ");
		Serial.print(accel.z / 16384.0f);
		Serial.println(")");
	}
}

void CalibrateRead()
{
	
}

void CalibrateWrite() 
{
	Serial.print("Length: ");
	Serial.println(EEPROM.length());

	//mpu.CalibrateAccel(6);
   // mpu.CalibrateGyro(6);

	
}
