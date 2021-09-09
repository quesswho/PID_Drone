#include <MPU6050_6Axis_MotionApps_V6_12.h>
#include <Wire.h>

#include <EEPROM.h> // Read/Write calibration offsets on the onboard eeprom

MPU6050 mpu;

uint8_t fifoBuffer[64];
Quaternion quatMeasured;
VectorInt16 accel;
VectorInt16 gyro;

Quaternion quatRef;
Quaternion quatErr;
VectorFloat axisPErr;
VectorFloat axisIErr;
VectorFloat axisDerr;

float motor[4];

const float kQP = 1.0f;
const float kQI = 1.0f;
const float kQD = 1.0f;

float throttle = 0;

// Do not run the program for more than 71 minutes because it will overflow
// Optionally can divide the micros() function by 4 because it only has a resolution of 4 micro seconds on 16MHz boards
unsigned long currentTime = 0;
unsigned long lastTime = 0;
unsigned long elapsedTime = 0;

inline const float zeroNeg(const float value) // If value is negative then return zero
{
  	return value > 0 ? value : 0;
}

void setup() 
{
 	Wire.begin();
 	Wire.setClock(400000);
	Serial.begin(115200);

	mpu.initialize();

	while (Serial.read() >= 0); // empty buffer
  
	if(mpu.dmpInitialize() == 0) {
		Serial.println("Successfully initialized dmp!");

		if(EEPROM.read(0) == 1) // Check whether offsets are written or not
		{
			calibrateRead();
		} else {
			calibrateWrite();
		}
		mpu.setDMPEnabled(true);
	} else
	{
		Serial.print("Failed to initialize dmp!");
	}
	lastTime = micros();
}

void loop() 
{
	if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
		GetRawSensor();
		CalculateError();
		CalculateMotorValues();
	}
}

void GetRawSensor() {
	mpu.dmpGetQuaternion(&quatMeasured, fifoBuffer);
	mpu.dmpGetAccel(&accel, fifoBuffer);
	mpu.dmpGetGyro(&gyro, fifoBuffer);
}

void CalculateError()
{
	quatErr = quatRef.getProduct(quatMeasured.getConjugate());
	if(quatErr.w < 0)
	{
		quatErr = quatErr.getConjugate();
	}
	axisPErr = VectorFloat(quatErr.x, quatErr.y, quatErr.z);
	currentTime = micros();
	elapsedTime = currentTime - lastTime; // elapsed time is used for calculating the inegral and derivative of the error
	// We need to leave 3d_mathhelper or rewrite it on a fork	
	axisIErr = VectorFloat(axisPErr.x * elapsedTime, axisPErr.y * elapsedTime, axisPErr.z * elapsedTime);
	const float invT = 1.0f / elapsedTime;
	axisDErr = VectorFloat(axisPErr.x * invT, axisPErr.y * invT, axisPErr.z * invT);	
	
	Serial.print("AxisIErr =(");
	Serial.print(axisIErr.x);
	Serial.print(", ");
	Serial.print(axisIErr.y);
	Serial.print(", ");
	Serial.print(axisIErr.z);
	Serial.println(")");
	
	Serial.print("AxisDErr =(");
	Serial.print(axisDErr.x);
	Serial.print(", ");
	Serial.print(axisDErr.y);
	Serial.print(", ");
	Serial.print(axisDErr.z);
	Serial.println(")");
}

// Motor layout
//
//   0     1
//      X
//   2     3

void CalculateMotorValues()
{
	// Assuming torque is proportional to the motor values
	motor[0] = throttle + kQP * (zeroNeg(axisPErr.x) + zeroNeg(-axisPErr.y));
	motor[1] = throttle + kQP * (zeroNeg(-axisPErr.x) + zeroNeg(-axisPErr.y));
	motor[2] = throttle + kQP * (zeroNeg(axisPErr.x) + zeroNeg(axisPErr.y));
	motor[3] = throttle + kQP * (zeroNeg(-axisPErr.x) + zeroNeg(axisPErr.y));
/*	Serial.print("Err =(");
	Serial.print(motor[0]);
	Serial.print(", ");
	Serial.print(motor[1]);
	Serial.print(", ");
	Serial.print(motor[2]);
	Serial.print(", ");
	Serial.print(motor[3]);
	Serial.println(")");
*/
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
