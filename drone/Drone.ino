// MPU6050
#include <MPU6050_6Axis_MotionApps_V6_12.h>
#include <Wire.h>

// Radio
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <printf.h>

// Read/Write to the onboard EEPROM
#include <EEPROM.h>

// Regulating electronic speed control with pulse width modulation
#include <Servo.h>

// Radio
RF24 radio(8, 9);
const byte address[6] = "01234";

// Controller input
bool light = false;
int mx = 0, my = 0, rot = 0, alt = 0;

// imu
MPU6050 mpu;

uint8_t fifoBuffer[64];
Quaternion quatMeasured;

// Reference and error quaternion
Quaternion quatRef;
Quaternion quatErr;

// PID components
VectorFloat axisPErr;
VectorFloat axisIErr;
VectorFloat axisDErr;

// PID koefficients
const float kQP = 0.5f;
const float kQI = 0.1f;
const float kQD = 1.2f;

// I_n = [0.0f, 1.0f]
float motor[4];

const float controlsens = 0.005f;

float throttle = 0.0f;

// ESC pins
const int pinMotor0 = 3;
const int pinMotor1 = 6;
const int pinMotor2 = 4;
const int pinMotor3 = 5;

// ESC objects
Servo motor0;
Servo motor1;
Servo motor2;
Servo motor3;

// Do not run the program for more than 71 minutes because the time will overflow
// Optionally we can divide the micros() function by 4 because it only has a resolution of 4 micro seconds on 16MHz boards
unsigned long currentTime = 0;
unsigned long lastTime = 0;
float elapsedTime = 0.0f;

inline const float negativeZero(const float value) { // If value is negative then return zero
  	return value > 0 ? value : 0;
}


// Initialize drone components
void setup() {
	Wire.begin();
	Wire.setClock(400000);
	Serial.begin(115200);

	// Initialize imu
	mpu.initialize();

	while (Serial.read() >= 0); // clear buffer

	if(mpu.dmpInitialize() == 0) {
		Serial.println("Successfully initialized dmp!");
		if(EEPROM.length() < 0 || EEPROM.read(0) == 0x4B) { // Check whether offsets are written or not by comparing with the unique magic number 0x4B
			calibrateRead();
		} else {
			calibrateWrite();
		}
		mpu.setDMPEnabled(true);
	} else {
		Serial.print("Failed to initialize dmp!");
	}

	// Initialize esc connection
	motor0.attach(pinMotor0, 1000, 2000);
	motor1.attach(pinMotor1, 1000, 2000);
	motor2.attach(pinMotor2, 1000, 2000);
	motor3.attach(pinMotor3, 1000, 2000);

	lastTime = micros();

	// Initialize radio connection
	radio.begin();
	radio.openReadingPipe(0, address);
	radio.setPALevel(RF24_PA_MAX);
	radio.setAutoAck(true);
	radio.setPayloadSize(10);
	radio.startListening();
	printf_begin();
	radio.printDetails();

	quatRef = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
}


// Main loop, radio and attitude system.
void loop() {
	readTransmitter();
	if(micros() - lastTime > 1000) { // 1ms minimum between imu read
		if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
     		applyControls();
			getRawSensor();
			calculateError();
			calculateMotorValues();
			applyControlledThrust();
		}
	}
}


// Read radio transmitter 
void readTransmitter() {
	static byte packet[10];

	if (radio.available()) {
		radio.read(&packet, 10);
		byte mask = packet[0];
		if(mask > 0) { // mask = 0 => no information was sent
			if((mask & 0b10000000) > 0) {
				if((mask & 0b00000001) > 0) {
					mx = (unsigned short)(packet[1] << 8) | packet[2];
				}
				if((mask & 0b00000010) > 0) {
					my = (unsigned short)(packet[3] << 8) | packet[4];
				}
				if((mask & 0b00000100) > 0) {
					rot = (unsigned short)(packet[5] << 8) | packet[6];
				}
				if((mask & 0b00001000) > 0) {
					alt = (unsigned short)(packet[7] << 8) | packet[8];
				}
				if((mask & 0b00010000) > 0) {
					light = packet[9];
				}
			}
		}
	} else if(radio.getARC() == 0) { // If connection is lost, try reestablish connectin
			radio.startListening();
	} else {
		Serial.println(radio.getARC());
	}
}


// Reference quaternion from radio signals
void applyControls() {
	throttle = max(min(throttle + (float)alt * 0.0000052125f, 0.6f), 0.0f); // 1/(256*750)

	static float sum_mx = 0.0f;
	static float sum_my = 0.0f;

	// Smooth input, easier on derivative component
	const float c = controlsens/512.0f;
	sum_mx = min((sum_mx + mx*c) * 0.95f, 0.2f); // s_{n+1} = (s_n+input) * decay, will be convergent if input is constant
	sum_my = min((sum_my + my*c) * 0.95f, 0.2f);

	quatRef = Quaternion(1.0f, -sum_mx, sum_my, 0.0f).getNormalized();
}


// Read quaternion
void getRawSensor() {
	mpu.dmpGetQuaternion(&quatMeasured, fifoBuffer);

	// Start measuring time
	currentTime = micros();
	elapsedTime = (currentTime - lastTime) / 1000000.0f;
	lastTime = currentTime;
}


// Calculate proportional, derivative and integral components
void calculateError(){
	quatErr = quatRef.getProduct(quatMeasured.getConjugate());
	if(quatErr.w < 0) {
		quatErr = quatErr.getConjugate();
	}
	const float invT = 1.0f / (elapsedTime*20);
	axisDErr = VectorFloat((quatErr.x - axisPErr.x) * invT, (quatErr.y - axisPErr.y) * invT, (quatErr.z - axisPErr.z) * invT); 
	axisPErr = VectorFloat(quatErr.x, quatErr.y, quatErr.z);
	const float decay = 0.98f;
	axisIErr = VectorFloat(axisIErr.x * decay + axisPErr.x * elapsedTime, axisIErr.y * decay + axisPErr.y * elapsedTime , axisIErr.z * decay + axisPErr.z * elapsedTime);
}



// Motor layout
//      |
//   0     2
//      X
//   1     3
//      |

// Calculate motor values with PID results
void calculateMotorValues() {
	if(throttle > 0) {
		motor[0] = min(throttle 
  	          + kQP * (negativeZero(axisPErr.x) + negativeZero(axisPErr.y))
              + kQI * (negativeZero(axisIErr.x) + negativeZero(axisIErr.y))
              + kQD * (negativeZero(axisDErr.x) + negativeZero(axisDErr.y))
  	          , 1.0f);
    
  		motor[1] = min(throttle 
  	          + kQP * (negativeZero(-axisPErr.x) + negativeZero(axisPErr.y))
              + kQI * (negativeZero(-axisIErr.x) + negativeZero(axisIErr.y))
  	          + kQD * (negativeZero(-axisDErr.x) + negativeZero(axisDErr.y))
  	          , 1.0f);
             
  		motor[2] = min(throttle 
  	          + kQP * (negativeZero(axisPErr.x) + negativeZero(-axisPErr.y))
              + kQI * (negativeZero(axisIErr.x) + negativeZero(-axisIErr.y))
              + kQD * (negativeZero(axisDErr.x) + negativeZero(-axisDErr.y))
  	          , 1.0f);
              
  		motor[3] = min(throttle 
  	          + kQP * (negativeZero(-axisPErr.x) + negativeZero(-axisPErr.y))
              + kQI * (negativeZero(-axisIErr.x) + negativeZero(-axisIErr.y))
              + kQD * (negativeZero(-axisDErr.x) + negativeZero(-axisDErr.y))
  	          , 1.0f);
  	} else {
		motor[0] = 0;
		motor[1] = 0;
		motor[2] = 0;
		motor[3] = 0;
  	}
	Serial.print("motor =(");
	Serial.print(motor[0]);
	Serial.print(", ");
	Serial.print(motor[1]);
	Serial.print(", ");
	Serial.print(motor[2]);
	Serial.print(", ");
	Serial.print(motor[3]);
	Serial.println(")");
}

// Apply thrust with given motor power
void applyControlledThrust() {
	motor0.write((int)(motor[0] * 180.0f));
	motor1.write((int)(motor[1] * 180.0f));
	motor2.write((int)(motor[2] * 180.0f));
	motor3.write((int)(motor[3] * 180.0f));
}

// Read EEPROM and set calibration values of the imu
void calibrateRead() {
	// Lambda expression to retrieve imu offsets
	auto readEEPROM = [](int index) -> int16_t {
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

// Calibrate and write to EEPROM
void calibrateWrite() {
	Serial.println("Calculating calibration offsets, please let the sensor rest.");
	mpu.CalibrateAccel(6);
  	mpu.CalibrateGyro(6);
	mpu.PrintActiveOffsets(); // Print offsets for debugging reasons
	if(EEPROM.length() > 12) {
		EEPROM.put(1, mpu.getXAccelOffset());
		EEPROM.put(3, mpu.getYAccelOffset());
		EEPROM.put(5, mpu.getZAccelOffset());

		EEPROM.put(7, mpu.getXGyroOffset());
		EEPROM.put(9, mpu.getYGyroOffset());
		EEPROM.put(11, mpu.getZGyroOffset());
		
		EEPROM.update(0, 0x4B); // First byte is set to the unique magic number 0x4B so that we know that the offsets are set in the future
	} else {
		Serial.println("You do not have a functional EEPROM. Offsets will not be written, consider using a chip with an EEPROM");
	}
}
