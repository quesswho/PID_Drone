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

// Radio
RF24 radio(8, 9);
const byte address[6] = "01234";

// Controller input
bool light = false;
int mx = 0, my = 0, rot = 0, alt = 0;

// Control system
MPU6050 mpu;

uint8_t fifoBuffer[64];
Quaternion quatMeasured;
VectorInt16 accel;
VectorInt16 gyro;

Quaternion quatRef;
Quaternion quatErr;
Quaternion axisPErr;
Quaternion axisIErr;
Quaternion axisDErr;

float motor[4];

const float kQP = 0.5f;
const float kQI = 0.0f;
const float kQD = 1.2f;

const float controlsens = 0.01f;

float throttle = 0.0f;

const int pinMotor0 = 9;
const int pinMotor1 = 8;
const int pinMotor2 = 10;
const int pinMotor3 = 11;

// Do not run the program for more than 71 minutes because the time will overflow
// Optionally we can divide the micros() function by 4 because it only has a resolution of 4 micro seconds on 16MHz boards
unsigned long currentTime = 0;
unsigned long lastTime = 0;
float elapsedTime = 0.0f;

inline const float negativeZero(const float value) { // If value is negative then return zero
  	return value > 0 ? value : 0;
}

void setup() {
 	Wire.begin();
 	Wire.setClock(400000);
	Serial.begin(115200);

	mpu.initialize();

	while (Serial.read() >= 0); // empty buffer
  
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

	pinMode(pinMotor0, OUTPUT);
	pinMode(pinMotor1, OUTPUT);
	pinMode(pinMotor2, OUTPUT);
	pinMode(pinMotor3, OUTPUT);
 
	lastTime = micros();

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

void loop() {

  readTransmitter();
	if(micros() - lastTime > 1000) { // 1ms minimum between each iteration
		if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      applyControls();
			getRawSensor();
			calculateError();
			calculateMotorValues();
			applyControlledThrust();
		}
	}
}

void readTransmitter() {
  static byte packet[10];
  
  if (radio.available()) {
    radio.read(&packet, 10);
    byte mask = packet[0];
    if(mask > 0) {
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
          //digitalWrite(5, light);
        //  Serial.print(" Light: ");
        //  Serial.print(light);
        }
      }
    }
  } else if(radio.getARC() == 0) {
       radio.startListening();
  } else {
    Serial.println(radio.getARC());
  }
}

void applyControls() {
  throttle = max(min(throttle + (float)alt * 0.0000078125f, 0.6f), 0.0f); // 1/(256*500)
  
  static float sum_mx = 0.0f;
  static float sum_my = 0.0f;

  // Smooth input, easier of derivative component
  const float c = controlsens/512.0f;
  sum_mx = min((sum_mx + mx*c) * 0.95f, 0.3f); // s_{n+1} = (s_n+input) * decay
  sum_my = min((sum_my + my*c) * 0.95f, 0.3f);
  
  quatRef = Quaternion(1.0f, sum_my, sum_mx, 0.0f).getNormalized();
}

void getRawSensor() {
	mpu.dmpGetQuaternion(&quatMeasured, fifoBuffer);
	mpu.dmpGetAccel(&accel, fifoBuffer);
	mpu.dmpGetGyro(&gyro, fifoBuffer);
	currentTime = micros();
	elapsedTime = (currentTime - lastTime) / 1000000.0f;
	lastTime = currentTime;
}

void calculateError(){
	quatErr = quatRef.getProduct(quatMeasured.getConjugate());
	if(quatErr.w < 0) {
		quatErr = quatErr.getConjugate();
	}

  /* Serial.print("quatRef =(");
  Serial.print(quatRef.x);
  Serial.print(", ");
  Serial.print(quatRef.y);
  Serial.print(", ");
  Serial.print(quatRef.z);
  Serial.print(", ");
  Serial.print(quatRef.w);
  Serial.println(")");*/

  /*
  Serial.print("quatErr =(");
  Serial.print(quatErr.x);
  Serial.print(", ");
  Serial.print(quatErr.y);
  Serial.print(", ");
  Serial.print(quatErr.z);
  Serial.print(", ");
  Serial.print(quatErr.w);
  Serial.println(")");*/
 

	const float invT = 1.0f / (elapsedTime*20);
 	axisDErr = Quaternion((quatErr.w - axisPErr.w) * invT, (quatErr.x - axisPErr.x) * invT, (quatErr.y - axisPErr.y) * invT, (quatErr.z - axisPErr.z) * invT); 
 	// Calculate derivate by getting the difference between current and last value over time
  //if(true || axisDErr.y > 1.0f) {
    /*Serial.print("AxisDErr =(");
    Serial.print(quatErr.y);
    Serial.print(", ");
    Serial.print(axisPErr.y);
    Serial.print(", ");
    Serial.print(axisDErr.y);
    Serial.print(", ");
    Serial.print(quatErr.y-axisPErr.y);
    Serial.println(")");
 // }*/
	axisPErr = Quaternion(quatErr.w, quatErr.x, quatErr.y, quatErr.z);
  const float decay = 0.98f;
	axisIErr = Quaternion(axisIErr.w * decay + axisPErr.w * elapsedTime, axisIErr.x * decay + axisPErr.x * elapsedTime, axisIErr.y * decay + axisPErr.y * elapsedTime , axisIErr.z * decay + axisPErr.z * elapsedTime);
	/*Serial.print("AxisPErr =(");
	Serial.print(axisPErr.x);
	Serial.print(", ");
	Serial.print(axisPErr.y);
	Serial.print(", ");
	Serial.print(axisPErr.z);
  Serial.print(", ");
  Serial.print(axisPErr.w);
	Serial.println(")");*/
	
	/*Serial.print("AxisIErr =(");
	Serial.print(axisIErr.x);
	Serial.print(", ");
	Serial.print(axisIErr.y);
	Serial.print(", ");
	Serial.print(axisIErr.z);
 Serial.print(", ");
 Serial.print(axisIErr.w);
	Serial.println(")");
	*/
	/*Serial.print("AxisDErr =(");
	Serial.print(axisDErr.x);
	Serial.print(", ");
	Serial.print(axisDErr.y);
	Serial.print(", ");
	Serial.print(axisDErr.z);
 Serial.print(", ");
 Serial.print(axisDErr.w);
	Serial.println(")");*/
}

// Motor layout
//      |
//   0     2
//      X
//   1     3
//      |

void calculateMotorValues() {
	// Currently assuming thrust is proportional to the motor values
	motor[0] = min(throttle 
	          //+ kQP * (negativeZero(axisPErr.x) + negativeZero(axisPErr.y))
            + kQI * (negativeZero(axisIErr.x) + negativeZero(axisIErr.y))
            //+ kQD * (negativeZero(axisDErr.x) + negativeZero(axisDErr.y))
	          , 0.95f);
  
	motor[1] = min(throttle 
	          //+ kQP * (negativeZero(-axisPErr.x) + negativeZero(axisPErr.y))
            + kQI * (negativeZero(-axisIErr.x) + negativeZero(axisIErr.y))
	          //+ kQD * (negativeZero(-axisDErr.x) + negativeZero(axisDErr.y))
	          , 0.95f);
           
	motor[2] = min(throttle 
	          //+ kQP * (negativeZero(axisPErr.x) + negativeZero(-axisPErr.y))
            + kQI * (negativeZero(axisIErr.x) + negativeZero(-axisIErr.y))
            //+ kQD * (negativeZero(axisDErr.x) + negativeZero(-axisDErr.y))
	          , 0.95f);
            
	motor[3] = min(throttle 
	          //+ kQP * (negativeZero(-axisPErr.x) + negativeZero(-axisPErr.y))
            + kQI * (negativeZero(-axisIErr.x) + negativeZero(-axisIErr.y))
            //+ kQD * (negativeZero(-axisDErr.x) + negativeZero(-axisDErr.y))
	          , 0.95f);
           
 // if(motor[0] == 0.95f) {
    /*Serial.print("AxisPErr =(");
    Serial.print(axisPErr.x);
    Serial.print(", ");
    Serial.print(axisPErr.y);
    Serial.print(", ");
    Serial.print(axisPErr.z);
    Serial.print(", ");
    Serial.print(axisPErr.w);
    Serial.println(")");
    */
    /*Serial.print("AxisDErr =(");
    Serial.print(axisDErr.x);
    Serial.print(", ");
    Serial.print(axisDErr.y);
    Serial.print(", ");
    Serial.print(axisDErr.z);
    Serial.print(", ");
    Serial.print(axisDErr.w);
    Serial.print(", ");
    Serial.print(quatErr.y);
    Serial.println(")");*/

    
 // }
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

void applyControlledThrust() {
	analogWrite(pinMotor0 * 256 - 1, OUTPUT);
	analogWrite(pinMotor1 * 256 - 1, OUTPUT);
	analogWrite(pinMotor2 * 256 - 1, OUTPUT);
	analogWrite(pinMotor3 * 256 - 1, OUTPUT);
}

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
