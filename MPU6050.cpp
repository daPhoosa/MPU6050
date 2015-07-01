/*	Invensense MPU-6050 Library
	by Phillip Schmidt
	v1.0
	
	! I2C buss must be initialized before the initialize() function !
	
	*** OBJECT INPUTS ***
	SampleRate	(3-1000)Hz with filtering, (31-8000)Hz w/o filtering.
				Hz = sampleClock / (1 + sampleDivider)  Only valid for integer values of sampleDivider.
				sampleClock = 1000Hz with filtering and 8000Hz w/o filtering.
	
	filterLevel	(0-6) ==> 0 = no filtering, 6=max filtering
	
	gyroRange:
				0 = +/- 250 deg/s
				1 = +/- 500 deg/s
				2 = +/- 1000deg/s
				3 = +/- 2000deg/s
				
	accelRange:
				0 = +/- 2g
				1 = +/- 4g
				2 = +/- 8g
				3 = +/-16g
	
	DLPF
		A_hz	A_del	G_hz	G_del	Fs_khz
	0	260	0.0	256	0.98	8
	1	184	2.0	188	1.9	1
	2	94		3.0	98		2.8	1
	3	44		4.9	42		4.8	1
	4	21		8.5	20		8.3	1
	5	10		13.8	10		13.4	1
	6	5		19.0	5		18.6	1
	
	
	*** FUNCTION USAGE ***
	some day soon...
	
*/

#include <wire.h>
#include "MPU6050.h"


#define MPU6050_ADD 0x68 // 7bit I2C sensor address, default on most boards
//#define MPU6050_ADD 0x69 // alternate address is 0x69 - todo: make this selectable at object creation for dual sensor usage

#define TEMP_SCALE  (1.0f / 340.0f)
#define TEMP_OFFSET 36.53f

#define ACCEL_BASE	2048.0f		// LSB per g   @ +/- 16g
#define GYRO_BASE		16.375f		// LSB per dps @ +/- 2000 deg/s

#define DEG_TO_RAD	(3.141592654f / 180.0f)



MPU6050::MPU6050(uint16_t sampleRate, byte filterLevel, byte gyroRange, byte accelRange)
{
	_filter = filterLevel;
	if(_filter > 0){
		_GyroClk = 1000;
	}else{
		_GyroClk = 8000;
	}
	
	
	_sampleRateDiv = byte((_GyroClk / sampleRate) - 1);
	samplePeriod = ((_sampleRateDiv + 1) * 1000000UL) / _GyroClk; // time between samples (us)
	_gFSR = gyroRange  << 3;	// bitshift to correct position for settings register
	_aFSR = accelRange << 3;
	
	accelToG  = 1.0f / (ACCEL_BASE * float(1 << (3 - accelRange)));		// constant to convert from raw int to float G

	gyroToRad = DEG_TO_RAD / (GYRO_BASE * float(1 << (3 - gyroRange)));	// constant to convert from raw int to float rad/sec
	
	aX_bias = 0;
	aY_bias = 0;
	aZ_bias = 0;
	
	gX_bias = 0;
	gY_bias = 0;
	gZ_bias = 0;

}


void MPU6050::writeTo(int16_t device, byte address, byte val) { // *** I2C Write Function ***
	Wire.beginTransmission(device);	//start transmission to device 
	Wire.write(address);			// send register address
	Wire.write(val);				// send value to write
	Wire.endTransmission();			//end transmission
}


void MPU6050::initialize(){
	
	delay(25); // make sure sensor has time to power up
	
	writeTo(MPU6050_ADD, 25, _sampleRateDiv);	//  sample rate divider: sample rate = mstrClock / (1 +  divider)
	writeTo(MPU6050_ADD, 26, _filter);			//  DLPF set.  (0 = 8kHz master clock else 1kHz master clock)
	writeTo(MPU6050_ADD, 27, _gFSR);			//  gyro full scale range
	writeTo(MPU6050_ADD, 28, _aFSR);			//  accel full scale range
	writeTo(MPU6050_ADD, 31, B00000000);		//  no motion detect
	writeTo(MPU6050_ADD, 35, B00000000);		//  no FIFO
	writeTo(MPU6050_ADD, 36, B00000000);		//  no mstr I2C
	writeTo(MPU6050_ADD, 55, B01110000);		//	configure interrupt  -- on when data ready, off on data read
	writeTo(MPU6050_ADD, 56, B00000001);		//	interrupt on
	writeTo(MPU6050_ADD, 106, B00000000);		//  no silly stuff
	writeTo(MPU6050_ADD, 107, B00000001);		//  no sleep and clock off gyro_X
	writeTo(MPU6050_ADD, 108, B00000000);		//  no goofball sleep mode
	
	// initialze read pointer to start of data
	Wire.beginTransmission(MPU6050_ADD); //start transmission to device 
	Wire.write(0x3B);			//sends address to read from
	Wire.endTransmission();		//end transmission
}


void MPU6050::retrieve(){

	Wire.beginTransmission(MPU6050_ADD);	//start transmission to device
	Wire.requestFrom(MPU6050_ADD, 14);		// request 14 bytes from device
	
	AX.byte[1] = Wire.read();	// high byte
	AX.byte[0] = Wire.read();	// low byte
	
	AY.byte[1] = Wire.read();	// high byte
	AY.byte[0] = Wire.read();	// low byte
	
	AZ.byte[1] = Wire.read();	// high byte
	AZ.byte[0] = Wire.read();	// low byte
	
	TempInt.byte[1] = Wire.read();	// high byte
	TempInt.byte[0] = Wire.read();	// low byte
	
	GX.byte[1] = Wire.read();	// high byte
	GX.byte[0] = Wire.read();	// low byte
	
	GY.byte[1] = Wire.read();	// high byte
	GY.byte[0] = Wire.read();	// low byte
	
	GZ.byte[1] = Wire.read();	// high byte
	GZ.byte[0] = Wire.read();	// low byte
	
	/* // old
	AX.full 		 = Wire.read() << 8 | Wire.read();
	AY.full  	 = Wire.read() << 8 | Wire.read();
	AZ.full  	 = Wire.read() << 8 | Wire.read();
	TempInt.full = Wire.read() << 8 | Wire.read();
	GX.full  	 = Wire.read() << 8 | Wire.read();
	GY.full  	 = Wire.read() << 8 | Wire.read();
	GZ.full  	 = Wire.read() << 8 | Wire.read();	
	*/
	
	Wire.write(0x3B);		// send pointer back to the beginning of data, saves time
	Wire.endTransmission(); //end transmission
	
	// Convert to float
	aX = float(AX.full) * accelToG + aX_bias;// todo: store biases in sensor
	aY = float(AY.full) * accelToG + aY_bias;
	aZ = float(AZ.full) * accelToG + aZ_bias;
	
	gX = float(GX.full) * gyroToRad + gX_bias;
	gY = float(GY.full) * gyroToRad + gY_bias;
	gZ = float(GZ.full) * gyroToRad + gZ_bias;
	
}


void MPU6050::accelZero() // Measure and store accelerometer bias offsets
{
	
	aX_bias = 0;
	aY_bias = 0;
	aZ_bias = 0;
	
	float sampleTempX = 0;
	float sampleTempY = 0;
	float sampleTempZ = 0;
	
	float sampleCount = 0;
	unsigned long sampleEnd = millis() + 1000;  //collect data for 1s
	
	while(millis() < sampleEnd){// loop to collect data
		unsigned long loopEnd = micros() + samplePeriod;
		
		retrieve();
		//convertToFloat();
		
		sampleTempX -= aX;
		sampleTempY -= aY;
		sampleTempZ -= aZ;
		sampleCount += 1.0f;
		
		delayMicroseconds(loopEnd - millis());	// delay to wait for new data to be sampled
	}
	
	// remove gravity (assumes one axis is parallel to gravity)
	if(sampleTempX > (.75f * sampleCount)){sampleTempX -= sampleCount;}
	if(sampleTempY > (.75f * sampleCount)){sampleTempY -= sampleCount;}
	if(sampleTempZ > (.75f * sampleCount)){sampleTempZ -= sampleCount;}
	
	if(sampleTempX < (-.75f * sampleCount)){sampleTempX += sampleCount;}
	if(sampleTempY < (-.75f * sampleCount)){sampleTempY += sampleCount;}
	if(sampleTempZ < (-.75f * sampleCount)){sampleTempZ += sampleCount;}
	
	aX_bias = sampleTempX / sampleCount;	// average bias readings
	aY_bias = sampleTempY / sampleCount;
	aZ_bias = sampleTempZ / sampleCount;

}


void MPU6050::gyroZero()	// Measure and store gyro bias offsets
{
	
	gX_bias = 0;
	gY_bias = 0;
	gZ_bias = 0;
	
	float sampleCount = 0;
	
	float sampleTempX = 0;
	float sampleTempY = 0;
	float sampleTempZ = 0;
	
	unsigned long sampleEnd = millis() + 1000;  //collect data for 1s
	
	while(millis() < sampleEnd){// loop to collect data
		unsigned long loopEnd = micros() + samplePeriod;
		
		retrieve();
		//convertToFloat();
		
		sampleTempX -= gX;
		sampleTempY -= gY;
		sampleTempZ -= gZ;
		sampleCount += 1.0;
		
		delayMicroseconds(loopEnd - millis());	// delay to wait for new data to be sampled
	}
	
	gX_bias = sampleTempX / sampleCount; // average
	gY_bias = sampleTempY / sampleCount;
	gZ_bias = sampleTempZ / sampleCount;

}


float MPU6050::temp_C()	// return sensor temperature
{
	return(float(TempInt.full) * TEMP_SCALE + TEMP_OFFSET);
}
