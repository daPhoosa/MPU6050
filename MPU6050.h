/*	Invensense MPU-6050 Library
	by Phillip Schmidt
	v1.0
	
	*** OBJECT INPUTS ***
	SampleRate	(3-1000)Hz with filtering, (31-8000)Hz w/o filtering.
				Hz = sampleClock / (1 + sampleDivider)  Only valid for integer values of sampleDivider.
				sampleClock = 1000Hz with filtering and 8000Hz w/o filtering.
	
	filterLevel	(0-6) ==> 0 = no filtering, 6=max filtering
	
	gyroRange	0 = +/- 250 deg/s
				1 = +/- 500 deg/s
				2 = +/- 1000deg/s
				3 = +/- 2000deg/s
				
	accelRange	0 = +/- 2g
				1 = +/- 4g
				2 = +/- 8g
				3 = +/-16g
	
	DLPF
		A_hz	A_del	G_hz	G_del	Fs_khz
	0	260		0.0		256		0.98	8
	1	184		2.0		188		1.9		1
	2	94		3.0		98		2.8		1
	3	44		4.9		42		4.8		1
	4	21		8.5		20		8.3		1
	5	10		13.8	10		13.4	1
	6	5		19.0	5		18.6	1
	
	
	*** FUNCTION USAGE ***
	some day soon...
	
*/


#ifndef MPU6050_h
#define MPU6050_h

#include <arduino.h>

class MPU6050
{
	public:
		MPU6050(uint16_t sampleRate, byte filterLevel, byte gyroRange, byte accelRange);
		
		void initialize();
		void retrieve();		// pull data from sensor and store locally
		void accelZero();
		void gyroZero();
		void convertToFloat();  // converts the integer results into floats and adds bias comp
		float temp();
		
		int16_t aXi, aYi, aZi;	// raw accelerometer results (int)
		int16_t gXi, gYi, gZi;	// raw gyro results (int)

		float aX, aY, aZ;		// accelerometer results
		float gX, gY, gZ;		// gyro results

		float aX_bias, aY_bias, aZ_bias;
		float gX_bias, gY_bias, gZ_bias;
		
		float accelToG, gyroToRad;
		
		unsigned long samplePeriod;  // [us] between when samples are updated
		
	private:
		void writeTo(int16_t device, byte address, byte val);  // I2C helper function
		
		byte _sampleRateDiv, _filter, _gFSR, _aFSR;
		byte _lowShift, _highShift;
		uint16_t _GyroClk;
		uint16_t _tempRaw;
		
};

#endif