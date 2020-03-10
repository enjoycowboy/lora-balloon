/*
	PARA TESTAR ESSE CÓDIGO:
	DEFINIR A VARIAVEL TEST NA LINHA 8

*/

#define test

#include <Arduino.h>
#include <MPU6050_6Axis_MotionApps_V6_12.h>
#include <SD.h>
#include "SPI.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "SFE_BMP180.h"
MPU6050 mpu;

#define INTERRUPT_PIN 2 // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13		// (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;		// return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;	// expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;		// count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;		 // [w, x, y, z]         quaternion container
VectorInt16 aa;		 // [x, y, z]            accel sensor measurements
VectorInt16 gy;		 // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];		 // [psi, theta, phi]    Euler angle container
float ypr[3];		 // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

#define SDFILE_PIN_CS 10
#define errorled 7
#define okled 8
double altura;
double pressao;
double temperatura;

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
	mpuInterrupt = true;
}

File sdFile;
SFE_BMP180 bmp;
char filename[7] = "00.txt";

void setup()
{
	int i = 0;
	bmp.begin();
	pinMode(SDFILE_PIN_CS, OUTPUT);
	if (!SD.begin())
	{
		digitalWrite(errorled, HIGH);
	}

	//evita arquivos duplicados

	while (SD.exists(filename))
	{

		if (i < 10)
		{

			sprintf(filename, "0%d.txt", i);
		}
		else
		{
			sprintf(filename, "%2d.txt", i);
		}
		i++;
	}

// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
	Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
#endif

	// initialize serial communication

	Serial.begin(9600);
	while (!Serial)

		// NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
		// Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
		// the baud timing being too misaligned with processor ticks. You must use
		// 38400 or slower in these cases, or use some kind of external separate
		// crystal solution for the UART timer.

		// initialize device
		Serial.println(F("Initializing I2C devices..."));
	mpu.initialize();
	pinMode(INTERRUPT_PIN, INPUT);

	// verify connection
	Serial.println(F("Testing device connections..."));
	Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

	bmp.begin();
	altura = bmp.altitude();
	temperatura = bmp.getTemperatureC();

	// load and configure the DMP
	devStatus = mpu.dmpInitialize();

	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(51);
	mpu.setYGyroOffset(8);
	mpu.setZGyroOffset(21);
	mpu.setXAccelOffset(1150);
	mpu.setYAccelOffset(-50);
	mpu.setZAccelOffset(1060);
	// make sure it worked (returns 0 if so)
	if (devStatus == 0)
	{
		// Calibration Time: generate offsets and calibrate our MPU6050
		mpu.CalibrateAccel(6);
		mpu.CalibrateGyro(6);
		Serial.println();
		mpu.PrintActiveOffsets();
		// turn on the DMP, now that it's ready
		Serial.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
		Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
		Serial.println(F(")..."));
		attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		Serial.println(F("DMP ready! Waiting for first interrupt..."));

		dmpReady = true;
		dmpReady ? digitalWrite(okled, HIGH) : digitalWrite(errorled, HIGH);

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
	}
	else
	{
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}

	// configure LED for output
	pinMode(LED_PIN, OUTPUT);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop()
{
	altura = bmp.altitude();
	temperatura = bmp.getTemperatureC();

	// if programming failed, don't try to do anything
	if (!dmpReady)
		return;
	// read a packet from FIFO
	if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
	{ // Get the Latest packet
		// display Euler angles in degrees
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#ifdef test
		Serial.print(millis());
		Serial.print(", ");
		Serial.print(altura);
		Serial.print(", ");
		Serial.print(temperatura);
		Serial.print(", ");
		Serial.print(ypr[0] * 180 / M_PI);
		Serial.print(", ");
		Serial.print(ypr[1] * 180 / M_PI);
		Serial.print(", ");
		Serial.print(ypr[2] * 180 / M_PI);
		Serial.println();
		Serial.flush();

#else

		sdFile.print(millis());
		sdFile.print(", ");
		sdFile.print(altura);
		sdFile.print(", ");
		sdFile.print(temperatura);
		sdFile.print(", ");
		sdFile.print(ypr[0] * 180 / M_PI);
		sdFile.print(", ");
		sdFile.print(ypr[1] * 180 / M_PI);
		sdFile.print(", ");
		sdFile.print(ypr[2] * 180 / M_PI);
		sdFile.println();
		sdFile.flush();
#endif
	}
}