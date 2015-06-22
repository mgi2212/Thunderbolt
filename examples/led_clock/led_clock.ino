

//#include "TBoltHWSerialPort.h"	// remove comments to use hardware serial ports 

#ifndef _TBOLT_HW_SERIALPORT_h
#include <SoftwareSerial.h>		// Thunderbolt library to use software serial ports. (on the Due the SoftwareSerial library is not available)
#include "TBoltSWSerialPort.h"
#endif

#define IEEE754_4BYTE_PRECISION	// comment out if there is double precision support on the device 
#include "Thunderbolt.h"

#ifndef _TBOLT_HW_SERIALPORT_h
#define TX_PIN 3
#define RX_PIN 4
SoftwareSerial _port(RX_PIN, TX_PIN, false); // RX, TX
TBoltSWSerialPort s_port(&_port);
Thunderbolt tbolt = Thunderbolt(&s_port);
#else
#define TBOLT_SERIAL_PORT 2  // remove comment and set correct port number to use hw serial
TBoltHWSerialPort s_port(TBOLT_SERIAL_PORT);
Thunderbolt tbolt = Thunderbolt(&s_port);
#endif

// clock display macros
#define ONES(x) (x % 10)
#define TENS(x) ((x / 10) % 10)

// Primary display 
#include <LedControl.h>
#define LED_CLK	9
#define LED_LOAD 8
#define LED_DIN	7
#define LED_INTENSITY 10  // (0-15)
#define LED_DEVICE_COUNT 1
LedControl primary_display = LedControl(LED_DIN, LED_CLK, LED_LOAD, LED_DEVICE_COUNT);

GPSStatus prevStatus;
GPSTime prevTime;

// displays core and app version numbers
void display_version(GPSVersion ver) {
	char buf[64];
	sprintf(buf, "Thunderbolt version: app:%d.%02d core:%d.%02d\n", ver.app.major_ver, ver.app.minor_ver, ver.core.major_ver, ver.core.minor_ver);
	Serial.print(buf);
}

// display basic status info
void display_status(GPSStatus s) {
	Serial.print("Status: ");
	Serial.print(s.rcvr_status);
	Serial.print(" Mode = ");
	Serial.print(s.rcvr_mode);
	Serial.print(" Lat = ");
	Serial.print(s.latitude.value.d * RAD_TO_DEG);
	Serial.print(" Lng = ");
	Serial.print(s.longitude.value.d * RAD_TO_DEG);
	Serial.print(" Alt = ");
	Serial.print(s.altitude.value.d * METERS_TO_FEET);
	Serial.print((s.critical_alarms) ? " (Critical Alarm!)" : "");
	Serial.println((s.minor_alarms) ? " (Minor Alarm!)" : "");
}

// write a decimal two digit value to the display
void display_clock_value(int display, int digit, int v, bool dec_point) {
	primary_display.setDigit(display, digit--, (byte)TENS(v), false);
	primary_display.setDigit(display, digit, (byte)ONES(v), dec_point);
}

// displayh current UTC time
void display_time(GPSTime t) {
	primary_display.setIntensity(0, LED_INTENSITY);
	primary_display.setChar(0, 7, ' ', false);
	primary_display.setChar(0, 6, ' ', false);
	display_clock_value(0, 5, t.hours, true);
	display_clock_value(0, 3, t.minutes, true);
	display_clock_value(0, 1, t.seconds, false);
}

// setup and clear display
void setup_display() {
	for (int index = 0; index < primary_display.getDeviceCount(); index++) {
		primary_display.shutdown(index, false);
		primary_display.setIntensity(index, LED_INTENSITY);  // brightness level (0 is min, 15 is max)
		primary_display.clearDisplay(index);					// clear display 
	}
}

void setup() {
	Serial.begin(115200);
	setup_display();
	tbolt.begin();
	if (tbolt.getSoftwareVersionInfo() == false) // this call is synchronous (waits for a response - but will timeout)
	{
		Serial.println("Unable to start Thunderbolt, nothing more to do.");
		while (1);
	}
	display_version(tbolt.getVersion());
	tbolt.setPacketBroadcastMask(PBM_TIMING_ONLY);	// this call is synchronous
	tbolt.setFixMode(RPT_FIX_POS_LLA_64, RPT_FIX_VEL_ENU, ALT_MSL, PPS_FIX, TME_UTCTIME);	// this call is synchronous
}

void loop()
{

#ifdef SoftwareSerial_h
	if (s_port.available()) {
		softSerialEvent();
	}
#endif
	GPSStatus s = tbolt.getStatus();
	if (prevStatus != s) {
		display_status(s);
		prevStatus = s;
	}

	GPSTime t = tbolt.getGPSTime();
	if (prevTime != t) {
		display_time(t);
		prevTime = t;
	}
}

#ifdef SoftwareSerial_h
void softSerialEvent() {
	tbolt.readSerial();
}
#elif TBOLT_SERIAL_PORT == 1
void serialEvent1() {
	tbolt.readSerial(); // Thunderbolt class serial in
}
#elif TBOLT_SERIAL_PORT == 2
void serialEvent2() {
	tbolt.readSerial(); // Thunderbolt class serial in
}
#elif TBOLT_SERIAL_PORT == 3
void serialEvent3() {
	tbolt.readSerial(); // Thunderbolt class serial in
}
#elif TBOLT_SERIAL_PORT == 4
void serialEvent4() {
	tbolt.readSerial(); // Thunderbolt class serial in
}
#endif
