// demonstrats how to use a software serial port with the Thunderbolt library.
// requires the LedControl library

#define IEEE754_4BYTE_PRECISION	// comment out if there is double precision support on the device 
#include "Thunderbolt.h"

#include <SoftwareSerial.h>		// Thunderbolt library to use software serial ports. (on the Due the SoftwareSerial library is not available)
#define TX_PIN 3
#define RX_PIN 4
SoftwareSerial s_port(RX_PIN, TX_PIN, false);	// Initialize serial port

Thunderbolt tbolt(&s_port);						// Intialize Thunderbolt

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
	Serial.print(s.latitude * RAD_TO_DEG);
	Serial.print(" Lng = ");
	Serial.print(s.longitude * RAD_TO_DEG);
	Serial.print(" Alt = ");
	Serial.print(s.altitude * METERS_TO_FEET);
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
	s_port.begin(TSIP_BAUD_RATE);

	setup_display();
	tbolt.flush();
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
	// read from the Thunderbolt when data is available
	if (s_port.available()) {
		tbolt.readSerial();
	}

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
