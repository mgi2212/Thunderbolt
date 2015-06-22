#include "TBoltHWSerialPort.h"	// comment out to use software serial ports

#ifndef _TBOLT_HW_SERIALPORT_h
#include <SoftwareSerial.h>		// Thunderbolt library to use software serial ports (on the Due the SoftwareSerial library is not available)
#include "TBoltSWSerialPort.h"
#endif

#define IEEE754_4BYTE_PRECISION	// comment out if there is double precision support on the device 
#include "Thunderbolt.h"

#ifndef _TBOLT_HW_SERIALPORT_h
#define TX_PIN 3
#define RX_PIN 4
TBoltSWSerialPort s_port(new SoftwareSerial(RX_PIN, TX_PIN, false));
Thunderbolt tbolt = Thunderbolt(&s_port);
#else
#define TBOLT_SERIAL_PORT 2  // remove comment and set correct port number to use hw serial
Thunderbolt tbolt = Thunderbolt(new TBoltHWSerialPort(TBOLT_SERIAL_PORT));
#endif

GPSStatus prevStatus;
GPSTime prevTime;

void display_version(GPSVersion ver) {
  char buf[64];
  sprintf(buf, "Thunderbolt version: app:%d.%02d core:%d.%02d\n", ver.app.major_ver, ver.app.minor_ver, ver.core.major_ver, ver.core.minor_ver);
  Serial.print(buf);
}

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


void displayTime(GPSTime t) {
  char buf[20];
  sprintf(buf, "time: %02d:%02d:%02d\n", t.hours, t.minutes, t.seconds);
  Serial.print(buf);
}

void setup() {
  Serial.begin(115200);
  tbolt.begin();
  if (tbolt.getSoftwareVersionInfo() == false) // this call is synchronous (waits for a response - but will timeout)
  {
	  Serial.println("Unable to start Thunderbolt, nothing more to do.");
	  while (1);
  }
  display_version(tbolt.getVersion());
  tbolt.setPacketBroadcastMask(PBM_TIMING_ONLY);
  tbolt.setFixMode(RPT_FIX_POS_LLA_64, RPT_FIX_VEL_ENU, ALT_MSL, PPS_FIX, TME_UTCTIME);
}

void loop()
{

#ifndef TBOLT_SERIAL_PORT
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
    displayTime(t);
    prevTime = t;
  }
}

#ifndef TBOLT_SERIAL_PORT
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

