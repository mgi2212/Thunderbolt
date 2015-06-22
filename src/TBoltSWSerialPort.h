// serialport.h
#ifndef _TBOLT_SW_SERIALPORT_h
#define _TBOLT_SW_SERIALPORT_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include <SoftwareSerial.h>
#include "TBoltSerialPort.h"

class TBoltSWSerialPort : public TBoltSerialPort
{

private:
	SoftwareSerial* _port;

public:

	TBoltSWSerialPort(SoftwareSerial* sw_serial_port) 
	{
		_port = sw_serial_port;
	};

	inline void begin(long speed) { _port->begin(speed); };
	inline void end() { _port->end(); };
	inline int peek() { return _port->peek(); };
	inline int read() { return _port->read(); };
	inline int available() { return _port->available(); };
	inline void flush() { return _port->flush(); };
	inline size_t write(int n) { return _port->write(n); };
	inline size_t write(unsigned int n) { return _port->write(n); };
	inline size_t write(long n) { _port->write(n); };
	inline size_t write(unsigned long n) { return _port->write(n); };
	inline size_t write(uint8_t byte) { return _port->write(byte); };
	inline size_t write(const char *str) { return _port->write(str); };
	inline size_t write(const char* buffer, size_t size) { return _port->write(buffer, size); };
};


#endif

