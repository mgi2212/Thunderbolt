// serialport.h
#ifndef _TBOLT_HW_SERIALPORT_h
#define _TBOLT_HW_SERIALPORT_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "TBoltSerialPort.h"

class TBoltHWSerialPort : public TBoltSerialPort
{
private:
	HardwareSerial* _port;

public:

	TBoltHWSerialPort(int serial_num)
	{
// ifdefs from HardwareSerial.h
		switch (serial_num) {
#if defined(UBRR1H) || defined(SERIAL_PORT_HARDWARE1)
			case 1: _port = &Serial1; break;
#endif
#if defined(UBRR2H) || defined(SERIAL_PORT_HARDWARE2)
			case 2: _port = &Serial2; break;
#endif
#if defined(UBRR3H) || defined(SERIAL_PORT_HARDWARE3)
			case 3: _port = &Serial3; break;
#endif
			default:
#if defined(UBRRH) || defined(UBRR0H) || defined(SERIAL_PORT_HARDWARE)
				_port = &Serial; break;
#else
				_port = NULL;
#endif
		}
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

