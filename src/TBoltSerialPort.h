// serialport.h
#ifndef _TBOLTSERIALPORT_h
#define _TBOLTSERIALPORT_h


class TBoltSerialPort
{
private:

public:

	virtual void begin(long speed);
	virtual void end();
	virtual int peek();
	virtual int read();
	virtual int available();
	virtual void flush();
	virtual size_t write(int n);
	virtual size_t write(unsigned int n);
	virtual size_t write(long n);
	virtual size_t write(unsigned long n);
	virtual size_t write(uint8_t byte);
	virtual size_t write(const char *str);
	virtual size_t write(const char* buffer, size_t size);
};

#endif

