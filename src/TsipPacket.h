#ifndef _TSIPPACKET_h
#define _TSIPPACKET_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "utility\debughelp.h"
#include "gpstype.h"
#include "chunk.h"

#define MAX_RPTBUF  256

enum TsipControlBytes 
{
	DLE = 0x10,     // TSIP message start code and byte stuffing escape value
	ETX = 0x03      // TSIP message end code
};

enum TsipParserState
{
	Empty,
	Full,
	Data,
	DLE1,
	DLE2
};

enum TsipTransmissionError
{
	TSIP_NoError,
	TSIP_Timeout,
	TSIP_Unknown
};

class TsipPacket
{
private:
	short counter;						// counter for data retrieval functions
	uint8_t packet_data[MAX_RPTBUF];	// TSIP data packet

protected:
	size_t length;						// received byte count < MAX_RPTBUF
	CommandID command;					// used for sending packets
	ReportType code;					// TSIP report code
	TsipParserState status;			// TSIP packet format/parse status
	void encode(uint8_t value);
	TsipTransmissionError error;
	uint8_t getNextByte();
	uint16_t getNextWord();
	uint32_t getNextDWord();
	uint64_t getNextDoubleDWord();
	float getNextFloat();
	double getNextDouble();
	void clear();

public:
	TsipPacket();

	bool isComplete() const;
	size_t getLength() const;
	ReportType getPacketType() const;
	byte getPacketByte(uint8_t index) const;
	word getPacketWord(uint8_t index) const;
	unsigned long getPacketDWord(uint8_t index) const;
	float getPacketFloat(uint8_t index) const;
	double getPacketDouble(uint8_t index) const;
	SubReportID_8F getSubReportID() const;
	TsipTransmissionError getLastError() const;

	friend class Thunderbolt;
} typedef const TSIP_PACKET;

#endif
