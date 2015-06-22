//  @brief
//  TSIP packet parsing and data accumulation, low-level validation and
//  primitives for removing sequential values from the packet data buffer.
#include "TsipPacket.h"

//************************************
// Method:    TsipPacket constructor
// FullName:  TsipPacket::TsipPacket
// Access:    public
//************************************
TsipPacket::TsipPacket()
{
	clear();
}

//************************************
// Method:    clear
// FullName:  TsipPacket::clear
// Access:    protected
//************************************
void TsipPacket::clear()
{
	counter = 0;
	length = 0;
	status = Empty;
	code = RPT_NONE;
	command = CMD_NONE;
	error = TSIP_NoError;
	memset((char *)packet_data, 0, sizeof(packet_data));
}

//************************************
// Method:    getLength
// FullName:  TsipPacket::getLength
// Access:    public 
// Returns:   size_t length of data in bytes held in the buffer
//************************************
size_t TsipPacket::getLength() const
{
	return length;
}

//  <summary>
//  Gets the super packet SubReportID
//  </summary>
SubReportID_8F TsipPacket::getSubReportID() const
{
	return (SubReportID_8F)packet_data[0];
}

//  <summary>
//  Returns the TSIP packet type (aka report code ... see TSIP docs)
//  </summary>
ReportType TsipPacket::getPacketType() const
{
	return code;
}

//  <summary>
//  Returns true when a complete TSIP packet has been parsed.
//  </summar
bool TsipPacket::isComplete() const
{
	bool retval = ((status == Full) && (code != RPT_NONE));
	return retval;
}

//  <summary>
//  Retrieves the next byte from the packet buffer.
//  </summary>
//  <returns>uint8_t</returns>
uint8_t TsipPacket::getNextByte()
{
	return packet_data[counter++];
}

byte TsipPacket::getPacketByte(uint8_t index) const
{
	if ((index >= MAX_RPTBUF) || (index >= length))
	{
		return 0xFF;
	}
	return packet_data[index];
}

//  <summary>
//  Retrieves the next 2-byte word value from the packet buffer.
//  </summary>
//  <returns>word</returns>
uint16_t TsipPacket::getNextWord()      // get next two byte (word) field device
{
	int16_t retval;
	copy_network_order(&retval, &packet_data[counter]);
	counter += sizeof(uint16_t);
	return retval;
}

word TsipPacket::getPacketWord(uint8_t index) const
{
	int16_t retval;
	copy_network_order(&retval, const_cast<uint8_t*>(&packet_data[index]));
	return retval;
}

//  <summary>
//  Retrieves the next 4-byte word value from the packet buffer.
//  </summary>
//  <returns></returns>
uint32_t TsipPacket::getNextDWord()
{
	int32_t retval;
	copy_network_order(&retval, &packet_data[counter]);
	counter += sizeof(uint32_t);
	return retval;
}

unsigned long TsipPacket::getPacketDWord(uint8_t index) const
{
	int32_t retval;
	copy_network_order(&retval, const_cast<uint8_t*>(&packet_data[index]));
	return retval;
}

//  <summary>
//  Retrieves the next 8-byte word value from the packet buffer.
//  </summary>
//  <returns></returns>
uint64_t TsipPacket::getNextDoubleDWord()
{
	int64_t retval;
	copy_network_order(&retval, &packet_data[counter]);
	counter += sizeof(uint64_t);
	return retval;
}

//  <summary>
//  Retrieves the next 4-byte (Single) precision value from the packet buffer.
//  </summary>
//  <returns>single</returns>
Float32 TsipPacket::getNextFloat()
{
	Float32 retval;
	copy_network_order(&retval, &packet_data[counter]);
	counter += sizeof(Float32);
	return retval;
}

float TsipPacket::getPacketFloat(uint8_t index) const
{
	Float32 retval;
	copy_network_order(&retval, const_cast<uint8_t*>(&packet_data[index]));
	return retval.f;
}
//  <summary>
//  Retrieves the next 8-byte (Double) precision value from the packet buffer.
//  Converts to 4-byte precision if 8-byte is unavailable
//  </summary>
//  <returns>double</returns>
Float64 TsipPacket::getNextDouble()
{
	Float64 retval;
#ifdef IEEE754_4BYTE_PRECISION
	// 8-byte to 4-byte IEEE754 conversion
	Float64 tmp;
	copy_network_order(&tmp, &packet_data[counter]);
	retval.sgl.filler = 0;
	retval.sgl.e = tmp.dbl.e - 1023 + 127;	// adjust exponent bias;
	retval.sgl.m = (tmp.dbl.m >> 29);		// shift for narrower mantissa width
	retval.sgl.s = tmp.dbl.s;				// get the sign
#else
	copy_network_order(&retval, &packet_data[counter]);
#endif
	counter += sizeof(Float64);
	return retval;
}

double TsipPacket::getPacketDouble(uint8_t index)  const
{
	Float64 retval;
	copy_network_order(&retval, const_cast<uint8_t*>(&packet_data[index]));
	return retval.value.d;
}

//  @brief
//  Accumulates bytes from the receiver, strips control bytes (DLE)
//  and checks for a valid packet end sequence (DLE ETX).
//  the isComplete() method determines if a complete packet is received.
//  packet structure: <DLE> <id> <data string bytes> <DLE> <ETX>

void TsipPacket::encode(uint8_t value)
{
	switch (status)
	{
		case DLE1:
		{
				switch (value)
				{
					case 0:
					case ETX:    // illegal TSIP id
						DEBUG_PRINTHEX("Parse Error: Illegal TSIP id = ", value);
						length = 0;
						status = Empty;
						break;
					case DLE:    // try normal message start again
						length = 0;
						status = DLE1;
						break;
					default:     // legal TSIP ID; start message
						code = (ReportType)value;
						length = 0;
						status = Data;
						break;
				}
				break;
		}
		case Data:
		{
				switch (value)
				{
					case DLE: // expect DLE or ETX next
						status = DLE2;
						break;
					default: // normal data here
						packet_data[length++] = value;
						break;
				}
				break;
		}
		case DLE2:
		{
				switch (value)
				{
					case DLE: // normal data byte
						packet_data[length++] = value;
						status = Data;
						break;
					case ETX: // end of message
						status = Full;
						DEBUG_PRINTHEX("full type=", code);
						DEBUG_PRINTHEX("subtype=", packet_data[0]);
						break;
					default: // error: treat as DLE1; start a new report packet
						DEBUG_PRINT("Parse Error: Treat as DLE1");
						code = (ReportType)value;
						length = 0;
						status = Data;
						break;
				}
					 break;
		}
		case Full:
		case Empty:
		default:
		{
				switch (value)
				{
					case DLE: // normal message start
						length = 0;
						status = DLE1;
						break;
					default: // error: ignore the new byte
						DEBUG_PRINTHEX("Parse Error: newbyte = ", value);
						DEBUG_PRINTHEX("	          status = ", status);
						length = 0;
						status = Empty;
						break;
				}
				break;
		}
	}
	if (length >= MAX_RPTBUF)
	{
		DEBUG_PRINT("Parse Error: Buffer Length Exceeded");
		length = 0;
		status = Empty;
	}
}