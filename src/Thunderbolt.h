
/***************************************************
This is a library for the Trimble Thunderbolt 
The unit uses either Hardware or software serial ports to 
communicate. 
****************************************************/
#ifndef _THUNDERBOLT_H
#define _THUNDERBOLT_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif


#define MAX_PKT_PROCESSORS 8
#define SERIAL_TIMEOUT 2000 // 2 seconds max wait for serial data
#define TSIP_BAUD_RATE 9600
#define TSIP_CMD_RETRYS  5  // commands sent to the thunderbolt are attempted 5 times before failing

#include "utility\debughelp.h"
#include <assert.h>
#include "chunk.h"
#include "gpstype.h"
#include "TsipPacket.h"

class Thunderbolt; // fwd decl

/**
* @brief Class for gaining access to raw TSIP packets.
*
* Packets managed by the Thunderbolt class will be also passed on to
* registered TsipExternalPacketProcessor, but have the 'isProcessed' flag set to
* indicate the packet has been processed.
*
* If the flag is not set, the packet is not used by this class.
*/
class TsipExternalPacketProcessor {
public:
	virtual ~TsipExternalPacketProcessor();

	virtual void tsipPacket(TSIP_PACKET tsip_packet, bool isProcessed);
};

/**
* @brief Thunderbolt.
*
*/
class Thunderbolt
{
public:

	Thunderbolt(Stream& _serial);
	Stream*	getSerial();
	bool getSoftwareVersionInfo();
	bool setFixMode(ReportType pos_fixmode, ReportType vel_fixmode, AltMode alt = ALT_NOCHANGE, PPSMode pps = PPS_NOCHANGE, GPSTimeMode time = TME_NOCHANGE);
	bool setPacketBroadcastMask(uint8_t mask);
	bool waitForPacket(ReportType type);
	bool waitForPacketAndSubReport(ReportType haltCommand, SubReportID_8F haltSubCommand);
	const GPSStatus& getStatus() const;
	const GPSTime& getGPSTime() const;
	const GPSVersion& getVersion() const;
	const PosFix& getPositionFix() const;
	const VelFix& getVelocityFix() const;

	int  readDataBytes(uint8_t *dst, int n);
	void writeDataBytes(const uint8_t *bytes, int n);
	bool flush();
	void beginCommand(CommandID cmd);
	void endCommand();
	void readSerial();

	bool addPacketProcessor(TsipExternalPacketProcessor *pcs);
	void removePacketProcessor(TsipExternalPacketProcessor *pcs);

private:

	Stream* m_serial;
	int m_serial_num;

	bool block_with_timeout();

	bool process_report();

	bool process_addl_status();
	bool process_datums();
	bool process_health();
	bool process_p_LLA_32();
	bool process_p_LLA_64();
	bool process_p_XYZ_32();
	bool process_p_XYZ_64();
	bool process_primary_timing();
	bool process_satellites();
	bool process_sbas_status();
	bool process_software_version_info();
	bool process_supplemental_timing();
	bool process_time();
	bool process_v_ENU();
	bool process_v_XYZ();

	void inform_external_processors(bool isProcessed);

	GPSStatus	m_status;
	GPSTime		m_time;
	GPSVersion  m_version;
	PosFix		m_pfix;
	VelFix		m_vfix;

	TsipExternalPacketProcessor *m_listeners[MAX_PKT_PROCESSORS];

	TsipPacket rcv_packet;
	TsipPacket xmt_packet;

	uint8_t m_n_listeners;
};

#endif
