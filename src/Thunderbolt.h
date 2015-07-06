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
#include <Time.h>
#include "chunk.h"
#include "gpstype.h"
#include "TsipPacket.h"
#include "ITimeSource.h"

// The epoch year
static const int EPOCH_YEAR = 1900;
// Time/date constants that will probably never change.
static const int DAYS_IN_YEAR = 365;
static const int HOURS_IN_DAY = 24;
static const int SECONDS_IN_MINUTE = 60;
static const int MINUTES_IN_HOUR = 60;
static const uint32_t  WATCHDOG_TIMEOUT = 1250;  //ms timeout

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



class RTCFallBack {
public:
	virtual const uint32_t getUnixTime() = 0;
	virtual void setTime(time_t unixTime);
	virtual uint32_t getSyncInterval();
};


/**
* @brief Thunderbolt.
*
*/
class Thunderbolt : public ITimeSource
{
public:
	Thunderbolt(Stream& _serial);
	bool readSerial();

	bool addPacketProcessor(TsipExternalPacketProcessor *pcs);
	void removePacketProcessor(TsipExternalPacketProcessor *pcs);

	bool flush();
	bool isWatchdogExpired() {
		return (millis() > _watchdog);
	}

	void registerFallback(RTCFallBack &fallback);
	bool processFallback();

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

	uint32_t getMilliSecondsPerSecond();
	uint32_t getSecondsSince1900Epoch();

	void beginCommand(CommandID cmd);
	void endCommand();
	void writeDataBytes(const uint8_t *bytes, int n);

	void now(uint32_t *secs, uint32_t *fract);

	time_t getUnixTime();

	virtual uint32_t timeRecv(uint32_t *secs, uint32_t *fract) const // not used
	{
		*secs = 0;
		*fract = 0;
		return 0;
	}

	static bool isLeapYear(uint16_t year)
	{
		/* Check if the year is divisible by 4 or is divisible by 400 */
		if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0))
			return true;
		else
			return false;
	}

private:
	Stream* _serial_stream;
	TsipExternalPacketProcessor *_listeners[MAX_PKT_PROCESSORS];

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
	void update_FractionalSeconds();
	void update_lastTimeUpdate(uint32_t tmrVal);
	void reset_GPS_watchdog();

	GPSStatus	_status;
	GPSTime		_time;
	GPSVersion  _version;
	PosFix		_pfix;
	VelFix		_vfix;

	TsipPacket _rcv_packet;
	TsipPacket _xmt_packet;
	RTCFallBack *_fallBack;

	uint8_t _num_listeners;
	uint32_t _fractionalSecondsSinceLastUpdate;
	uint32_t _milliSecondsOfLastUpdate;
	uint32_t _milliSecondsPerSecond;
	time_t _watchdog;
};

#endif
