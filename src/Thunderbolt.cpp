//  @brief
// Arduino Thunderbolt support This library is designed to handle asynchronous (broadcast) TSIP packets from a Trimble Thunderbolt
#include "Thunderbolt.h"

Thunderbolt::Thunderbolt(Stream& _serial) :
	_serial_stream(&_serial),
	_num_listeners(0),
	_fractionalSecondsSinceLastUpdate(0),
	_milliSecondsOfLastUpdate(0),
	_milliSecondsPerSecond(0)
{
	// empty
	_watchdog = 0;
}

uint32_t Thunderbolt::getMilliSecondsPerSecond() {
	return _milliSecondsPerSecond;
}

void Thunderbolt::registerFallback(RTCFallBack &fallBack) {
	_fallBack = &fallBack;
}

time_t Thunderbolt::getUnixTime() {
		
	TimeElements tm;
	tm.Day = _time.day;
	tm.Hour = _time.hours;
	tm.Minute = _time.minutes;
	tm.Month = _time.month;
	tm.Second = _time.seconds;
	tm.Year = _time.year - 1970;
	return makeTime(tm);
}

bool Thunderbolt::processFallback() {
	if (!_fallBack) 	{
		return false;
	}

	if (!isWatchdogExpired())	{
		return false;
	}

	time_t t = _fallBack->getUnixTime();
	if (getUnixTime() != t) {
		uint16_t tmrValue = millis();
		TimeElements tm;
		breakTime(t, tm);
		_time.seconds = tm.Second;
		_time.minutes = tm.Minute;
		_time.hours = tm.Hour;
		_time.day = tm.Day;
		_time.month = tm.Month;
		_time.year = tm.Year + 1970;
		update_lastTimeUpdate(tmrValue);
		return true;
	}
}


void Thunderbolt::now(uint32_t *secs, uint32_t *fract) {
	if (secs)
	{
		*secs = getSecondsSince1900Epoch();
	}
	update_FractionalSeconds();
	if (fract)
	{
		*fract = _fractionalSecondsSinceLastUpdate;
	}
}

void Thunderbolt::update_FractionalSeconds(void)
{
	// Calculate new fractional value based on system runtime
	// since the Thunderbolt does not supply anything other than whole seconds.
	uint32_t currTime = millis();
	uint32_t millisecondDifference = currTime - _milliSecondsOfLastUpdate;
	_fractionalSecondsSinceLastUpdate = (millisecondDifference % _milliSecondsPerSecond) * (0xFFFFFFFF / _milliSecondsPerSecond);
}

void Thunderbolt::reset_GPS_watchdog() {
	_watchdog = millis() + WATCHDOG_TIMEOUT;
}

bool Thunderbolt::flush()
{
	if (_serial_stream == NULL) {
		DEBUG_PRINT("No serial port defined...");
		return false;
	}
	_serial_stream->flush();
	return true;
}

/***************************
* i/o                     *
***************************/

/**
* Begin a TSIP command by sending the header bytes for the given
* command type.
* @param cmd Command to begin.
*/
void Thunderbolt::beginCommand(CommandID cmd) {
	assert(_serial_stream);
#ifdef DEBUG_PACKETS
	Serial.print("packet->");
	Serial.print((uint8_t)CTRL_DLE, HEX);
	Serial.print(" ");
	Serial.print((uint8_t)cmd, HEX);
	Serial.print(" ");
#endif

	_serial_stream->write((uint8_t)DLE);
	_serial_stream->write((uint8_t)cmd);
}

/**
* End a command by sending the end-of-transmission byte sequence.
*/
void Thunderbolt::endCommand() {
	assert(_serial_stream);

#ifdef DEBUG_PACKETS
	Serial.print((uint8_t)CTRL_DLE, HEX);
	Serial.print(" ");
	Serial.println((uint8_t)CTRL_ETX, HEX);
#endif

	_serial_stream->write((uint8_t)DLE);
	_serial_stream->write((uint8_t)ETX);
}

/**
* This is the entry point to the parser. Reads data bytes from a TSIP report packet,
* unpacking any escape sequences in the process, placing `n` decoded bytes into a packet
* buffer. Does not block, partial packets are retained until they are complete or reset.
*/
bool Thunderbolt::readSerial() {
	assert(_serial_stream);

	bool retval = false;
	while (_serial_stream->available() > 0)
	{
		_rcv_packet.encode(_serial_stream->read());
		if (_rcv_packet.isComplete())
		{
			process_report();
			return true;
		}
	}
	return retval;
}

/**
* Encode `n` data bytes as part of a TSIP command packet and send them to the
* gps module. Must be called only if a command has been opened with a call to
* `beginCommand()`, and may be called multiple times before a call to `endCommand()`.
* @param bytes Data bytes to encode and send.
* @param n Number of bytes to encode.
*/
void Thunderbolt::writeDataBytes(const uint8_t* bytes, int n) {
	assert(_serial_stream);

	for (int i = 0; i < n; i++) {
		_serial_stream->write(bytes[i]);

#ifdef DEBUG_PACKETS
		Serial.print(bytes[i], HEX);
		Serial.print(" ");
#endif

		if (bytes[i] == DLE) {
			// avoid ambiguity with "end txmission" byte sequence
			_serial_stream->write(bytes[i]);

#ifdef DEBUG_PACKETS
			Serial.print(bytes[i], HEX);
			Serial.print(" ");
#endif
		}
	}
}

bool Thunderbolt::block_with_timeout()
{
	unsigned long timeout = millis() + SERIAL_TIMEOUT;
	while ((_serial_stream->available() <= 0))
	{
		if (timeout <= millis())
		{
			return false;
		}
	}
	return true;
}

/// <summary>
/// Sends a prepared packet, then waits to receive a TSIP packet of a supplied type
/// This function will timeout and resend the packet
/// if there is a brief communications problem
/// </summary>
/// <param name="haltAt">The halt at.</param>
/// <returns>
/// Returns true if the packet was successfully sent and the expected information packet received.
///</returns>
bool Thunderbolt::waitForPacket(ReportType haltAt)
{
	assert(_serial_stream);

	for (short s = 0; s < TSIP_CMD_RETRYS; s++)
	{
		beginCommand(_xmt_packet.command);
		writeDataBytes(_xmt_packet.packet_data, _xmt_packet.length);
		endCommand();

		while (!_rcv_packet.isComplete())
		{
			if (!block_with_timeout())
			{
				_xmt_packet.error = TSIP_Timeout; // communications timeout
				return false;
			}
			_rcv_packet.encode(_serial_stream->read());
		}

		if (_rcv_packet.code == haltAt)		// return when the expected packet is received
		{
			return true;
		}
		else if (_rcv_packet.isComplete())	// could be a packet ahead of this request
		{
			process_report();				// process the unknown report (this clears the receive packet)
		}
		DEBUG_PRINTDEC("TSIP: retries =", s);
	}
	_xmt_packet.error = TSIP_Timeout;		// communications timeout
	return false;
}

// waits to receive a tsip packet of a supplied ReportType then processes it
// returns true if the condition was met, otherwise false indicates a timeout
bool Thunderbolt::waitForPacketAndSubReport(ReportType haltCommand, SubReportID_8F haltSubCommand)
{
	//DEBUG_PRINT(__FUNCTION__);
	assert(_serial_stream);

	for (short s = 0; s < TSIP_CMD_RETRYS; s++)
	{
		beginCommand(_xmt_packet.command);
		writeDataBytes(_xmt_packet.packet_data, _xmt_packet.length);
		endCommand();

		while (!_rcv_packet.isComplete())
		{
			if (!block_with_timeout())  // will timeout
			{
				_xmt_packet.error = TSIP_Timeout; // communications timeout
				return false;
			}
			_rcv_packet.encode(_serial_stream->read());
		}
		if ((_rcv_packet.code == haltCommand) && (_rcv_packet.getSubReportID() == haltSubCommand))
		{
			return true;
		}
		else if (_rcv_packet.isComplete())	// could be a packet ahead of this request
		{
			process_report();			// process the unknown report (this clears the receive packet)
		}
		DEBUG_PRINTDEC("TSIP: retries =", s);
	}
	_xmt_packet.error = TSIP_Timeout;	// communications timeout
	return false;
}

/***********************
* Commands            *
***********************/

bool Thunderbolt::getSoftwareVersionInfo()
{
	DEBUG_PRINT(__FUNCTION__);
	_xmt_packet.clear();
	_xmt_packet.command = CMD_REQ_SOFTWARE_VERSION;
	_xmt_packet.length = 0;
	if (waitForPacket(RPT_SOFTWARE_VERSION))
	{
		return process_report();
	}
	//assert(false);
	return false;
}

bool Thunderbolt::setPacketBroadcastMask(uint8_t mask)
{
	DEBUG_PRINT(__FUNCTION__);
	_xmt_packet.clear();
	_xmt_packet.command = CMD_TSIP_SUPERPACKET_8E;
	_xmt_packet.packet_data[0] = SUBCMD_SET_REQ_PACKET_BROADCAST_MASK;
	//xmt_packet.packet_data[1] = 0;
	_xmt_packet.packet_data[2] = mask;  // byte order
	//xmt_packet.packet_data[3] = 0;
	//xmt_packet.packet_data[4] = 0;
	_xmt_packet.length = 5;
	if (waitForPacketAndSubReport(RPT_TSIP_SUPERPACKET_8F, SUBRPT_PACKET_BROADCAST_MASK))
	{
		DEBUG_PRINT("broadcast mask set");
		_rcv_packet.clear();
		return true;
	}

	assert(false);
	return false;
}

/**
* Set the format of position, velocity, and altitude fixes.
* PPS settings and GPS time format may also be set with this command.
*
* To leave a fix mode unchanged, pass `RPT_NONE`. Other mode settings
* have `NOCHANGE` constants which will preserve the current settings.
*
* @param pos_fixmode New position fix format. Any of the `RPT_FIX_POS_*` constants, or `RPT_NONE`.
* @param vel_fixmode New velocity fix format. Any of the `RPT_FIX_VEL_*` constants, or `RPT_NONE`.
* @param alt New altitude format.
* @param pps New PPS setting.
* @param time New GPS time format.
* @param block Whether to wait for a confirmation from the receiver that the settings
* have taken effect.
* @return `true` if the settings were change successfully, `false` if an I/O problem occurred.
*/
bool Thunderbolt::setFixMode(ReportType pos_fixmode,
	ReportType vel_fixmode,
	AltMode alt,
	PPSMode pps,
	GPSTimeMode time)
{
	DEBUG_PRINT(__FUNCTION__);
	DEBUG_PRINT("request I/O options");
	_xmt_packet.clear();
	_xmt_packet.command = CMD_SET_REQ_IO_OPTIONS;
	_xmt_packet.length = 0;
	if (!waitForPacket(RPT_IO_SETTINGS))
	{
		assert(false);
		return false;
	}
	DEBUG_PRINT("received I/O options");

	_xmt_packet.clear(); // prepare
	memcpy(_xmt_packet.packet_data, _rcv_packet.packet_data, _rcv_packet.length);

	const uint8_t pos_mask = POS_MASK;
	const uint8_t vel_mask = VEL_MASK;
	const uint8_t alt_mask = 0x04;
	const uint8_t tme_mask = TIM_MASK;

#ifdef COPERNICUS
	const uint8_t pps_mask = 0x60;  // not supported in tbolt?
#endif

	// alter position fixmode
	switch (pos_fixmode) {
		case RPT_FIX_POS_LLA_32:  // LLA = Latitude Longitude Altitude
			_xmt_packet.packet_data[0] = (_xmt_packet.packet_data[0] & ~pos_mask) | (POS_ECEF_ON | POS_LLA_ON); break;
		case RPT_FIX_POS_LLA_64:
			_xmt_packet.packet_data[0] = (_xmt_packet.packet_data[0] & ~pos_mask) | (POS_ECEF_ON | POS_LLA_ON | POS_DOUBLE); break;
		case RPT_FIX_POS_XYZ_32:
			_xmt_packet.packet_data[0] = (_xmt_packet.packet_data[0] & ~pos_mask) | (POS_ECEF_ON); break;
		case RPT_FIX_POS_XYZ_64:
			_xmt_packet.packet_data[0] = (_xmt_packet.packet_data[0] & ~pos_mask) | (POS_ECEF_ON | POS_DOUBLE); break;
		default: break; // do nothing
	}
	// alter velocity fixmode
	switch (vel_fixmode) {
		case RPT_FIX_VEL_XYZ:
			_xmt_packet.packet_data[1] = (_xmt_packet.packet_data[1] & ~vel_mask) | (VEL_ECEF_ON); break;
		case RPT_FIX_VEL_ENU:
			_xmt_packet.packet_data[1] = (_xmt_packet.packet_data[1] & ~vel_mask) | (VEL_ENU_ON); break;
		default: break; // do nothing
	}
	// alter other fixmode settings
	if (alt != ALT_NOCHANGE) _xmt_packet.packet_data[0] = (_xmt_packet.packet_data[0] & ~alt_mask) | alt;   // altitude
#ifdef COPERNICUS
	if (pps != PPS_NOCHANGE) _xmt_packet.packet_data[2] = (_xmt_packet.packet_data[2] & ~pps_mask) | pps; // Thunderbolt docs don't mention this Copernicus option
#endif
	if (time != TME_NOCHANGE) _xmt_packet.packet_data[2] = (_xmt_packet.packet_data[2] & ~tme_mask) | time; // time in GPS or UTC

	DEBUG_PRINT("modify I/O options");
	_xmt_packet.command = CMD_SET_REQ_IO_OPTIONS;
	_xmt_packet.length = 4;
	if (!waitForPacket(RPT_IO_SETTINGS))
	{
		assert(false);
		return false;
	}
	DEBUG_PRINT("I/O options modified");

	process_report();  // allow this packet through, to give any user packetProcessors a swipe at it

	return true;
}

bool Thunderbolt::process_report() {
	bool ok = false;
	switch (_rcv_packet.code)
	{
		case RPT_FIX_POS_LLA_32:
			ok = process_p_LLA_32(); break;
		case RPT_FIX_POS_LLA_64:
			ok = process_p_LLA_64(); break;
		case RPT_FIX_POS_XYZ_32:
			ok = process_p_XYZ_32(); break;
		case RPT_FIX_POS_XYZ_64:
			ok = process_p_XYZ_64(); break;
		case RPT_FIX_VEL_XYZ:
			ok = process_v_XYZ(); break;
		case RPT_FIX_VEL_ENU:
			ok = process_v_ENU(); break;
		case RPT_GPSTIME:
			ok = process_primary_timing(); break;
		case RPT_IO_SETTINGS:  // no need to process
			ok = true; break;
		case RPT_HEALTH:
			ok = process_health(); break;
		case RPT_ADDL_STATUS:
			ok = process_addl_status(); break;
		case RPT_TSIP_SUPERPACKET_8F:
		{
			switch (_rcv_packet.getSubReportID())
			{
				case SUBRPT_PACKET_BROADCAST_MASK:
					ok = true; break;
				case	SUBRPT_PRIMARY_TIMING_PACKET:
					ok = process_time(); break;
				case SUBRPT_SUPPLEMENTAL_TIMING_PACKET:
					ok = process_supplemental_timing(); break;
				default:
					DEBUG_PRINTHEX("Unhandled 8F Report = ", _rcv_packet.getSubReportID());
					break;
			}
		}
			break;
		case RPT_SATELLITES:
			ok = process_satellites(); break;
		case RPT_SOFTWARE_VERSION:
			ok = process_software_version_info(); break;
		default:
			DEBUG_PRINTHEX("Unhandled Report = ", _rcv_packet.code);
			break;
	}

	// give user's packet processors a swipe
	inform_external_processors(ok);

	// ready the packet buffer
	_rcv_packet.clear();
	return ok;
}

void Thunderbolt::inform_external_processors(bool isProcessed)
{
	for (int i = 0; i < _num_listeners; i++)
	{
		_listeners[i]->tsipPacket(this->_rcv_packet, isProcessed);
	}
}

bool Thunderbolt::process_software_version_info()
{
	_version.app.major_ver = _rcv_packet.getNextByte();
	_version.app.minor_ver = _rcv_packet.getNextByte();
	_version.app.month = _rcv_packet.getNextByte();
	_version.app.day = _rcv_packet.getNextByte();
	_version.app.year1900 = _rcv_packet.getNextByte();

	_version.core.major_ver = _rcv_packet.getNextByte();
	_version.core.minor_ver = _rcv_packet.getNextByte();
	_version.core.month = _rcv_packet.getNextByte();
	_version.core.day = _rcv_packet.getNextByte();
	_version.core.year1900 = _rcv_packet.getNextByte();
	return true;
}

bool Thunderbolt::process_satellites()
{
	DEBUG_PRINT(__FUNCTION__);
	// todo process sat packets
	return true;
}

void Thunderbolt::update_lastTimeUpdate(uint32_t tmrVal)  {
	_milliSecondsPerSecond = (_milliSecondsPerSecond + (tmrVal - _milliSecondsOfLastUpdate)) / 2;
	_fractionalSecondsSinceLastUpdate = 0;
	_milliSecondsOfLastUpdate = tmrVal;
}


// This function decodes the primary timing packet it is called at a rate of 1pps and used to track
// the micro controller's internal timing (ms/sec) it also resets any fractional seconds
bool Thunderbolt::process_time() {
	DEBUG_PRINT(__FUNCTION__);
	uint32_t tmrVal = millis();
	_rcv_packet.getNextByte(); // waste the sub code
	_time.time_of_week = _rcv_packet.getNextDWord();
	_time.week_no = _rcv_packet.getNextWord();
	_time.utc_offs = _rcv_packet.getNextWord();
	_time.timing_flags = _rcv_packet.getNextByte();
	_time.seconds = _rcv_packet.getNextByte();
	_time.minutes = _rcv_packet.getNextByte();
	_time.hours = _rcv_packet.getNextByte();
	_time.day = _rcv_packet.getNextByte();
	_time.month = _rcv_packet.getNextByte();
	_time.year = _rcv_packet.getNextWord();
	update_lastTimeUpdate(tmrVal);

	if (_fallBack) {
		time_t t = _fallBack->getUnixTime();
		if ((t % _fallBack->getSyncInterval()) == 0) {
			_fallBack->setTime(getUnixTime());
		}
	}
	reset_GPS_watchdog();
	return true;
}

bool Thunderbolt::process_supplemental_timing()
{
	DEBUG_PRINT(__FUNCTION__);
	_rcv_packet.getNextByte(); // waste the subcode
	_status.rcvr_mode = static_cast<ReceiverMode>(_rcv_packet.getNextByte());
	_status.disc_mode = (DiscipliningMode)_rcv_packet.getNextByte();
	_status.self_survey_progress = _rcv_packet.getNextByte();
	_status.holdover_duration = _rcv_packet.getNextDWord();
	_status.critical_alarms = _rcv_packet.getNextWord();
	_status.minor_alarms = _rcv_packet.getNextWord();
	_status.rcvr_status = (ReceiverStatus)_rcv_packet.getNextByte();
	_status.disc_activity = (DiscipliningActivity)_rcv_packet.getNextByte();

	_rcv_packet.getNextWord(); // spares

	_status.pps_offset = _rcv_packet.getNextFloat();
	_status.mhz_offset = _rcv_packet.getNextFloat();
	_status.dac_value = _rcv_packet.getNextDWord();
	_status.dac_voltage = _rcv_packet.getNextFloat();
	_status.temperature = _rcv_packet.getNextFloat();
	_status.latitude = _rcv_packet.getNextDouble();
	_status.longitude = _rcv_packet.getNextDouble();
	_status.altitude = _rcv_packet.getNextDouble();
	return true;
}

bool Thunderbolt::process_p_LLA_32() {
	DEBUG_PRINT(__FUNCTION__);
	_pfix.type = RPT_FIX_POS_LLA_32;
	LLA_Fix<Float32> *fix = &_pfix.lla_32;
	fix->lat.f = _rcv_packet.getNextFloat();
	fix->lng.f = _rcv_packet.getNextFloat();
	fix->alt.f = _rcv_packet.getNextFloat();
	fix->bias.f = _rcv_packet.getNextFloat();
	fix->fixtime.f = _rcv_packet.getNextFloat();
	return true;
}

bool Thunderbolt::process_p_LLA_64() {
	DEBUG_PRINT(__FUNCTION__);
	_pfix.type = RPT_FIX_POS_LLA_64;
	LLA_Fix<Float64> *fix = &_pfix.lla_64;
	fix->lat.value.d = _rcv_packet.getNextDouble();
	fix->lng.value.d = _rcv_packet.getNextDouble();
	fix->alt.value.d = _rcv_packet.getNextDouble();
	fix->bias.value.d = _rcv_packet.getNextDouble();
	fix->fixtime.f = _rcv_packet.getNextFloat(); // fixtime is always a 4-byte float
	return true;
}

bool Thunderbolt::process_p_XYZ_32() {
	DEBUG_PRINT(__FUNCTION__);
	_pfix.type = RPT_FIX_POS_XYZ_32;
	XYZ_Fix<Float32> *fix = &_pfix.xyz_32;

	fix->x.f = _rcv_packet.getNextFloat();
	fix->y.f = _rcv_packet.getNextFloat();
	fix->z.f = _rcv_packet.getNextFloat();
	fix->bias.f = _rcv_packet.getNextFloat();
	fix->fixtime.f = _rcv_packet.getNextFloat();
	return true;
}

bool Thunderbolt::process_p_XYZ_64() {
	DEBUG_PRINT(__FUNCTION__);
	_pfix.type = RPT_FIX_POS_XYZ_64;
	XYZ_Fix<Float64> *fix = &_pfix.xyz_64;

	fix->x.value.d = _rcv_packet.getNextDouble();
	fix->y.value.d = _rcv_packet.getNextDouble();
	fix->z.value.d = _rcv_packet.getNextDouble();
	fix->bias.value.d = _rcv_packet.getNextDouble();
	fix->fixtime.f = _rcv_packet.getNextFloat(); // fixtime is always a 4-byte float
	return true;
}

bool Thunderbolt::process_v_XYZ() {
	DEBUG_PRINT(__FUNCTION__);
	_vfix.type = RPT_FIX_VEL_XYZ;
	XYZ_VFix *fix = &_vfix.xyz;

	fix->x.f = _rcv_packet.getNextFloat();
	fix->y.f = _rcv_packet.getNextFloat();
	fix->z.f = _rcv_packet.getNextFloat();
	fix->bias.f = _rcv_packet.getNextFloat();
	fix->fixtime.f = _rcv_packet.getNextFloat();
	return true;
}

bool Thunderbolt::process_v_ENU() {
	DEBUG_PRINT(__FUNCTION__);
	_vfix.type = RPT_FIX_VEL_ENU;
	ENU_VFix *fix = &_vfix.enu;

	fix->e.f = _rcv_packet.getNextFloat();
	fix->n.f = _rcv_packet.getNextFloat();
	fix->u.f = _rcv_packet.getNextFloat();
	fix->bias.f = _rcv_packet.getNextFloat();
	fix->fixtime.f = _rcv_packet.getNextFloat();
	return true;
}

bool Thunderbolt::process_primary_timing() {
	DEBUG_PRINT(__FUNCTION__);
	_time.time_of_week = _rcv_packet.getNextFloat();
	_time.week_no = _rcv_packet.getNextWord();
	_time.utc_offs = _rcv_packet.getNextFloat();
	return true;
}

bool Thunderbolt::process_health() {
	DEBUG_PRINT(__FUNCTION__);
	_status.health = static_cast<GPSHealth>(_rcv_packet.packet_data[0]);
	return true;
}

bool Thunderbolt::process_addl_status() {
	DEBUG_PRINT(__FUNCTION__);
	_status.rtclock_unavailable = (_rcv_packet.packet_data[1] & 0x02) != 0;
	_status.almanac_incomplete = (_rcv_packet.packet_data[1] & 0x08) != 0;
	return true;
}

bool Thunderbolt::process_sbas_status() {
	DEBUG_PRINT(__FUNCTION__);
	_status.sbas_corrected = (_rcv_packet.packet_data[0] & 0x01) != 0;
	_status.sbas_enabled = (_rcv_packet.packet_data[0] & 0x02) != 0;
	return true;
}

/***************************
* access                  *
***************************/

const GPSVersion& Thunderbolt::getVersion() const {
	return _version;
}

/**
* Get the status and health of the reciever.
* If the unit has a GPS lock, `getStatus().health` will equal `HLTH_DOING_FIXES`.
*/
const GPSStatus& Thunderbolt::getStatus() const {
	return _status;
}

/**
* Get the most current position fix.
*/
const PosFix& Thunderbolt::getPositionFix() const {
	return _pfix;
}

/**
* Get the most current velocity fix.
*/
const VelFix& Thunderbolt::getVelocityFix() const {
	return _vfix;
}

/**
* Get the most recent GPS time report. For accurate current time,
* this datum must be correlated with a PPS pulse signal.
*/
const GPSTime& Thunderbolt::getGPSTime() const {
	return _time;
}

/**
* Add a `GPSPacketProcessor` to be notified of incoming TSIP packets. At most
* `MAX_PKT_PROCESSORS` (8) are supported at a time.
* @param pcs Processor to add.
* @return `false` if there was not enough space to add the processor, `true` otherwise.
*/
bool Thunderbolt::addPacketProcessor(TsipExternalPacketProcessor *pcs) {
	for (int i = 0; i < _num_listeners; i++) {
		if (_listeners[i] == pcs) return true;
	}
	if (_num_listeners >= MAX_PKT_PROCESSORS) return false;
	_listeners[_num_listeners++] = pcs;
	return true;
}

/**
* Cease to notify the given `GPSPacketProcessor` of incoming TSIP packets.
* @param pcs Processor to remove.
*/
void Thunderbolt::removePacketProcessor(TsipExternalPacketProcessor *pcs) {
	bool found = false;
	for (int i = 0; i < _num_listeners; i++) {
		if (_listeners[i] == pcs) {
			found = true;
		}
		if (found && i < _num_listeners - 1) {
			_listeners[i] = _listeners[i + 1];
		}
	}
	if (found) {
		_num_listeners -= 1;
	}
}

// February is 28 here, but we will account for leap years further down.
static int numDaysInMonths[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

//const unsigned long seventyYears = 2208988800UL; // to convert unix time to epoch

uint32_t Thunderbolt::getSecondsSince1900Epoch()
{
	uint32_t returnValue = 0;
	// Hours, minutes and regular seconds 
	returnValue =
		_time.seconds +
		(_time.minutes * SECONDS_IN_MINUTE) +
		(_time.hours * SECONDS_IN_MINUTE * MINUTES_IN_HOUR);

	// Days, months and years accounting for past leap years and for different sized months.
	uint32_t numDays = 0;
	for (uint32_t currentYear = EPOCH_YEAR; currentYear < _time.year; currentYear++)
	{
		if (isLeapYear(currentYear)) {
			numDays++;
		}
	}
	numDays += DAYS_IN_YEAR * (_time.year - EPOCH_YEAR);
	// calculate elapsed days, current year using a table, don't count current month (it's not over yet)
	// numDaysInMonths is zero-based
	for (uint8_t idx = 0; idx < _time.month - 1; idx++)
	{
		numDays += numDaysInMonths[idx]; 
	}
	// add days for this month, not today as it's not over yet either!
	numDays += _time.day - 1;
	// account for current leap year if applicable
	if (isLeapYear(_time.year) && (_time.month > 2)) {
		numDays++;
	}
	returnValue += numDays * SECONDS_IN_MINUTE * MINUTES_IN_HOUR * HOURS_IN_DAY;
	// Return final result.
	return returnValue;
}

/****************************
* gps listener             *
****************************/

TsipExternalPacketProcessor::~TsipExternalPacketProcessor() {}