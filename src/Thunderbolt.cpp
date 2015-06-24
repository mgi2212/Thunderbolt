//  @brief
// Arduino Thunderbolt support This library is designed to handle asynchronous (broadcast) TSIP packets from a Trimble Thunderbolt
#include "Thunderbolt.h"

Thunderbolt::Thunderbolt(Stream* _serial) : m_n_listeners(0)
{
	m_serial = _serial;
}


bool Thunderbolt::flush()
{
	if (m_serial == NULL) {
		DEBUG_PRINT("No serial port defined...");
		return false;
	}
	m_serial->flush();
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
	assert(m_serial);
#ifdef DEBUG_PACKETS
	Serial.print("packet->");
	Serial.print((uint8_t)CTRL_DLE, HEX);
	Serial.print(" ");
	Serial.print((uint8_t)cmd, HEX);
	Serial.print(" ");
#endif

	m_serial->write((uint8_t)DLE);
	m_serial->write((uint8_t)cmd);
}

/**
* End a command by sending the end-of-transmission byte sequence.
*/
void Thunderbolt::endCommand() {
	assert(m_serial);

#ifdef DEBUG_PACKETS
	Serial.print((uint8_t)CTRL_DLE, HEX);
	Serial.print(" ");
	Serial.println((uint8_t)CTRL_ETX, HEX);
#endif

	m_serial->write((uint8_t)DLE);
	m_serial->write((uint8_t)ETX);
}

/**
* This is the entry point to the parser. Reads data bytes from a TSIP report packet, 
* unpacking any escape sequences in the process, placing `n` decoded bytes into a packet 
* buffer. Does not block, partial packets are retained until they are complete or reset.
*/
void Thunderbolt::readSerial() {
	assert(m_serial);

	while (m_serial->available() > 0)
	{
		rcv_packet.encode(m_serial->read());
		if (rcv_packet.isComplete())
		{
			process_report();
		}
	}
}

/**
* Encode `n` data bytes as part of a TSIP command packet and send them to the
* gps module. Must be called only if a command has been opened with a call to
* `beginCommand()`, and may be called multiple times before a call to `endCommand()`.
* @param bytes Data bytes to encode and send.
* @param n Number of bytes to encode.
*/
void Thunderbolt::writeDataBytes(const uint8_t* bytes, int n) {
	assert(m_serial);

	for (int i = 0; i < n; i++) {
		m_serial->write(bytes[i]);

#ifdef DEBUG_PACKETS
		Serial.print(bytes[i], HEX);
		Serial.print(" ");
#endif

		if (bytes[i] == DLE) {
			// avoid ambiguity with "end txmission" byte sequence
			m_serial->write(bytes[i]);

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
	while ((m_serial->available() <= 0))
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
	assert(m_serial);

	for (short s = 0; s < TSIP_CMD_RETRYS; s++)
	{
		beginCommand(xmt_packet.command);
		writeDataBytes(xmt_packet.packet_data, xmt_packet.length);
		endCommand();

		while (!rcv_packet.isComplete())
		{
			if (!block_with_timeout())
			{
				xmt_packet.error = TSIP_Timeout; // communications timeout
				return false;
			}
			rcv_packet.encode(m_serial->read());
		}

		if (rcv_packet.code == haltAt)		// return when the expected packet is received
		{
			return true;
		}
		else if (rcv_packet.isComplete())	// could be a packet ahead of this request
		{
			process_report();				// process the unknown report (this clears the receive packet)
		}
		DEBUG_PRINTDEC("TSIP: retries =", s);
	}
	xmt_packet.error = TSIP_Timeout;		// communications timeout
	return false;
}

// waits to receive a tsip packet of a supplied ReportType then processes it
// returns true if the condition was met, otherwise false indicates a timeout
bool Thunderbolt::waitForPacketAndSubReport(ReportType haltCommand, SubReportID_8F haltSubCommand)
{
	//DEBUG_PRINT(__FUNCTION__);
	assert(m_serial);

	for (short s = 0; s < TSIP_CMD_RETRYS; s++)
	{
		beginCommand(xmt_packet.command);
		writeDataBytes(xmt_packet.packet_data, xmt_packet.length);
		endCommand();

		while (!rcv_packet.isComplete())
		{
			if (!block_with_timeout())  // will timeout
			{
				xmt_packet.error = TSIP_Timeout; // communications timeout
				return false;
			}
			rcv_packet.encode(m_serial->read());
		}
		if ((rcv_packet.code == haltCommand) && (rcv_packet.getSubReportID() == haltSubCommand))
		{
			return true;
		}
		else if (rcv_packet.isComplete())	// could be a packet ahead of this request
		{
			process_report();			// process the unknown report (this clears the receive packet)
		}
		DEBUG_PRINTDEC("TSIP: retries =", s);
	}
	xmt_packet.error = TSIP_Timeout;	// communications timeout
	return false;
}

/***********************
* Commands            *
***********************/

bool Thunderbolt::getSoftwareVersionInfo()
{
	DEBUG_PRINT(__FUNCTION__);
	xmt_packet.clear();
	xmt_packet.command = CMD_REQ_SOFTWARE_VERSION;
	xmt_packet.length = 0;
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
	xmt_packet.clear();
	xmt_packet.command = CMD_TSIP_SUPERPACKET_8E;
	xmt_packet.packet_data[0] = SUBCMD_SET_REQ_PACKET_BROADCAST_MASK;
	//xmt_packet.packet_data[1] = 0;
	xmt_packet.packet_data[2] = mask;  // byte order
	//xmt_packet.packet_data[3] = 0;
	//xmt_packet.packet_data[4] = 0;
	xmt_packet.length = 5;
	if (waitForPacketAndSubReport(RPT_TSIP_SUPERPACKET_8F, SUBRPT_PACKET_BROADCAST_MASK))
	{
		DEBUG_PRINT("broadcast mask set");
		rcv_packet.clear();
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
	xmt_packet.clear();
	xmt_packet.command = CMD_SET_REQ_IO_OPTIONS;
	xmt_packet.length = 0;
	if (!waitForPacket(RPT_IO_SETTINGS))
	{
		assert(false);
		return false;
	}
	DEBUG_PRINT("received I/O options");

	xmt_packet.clear(); // prepare
	memcpy(xmt_packet.packet_data, rcv_packet.packet_data, rcv_packet.length);

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
			xmt_packet.packet_data[0] = (xmt_packet.packet_data[0] & ~pos_mask) | (POS_ECEF_ON | POS_LLA_ON); break;
		case RPT_FIX_POS_LLA_64:
			xmt_packet.packet_data[0] = (xmt_packet.packet_data[0] & ~pos_mask) | (POS_ECEF_ON | POS_LLA_ON | POS_DOUBLE); break;
		case RPT_FIX_POS_XYZ_32:
			xmt_packet.packet_data[0] = (xmt_packet.packet_data[0] & ~pos_mask) | (POS_ECEF_ON); break;
		case RPT_FIX_POS_XYZ_64:
			xmt_packet.packet_data[0] = (xmt_packet.packet_data[0] & ~pos_mask) | (POS_ECEF_ON | POS_DOUBLE); break;
		default: break; // do nothing
	}
	// alter velocity fixmode
	switch (vel_fixmode) {
		case RPT_FIX_VEL_XYZ:
			xmt_packet.packet_data[1] = (xmt_packet.packet_data[1] & ~vel_mask) | (VEL_ECEF_ON); break;
		case RPT_FIX_VEL_ENU:
			xmt_packet.packet_data[1] = (xmt_packet.packet_data[1] & ~vel_mask) | (VEL_ENU_ON); break;
		default: break; // do nothing
	}
	// alter other fixmode settings
	if (alt != ALT_NOCHANGE) xmt_packet.packet_data[0] = (xmt_packet.packet_data[0] & ~alt_mask) | alt;   // altitude
#ifdef COPERNICUS
	if (pps != PPS_NOCHANGE) xmt_packet.packet_data[2] = (xmt_packet.packet_data[2] & ~pps_mask) | pps; // Thunderbolt docs don't mention this Copernicus option
#endif
	if (time != TME_NOCHANGE) xmt_packet.packet_data[2] = (xmt_packet.packet_data[2] & ~tme_mask) | time; // time in GPS or UTC

	DEBUG_PRINT("modify I/O options");
	xmt_packet.command = CMD_SET_REQ_IO_OPTIONS;
	xmt_packet.length = 4;
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
	switch (rcv_packet.code)
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
				switch (rcv_packet.getSubReportID())
				{
					case SUBRPT_PACKET_BROADCAST_MASK:
						ok = true; break;
					case	SUBRPT_PRIMARY_TIMING_PACKET:
						ok = process_time(); break;
					case SUBRPT_SUPPLEMENTAL_TIMING_PACKET:
						ok = process_supplemental_timing(); break;
					default:
						DEBUG_PRINTHEX("Unhandled 8F Report = ", rcv_packet.getSubReportID());
						break;
				}
		}
			break;
		case RPT_SATELLITES:
			ok = process_satellites(); break;
		case RPT_SOFTWARE_VERSION:
			ok = process_software_version_info(); break;
		default:
			DEBUG_PRINTHEX("Unhandled Report = ", rcv_packet.code);
			break;
	}

	// give user's packet processors a swipe
	inform_external_processors(ok);

	// ready the packet buffer
	rcv_packet.clear();
	return ok;
}

void Thunderbolt::inform_external_processors(bool isProcessed)
{
	for (int i = 0; i < m_n_listeners; i++)
	{
		m_listeners[i]->tsipPacket(this->rcv_packet, isProcessed);
	}
}

bool Thunderbolt::process_software_version_info()
{
	m_version.app.major_ver = rcv_packet.getNextByte();
	m_version.app.minor_ver = rcv_packet.getNextByte();
	m_version.app.month = rcv_packet.getNextByte();
	m_version.app.day = rcv_packet.getNextByte();
	m_version.app.year1900 = rcv_packet.getNextByte();

	m_version.core.major_ver = rcv_packet.getNextByte();
	m_version.core.minor_ver = rcv_packet.getNextByte();
	m_version.core.month = rcv_packet.getNextByte();
	m_version.core.day = rcv_packet.getNextByte();
	m_version.core.year1900 = rcv_packet.getNextByte();
	return true;
}

bool Thunderbolt::process_satellites()
{
	DEBUG_PRINT(__FUNCTION__);
	// todo process sat packets
	return true;
}

bool Thunderbolt::process_time() {
	rcv_packet.getNextByte(); // waste the subcode
	m_time.time_of_week = rcv_packet.getNextDWord();
	m_time.week_no = rcv_packet.getNextWord();
	m_time.utc_offs = rcv_packet.getNextWord();
	m_time.timing_flags = rcv_packet.getNextByte();

	m_time.seconds = rcv_packet.getNextByte();
	m_time.minutes = rcv_packet.getNextByte();
	m_time.hours = rcv_packet.getNextByte();
	m_time.day = rcv_packet.getNextByte();
	m_time.month = rcv_packet.getNextByte();
	m_time.year = rcv_packet.getNextWord();
	return true;
}

bool Thunderbolt::process_supplemental_timing()
{
	rcv_packet.getNextByte(); // waste the subcode
	m_status.rcvr_mode = static_cast<ReceiverMode>(rcv_packet.getNextByte());
	m_status.disc_mode = (DiscipliningMode)rcv_packet.getNextByte();
	m_status.self_survey_progress = rcv_packet.getNextByte();
	m_status.holdover_duration = rcv_packet.getNextDWord();
	m_status.critical_alarms = rcv_packet.getNextWord();
	m_status.minor_alarms = rcv_packet.getNextWord();
	m_status.rcvr_status = (ReceiverStatus)rcv_packet.getNextByte();
	m_status.disc_activity = (DiscipliningActivity)rcv_packet.getNextByte();

	rcv_packet.getNextWord(); // spares

	m_status.pps_offset = rcv_packet.getNextFloat();
	m_status.mhz_offset = rcv_packet.getNextFloat();
	m_status.dac_value = rcv_packet.getNextDWord();
	m_status.dac_voltage = rcv_packet.getNextFloat();
	m_status.temperature = rcv_packet.getNextFloat();
	m_status.latitude = rcv_packet.getNextDouble();
	m_status.longitude = rcv_packet.getNextDouble();
	m_status.altitude = rcv_packet.getNextDouble();
	return true;
}

bool Thunderbolt::process_p_LLA_32() {
	DEBUG_PRINT(__FUNCTION__);
	m_pfix.type = RPT_FIX_POS_LLA_32;
	LLA_Fix<Float32> *fix = &m_pfix.lla_32;
	fix->lat.f = rcv_packet.getNextFloat();
	fix->lng.f = rcv_packet.getNextFloat();
	fix->alt.f = rcv_packet.getNextFloat();
	fix->bias.f = rcv_packet.getNextFloat();
	fix->fixtime.f = rcv_packet.getNextFloat();
	return true;
}

bool Thunderbolt::process_p_LLA_64() {
	DEBUG_PRINT(__FUNCTION__);
	m_pfix.type = RPT_FIX_POS_LLA_64;
	LLA_Fix<Float64> *fix = &m_pfix.lla_64;
	fix->lat.value.d = rcv_packet.getNextDouble();
	fix->lng.value.d = rcv_packet.getNextDouble();
	fix->alt.value.d = rcv_packet.getNextDouble();
	fix->bias.value.d = rcv_packet.getNextDouble();
	fix->fixtime.f = rcv_packet.getNextFloat(); // fixtime is always a 4-byte float
	return true;
}

bool Thunderbolt::process_p_XYZ_32() {
	DEBUG_PRINT(__FUNCTION__);
	m_pfix.type = RPT_FIX_POS_XYZ_32;
	XYZ_Fix<Float32> *fix = &m_pfix.xyz_32;

	fix->x.f = rcv_packet.getNextFloat();
	fix->y.f = rcv_packet.getNextFloat();
	fix->z.f = rcv_packet.getNextFloat();
	fix->bias.f = rcv_packet.getNextFloat();
	fix->fixtime.f = rcv_packet.getNextFloat();
	return true;
}

bool Thunderbolt::process_p_XYZ_64() {
	DEBUG_PRINT(__FUNCTION__);
	m_pfix.type = RPT_FIX_POS_XYZ_64;
	XYZ_Fix<Float64> *fix = &m_pfix.xyz_64;

	fix->x.value.d = rcv_packet.getNextDouble();
	fix->y.value.d = rcv_packet.getNextDouble();
	fix->z.value.d = rcv_packet.getNextDouble();
	fix->bias.value.d = rcv_packet.getNextDouble();
	fix->fixtime.f = rcv_packet.getNextFloat(); // fixtime is always a 4-byte float
	return true;
}

bool Thunderbolt::process_v_XYZ() {
	DEBUG_PRINT(__FUNCTION__);
	m_vfix.type = RPT_FIX_VEL_XYZ;
	XYZ_VFix *fix = &m_vfix.xyz;

	fix->x.f = rcv_packet.getNextFloat();
	fix->y.f = rcv_packet.getNextFloat();
	fix->z.f = rcv_packet.getNextFloat();
	fix->bias.f = rcv_packet.getNextFloat();
	fix->fixtime.f = rcv_packet.getNextFloat();
	return true;
}

bool Thunderbolt::process_v_ENU() {
	DEBUG_PRINT(__FUNCTION__);
	m_vfix.type = RPT_FIX_VEL_ENU;
	ENU_VFix *fix = &m_vfix.enu;

	fix->e.f = rcv_packet.getNextFloat();
	fix->n.f = rcv_packet.getNextFloat();
	fix->u.f = rcv_packet.getNextFloat();
	fix->bias.f = rcv_packet.getNextFloat();
	fix->fixtime.f = rcv_packet.getNextFloat();
	return true;
}

bool Thunderbolt::process_primary_timing() {
	DEBUG_PRINT(__FUNCTION__);
	m_time.time_of_week = rcv_packet.getNextFloat();
	m_time.week_no = rcv_packet.getNextWord();
	m_time.utc_offs = rcv_packet.getNextFloat();
	return true;
}

bool Thunderbolt::process_health() {
	DEBUG_PRINT(__FUNCTION__);
	m_status.health = static_cast<GPSHealth>(rcv_packet.packet_data[0]);
	return true;
}

bool Thunderbolt::process_addl_status() {
	DEBUG_PRINT(__FUNCTION__);
	m_status.rtclock_unavailable = (rcv_packet.packet_data[1] & 0x02) != 0;
	m_status.almanac_incomplete = (rcv_packet.packet_data[1] & 0x08) != 0;
	return true;
}

bool Thunderbolt::process_sbas_status() {
	DEBUG_PRINT(__FUNCTION__);
	m_status.sbas_corrected = (rcv_packet.packet_data[0] & 0x01) != 0;
	m_status.sbas_enabled = (rcv_packet.packet_data[0] & 0x02) != 0;
	return true;
}

/***************************
* access                  *
***************************/

const GPSVersion& Thunderbolt::getVersion() const {
	return m_version;
}

Stream* Thunderbolt::getSerial() {
	return m_serial;
}

/**
* Get the status and health of the reciever.
* If the unit has a GPS lock, `getStatus().health` will equal `HLTH_DOING_FIXES`.
*/
const GPSStatus& Thunderbolt::getStatus() const {
	return m_status;
}

/**
* Get the most current position fix.
*/
const PosFix& Thunderbolt::getPositionFix() const {
	return m_pfix;
}

/**
* Get the most current velocity fix.
*/
const VelFix& Thunderbolt::getVelocityFix() const {
	return m_vfix;
}

/**
* Get the most recent GPS time report. For accurate current time,
* this datum must be correlated with a PPS pulse signal.
*/
const GPSTime& Thunderbolt::getGPSTime() const {
	return m_time;
}

/**
* Add a `GPSPacketProcessor` to be notified of incoming TSIP packets. At most
* `MAX_PKT_PROCESSORS` (8) are supported at a time.
* @param pcs Processor to add.
* @return `false` if there was not enough space to add the processor, `true` otherwise.
*/
bool Thunderbolt::addPacketProcessor(TsipExternalPacketProcessor *pcs) {
	for (int i = 0; i < m_n_listeners; i++) {
		if (m_listeners[i] == pcs) return true;
	}
	if (m_n_listeners >= MAX_PKT_PROCESSORS) return false;
	m_listeners[m_n_listeners++] = pcs;
	return true;
}

/**
* Cease to notify the given `GPSPacketProcessor` of incoming TSIP packets.
* @param pcs Processor to remove.
*/
void Thunderbolt::removePacketProcessor(TsipExternalPacketProcessor *pcs) {
	bool found = false;
	for (int i = 0; i < m_n_listeners; i++) {
		if (m_listeners[i] == pcs) {
			found = true;
		}
		if (found && i < m_n_listeners - 1) {
			m_listeners[i] = m_listeners[i + 1];
		}
	}
	if (found) {
		m_n_listeners = m_n_listeners - 1;
	}
}

/****************************
* gps listener             *
****************************/

TsipExternalPacketProcessor::~TsipExternalPacketProcessor() {}