/*
 * File:   gpstype.h
 * Author: tbabb
 * Tbolt additions by: n7hq
 *
 * Created on October 7, 2013, 11:04 PM
 * Revised on June 15, 2014
 */

#ifndef GPSTYPE_H
#define	GPSTYPE_H

//#define IEEE754_4BYTE_PRECISION

#include <stdint.h>
#include <float.h>
#include <time.h>

#define EXTERN extern
#define COORD int

// conversions
#define SQRT3			(1.732050808)
#define METERS_TO_FEET	(3.2808)
#define FEET_TO_METERS	(1.0000 / METERS_TO_FEET)

#define IABS(x)     (((x)<0)?(0-(x)):(x))
#define ABS(x)      (((x)<0.0F)?(0.0F-(x)):(x))
#define DABS(x)     (((x)<0.0)?(0.0-(x)):(x))
#define ROUND(x,to) ((((x)+(to)-1)/(to))*to)

// bits in the sat_flags byte
#define TEMP_SPIKE   HOLDOVER
#define CONST_CHANGE 0x80
#define TIME_SKIP    0x40
#define UTC_TIME     0x20
#define HOLDOVER     0x10
#define SAT_FLAGS    (CONST_CHANGE | TIME_SKIP | UTC_TIME | HOLDOVER)
#define SAT_COUNT    0x0F

#define SENSOR_TC    10.0F          // tbolt firmware temperature sensor averaging time (in seconds)

// TSIP message handler error codes
#define MSG_ID      0x1000
#define MSG_END     0x0300
#define MSG_TIMEOUT 0xFF00

// bits in the 0x35 command packet position byte[0]
#define POS_ECEF_ON	 0x01
#define POS_ECEF_OFF 0x00
#define POS_LLA_ON   0x02
#define POS_LLA_OFF  0x00
#define POS_MSL      0x04
#define POS_HAE      0x00
#define POS_UNUSED   0x08
#define POS_SINGLE   0x00
#define POS_DOUBLE   0x10
#define POS_MASK (POS_ECEF_ON | POS_LLA_ON | POS_MSL | POS_DOUBLE)

// bits in the 0x35 command packet velocity byte[1]
#define VEL_ECEF_ON    0x01
#define VEL_ENU_ON     0x02
#define VEL_MASK (VEL_ECEF_ON | VEL_ENU_ON)

// bits in the 0x35 command packet timing byte[2]
#define TIM_GPS     0x00
#define TIM_UTC     0x01
#define TIM_MASK	TIM_GPS

// bits in the 0x35 command packet aux byte[3]
#define AUX_5A_ON		  0x01
#define AUX_5A_OFF		  0x00
#define AUX_PR_FILTER_ON  0x02
#define AUX_PR_FILTER_OFF 0x00
#define AUX_UNUSED		  0x04
#define AUX_OUTPUT_DBHZ   0x08
#define AUX_OUTPUT_AME    0x00
#define AUX_MASK (AUX_5A_ON | AUX_PR_FILTER_ON | AUX_OUTPUT_DBHZ)

// bits for packet broadcast mask
#define PBM_NONE						B00000000  // no broadcast packets
#define PBM_PRIMARY_TIMING_INFO			B00000001  // rpt packet 0x8F-AB subcmd+16 bytes (time of week, week number, UTC offset, Timing Flags, Time of Day, Date) - see UTC or GPS
#define PBM_SUPPLEMENTAL_TIMING_INFO	B00000100  // rpt packet 0x8F-AC subcmd+68 bytes (rcvr mode, discipline mode, self-survey progress, holdover duration, crit alarms, minor alarms, GPS decode status, ... lat, long, altitude)
#define PBM_IND_SAT_SOLUTION_FMT0		B00010000  // rpt packet 0x8F-A7 (Format 0) float (Time of fix, Clock bias (combined), Clock bias rate (combined), per sat(Sat ID, Clock bias by sat)
#define PBM_IND_SAT_SOLUTION_FMT1		B00100000  // rpt packet 0x8F-A7 (Format 1) int
#define PBM_GPS_EPHEMERIS_SELLIST		B01000000  // rpt packet 0x6D (Fix dimension, Fix mode, Number of SVs in fix, PDOP, HDOP, VDOP, TDOP, per sat(SV PRN)
#define PBM_TIMING_ONLY (PBM_PRIMARY_TIMING_INFO | PBM_SUPPLEMENTAL_TIMING_INFO)
#define PBM_DEFAULT (PBM_PRIMARY_TIMING_INFO | PBM_SUPPLEMENTAL_TIMING_INFO | PBM_IND_SAT_SOLUTION_FMT0 | PBM_GPS_EPHEMERIS_SELLIST)  // from lady heather


EXTERN struct SAT_INFO sat[1 + 32];

/***************************
 * enums                   *
 ***************************/

// valid sub-commands for 0x8E superpacket
enum SubCommandID_8E
{
	SUBCMD_REQ_MANUFACTURING_PARAMS = 0x41,
	SUBCMD_REQ_PRODUCTION_PARAMS = 0x42,
	SUBCMD_REVERT_TO_DEFAULT = 0x45,
	SUBCMD_SET_REQ_PPS_CHARACTERISTICS = 0x4A,
	SUBCMD_SAVE_SEGMENT_EEPROM = 0x4C,
	SUBCMD_SET_DAC_VALUE = 0xA0,
	SUBCMD_10MHZ_OUTPUT_SENSE = 0xA1,
	SUBCMD_UTC_GPS_TIMING = 0xA2,
	SUBCMD_ISSUE_DISCIPLINING = 0xA3,
	SUBCMD_TEST_MODES = 0xA4,
	SUBCMD_SET_REQ_PACKET_BROADCAST_MASK = 0xA5,
	SUBCMD_ISSUE_SELF_SURVEY = 0xA6,
	SUBCMD_SET_REQ_DISCIPLINING_PARAMS = 0xA8,
	SUBCMD_SET_REQ_SELF_SURVEY_PARAMS = 0xA9,
	SUBCMD_REQ_PRIMARY_TIMING_PACKET = 0xAB,
	SUBCMD_REQ_SUPPLEMENTAL_TIMING_PACKET = 0xAC,
};

// sub-packet types for report superpacket 8F
enum SubReportID_8F
{
	SUBRPT_MANUFACTURING_PARAMS = 0x41,
	SUBRPT_PRODUCTION_PARAMS = 0x42,
	SUBRPT_REVERT_TO_DEFAULT = 0x45,
	SUBRPT_PPS_CHARACTERISTICS = 0x4A,
	SUBRPT_SEGMENT_EEPROM = 0x4C,
	SUBRPT_DAC_VALUE = 0xA0,
	SUBRPT_10MHZ_OUTPUT_SENSE = 0xA1,
	SUBRPT_UTC_GPS_TIMING = 0xA2,
	SUBRPT_TEST_MODES = 0xA4,
	SUBRPT_PACKET_BROADCAST_MASK = 0xA5,
	SUBRPT_INDIVIDUAL_SATELLITE_SOLUTIONS = 0xA7,
	SUBRPT_DISCIPLINING_PARAMS = 0xA8,
	SUBRPT_SELF_SURVEY_PARAMS = 0xA9,
	SUBRPT_PRIMARY_TIMING_PACKET = 0xAB,
	SUBRPT_SUPPLEMENTAL_TIMING_PACKET = 0xAC
};

// valid tsip commands
enum CommandID {
	CMD_NONE = 0x00,
	CMD_FACTORY_RESET = 0x1E,
	CMD_REQ_SOFTWARE_VERSION = 0x1F,
	CMD_REQ_ALMANAC = 0x20,
	CMD_REQ_GPS_SELECTION_LIST = 0x24,
	CMD_WARM_START_SELF_TEST = 0x25,
	CMD_REQ_SIGNAL_LEVELS = 0x27,
	CMD_REQ_ALMANC_HEALTH_PAGE = 0x29,
	CMD_SET_ACCURATE_INITAL_POS_CARTESIAN = 0x31,
	CMD_SET_ACCURATE_INITAL_POS_LATLONG = 0x32,
	CMD_SET_SINGLE_SAT_MODE = 0x34,
	CMD_SET_REQ_IO_OPTIONS = 0x35,
	CMD_REQ_LAST_POS = 0x37,
	CMD_REQ_LAST_RAW = 0x38,
	CMD_REQ_EPHEMERIS_STATUS = 0x3B,
	CMD_REQ_SAT_TRACKING_STATUS = 0x3C,
	CMD_TSIP_SUPERPACKET_8E = 0x8E
};

// valid tsip report (response) types
enum ReportType {
	//  Set if a known packet was corrupted or could not be processed.
	RPT_ERROR = -1,
	//  Set in fixes if the fix is invalid and/or no fix has been obtained yet.
	RPT_NONE = 0x00,
	//  GPS time report.
	RPT_GPSTIME = 0x41,
	//  Position fix, XYZ Earth-centered Earth-fixed, single-precision.
	RPT_FIX_POS_XYZ_32 = 0x42,
	//  Velocity fix, XYZ Earth-centered Earth-fixed.
	RPT_FIX_VEL_XYZ = 0x43,
	// software version
	RPT_SOFTWARE_VERSION = 0x45,
	//  Receiver health report.
	RPT_HEALTH = 0x46,
	// all tracked satellite signal levels
	RPT_ALL_SAT_SIG_LEVELS = 0x47,
	//  Position fix, Lat/Lng/Alt, single-precision (32 bit).
	RPT_ALMANAC_HEALTH_PAGE = 0x49,
	//  Position fix, Lat/Lng/Alt, single-precision (32 bit).
	RPT_FIX_POS_LLA_32 = 0x4A,
	//  Additional receiver status report (almanac / realtime clock availability)  Copernicus not Thunderbolt
	RPT_ADDL_STATUS = 0x4B,
	//  GPS IO settings.
	RPT_IO_SETTINGS = 0x55,
	//  Velocity fix, East/North/Up.
	RPT_FIX_VEL_ENU = 0x56,
	//  Last computed fix information report.
	RPT_LAST_FIX_INFO = 0x57,
	//  Status of Satellite Disable or Ignore Health
	RPT_HEALTH_DISABLE_OR_IGNORE = 0x59,
	//  Raw Measurement Data
	RPT_RAW_MEASUREMENT_DATA = 0x5A,
	//  Satellite Ephemeris Status
	RPT_SATELLITE_EPHEMERIS_STATUS = 0x5B,
	//  Satellite Tracking Status
	RPT_SATELLITE_TRACKING_STATUS = 0x5C,
	//  Satellite report.
	RPT_SATELLITES = 0x6D,
	//  Filter Configuration
	RPT_FILTER_CONFIG = 0x70,
	//  SBAS (Satellite-based augmentation system) mode report.
	RPT_SBAS_MODE = 0x82,
	//  Position fix, XYZ Earth-centered Earth-fixed, double-precision.
	RPT_FIX_POS_XYZ_64 = 0x83,
	//  Position fix, Lat/Lng/Alt, double-precision (64 bit).
	RPT_FIX_POS_LLA_64 = 0x84,
	// Report from a 0x8E superpacket request
	RPT_TSIP_SUPERPACKET_8F = 0x8F
};

// Health status
enum GPSHealth {
	//  Set if GPS health has not been established yet.
	HLTH_UNKNOWN = 0xFF,
	//  Set if reciever has a GPS lock and is obtaining valid fixes.
	HLTH_DOING_FIXES = 0x00,
	//  Set if GPS time has not been obtained yet.
	HLTH_NO_GPSTIME = 0x01,
	//  Set if satellite geometry is too poor to obtain a fix.
	HLTH_PDOP_TOO_HIGH = 0x03,
	//  Set if the chosen SV is unavailable.
	HLTH_SV_UNAVAILABLE = 0x04,
	//  Set if no useable satellites have been locked.
	HLTH_SATELLITES_NONE = 0x08,
	//  Set if only one useable satellite has been locked.
	HLTH_SATELLITES_ONE = 0x09,
	//  Set if only two useable satellites have been locked.
	HLTH_SATELLITES_TWO = 0x0A,
	//  Set if only three useable satellites have been locked.
	HLTH_SATELLITES_THREE = 0x0B,
	//  Set if operating in overdetermined mode.
	HLTH_SATELLITES_OVERDETERMINED = 0xBB,
};

enum PacketStatus {
	//  Indicates the GPSPacketProcessor does not wish to intercept this packet and no bytes have been consumed.
	PKT_IGNORE,
	//  Indicates that the GPSPacketProcessor has consumed and processed the packet, including the end-of-packet sequence.
	PKT_CONSUMED,
	//  Indicates that an error has occurred while processing the packet, and the stream should be advanced to a safe state.
	PKT_ERROR,
	//  Indicates that the GPSPacketProcessor has consumed some bytes of the packet, and that the remainder of the packet should be consumed.
	PKT_PARTIAL,
};

enum AltMode {
	//  Height above WGS-84 ellipsoid.
	ALT_HAE = 0x00,
	//  Height above mean sea level.
	ALT_MSL = 0x01,
	//  Flag to leave altitude mode unchanged.
	ALT_NOCHANGE = 0xFF,
};

enum PPSMode {
	//  PPS always on.
	PPS_ALWAYS = 0x00,
	//  PPS fix-based.
	PPS_FIX = 0x20,
	//  PPS off.
	PPS_OFF = 0x40,
	//  Flag to leave PPS unchanged.
	PPS_NOCHANGE = 0x60,
};

enum GPSTimeMode {
	//  Report GPS time.
	TME_GPSTIME = 0x00,
	//  Report UTC.
	TME_UTCTIME = 0x01,
	//  Flag to leave time reporting mode unchanged.
	TME_NOCHANGE = 0xFF
};

//enum TimingModes
//{
//	GPS,
//	UTC
//};

//  <summary>
//  Time Status
//  </summary>
//enum TimeType
//{
//	//  No Time Available
//	NoTimeAvailable,
//	//  No UTC Offset
//	NoUTCOffset,
//	//  User Set Time
//	UserSetTime,
//	//  UTC Time OK
//	UTCTimeOk,
//	//  GPS Time OK
//	GPSTimeOk
//};

//enum FixMode
//{
//	Auto = 0,
//	Manual = 1
//};

//  <summary>
//  Values that describe the demension of the current fix
//  </summary>
//enum FixDimension
//{
//	//  None
//	None,
//	//  1D Clock
//	Clock_1D,
//	//  2D Position
//	Position_2D,
//	//  3D Position
//	Position_3D,
//	//  Over Determined
//	OverDetermined
//};

//  <summary>
//  1	    Ideal	    This is the highest possible confidence level to be used for applications
//                      demanding the highest possible precision at all times.
//  1-2	    Excellent	At this confidence level, positional measurements are considered accurate
//                      enough to meet all but the most sensitive applications.
//  2-5	    Good	    Represents a level that marks the minimum appropriate for making business
//                      decisions. Positional measurements could be used to make reliable in-route
//                      navigation suggestions to the user.
//  5-10    Moderate	Positional measurements could be used for calculations, but the fix quality
//                      could still be improved. A more open view of the sky is recommended.
//  10-20	Fair	    Represents a low confidence level. Positional measurements should be discarded
//                      or used only to indicate a very rough estimate of the current location.
//  >20	    Poor	    At this level, measurements are inaccurate by as much as 300 meters with a
//                      6 meter accurate device (50 DOP × 6 meters) and should be discarded.
//  </summary>
//enum FixPrecision
//{
//	Ideal,
//	Excellent,
//	Good,
//	Moderate,
//	Fair,
//	Poor
//};

enum ReceiverStatus
{
	//  Doing Fixes
	DoingFixes = 0x00,
	//  No GPS Time
	NoGPSTime = 0x01,
	//  Reserved
	Reserved = 0x02,
	//  PDOP Too High
	PDOPTooHigh = 0x03,
	//  No Useable Sats
	NoUsableSats = 0x08,
	//  Only One Sat
	Only1Sat = 0x09,
	//  Only 2 Sats
	Only2Sats = 0x0A,
	//  Only 3 Sats
	Only3Sats = 0x0B,
	//  Sat Unuseable
	SatUnusable = 0x0C, // This message is included only when the one-satellite mode is in effect and
	// a specific satellite is chosen with Command Packet 0x34,
	// the selected satellite is not usable.
	//  TAIM Rejected
	TAIMRejected = 0x10,
	//  Unknown
	Unknown = 0xFF
};

//static const char* ReceiverStatusStrings[] = { "Doing Fixes", "No GPS Time", "Reserved", "PDOP Too High", "No Useable Stats", "Only 1 Sat", "Only 2 Sats", "Only 3 Sats", "Sat Unusable", "TAIMRejected", "Unknown"};

enum ReceiverMode
{
	//  Automatic
	Automatic = 0,
	//  Single Satellite
	SingleSatellite = 1,
	//  Horizontal
	Horizontal = 3,
	//  Full Position
	FullPosition = 4,
	//  Over Determined Clock
	OverDeterminedClock = 7,
};

enum DiscipliningMode
{
	//  Normal
	Normal = 0,
	//  Power Up
	PowerUp = 1,
	//  Auto Hold Over
	AutoHoldover = 2,
	//  Manual Hold Over
	ManualHoldover = 3,
	//  Recovery
	Recovery = 4,
	//  Disabled
	Disabled = 6
};

enum DiscipliningActivity
{
	//  Phase Locking
	PhaseLocking = 0,
	//  Osc Warm Up
	OscillatorWarmUp = 1,
	//  Frequency Locking
	FrequencyLocking = 2,
	//  Placing PPS
	PlacingPPS = 3,
	//  Init Loop Filter
	InitializingLoopFilter = 4,
	//  Compensating OCXO
	CompensatingOCXO = 5,
	//  Inactive
	Inactive = 6,
	//  Recovery Mode
	RecoveryMode = 8,
	//  Calibration Voltage
	CalibrationVoltage = 9
};

/***************************
 * storage types           *
 ***************************/

// many/most arduino chipsets do not support 64 bit floats, 
// but we still need to store the bits of a 64-bit float and convert them.

#pragma pack(4)		// set 4-byte alignment

// IEEE754 float layout;
struct IEEEfloat {
	uint32_t m : 23;
	uint8_t e : 8;
	uint8_t s : 1;
};

// IEEE754 double layout;
struct IEEEdouble {
	uint64_t m : 52;
	uint16_t e : 11;
	uint8_t s : 1;
};

//
// for conversion between 32-bit and 64-bit floating point
// a filler is added.
struct IEEEdouble2float {
	uint32_t filler;
	uint32_t m : 23;
	uint16_t e : 8;
	uint8_t  s : 1;
};

union Float32 {
	//  The bits of an IEEE 754 32-bit float.
	uint32_t bits;
	uint8_t bytes[4];
	IEEEfloat p;
	float f;
};

union Float64 {
	//  The bits of an IEEE 754 64-bit float.
	uint64_t bits;
	uint8_t bytes[8];
	IEEEdouble dbl;
	IEEEdouble2float sgl;		// for IEEE754 precision conversions
	struct {
#ifdef IEEE754_4BYTE_PRECISION
		uint8_t filler[4];	// 4-byte filler
#endif
		double d;
	} value;
};

/***************************
 * datapoints              *
 ***************************/

// all angles are in radians.
// fix times are -1 if the fix is not valid.

struct TBOLT_VERSION_REC {
	uint8_t major_ver;
	uint8_t minor_ver;
	uint8_t month;
	uint8_t day;
	uint8_t year1900; // minus 1900
};

struct GPSVersion {		// packet 0x45
	GPSVersion();
	TBOLT_VERSION_REC app;
	TBOLT_VERSION_REC core;
};

typedef struct SAT_INFO {  // the accumulated wisdom of the ages
	// packet 0x49 (Almanac Health Page)
	unsigned char health_flag;

	// packet 0x59 ( Status of Satellite Disable or Ignore Health)
	unsigned char disabled;
	unsigned char forced_healthy;

	// packet 0x5A ( Raw Measurement Data)
	float sample_len;
	float sig_level;				// (also packet 0x47, 0x5C)
	unsigned char level_msg;		// the message type that set sig_level
	float code_phase;
	float doppler;
	double raw_time;

	// packet 0x5B (satellite ephemeris status report)
	float eph_time;
	unsigned char eph_health;
	unsigned char iode;
	float toe;
	unsigned char fit_flag;
	float sv_accuracy;

	// packet 0x5C (Satellite Tracking Status)
	unsigned char slot;
	unsigned char chan;
	unsigned char acq_flag;
	unsigned char eph_flag;
	float time_of_week;
	float azimuth;
	float elevation;
	unsigned char el_dir;
	unsigned char age;
	unsigned char msec;
	unsigned char bad_flag;
	unsigned char collecting;

	// packet 0x6D (Satellite Selection List)
	int tracking;

	// packet 0xA7 (Individual Satellite Solutions) format 1
	float sat_bias;
	float time_of_fix;
	unsigned char last_bias_msg;		// flag set if sat info was from last message
} SatelliteInfo, * lpSatelliteInfo;


template <typename T>
struct LLA_Fix {
	T lat;
	T lng;
	T alt;
	T bias;
	Float32 fixtime;
};

template <typename T>
struct XYZ_Fix {
	T x;
	T y;
	T z;
	T bias;
	Float32 fixtime;
};

struct XYZ_VFix {
	Float32 x;
	Float32 y;
	Float32 z;
	Float32 bias;
	Float32 fixtime;
};

struct ENU_VFix {
	Float32 e;
	Float32 n;
	Float32 u;
	Float32 bias;
	Float32 fixtime;
};

/**
 * @brief Position fix.
 *
 * Depending on the reporting mode of the receiver, this structure may store
 * a report of type `RPT_FIX_POS_LLA_32`, `RPT_FIX_POS_LLA_64`, `RPT_FIX_POS_XYZ_32`,
 * `RPT_FIX_POS_XYZ_64`, or `RPT_NONE`. The stored type is indicated by
 * `PosFix.type`; use one of the four access methods to obtain an object storing
 * the actual position data. Access methods which don't currently correspond to
 * the store type will return `NULL`.
 *
 * If the report type is unknown, there is not yet a valid fix and all accessors
 * will return `NULL`.
 *
 * For example:
 *
 *      const PosFix &fix = gps.GetPositionFix();
 *      if (fix.type == RPT_FIX_POS_LLA_32) {
 *          LLA_Fix<Float32> fixdata = fix.getLLA_32();
 *          // ...
 *      } else if (fix.type == RPT_FIX_POS_XYZ_32) {
 *          XYZ_FIX<Float32> fixdata = fix.getXYZ_32();
 *          // ...
 *      } // etc.
 *
 */
struct PosFix {
	//  Format of stored position fix.
	ReportType type;

	PosFix();

	const LLA_Fix<Float32> *getLLA_32() const;
	const LLA_Fix<Float64> *getLLA_64() const;
	const XYZ_Fix<Float32> *getXYZ_32() const;
	const XYZ_Fix<Float64> *getXYZ_64() const;

protected:

	union {
		XYZ_Fix<Float32> xyz_32;
		XYZ_Fix<Float64> xyz_64;
		LLA_Fix<Float32> lla_32;
		LLA_Fix<Float64> lla_64;
	};

	friend class Thunderbolt;
};

/**
 * @brief Velocity fix.
 *
 * Depending on the reporting mode of the receiver, this structure may store
 * a report of type `RPT_FIX_VEL_XYZ`, `RPT_FIX_VEL_ENU`, or `RPT_NONE`. The
 * stored type is indicated by `VelFix.type`; use one of the two access methods
 * to obtain an object storing the actual velocity data. Access methods which
 * don't currently correspond to the store type will return `NULL`.
 *
 * If the report type is unknown, there is not yet a valid fix and all accessors
 * will return `NULL`.
 *
 * For example:
 *
 *     const VelFix &fix = gps.GetVelocityFix();
 *     if (fix.type == RPT_FIX_VEL_ENU) {
 *         ENU_VFix fixdata = fix.getENU();
 *         // ...
 *     } else {
 *         XYZ_VFix fixdata = fix.getXYZ();
 *         // ...
 *     }
 * .
 */
struct VelFix {
	//  Format of stored velocity fix.
	ReportType type;

	VelFix();

	const XYZ_VFix *getXYZ() const;
	const ENU_VFix *getENU() const;

protected:

	union {
		XYZ_VFix xyz;
		ENU_VFix enu;
	};

	friend class Thunderbolt;
};

struct GPSTime {				// 8F.AB
	GPSTime();

	uint32_t time_of_week;
	uint16_t week_no;
	int16_t utc_offs;			// signed int!
	uint8_t timing_flags;
	uint8_t Second;
	uint8_t Minute;
	uint8_t Hour;
	uint8_t Day;
	uint8_t Month;
	uint16_t Year;

	bool operator ==(const GPSTime& x) {
		return ((x.Second == this->Second) ||
			(x.Minute == this->Minute) ||
			(x.Hour == this->Hour) ||
			(x.Day == this->Day) ||
			(x.Month == this->Month) ||
			(x.Year == this->Year));
	}

	bool operator !=(const GPSTime& x) {
		return ((x.Second != this->Second) ||
			(x.Minute != this->Minute) ||
			(x.Hour != this->Hour) ||
			(x.Day != this->Day) ||
			(x.Month != this->Month) ||
			(x.Year != this->Year));
	}
};

/**
* @brief GPSStatus
*
*/
struct GPSStatus {
	GPSStatus();

	GPSHealth health;						// pkt 0x46
	int n_satellites;						// pkt 0x6d
	bool almanac_incomplete;				// pkt 0x4b
	bool rtclock_unavailable;				// pkt 0x4b
	bool sbas_enabled;						// pkt 0x82
	bool sbas_corrected;					// pkt 0x82

	ReceiverMode rcvr_mode;					// 8F.AC
	ReceiverStatus rcvr_status;				// 8F.AC
	DiscipliningMode disc_mode;				// 8F.AC
	DiscipliningActivity disc_activity;		// 8F.AC

	uint8_t self_survey_progress;			// 8F.AC 0-100%
	uint32_t holdover_duration;				// 8F.AC seconds
	uint16_t critical_alarms;				// 8F.AC bitfield
	uint16_t minor_alarms;					// 8F.AC bitfield
	float pps_offset;						// 8F.AC estimate of UTC/GPS in ns
	float mhz_offset;						// 8F.AC estimate of UTC/GPS in ppb
	uint32_t dac_value;						// 8F.AC offset binary (0x00 - 0xFFFFF)
	float dac_voltage;						// 8F.AC volts
	float temperature;						// 8F.AC degrees C
	double latitude;						// 8F.AC radians
	double longitude;						// 8F.AC radians
	double altitude;						// 8F.AC meters

	bool operator ==(const GPSStatus& x) {
		return ((x.altitude == this->altitude) ||
			(x.longitude == this->longitude) ||
			(x.latitude == this->latitude) ||
			(x.rcvr_mode == this->rcvr_mode) ||
			(x.rcvr_status == this->rcvr_status) ||
			(x.critical_alarms == this->critical_alarms) ||
			(x.minor_alarms == this->minor_alarms));
	}

	bool operator !=(const GPSStatus& x) {
		return ((x.altitude != this->altitude) ||
			(x.longitude != this->longitude) ||
			(x.latitude != this->latitude) ||
			(x.rcvr_mode != this->rcvr_mode) ||
			(x.rcvr_status != this->rcvr_status) ||
			(x.critical_alarms != this->critical_alarms) ||
			(x.minor_alarms != this->minor_alarms));
	}
};

//  @} // addtogroup datapoint

#endif	/* GPSTYPE_H */
