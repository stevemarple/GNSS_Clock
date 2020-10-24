#include <string.h>
#include <util/atomic.h>

#include "GNSS_Clock.h"
#include <MicroNMEA.h>

GNSS_Clock gnss_clock;


void GNSS_Clock::isr(void)
{
	gnss_clock.ppsHandler();
}


bool GNSS_Clock::begin(void* buffer, uint8_t len, uint8_t ppsPin, uint8_t edge)
{
	_nmea.setBuffer(buffer, len);
	pinMode(ppsPin, INPUT);
	attachInterrupt(digitalPinToInterrupt(ppsPin), isr, edge);
	return true;
}


bool GNSS_Clock::isValid(void) const volatile
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		return _isValid;
	}
}


bool GNSS_Clock::readClock(RTCx::time_t *t) const
{
	bool r = false;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		if (_isValid) {
			if (t != NULL)
				*t = _secondsSinceEpoch;
			r = true;
		}
	}
	return r;
}

bool GNSS_Clock::readClock(RTCx::time_t& t,
						   long& latitude,
						   long& longitude,
						   long& altitude,
						   bool& altitudeValid,
						   char& navSystem,
						   uint8_t& numSat,
						   uint8_t& hdop) const
{
	bool r;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		t = _secondsSinceEpoch;
		r = _isValid;
		if (_isValid) {
			latitude = _latitude;
			longitude = _longitude;
			altitudeValid = _altitudeValid;
			if (_altitudeValid)
				altitude = _altitude;
			navSystem = _navSystem;
			numSat = _numSat;
			hdop = _hdop;
		}
	}
	return r;
}

bool GNSS_Clock::readClock(struct RTCx::tm *tm) const
{
	bool r;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		RTCx::gmtime_r((const RTCx::time_t*)&_secondsSinceEpoch, tm);
		r = _isValid;
	}
	return r;
}

void GNSS_Clock::clear(void)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		_isValid = false;
		_navSystem = '\0';
		_numSat = 0;
		_hdop = 255;
		_latitude = 999000000L;
		_longitude = 999000000L;
		_altitude = _speed = _course = LONG_MIN;
		_altitudeValid = false;
	// _tm.tm_year = _tm.tm_mon = _tm.tm_mday = 0;
	// _tm.tm_yday = -1;
	// _tm.tm_hour = _tm.tm_min = _tm.tm_sec = 99;
	}
}

void GNSS_Clock::ppsHandler(void)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		// Try to obtain the time of fix, even if the time/position is not valid
		struct RTCx::tm tm;
		tm.tm_year = _nmea.getYear() - 1900;
		tm.tm_mon = _nmea.getMonth() - 1;
		tm.tm_mday = _nmea.getDay();
		tm.tm_hour = _nmea.getHour();
		tm.tm_min = _nmea.getMinute();
		tm.tm_sec = _nmea.getSecond();
		tm.tm_yday = -1;
		_secondsSinceEpoch = RTCx::mktime(tm);
		if (_nmea.isValid()) {
			if (_secondsSinceEpoch == -1)
				_isValid = false;
			else {
				// Increment to get current time. NB this doesn't take into
				// account any leap seconds which may be added.
				++_secondsSinceEpoch;
				_isValid = true;
			}
			_navSystem = _nmea.getNavSystem();
			_numSat = _nmea.getNumSatellites();
			_hdop = _nmea.getHDOP();
			_latitude = _nmea.getLatitude();
			_longitude = _nmea.getLongitude();
			long tmp = LONG_MIN;
			_altitudeValid = _nmea.getAltitude(tmp);
			if (_altitudeValid)
				_altitude = tmp;
		}
		else {
			clear();
		}
		_nmea.clear();

		if (_1ppsCallback)
			(*_1ppsCallback)(*this);
	}
}
