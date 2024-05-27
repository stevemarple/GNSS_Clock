#ifndef GNSS_CLOCK_H
#define GNSS_CLOCK_H

#define GNSS_CLOCK_VERSION "0.2.2"

#include "Arduino.h"
#include <MicroNMEA.h>
#include <RTCx.h>


class GNSS_Clock {
	// friend void isr(void);

public:

	inline GNSS_Clock(void) :
		_secondsSinceEpoch(0),
		_1ppsCallback(NULL) {
		clear();
	}

	bool begin(void* buffer, uint8_t len, uint8_t ppsPin, uint8_t edge=RISING);

	inline bool process(char c) {
		return _nmea.process(c);
	}

	bool isValid(void) const volatile;

	bool readClock(RTCx::time_t *t) const;

	inline bool readClock(RTCx::time_t &t) const {
		return readClock(&t);
	}

	bool readClock(RTCx::time_t &t, long& latitude, long& longitude,
				   long& altitude, bool& altitudeValid, char& navSystem,
				   uint8_t& numSat, uint8_t& hdop) const;

	bool readClock(struct RTCx::tm *tm) const;

	inline bool readClock(struct RTCx::tm &tm) const {
		return readClock(&tm);
	}

	void clear(void);

	inline void set1ppsCallback(void (*callback)(volatile const GNSS_Clock& clock)) {
		_1ppsCallback = callback;
	}

	inline const char* getSentence(void) const {
		return _nmea.getSentence();
	}

	inline MicroNMEA& getMicroNMEA(void) {
	    return _nmea;
	}

private:
	volatile char _navSystem;
	volatile bool _isValid;
	volatile long _latitude, _longitude; // In millionths of a degree
	volatile long _altitude; // In millimetres
	volatile bool _altitudeValid;
	volatile long _speed, _course;
	volatile int32_t _secondsSinceEpoch;
	//volatile struct RTCx::tm _tm;
	// volatile uint16_t _year;
	// volatile uint8_t _month, _day, _hour, _minute, _second;
	volatile uint8_t _numSat;
	volatile uint8_t _hdop;
	MicroNMEA _nmea;
	void (*_1ppsCallback)(volatile const GNSS_Clock &clock);

	static void isr(void);
	void ppsHandler(void);
};

extern GNSS_Clock gnss_clock;


#endif
