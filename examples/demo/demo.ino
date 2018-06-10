#include <stdio.h>
#include <GNSS_Clock.h>
#include <AsyncDelay.h>

// Refer to serial devices logically
HardwareSerial& console = Serial;
HardwareSerial& gnss = Serial1;

const int ppsPin = 6;
char buffer[85];
bool ledState = LOW;
volatile bool ppsTriggered = false;


void ppsCallback(const GNSS_Clock __attribute__((unused)) &clock)
{
	ppsTriggered = true;
}


void gnssHardwareReset()
{
	console.write("gnssHardwareReset()");
	// Empty input buffer
	while (gnss.available())
		gnss.read();

	// Toggle reset line
	digitalWrite(A0, LOW);
	delay(50);
	digitalWrite(A0, HIGH);

	// Reset is complete when the first valid NMEA sentence (not
	// necessarily a valid fix) is received. Include a timeout in case
	// nothing is received.
	AsyncDelay timeout;
	timeout.start(10000, AsyncDelay::MILLIS);
	while (!timeout.isExpired()) {
		while (gnss.available()) {
			char c = gnss.read();
			if (gnss_clock.process(c))
				return;
		}
	}
}



void setup(void)
{
	console.begin(115200);
	gnss.begin(115200);
	delay(50);
	console.println("\nsetup()");

	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, ledState);

	pinMode(A0, OUTPUT);
	digitalWrite(A0, HIGH);

	// Initialize the GNSS clock library. Do this before resetting the
	// GNSS hardware because the reset function attempts to wait for
	// a valid NMEA sentence.
	pinMode(ppsPin, INPUT);
	gnss_clock.begin(buffer, sizeof(buffer), ppsPin);

	console.println("Resetting GNSS module ...");
	gnssHardwareReset();
	console.println("... done");

	// Compatibility mode off
	// MicroNMEA::sendSentence(gnss, "$PONME,2,6,1,0");
	//MicroNMEA::sendSentence(gnss, "$PONME,,,1,0");
	MicroNMEA::sendSentence(gnss, "$PNVGNME,2,7,1");

	// Clear the list of messages which are sent.
	MicroNMEA::sendSentence(gnss, "$PORZB");

	// Send RMC and GGA messages.
	MicroNMEA::sendSentence(gnss, "$PORZB,RMC,1,GGA,1");
	// MicroNMEA::sendSentence(gnss, "$PORZB,RMC,1,GGA,1,GSV,1");

	gnss_clock.set1ppsCallback(ppsCallback);
	console.println("end of setup()");
}

void loop(void)
{

	if (ppsTriggered) {
		ppsTriggered = false;
		ledState = !ledState;
		digitalWrite(LED_BUILTIN, ledState);
		console.println("=====================");
		console.println("===> PPS");
		RTCx::time_t t;
		bool valid = gnss_clock.readClock(t);
		console.print("Valid: ");
		console.println(valid ? "true" : "false");
		console.print("Seconds since epoch: ");
		console.println(t);

		if (valid) {
			struct RTCx::tm tm;
			gnss_clock.readClock(tm);

			char isoDateTime[25];
			snprintf(isoDateTime, sizeof(isoDateTime),
					 "%04d-%02d-%02d %02d:%02d:%02dZ",
					 tm.tm_year + 1900, tm.tm_mon+1, tm.tm_mday,
					 tm.tm_hour, tm.tm_min, tm.tm_sec);
			console.print("Date/time: ");
			console.println(isoDateTime);
		}
	}

	while (!ppsTriggered && gnss.available()) {
		char c = gnss.read();
		console.print(c);
		gnss_clock.process(c);
	}
}
