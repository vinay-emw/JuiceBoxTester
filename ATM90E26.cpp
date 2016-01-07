#include "ATM90E26.h"

#undef DEBUG
//#define DEBUG

#ifdef DEBUG
#define DPRINT(item) debug.print(item)
#define DPRINTLN(item) debug.println(item)
#define DPRINTHEX(item) debug.print(item,HEX);
#define DPRINTLNHEX(item) debug.println(item,HEX);
#else
#define DPRINT(item)
#define DPRINTLN(item)
#define DPRINTHEX(item)
#define DPRINTLNHEX(item)
#endif

/* Work around a bug with PROGMEM and PSTR where the compiler always
 * generates warnings.
 */
/*#undef PROGMEM 
#define PROGMEM __attribute__(( section(".progmem.data") )) 
#undef PSTR 
#define PSTR(s) (__extension__({static prog_char __c[] PROGMEM = (s); &__c[0];})) 
*/

ATM90E26Debug::ATM90E26Debug()
{
    debug = NULL;
}

void ATM90E26Debug::begin(Stream *debugPrint)
{
    debug = debugPrint;
}

size_t ATM90E26Debug::write(uint8_t data)
{
    if (debug != NULL) {
	return debug->write(data);
    }

    return 0;
}

ATM90E26::ATM90E26() {
	debugOn = true;
	// nothing to do here
}

boolean ATM90E26::begin(Stream *serialptr, Stream *debugstream) {
	debug.begin(debugstream);
	serial = serialptr;
	return true;
}

byte ATM90E26::lastError() {
	return errState;
}

inline boolean ATM90E26::isBuffered() {
	return serial->available();
}

boolean ATM90E26::isValid() {
	return (errState == EM_Valid);
}

void ATM90E26::setBit(byte theRegister, unsigned int bitMask) {
	unsigned int regData;
	regData = readNow(theRegister);
	regData &= bitMask;
	write(theRegister, regData);
}

void ATM90E26::clearBit(byte theRegister, unsigned int bitMask) {
	unsigned int regData;
	regData = readNow(theRegister);
	regData ^= bitMask;
	write(theRegister, regData);
}

unsigned int ATM90E26::readNow(byte theRegister) {
	// purge any junk data - this is the only operation that should be in process
	purgeSerial();

	theRegister |= 0x80; // mask bit 7 to set "read"
	serial->write(0xFE);
	serial->write(theRegister);
	serial->write(theRegister); // read checksum = address (including R/W flag - thx Atmel ;) )
	DPRINT(F("Send: "));
	DPRINTHEX(0xFE);
	DPRINT(" ");
	DPRINTHEX(theRegister);
	DPRINT(" ");
	DPRINTLNHEX(theRegister);
	return readBuffer();
}

float ATM90E26::readVolts() {
	return ((float)readNow(EM_Urms) / 100);
}

float ATM90E26::readCurrent() {
	return ((float)readNow(EM_Irms2) / 1000);
}

float ATM90E26::readFrequency() {
	return ((float)readNow(EM_Freq) / 100);
}

float ATM90E26::readPowerF() {
	return ((float)readNow(EM_PowerF) / 1000);
}
float ATM90E26::readPowerF2() {
	return ((float)readNow(EM_PowerF2) / 1000);
}

void ATM90E26::readLater(byte theRegister) {
	theRegister |= 0x80;
	serial->write(0xFE);
	serial->write(theRegister);
	serial->write(theRegister);
}

boolean ATM90E26::write(byte theRegister, unsigned int theData) {
	byte checksum;
	// purge any junk data
	purgeSerial();
	
	// get checksum ready
	checksum = theRegister;
	checksum += theData >> 8; // 1111 1111 0101 0101 => 1111 1111
	checksum += theData & 255; // 1111 1111 0101 0101 => 0101 0101
	serial->write(0xFE);
	serial->write(theRegister);
	serial->write(theData >> 8); // see above :)
	serial->write(theData & 255);
	serial->write(checksum);
	DPRINT(F("Send: "));
	DPRINTHEX(0xFE);
	DPRINT(" ");
	DPRINTHEX(theRegister);
	DPRINT(" ");
	DPRINTHEX(theData >> 8);
	DPRINT(" ");
	DPRINTHEX(theData & 255);
	DPRINT(" ");
	DPRINTLNHEX(checksum);

	if (!waitForData()) return false;
	if ((serial->read()) == checksum) {
                delay(250);
		errState = EM_Valid;
		return true;
	} else {
		errState = EM_ChksumFail;
		return false;
	}
}	

unsigned int ATM90E26::readBuffer() {
	unsigned int result;
	byte checksum = 0;

	// DATA_MSB
	if (!waitForData()) return false;
	checksum += serial->peek(); // make a checksum outta it
	result = serial->read() << 8; // 1111 1111 => 1111 1111 0000 0000
	// DATA_LSB
	if (!waitForData()) return false;
	checksum += serial->peek(); // drops overflow
	result += serial->read(); // 1010 1010 => 1111 1111 1010 1010
	// CHKSUM
	if (!waitForData()) return false;
	if (checksum == serial->read())
		errState = EM_Valid;
	else
		errState = EM_ChksumFail;
	return result;
}

// boolean waitForData()
// -- Blocks program until data is available
boolean ATM90E26::waitForData() {
	unsigned long timeout;
	timeout = millis() + 40; // 20ms max time to wait (5ms typical response time)
	while (millis() < timeout) {
		if (serial->available()) return true;
	}
	errState = EM_Timeout;
	return false;
}

void ATM90E26::purgeSerial() {
	while (serial->available()) serial->read();
}

