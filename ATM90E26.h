#ifndef _ATM90E26_H_
#define _ATM90E26_H_

#include <Arduino.h>
#include <Stream.h>
#include <avr/pgmspace.h>

// Register Map from Atmel ATM90E26 Datasheet pp.21-22
// Status and Special Register
#define EM_SoftReset	0x00	// Software Reset
#define EM_SysStatus	0x01	// System Status
#define EM_FuncEn		0x02	// Function Enable
#define EM_SagTh		0x03	// Voltage Sag Threshold * Format: XXX.XX; unit is V
#define EM_SmallPMod	0x04	// Small-Power Mode * 0xA987 to enable; others to disable.
#define EM_LastData		0x06	// Last Read/Write SPI/UART Value

// Metering Calibration and Configuration Register
#define EM_LSB			0x08	// RMS/Power 16-bit LSB
#define EM_CalStart		0x20	// Calibration Start Command
								//   0x6886 - Power-on value. Metering function is disabled.
								//   0x5678 - Metering calibration startup command. After 0x5678 is written to this register, registers 0x21-0x2B resume
								//            to their power-on values. The M90E26 starts to meter and output energy pulses regardless of the
								//            correctness of diagnosis. The CalErr[1:0] bits (SysStatus) are not set and the WarnOut/IRQ pins
								//            do not report any warning/interrupt.
								//   0x8765 - Check the correctness of the 0x21-0x2B registers. If correct, normal metering. If not correct, metering
								//            function is disabled, the CalErr[1:0] bits (SysStatus) are set and the WarnOut/IRQ pins report
								//            warning/interrupt.
								//   Others - Metering function is disabled. The CalErr[1:0] bits (SysStatus) are set and WarnOut is asserted.
#define EM_PLConstH		0x21	// High word of PL_Constant
#define EM_PLconstL		0x22	// Low word of PL_Constant
#define EM_Lgain		0x23	// L Line Calibration Gain
#define EM_Lphi			0x24	// L Line Calibration Angle
#define EM_Ngain		0x25	// N Line Calibration Gain
#define EM_Nphi			0x26	// N Line Calibration Angle
#define EM_PStartTH		0x27	// Active Startup Power Threshold
#define	EM_PNolTh		0x28	// Active No-Load Power Threshold
#define EM_QStartTh		0x29	// Reactive Startup Power Threshold
#define EM_QNolTh		0x2A	// Reactive No-Load Power Threshold
#define EM_MMode		0x2B	// Metering Mode Configuration
#define EM_CS1			0x2C	// Checksum 1

// Measurement Calibration Register
#define EM_AdjStart		0x30	// Measurement Calibration Start Command
#define EM_Ugain		0x31	// Voltage rms Gain
#define EM_IgainL		0x32	// L Line Current rms Gain
#define EM_IgainN		0x33	// N Line Current rms Gain
#define EM_Uoffset		0x34	// Voltage Offset
#define EM_IoffsetL		0x35	// L Line Current Offset
#define EM_IoffsetN		0x36	// N Line Current Offset
#define EM_PoffsetL		0x37	// L Line Active Power Offset
#define EM_QoffsetL		0x38	// L Line Reactive Power Offset
#define EM_PffsetN		0x39	// N Line Active Power Offset
#define EM_QoffsetN		0x3A	// N Line Reactive Power Offset
#define EM_CS2			0x3B	// Checksum 2

// Energy Register
#define EM_APenergy		0x40	// Forward Active Energy
#define EM_ANenergy		0x41	// Reverse Active Energy
#define EM_ATenergy		0x42	// Absolute Active Energy
#define EM_RPenergy		0x43	// Forward (Inductive) Reactive Energy
#define EM_RNenergy		0x44	// Reverse (Capacitive) Reactive Energy
#define EM_RTenergy		0x45	// Absolute Reactive Energy
#define EM_EnStatus		0x46	// Metering Status

// Measurement Register
#define EM_Irms			0x48	// L Line Current rms
#define EM_Urms			0x49	// Voltage rms
#define EM_Pmean		0x4A	// L Line Mean Active Power
#define EM_Qmean		0x4B	// L Line Mean Reactive Power
#define EM_Freq			0x4C	// Voltage Frequency
#define EM_PowerF		0x4D	// L Line Power Factor
#define EM_Pangle		0x4E	// Phase Angle between Voltage and L Line Current
#define EM_Smean		0x4F	// L Line Mean Apparent Power
#define EM_Irms2		0x68	// N Line Current rms
#define EM_Pmean2		0x6A	// N Line Mean Active Power
#define EM_Qmean2		0x6B	// N Line Mean Reactive Power
#define EM_PowerF2		0x6D	// N Line Power Factor
#define EM_Pangle2		0x6E	// Phase Angle between Voltage and N Line Current
#define EM_Smean2		0x6F	// N Line Mean Apparent Power

// Bit masks
//  SysStatus
#define EM_CalErr1		0x8000	// Indicates CS1 checksum status / 00: CS1 checksum correct (default)
#define EM_CalErr0		0x4000	//   11: CS1 checksum error. At the same time, WarnOut pin is asserted.
#define EM_AdjErr1		0x2000	// Indicate CS2 checksum status / 00: CS2 checksum correct (default)
#define EM_AdjErr0		0x1000	//   11: CS2 checksum error.
#define EM_LNchange		0x80	// Indicates whether there is any change of metering line (L/N) / 0: no change / 1: changed
#define EM_RevQchg		0x40	// Indicates change with direction of reactive energy / 0: no change / 1: changed (enabled with RevQEn bit of FuncEn)
#define EM_RevPchg		0x20	// Indicates change with direction of active energy / 0: no change / 1: changed (enabled with RevQEn bit of FuncEn)
#define EM_SagWarn		0x2		// Indicates voltage sag status / 0: no voltage sag / 1: voltage sag (enabled with SagEn bit of FuncEn; can also assert WarnOut when SagWo of FuncEn is enabled)
// Any of the above events will prompt the IRQ pin to be asserted as well.

// FuncEn
#define EM_SagEn		0x20	// Determines whether to enable the voltage sag interrupt / 0: disable (default) / 1: enable
#define EM_SagWo		0x10	// Voltage sag reported by WarnOut / 0: disable (default) / 1: enable
#define EM_RevQEn		0x8		// Enable direction change interrupt of reactive energy / 0: disable / 1: enable (default)
#define EM_RevPEn		0x4		// Enable direction change interrupt of active energy / 0: disable / 1: enable (default)

// MMode
#define EM_Lgain2		0x8000	// L line current gain, default value is 0b100
#define EM_Lgain1		0x4000	// 1xx: gain 1 | 000: gain 4 | 001: gain 8 | 010: gain 16 | 011: gain 24
#define EM_Lgain0		0x2000
#define EM_Ngain1		0x1000	// N line current gain
#define EM_Ngain0		0x800	// 00: 2 | 01: 4 | 10: 1 (default) | 11: 1
#define EM_LNSel		0x400	// Metering as L-line or N-line when using flexible-mode pins / 0: N line / 1: L line (default)
#define EM_DisHPF1		0x200	// Configure High Filter Pass (HPF) after ADC. This bit: 0 = enable HPF1 (default), 1 = disable HPF1
#define EM_DisHPF0		0x100	//   This bit: 0 = enable HPF0 (default), 1 = disable HPF0
#define EM_Amod			0x80	// CF1 output for active power: 0 = forward or reverse pulse output (default) / 1: absolute pulse output
#define EM_Rmod			0x40	// CF2 output for reactive power - same as above (for inductive/capacitive)
#define EM_ZXCon1		0x20	// Configure zero-crossing mode. ZX pin outputs 5ms-width high level when voltage crosses zero.
#define EM_ZXCon0		0x10	//   00: positive zero-crossing / 01: negative z-c / 10: all z-c (default) / 11: no z-c output
#define EM_Pthresh3		0x8		// Configure the L & N-line power difference in anti-tampering mode
#define EM_Pthresh2		0x4		//   0000 = 12.5% / 0001 = 6.25% / 0010 = 3.125% (default) / 0011 = 1.5625% / 0100 = 1% / 0101 = 2%
#define EM_Pthresh1		0x2		//   0110 = 3% / 0111 = 4% / 1000 = 5% / 1001 = 6% / 1010 = 7% / 1011 = 8% / 1100 = 9% / 1101 = 10%
#define EM_Pthresh0		0x1		//   1111 = 12%

// EnStatus
#define EM_Qnoload		0x8000	// Indicates whether the M90E26 is in reactive no-load status / 0: no / 1: yes
#define EM_Pnoload		0x4000	// Indicates whether the M90E26 is in active no-load status / 0: no / 1: yes
#define EM_RevQ			0x2000	// Indicates the direction of the last CF2 (reactive output) / 0: reactive forward / 1: reactive reverse
#define EM_RevP			0x1000	// Indicates the direction of the last CF1 (active output) / 0: active forward / 1: active reverse
#define EM_Lline		0x800	// Indicates the current metering line in anti-tampering mode / 0: N line / 1: L line
#define EM_LNMode1		0x2		// Indicates the configuration of MMD1 pin
#define EM_LNMode0		0x1		// Indicates the configuration of MMD0 pin

// Program status codes
#define EM_Valid		0x00	// http://knowyourmeme.com/memes/everything-went-better-than-expected
#define EM_Timeout		0x01	// didn't receive any data in time
#define EM_ChksumFail	0x02	// got a checksum byte, but... not the right one

class ATM90E26Debug : public Stream {
public:
    ATM90E26Debug();
    void begin(Stream *debugPrint);

    virtual size_t write(uint8_t byte);
    virtual int read() { return debug->read(); }
    virtual int available() { return debug->available(); }
    virtual void flush() { return debug->flush(); }
    virtual int peek() { return debug->peek(); }

    using Print::write;
private:
    Stream *debug;
};

class ATM90E26 {
	public:
		ATM90E26();
		
		boolean begin(Stream *serialptr, Stream *debugstream);
		unsigned int readNow(byte register);
		void readLater(byte register);
		float readVolts();
		float readCurrent();
                float readFrequency();
                float readPowerF();
                float readPowerF2();
		unsigned int readBuffer();
		void setBit(byte theRegister, unsigned int bitMask);
		void clearBit(byte theRegister, unsigned int bitMask);
		boolean write(byte theRegister, unsigned int theData);
		byte lastError();
		inline boolean isBuffered();
		inline boolean waitForData();
		boolean isValid();
		boolean debugOn;
		ATM90E26Debug debug;
		Stream *serial;
	private:
		byte errState;
		void purgeSerial();
};

#endif
