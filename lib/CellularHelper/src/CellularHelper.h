#ifndef __CELLULARHELPER_H
#define __CELLULARHELPER_H

#if Wiring_Cellular

#include "Particle.h"

// Class for quering infromation directly from the ublox SARA modem

/**
 * All response objects inherit from this, so the parse() method can be called
 * in the subclass, and also the resp and enableDebug members are always available.
 */
class CellularHelperCommonResponse {
public:
	int resp = RESP_ERROR;
	bool enableDebug = false;

	virtual int parse(int type, const char *buf, int len) = 0;

	void logCellularDebug(int type, const char *buf, int len) const;
};

/**
 * Things that return a simple string, like the manufacturer string, use this
 *
 * Since it inherits from CellularHelperPlusStringResponse and CellularHelperCommonResponse you
 * can check resp == RESP_OK to make sure the call succeeded.
 */
class CellularHelperStringResponse : public CellularHelperCommonResponse {
public:
	String string;

	virtual int parse(int type, const char *buf, int len);
};

/**
 * Things that return a + response and a string use this.
 *
 * Since it inherits from CellularHelperPlusStringResponse and CellularHelperCommonResponse you
 * can check resp == RESP_OK to make sure the call succeeded.
 */
class CellularHelperPlusStringResponse : public CellularHelperCommonResponse {
public:
	String command;
	String string;

	virtual int parse(int type, const char *buf, int len);
	String getDoubleQuotedPart(bool onlyFirst = true) const;
};

/**
 * This class is used to return the rssi and qual values.
 *
 * Note that for 2G, qual is not available and 99 is always returned.
 *
 * Since it inherits from CellularHelperPlusStringResponse and CellularHelperCommonResponse you
 * can check resp == RESP_OK to make sure the call succeeded.
 */
class CellularHelperRSSIQualResponse : public CellularHelperPlusStringResponse {
public:
	int rssi = 0;
	int qual = 0;

	void postProcess();
};

/**
 * Used to hold the results for one cell (service or neighbor) from the AT+CGED command
 */
class CellularHelperEnvironmentCellData { // 44 bytes
public:
	int mcc = 65535; 		// Mobile Country Code, range 0 - 999 (3 digits). Other values are to be considered invalid / not available
	int mnc = 255; 			// Mobile Network Code, range 0 - 999 (1 to 3 digits). Other values are to be considered invalid / not available
	int lac; 				// Location Area Code, range 0h-FFFFh (2 octets)
	int ci; 				// Cell Identity: 2G cell: range 0h-FFFFh (2 octets); 3G cell: range 0h-FFFFFFFh (28 bits)
	int bsic; 				// Base Station Identify Code, range 0h-3Fh (6 bits) [2G]
	int arfcn; 				// Absolute Radio Frequency Channel Number, range 0 - 1023 [2G]
	// The parameter value also decodes the band indicator bit (DCS or PCS) by means of the
	// most significant byte (8 means 1900 band) (i.e. if the parameter reports the value 33485, it corresponds to 0x82CD, in the most significant byte there is the band indicator bit, so the <arfcn> is 0x2CD (717) and belongs to 1900 band).
	int rxlev;				// Received signal level on the cell, range 0 - 63; see the 3GPP TS 05.08 [2G]
	bool isUMTS = false;	// RAT is GSM (false) or UMTS (true)
	int dlf;				// Downlink frequency. Range 0 - 16383 [3G]
	int ulf;				// Uplink frequency. Range 0 - 16383 [3G]
	int rscpLev = 255;		// Received signal level [3G]

	bool isValid(bool ignoreCI = false) const;
	void parse(const char *str);
	void addKeyValue(const char *key, const char *value);
	String toString() const;

	// Calculated
	int getBand() const;
	String getBandString() const;
	int getRSSI() const;
	int getBars() const;
};

/**
 * Used to hold the results from the AT+CGED command
 */
class CellularHelperEnvironmentResponse : public CellularHelperPlusStringResponse {
public:
	CellularHelperEnvironmentResponse(CellularHelperEnvironmentCellData *neighbors, size_t numNeighbors);

	CellularHelperEnvironmentCellData service;
	CellularHelperEnvironmentCellData *neighbors;
	size_t numNeighbors;
	int curDataIndex = -1;

	virtual int parse(int type, const char *buf, int len);
	void clear();
	void postProcess();
	void logResponse() const;
	size_t getNumNeighbors() const;
};

template <size_t MAX_NEIGHBOR_CELLS>
class CellularHelperEnvironmentResponseStatic : public CellularHelperEnvironmentResponse {
public:
	explicit CellularHelperEnvironmentResponseStatic() : CellularHelperEnvironmentResponse(staticNeighbors, MAX_NEIGHBOR_CELLS) {
	}

protected:
	CellularHelperEnvironmentCellData staticNeighbors[MAX_NEIGHBOR_CELLS];
};


class CellularHelperLocationResponse : public CellularHelperPlusStringResponse {
public:
	bool valid = false;
	float lat = 0.0;
	float lon = 0.0;
	int alt = 0;
	int uncertainty = 0;

	void postProcess();
	String toString() const;
};


/**
 * Class for calling the u-blox SARA modem directly
 */
class CellularHelperClass {
public:
	String getManufacturer() const;

	String getModel() const;

	String getOrderingCode() const;

	String getFirmwareVersion() const;

	String getIMEI() const;

	String getIMSI() const;

	String getICCID() const;

	String getOperatorName(int operatorNameType = OPERATOR_NAME_LONG_EONS) const;


	/**
	 * Get the RSSI and qual values for the receiving cell site.
	 *
	 * The qual value is always 99 for me on the G350 (2G).
	 */
	CellularHelperRSSIQualResponse getRSSIQual() const;


	void getEnvironment(int mode, CellularHelperEnvironmentResponse &resp) const;


	CellularHelperLocationResponse getLocation(unsigned long timeoutMs = DEFAULT_TIMEOUT) const;

	/**
	 * Pings an address. Typically a dotted octet string, but can pass a hostname as well.
	 *
	 * If you have a Particle IPAddress, you can use ipaddr.toString() to get the dotted octet string
	 * that this method wants
	 */
	bool ping(const char *addr) const;

	/**
	 * Looks up the IPAddress of a hostname
	 */
	IPAddress dnsLookup(const char *hostname) const;

	void appendBufferToString(String &str, const char *buf, int len, bool noEOL = true) const;

	// Default timeout in milliseconds
	static const system_tick_t DEFAULT_TIMEOUT = 10000;

	static const int ENVIRONMENT_SERVING_CELL = 3;
	static const int ENVIRONMENT_SERVING_CELL_AND_NEIGHBORS = 5;

	/**
	 * Constants for getOperatorName. Default and recommended value is OPERATOR_NAME_LONG_EONS.
	 */
	static const int OPERATOR_NAME_NUMERIC = 0;
	static const int OPERATOR_NAME_SHORT_ROM = 1;
	static const int OPERATOR_NAME_LONG_ROM = 2;
	static const int OPERATOR_NAME_SHORT_CPHS = 3;
	static const int OPERATOR_NAME_LONG_CPHS = 4;
	static const int OPERATOR_NAME_SHORT_NITZ = 5;
	static const int OPERATOR_NAME_LONG_NITZ = 6;
	static const int OPERATOR_NAME_SERVICE_PROVIDER = 7;
	static const int OPERATOR_NAME_SHORT_EONS = 8;
	static const int OPERATOR_NAME_LONG_EONS = 9;
	static const int OPERATOR_NAME_SHORT_NETWORK_OPERATOR = 11;
	static const int OPERATOR_NAME_LONG_NETWORK_OPERATOR = 12;


	static int responseCallback(int type, const char* buf, int len, void *param);

	static int rssiToBars(int rssi);


};

extern CellularHelperClass CellularHelper;

#endif /* Wiring_Cellular */

#endif /* __CELLULARHELPER_H */