#include "helper.h"
#include "obd2.h"
#include <Arduino.h>
#define ARDUINOJSON_ENABLE_PROGMEM 0
#include <ArduinoJson.h>

// Utilities
void setLEDTheme() {
    LEDSystemTheme theme;
    theme.setColor(LED_SIGNAL_NETWORK_OFF, RGB_COLOR_GREEN);
    theme.setPattern(LED_SIGNAL_NETWORK_OFF, LED_PATTERN_SOLID);
    theme.apply();
}

void setCharging(bool enable) {

	PMIC pmic;

	// DisableCharging turns off charging. DisableBATFET completely disconnects the battery.
	if (enable) {
		pmic.enableCharging();
		pmic.enableBATFET();
	}
	else {
		pmic.disableCharging();
		pmic.disableBATFET();
	}

	// Disabling the watchdog is necessary, otherwise it will kick in and turn
	// charing at random times, even when sleeping.

	// This disables both the watchdog and the charge safety timer in
	// Charge Termination/Timer Control Register REG05
	// pmic.disableWatchdog() disables the watchdog, but doesn't disable the
	// charge safety timer, so the red LED will start blinking slowly after
	// 1 hour if you don't do both.
	byte DATA = pmic.readChargeTermRegister();

	if (enable) {
		DATA |= 0b00111000;
	}
	else {
		// 0b11001110 = disable watchdog
		// 0b11000110 = disable watchdog and charge safety timer
		DATA &= 0b11000110;
	}

	// This would be easier if pmic.writeRegister wasn't private (or disable
	// charge safety timer had an exposed method
	Wire3.beginTransmission(PMIC_ADDRESS);
	Wire3.write(CHARGE_TIMER_CONTROL_REGISTER);
	Wire3.write(DATA);
	Wire3.endTransmission(true);

}

bool isWithinNumberRange(float number, int A, int B) {
    if (number >= A && number <= B) {
        return true;
    }
    return false;
}

String intToStr(int integer) {
    String string = String(integer);
    //sprintf(string, "%d", integer);
    return string;
}

String fToStr(float f) {
    if (f == floor(f)) return intToStr(floor(f));

    String string = String(f,1);
    //sprintf(string, "%.01f", d);
    return string;
}

// Takes into account send_pids
int getNextPID(int currentPid, bool all, uint8_t *send_pids, unsigned send_pid_size) {

    static unsigned current_send_pid_index = 0;

    // All supported PIDs
    if (all || send_pids == NULL || send_pid_size == 0) {
        current_send_pid_index = 0;
        return basicGetNextPID(currentPid);
    }

    bool support_pid_found = false;

    // Support PIDs are always queried
    for (unsigned i=0; i<sizeof(PID_SUPPORT_PIDS)/sizeof(PID_SUPPORT_PIDS[0]); i++) {
        
        // Second Time Found (We just finished a support PID, so next Support PID is prioritized)
        if (support_pid_found)
            return PID_SUPPORT_PIDS[i];

        // First Time Found (Current PID is a Support PID)
        if (currentPid == PID_SUPPORT_PIDS[i])
            support_pid_found = true;

    }

    // Right after last Support PID is the first send_pid
    if (support_pid_found) {
        current_send_pid_index = 0;
        return send_pids[current_send_pid_index];
    }

    current_send_pid_index++;

    // Support PID after last send_pids index
    if (current_send_pid_index == send_pid_size) {
        current_send_pid_index = 0;
        return PID_SUPPORT_PIDS[0];
    }

    return send_pids[current_send_pid_index];
}

// Above func but all=true
int basicGetNextPID(int currentPid) {

    bool support_pid_found = false;
    int increment = 1;
    int newPid = currentPid;

    // PIDs Enabled PIDs first
    for (unsigned i=0; i<sizeof(PID_SUPPORT_PIDS)/sizeof(PID_SUPPORT_PIDS[0]); i++) {

        // Second Time Found (We just finished a support PID, so next Support PID is prioritized)
        if (support_pid_found)
            return PID_SUPPORT_PIDS[i];

        // Skip PIDs Enabled PID
        if (currentPid + 1 == PID_SUPPORT_PIDS[i])
            increment++;

        // First Time Found (Current PID is a Support PID)
        if (currentPid == PID_SUPPORT_PIDS[i])
            support_pid_found = true;
    }

    // First PID is 0x01 afer 0x00;
    if (support_pid_found) return 0x01;

    if (newPid == PID_SIZE) newPid = 0x00;

    // Other PIDs
    while (increment > 0) {
        newPid++;
        increment--;
        if (newPid == PID_SIZE) newPid = 0x00;
    }

    return newPid;
}

// Algorithms
int getCurrentGear(float speed, float rpm) {

    // first gear?
    if (speed < 5) {
        return 1;
    }

    float ratio = rpm / speed;

    // gear 6 = 20
    if (isWithinNumberRange(ratio, 17, 23)) {
        return 6;
    }

    // gear 5 = 27
    if (isWithinNumberRange(ratio, 24, 30)) {
        return 5;
    }

    // gear 4 = 36
    if (isWithinNumberRange(ratio, 31, 41)) {
        return 4;
    }

    // gear 3 = 48
    if (isWithinNumberRange(ratio, 42, 54)) {
        return 3;
    }

    // gear 2 = 70
    if (isWithinNumberRange(ratio, 60, 80)) {
        return 2;
    }

    // gear 4 = 112.7
    if (isWithinNumberRange(ratio, 90, 136)) {
        return 1;
    }

    return 255;
}