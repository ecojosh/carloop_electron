#include "helper.h"
#include "obd2.h"

// Utilities
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
    String string = String(f,1);
    //sprintf(string, "%.01f", d);
    return string;
}

int getNextPID(int currentPid) {

    bool pids_enabled_pid_found = false;
    int increment = 1;
    int newPid = currentPid; 

    // PIDs Enabled PIDs first
    for (unsigned i=0; i<sizeof(PID_SUPPORT_PIDS)/sizeof(PID_SUPPORT_PIDS[0]); i++) {

        // Second Time Found
        if (pids_enabled_pid_found)
            return PID_SUPPORT_PIDS[i];

        // Skip PIDs Enabled PID
        if (currentPid + 1 == PID_SUPPORT_PIDS[i])
            increment++;

        // First Time Found
        if (currentPid == PID_SUPPORT_PIDS[i])
            pids_enabled_pid_found = true;
    }

    // First PID is 0x01 afer 0x00;
    if (pids_enabled_pid_found) return 0x01;

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