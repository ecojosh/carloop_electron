#include "helper.h"
#include "obd2.h"

// Utilities
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

    if (newPid == 0x60) newPid = 0x00;

    // Other PIDs
    while (increment > 0) {
        newPid++;
        increment--;
        if (newPid == 0x60) newPid = 0x00;
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