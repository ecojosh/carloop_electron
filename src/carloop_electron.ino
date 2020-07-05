#include <carloop.h>
#include "obd2.h"
#include "helper.h"

// Helper Functions
int sendObdRequest();
void getObdResponse(int request_pid);
void storeMessageValue(byte data[]);
void setPidEnabled(bool (&pid_array)[PID_SIZE]);
// DeviceID: 4f002f000550483553353520

// System
SYSTEM_MODE(MANUAL);
SYSTEM_THREAD(ENABLED);

// carloop
Carloop<CarloopRevision2> carloop;

const auto OBD_REQUEST_ID      = 0x7E0;
const auto OBD_REPLY_ID        = 0x7E8;
const auto OBD_PID_SERVICE     = 0x01;

// obd data
float alldata[PID_SIZE] = {-1};
bool pid_enabled[PID_SIZE] = {0};

int current_gear = 0;

// algorithms
void doAlgorithms() {
    current_gear = getCurrentGear(alldata[VEHICLE_SPEED], alldata[ENGINE_RPM]);
}

void setup() {

    Serial.begin(9600);

    carloop.begin();

    // init arrays
    for (unsigned i = 0; i < PID_SIZE; i++) {
      alldata[i] = -1;
      pid_enabled[i] = 0;
    }

    // Enable Read Supported PIDs
    for (unsigned i=0; i<sizeof(PID_SUPPORT_PIDS)/sizeof(PID_SUPPORT_PIDS[0]); i++) {
        pid_enabled[PID_SUPPORT_PIDS[i]] = 1;
    }
}

void loop() {

    static auto loop_delay = millis();
    static auto print_delay = millis();

    // carloop
    if (millis() - loop_delay > 250) {
        carloop.update();
        int currentPid = sendObdRequest();
        getObdResponse(currentPid);
        doAlgorithms();
        loop_delay = millis();
    }

    // print results
    if (millis() - print_delay > 30000) {
        Serial.println("Printing CAN Bus PIDs:");

        String content = "";
        for (unsigned i=0; i < PID_SIZE; i++) {
            if (alldata[i] == -1) continue;
            content += getPidName(i) + ": " + fToStr(alldata[i]) + " " + getPidUnits(i) + "\n";
        }

        Serial.print(content);
        print_delay = millis();
    }
}

// OBD Functions
int sendObdRequest() {

    static int currentPidIndex = 0;

    int request_pid = currentPidIndex;
    currentPidIndex = getNextPID(currentPidIndex);
    // Only query PIDs that are supported
    while (!pid_enabled[currentPidIndex]) {
        currentPidIndex = getNextPID(currentPidIndex);
    }

    CANMessage message;
    message.id = OBD_REQUEST_ID;
    message.len = 8;
    message.data[0] = 0x02;
    message.data[1] = OBD_PID_SERVICE;
    message.data[2] = request_pid;
    carloop.can().transmit(message);

    return request_pid;
}

void getObdResponse(int request_pid) {
    CANMessage message;

    auto delay_time = millis();
    bool message_delayed = true;
    while (carloop.can().receive(message) || message_delayed) {
        //Serial.println("RECEIVED");
        if(message.id == OBD_REPLY_ID && message.data[2] == request_pid) {
            storeMessageValue(message.data);
        }

        if (millis() - delay_time > 100) {
            message_delayed = false;
        }
    }
}

void storeMessageValue(byte data[]) {

    byte pid = data[2];

    uint8_t values[4] = {data[3], data[4], data[5], data[6]};
    alldata[pid] = getPidValue(pid, values);

    // Set PIDs to Query, turn off the flag
    for (unsigned i=0; i<sizeof(PID_SUPPORT_PIDS)/sizeof(PID_SUPPORT_PIDS[0]); i++) {
        if (pid == PID_SUPPORT_PIDS[i]) {
            setPidEnabled(pid);
            pid_enabled[pid] = 0;
            break;
        }
    }
}

void setPidEnabled(int pid) {

    uint32_t mask = (uint32_t)alldata[pid];

    unsigned shift = 0;
    if (pid == PIDS_SUPPORT_01_20) shift = 0x00;
    if (pid == PIDS_SUPPORT_21_40) shift = 0x20;
    if (pid == PIDS_SUPPORT_41_60) shift = 0x40;

    for (int i=0; i<0x20; i++) {
        pid_enabled[(0x20 - i + shift)] = (mask & (1<<i)) != 0;
    }

}