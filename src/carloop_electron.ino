#include <carloop.h>
#include <locator.h>
#include <Arduino.h>
#define ARDUINOJSON_ENABLE_PROGMEM 0
#include <ArduinoJson.h>
#include "obd2.h"
#include "helper.h"
#include "Serial4/Serial4.h"

#define FF_LOCATOR_ENABLED false
#define FF_DEBUG_PRINT false
#define FF_SNIFF_MODE false
#define SUPPORT_PID_REQUEST_PERIOD_SECONDS 30

// DeviceID: 4f002f000550483553353520

// Helper Functions
void debug_print(String msg) {
    if (FF_DEBUG_PRINT) Serial.println(msg);
}
void resetOBDSupportData();
void setReadSupportPIDsLoop(bool override = false);
int sendObdRequest();
void getObdResponse(int request_pid);
void storeMessageValue(byte data[8]);
void setPidEnabled(uint8_t pid);
void receiveSendPIDsLoop();
String dataToJsonStr();
void sniff_loop();

// System
SYSTEM_MODE(MANUAL);
SYSTEM_THREAD(ENABLED);

Locator locator;

// carloop
Carloop<CarloopRevision2> carloop;

const auto OBD_REQUEST_ID      = 0x7E0;
const auto OBD_REPLY_ID        = 0x7E8;
const auto OBD_PID_SERVICE     = 0x01;

// obd data
float alldata[PID_SIZE];
uint8_t *send_pids;
unsigned send_pid_size;
bool send_all_pids;
bool pid_enabled[PID_SIZE];
bool can_ready = false;

int current_gear = 0;

// algorithms
void doCustomAlgorithms() {
    //current_gear = getCurrentGear(alldata[VEHICLE_SPEED], alldata[ENGINE_RPM]);
    alldata[CUSTOM_CARLOOP_BATTERY_VOLTAGE] = carloop.battery();
}

void setup() {
    setLEDTheme(false);
    setCharging(false);
    pinMode(WKP, INPUT_PULLDOWN);
    Serial4.begin(230400);
    carloop.begin();
    resetOBDSupportData();
    if (FF_LOCATOR_ENABLED) {
        //if (Particle.connected) locator.publishLocation();
    }
    if (FF_SNIFF_MODE) {
        send_one_message();
    }
}

void loop() {

    if (FF_SNIFF_MODE) {
        sniff_loop();
        return;
    }

    receiveSendPIDsLoop();
    setReadSupportPIDsLoop();

    // carloop
    static auto loop_delay = millis();
    if (millis() - loop_delay > 100) {
        debug_print("Test all: " + String(send_all_pids));

        if (carloop.can().errorStatus() != CAN_NO_ERROR) {

            // Car isn't ready first time
            if (can_ready) setLEDTheme(false);
            can_ready = false;

        } else {

            // Car is ready first time
            if (!can_ready) {
                setLEDTheme(true);
                setReadSupportPIDsLoop(true);
            }
            can_ready = true;

            carloop.update();
            int currentPid = sendObdRequest();
            getObdResponse(currentPid);
            doCustomAlgorithms();
        }
        loop_delay = millis();
    }

    // print results through Serial4
    static auto print_delay = millis();
    if (millis() - print_delay > 1000) {
        String jsonString = dataToJsonStr();
        Serial4.println(jsonString);
        debug_print("Sending JSON: " + jsonString);
        Serial4.flush();
        print_delay = millis();
    }

    // Sleep when car off
    SystemSleepConfiguration locator_sleep;
    locator_sleep.mode(SystemSleepMode::HIBERNATE)
        .gpio(WKP, RISING)
        .duration(15min);

    SystemSleepConfiguration normal_sleep;
    normal_sleep.mode(SystemSleepMode::HIBERNATE)
        .gpio(WKP, RISING);

    // Sleep when pin XX is PULLED_DOWN (LOW)
    if (!digitalRead(WKP)) {
        while (!digitalRead(WKP)) {

            if (FF_LOCATOR_ENABLED) {
                System.sleep(locator_sleep);
                delay(1000);
                setCharging(false);
            } else {
                System.sleep(normal_sleep);
            }
        }
        System.reset();
    }
}

void resetOBDSupportData() {

    send_pids = NULL;
    send_all_pids = true;

    // init arrays
    for (unsigned i = 0; i < PID_SIZE; i++) {
      alldata[i] = EMPTY_VALUE;
      pid_enabled[i] = false;
    }

    setReadSupportPIDsLoop();
}

void setReadSupportPIDsLoop(bool override) {

    static auto wait = millis();
    if (millis() - wait < SUPPORT_PID_REQUEST_PERIOD_SECONDS && !override) return;

    // Enable Read Supported PIDs
    for (unsigned i=0; i<PID_SUPPORT_PIDS_SIZE; i++) {
        pid_enabled[PID_SUPPORT_PIDS[i]] = true;
    }

    wait = millis();
}

int sendObdRequest() {

    // This is not accurate
    static int currentPidIndex = 0;

    int request_pid = currentPidIndex;
    currentPidIndex = getNextPID(currentPidIndex, send_all_pids, send_pids, send_pid_size);

    // Only query PIDs that are supported
    while (!pid_enabled[currentPidIndex]) {
        currentPidIndex = getNextPID(currentPidIndex, send_all_pids, send_pids);
    }

    debug_print("Request PID: " + String(request_pid));

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

        if (millis() - delay_time > 50) {
            message_delayed = false;
        }

        if (message.id != OBD_REPLY_ID) continue;

        // If we find exactly what we are asking for
        if (message.data[2] == request_pid) {
            storeMessageValue(message.data);
            break;
        }

        // Not quite what we asked for but useful. Keep looking
        for (unsigned i=0; i<PID_SIZE; i++) {
            if (message.data[2] >= PID_SIZE) break;

            if (pid_enabled[message.data[2]]) {
                storeMessageValue(message.data);
                break;
            }
        }
    }
}

void storeMessageValue(byte data[8]) {

    uint8_t pid = data[2];

    uint8_t values[4] = {data[3], data[4], data[5], data[6]};
    alldata[pid] = getPidValue(pid, values);

    // Set PIDs to Query, turn off the flag
    
    for (unsigned i=0; i<PID_SUPPORT_PIDS_SIZE; i++) {
        if (pid >= PID_SIZE) continue;
        if (pid == PID_SUPPORT_PIDS[i]) {
            setPidEnabled(pid);
            pid_enabled[pid] = false;
            break;
        }
    }

    debug_print("Store PID " + String(pid) + " Value: " + alldata[pid]);
}

void setPidEnabled(uint8_t pid) {
    
    if (pid >= PID_SIZE) return;

    uint32_t mask = (uint32_t)alldata[pid];

    unsigned shift = 0;
    if (pid == PIDS_SUPPORT_01_20) shift = 0x00;
    if (pid == PIDS_SUPPORT_21_40) shift = 0x20;
    if (pid == PIDS_SUPPORT_41_60) shift = 0x40;

    
    for (int i=0; i<0x20; i++) {
        unsigned index = (0x20 - i + shift);
        if (index >= PID_SIZE) continue;
        pid_enabled[index] = (mask & (1<<i)) != 0;
    }
}

void receiveSendPIDsLoop() {

    DynamicJsonDocument json(4096);
    String str = "";
    while (Serial4.available()) {
        str = Serial4.readString();
    }

    if (str == "") return;

    debug_print("Received msg: " + str);

    DeserializationError err = deserializeJson(json, str);
    if (err != DeserializationError::Ok) {
        return;
    }

    if (json.isNull()) {
        return;
    }

    unsigned count = json["count"] | 0;
    if (count <= 0) {
        return;
    }

    if (send_pids != NULL) delete []send_pids;
    send_pids = NULL;

    int all = json["all"] | -1;
    debug_print("All: " + String(all));
    if (all) {
        send_pid_size = 0;
        send_all_pids = true;
        return;
    }

    if (all != 0) return;

    send_pids = new uint8_t[count];
    unsigned actual_count = count;

    for (unsigned i=0; i<count; i++) {
        uint8_t _send_pid = json["pids"][i] | 0xFF;
        if (_send_pid == 0xFF) {
            actual_count--;
            continue;
        }
        send_pids[i] = _send_pid;
        debug_print("Send PID index " + String(i) + " value: " + send_pids[i]);
    }

    send_pid_size = count;
    send_all_pids = false;

    // { "count": 2, "all": 0, "pids": [13, 14] }
    debug_print("Send PIDs count = " + String(count));
}

String dataToJsonStr() {
    DynamicJsonDocument json(4096);
    String output;

    json["cr"] = can_ready;
    json["a"] = unsigned(send_all_pids);

    if (!can_ready) {
        serializeJson(json, output);
        return output;
    }

    unsigned count = 0;

    if (send_all_pids) {
        for (unsigned i=0; i<PID_SIZE; i++) {
            if (alldata[i] == EMPTY_VALUE) continue;
            if (fToStr(alldata[i]) == EMPTY_STRING) continue;

            // { "car_ready": true, "count": 2, "metrics": [ { "pid": 12, "name": "eng", "value": "2504", "unit": "RPM" }, { "pid": 13, "name": "spd", "value": "105", "unit": "km/h" } ] }
            json["m"][count]["pid"] = i;
            json["m"][count]["n"] = getPidName(i);
            json["m"][count]["v"] = fToStr(alldata[i]);
            json["m"][count]["u"] = getPidUnits(i);

            count++;
        }
    } else {
        for (unsigned i=0; i<send_pid_size; i++) {

            //debug_print("Composing PID: " + send_pids[i]);

            if (alldata[send_pids[i]] == EMPTY_VALUE) continue;
            if (fToStr(alldata[send_pids[i]]) == EMPTY_STRING) continue;

            // { "car_ready": true, "count": 2, "metrics": [ { "pid": 12, "name": "eng", "value": "2504", "unit": "RPM" }, { "pid": 13, "name": "spd", "value": "105", "unit": "km/h" } ] }
            json["m"][count]["pid"] = send_pids[i];
            //json["metrics"][count]["n"] = getPidName(i);
            json["m"][count]["v"] = fToStr(alldata[send_pids[i]]);
            json["m"][count]["u"] = getPidUnits(send_pids[i]);

            count++;
        }
    }

    json["c"] = count;

    serializeJson(json, output);
    return output;
}

void sniff_loop() {
    auto time = millis();
    CANMessage message;
    while (carloop.can().receive(message) || millis() - time < 10000) {
        String str = "";
        for (unsigned i=0; i<8; i++) {
            if (message.data[i] < 0x10) str += "0";
            str += String(int(message.data[i]), HEX);
            str += " ";
        }
        Serial.println(str);
    }
}

void send_one_message() {
    CANMessage message;
    //fd ff 00 07 63 00 00 00 
    message.data[0] = 0xfd;
    message.data[1] = 0xff;
    message.data[2] = 0x00;
    message.data[3] = 0x07;
    message.data[4] = 0x63;
    message.data[5] = 0x00;
    message.data[6] = 0x00;
    message.data[7] = 0x00;
    message.len = 8;
    carloop.can().transmit(message);
}