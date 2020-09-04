#include "Particle.h"

#define EMPTY_VALUE -100.1
#define EMPTY_STRING fToStr(EMPTY_VALUE)

void setLEDTheme(bool ready);
void setCharging(bool enable);
bool isWithinNumberRange(float number, int A, int B);
String intToStr(int integer);
String fToStr(float f);
int basicGetNextPID(int currentPid);
int getNextPID(int currentPid, bool all, uint8_t *send_pids = NULL, unsigned send_pid_size = 0);
int getCurrentGear(float speed, float rpm);