#include "Particle.h"

#define PID_SIZE 0x60

bool isWithinNumberRange(float number, int A, int B);
String intToStr(int integer);
String fToStr(float f);
int getNextPID(int currentPid);
int getCurrentGear(float speed, float rpm);