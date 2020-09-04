#include "Particle.h"
#include <math.h>

#ifdef __AVR__
#include <avr/pgmspace.h>
#else
#define PROGMEM
#endif

#define PID_SIZE 0x61

String getPidName(uint8_t pid);
String getPidUnits(uint8_t pid);
float getPidValue(uint8_t pid, uint8_t value[4]);

// Sourced from https://github.com/sandeepmistry/arduino-OBD2/blob/master/src/OBD2.h

enum {
  PIDS_SUPPORT_01_20                                = 0x00,
  MONITOR_STATUS_SINCE_DTCS_CLEARED                 = 0x01,
  FREEZE_DTC                                        = 0x02,
  FUEL_SYSTEM_STATUS                                = 0x03,
  CALCULATED_ENGINE_LOAD                            = 0x04,
  ENGINE_COOLANT_TEMPERATURE                        = 0x05,
  SHORT_TERM_FUEL_TRIM_BANK_1                       = 0x06,
  LONG_TERM_FUEL_TRIM_BANK_1                        = 0x07,
  SHORT_TERM_FUEL_TRIM_BANK_2                       = 0x08,
  LONG_TERM_FUEL_TRIM_BANK_2                        = 0x09,
  FUEL_PRESSURE                                     = 0x0a,
  INTAKE_MANIFOLD_ABSOLUTE_PRESSURE                 = 0x0b,
  ENGINE_RPM                                        = 0x0c,
  VEHICLE_SPEED                                     = 0x0d,
  TIMING_ADVANCE                                    = 0x0e,
  AIR_INTAKE_TEMPERATURE                            = 0x0f,
  MAF_AIR_FLOW_RATE                                 = 0x10,
  THROTTLE_POSITION                                 = 0x11,
  COMMANDED_SECONDARY_AIR_STATUS                    = 0x12,
  OXYGEN_SENSORS_PRESENT_IN_2_BANKS                 = 0x13,
  OXYGEN_SENSOR_1_SHORT_TERM_FUEL_TRIM              = 0x14,
  OXYGEN_SENSOR_2_SHORT_TERM_FUEL_TRIM              = 0x15,
  OXYGEN_SENSOR_3_SHORT_TERM_FUEL_TRIM              = 0x16,
  OXYGEN_SENSOR_4_SHORT_TERM_FUEL_TRIM              = 0x17,
  OXYGEN_SENSOR_5_SHORT_TERM_FUEL_TRIM              = 0x18,
  OXYGEN_SENSOR_6_SHORT_TERM_FUEL_TRIM              = 0x19,
  OXYGEN_SENSOR_7_SHORT_TERM_FUEL_TRIM              = 0x1a,
  OXYGEN_SENSOR_8_SHORT_TERM_FUEL_TRIM              = 0x1b,
  OBD_STANDARDS_THIS_VEHICLE_CONFORMS_TO            = 0x1c,
  OXYGEN_SENSORS_PRESENT_IN_4_BANKS                 = 0x1d,
  AUXILIARY_INPUT_STATUS                            = 0x1e,
  RUN_TIME_SINCE_ENGINE_START                       = 0x1f,

  PIDS_SUPPORT_21_40                                = 0x20,
  DISTANCE_TRAVELED_WITH_MIL_ON                     = 0x21,
  FUEL_RAIL_PRESSURE                                = 0x22,
  FUEL_RAIL_GAUGE_PRESSURE                          = 0x23,
  OXYGEN_SENSOR_1_FUEL_AIR_EQUIVALENCE_RATIO        = 0x24,
  OXYGEN_SENSOR_2_FUEL_AIR_EQUIVALENCE_RATIO        = 0x25,
  OXYGEN_SENSOR_3_FUEL_AIR_EQUIVALENCE_RATIO        = 0x26,
  OXYGEN_SENSOR_4_FUEL_AIR_EQUIVALENCE_RATIO        = 0x27,
  OXYGEN_SENSOR_5_FUEL_AIR_EQUIVALENCE_RATIO        = 0x28,
  OXYGEN_SENSOR_6_FUEL_AIR_EQUIVALENCE_RATIO        = 0x29,
  OXYGEN_SENSOR_7_FUEL_AIR_EQUIVALENCE_RATIO        = 0x2a,
  OXYGEN_SENSOR_8_FUEL_AIR_EQUIVALENCE_RATIO        = 0x2b,
  COMMANDED_EGR                                     = 0x2c,
  EGR_ERROR                                         = 0x2d,
  COMMANDED_EVAPORATIVE_PURGE                       = 0x2e,
  FUEL_TANK_LEVEL_INPUT                             = 0x2f,
  WARM_UPS_SINCE_CODES_CLEARED                      = 0x30,
  DISTANCE_TRAVELED_SINCE_CODES_CLEARED             = 0x31,
  EVAP_SYSTEM_VAPOR_PRESSURE                        = 0x32,
  ABSOLULTE_BAROMETRIC_PRESSURE                     = 0x33,
/*OXYGEN_SENSOR_1_FUEL_AIR_EQUIVALENCE_RATIO        = 0x34,
  OXYGEN_SENSOR_2_FUEL_AIR_EQUIVALENCE_RATIO        = 0x35,
  OXYGEN_SENSOR_3_FUEL_AIR_EQUIVALENCE_RATIO        = 0x36,
  OXYGEN_SENSOR_4_FUEL_AIR_EQUIVALENCE_RATIO        = 0x37,
  OXYGEN_SENSOR_5_FUEL_AIR_EQUIVALENCE_RATIO        = 0x38,
  OXYGEN_SENSOR_6_FUEL_AIR_EQUIVALENCE_RATIO        = 0x39,
  OXYGEN_SENSOR_7_FUEL_AIR_EQUIVALENCE_RATIO        = 0x3a,
  OXYGEN_SENSOR_8_FUEL_AIR_EQUIVALENCE_RATIO        = 0x3b,*/
  CATALYST_TEMPERATURE_BANK_1_SENSOR_1              = 0x3c,
  CATALYST_TEMPERATURE_BANK_2_SENSOR_1              = 0x3d,
  CATALYST_TEMPERATURE_BANK_1_SENSOR_2              = 0x3e,
  CATALYST_TEMPERATURE_BANK_2_SENSOR_2              = 0x3f,

  PIDS_SUPPORT_41_60                                = 0x40,
  MONITOR_STATUS_THIS_DRIVE_CYCLE                   = 0x41,
  CONTROL_MODULE_VOLTAGE                            = 0x42,
  ABSOLUTE_LOAD_VALUE                               = 0x43,
  FUEL_AIR_COMMANDED_EQUIVALENCE_RATE               = 0x44,
  RELATIVE_THROTTLE_POSITION                        = 0x45,
  AMBIENT_AIR_TEMPERATURE                           = 0x46,
  ABSOLUTE_THROTTLE_POSITION_B                      = 0x47,
  ABSOLUTE_THROTTLE_POSITION_C                      = 0x48,
  ABSOLUTE_THROTTLE_POSITION_D                      = 0x49,
  ABSOLUTE_THROTTLE_POSITION_E                      = 0x4a,
  ABSOLUTE_THROTTLE_POSITION_F                      = 0x4b,
  COMMANDED_THROTTLE_ACTUATOR                       = 0x4c,
  TIME_RUN_WITH_MIL_ON                              = 0x4d,
  TIME_SINCE_TROUBLE_CODES_CLEARED                  = 0x4e,
/*                                                  = 0x4f,
                                                    = 0x50,*/
  FUEL_TYPE                                         = 0x51,
  ETHANOL_FUEL_PERCENTAGE                           = 0x52,
  ABSOLUTE_EVAP_SYSTEM_VAPOR_PRESSURE               = 0x53,
/*EVAP_SYSTEM_VAPOR_PRESSURE                        = 0x54,*/
/*                                                  = 0x55,
                                                    = 0x56,
                                                    = 0x57,
                                                    = 0x58,*/
  FUEL_RAIL_ABSOLUTE_PRESSURE                       = 0x59,
  RELATIVE_ACCELERATOR_PEDAL_POSITTION              = 0x5a,
  HYBRID_BATTERY_PACK_REMAINING_LIFE                = 0x5b,
  ENGINE_OIL_TEMPERATURE                            = 0x5c,
  FUEL_INJECTION_TIMING                             = 0x5d,
  ENGINE_FUEL_RATE                                  = 0x5e,
  EMISSION_REQUIREMENT_TO_WHICH_VEHICLE_IS_DESIGNED = 0x5f,

  CUSTOM_CARLOOP_BATTERY_VOLTAGE                    = 0x60,

  // more PIDs can be added from: https://en.wikipedia.org/wiki/OBD-II_PIDs
};

const int PID_SUPPORT_PIDS[] = {PIDS_SUPPORT_01_20, PIDS_SUPPORT_21_40, PIDS_SUPPORT_41_60};
const unsigned PID_SUPPORT_PIDS_SIZE = 3;

// Sourced from https://github.com/sandeepmistry/arduino-OBD2/blob/master/src/OBD2.cpp

const char PID_NAME_0x00[] PROGMEM = "PIDs supported [01 - 20]";
const char PID_NAME_0x01[] PROGMEM = "Monitor status since DTCs cleared";
const char PID_NAME_0x02[] PROGMEM = "Freeze DTC";
const char PID_NAME_0x03[] PROGMEM = "Fuel system status";
const char PID_NAME_0x04[] PROGMEM = "Calculated engine load";
const char PID_NAME_0x05[] PROGMEM = "Engine coolant temperature";
const char PID_NAME_0x06[] PROGMEM = "Short term fuel trim — Bank 1";
const char PID_NAME_0x07[] PROGMEM = "Long term fuel trim — Bank 1";
const char PID_NAME_0x08[] PROGMEM = "Short term fuel trim — Bank 2";
const char PID_NAME_0x09[] PROGMEM = "Long term fuel trim — Bank 2";
const char PID_NAME_0x0a[] PROGMEM = "Fuel pressure";
const char PID_NAME_0x0b[] PROGMEM = "Intake manifold absolute pressure";
const char PID_NAME_0x0c[] PROGMEM = "Engine RPM";
const char PID_NAME_0x0d[] PROGMEM = "Vehicle speed";
const char PID_NAME_0x0e[] PROGMEM = "Timing advance";
const char PID_NAME_0x0f[] PROGMEM = "Intake air temperature";
const char PID_NAME_0x10[] PROGMEM = "MAF air flow rate";
const char PID_NAME_0x11[] PROGMEM = "Throttle position";
const char PID_NAME_0x12[] PROGMEM = "Commanded secondary air status";
const char PID_NAME_0x13[] PROGMEM = "Oxygen sensors present (in 2 banks)";
const char PID_NAME_0x14[] PROGMEM = "Oxygen Sensor 1 - Short term fuel trim";
const char PID_NAME_0x15[] PROGMEM = "Oxygen Sensor 2 - Short term fuel trim";
const char PID_NAME_0x16[] PROGMEM = "Oxygen Sensor 3 - Short term fuel trim";
const char PID_NAME_0x17[] PROGMEM = "Oxygen Sensor 4 - Short term fuel trim";
const char PID_NAME_0x18[] PROGMEM = "Oxygen Sensor 5 - Short term fuel trim";
const char PID_NAME_0x19[] PROGMEM = "Oxygen Sensor 6 - Short term fuel trim";
const char PID_NAME_0x1a[] PROGMEM = "Oxygen Sensor 7 - Short term fuel trim";
const char PID_NAME_0x1b[] PROGMEM = "Oxygen Sensor 8 - Short term fuel trim";
const char PID_NAME_0x1c[] PROGMEM = "OBD standards this vehicle conforms to";
const char PID_NAME_0x1d[] PROGMEM = "Oxygen sensors present (in 4 banks)";
const char PID_NAME_0x1e[] PROGMEM = "Auxiliary input status";
const char PID_NAME_0x1f[] PROGMEM = "Run time since engine start";
const char PID_NAME_0x20[] PROGMEM = "PIDs supported [21 - 40]";
const char PID_NAME_0x21[] PROGMEM = "Distance traveled with malfunction indicator lamp (MIL) on";
const char PID_NAME_0x22[] PROGMEM = "Fuel Rail Pressure (relative to manifold vacuum)";
const char PID_NAME_0x23[] PROGMEM = "Fuel Rail Gauge Pressure (diesel, or gasoline direct injection)";
const char PID_NAME_0x24[] PROGMEM = "Oxygen Sensor 1 - Fuel–Air Equivalence Ratio";
const char PID_NAME_0x25[] PROGMEM = "Oxygen Sensor 2 - Fuel–Air Equivalence Ratio";
const char PID_NAME_0x26[] PROGMEM = "Oxygen Sensor 3 - Fuel–Air Equivalence Ratio";
const char PID_NAME_0x27[] PROGMEM = "Oxygen Sensor 4 - Fuel–Air Equivalence Ratio";
const char PID_NAME_0x28[] PROGMEM = "Oxygen Sensor 5 - Fuel–Air Equivalence Ratio";
const char PID_NAME_0x29[] PROGMEM = "Oxygen Sensor 6 - Fuel–Air Equivalence Ratio";
const char PID_NAME_0x2a[] PROGMEM = "Oxygen Sensor 7 - Fuel–Air Equivalence Ratio";
const char PID_NAME_0x2b[] PROGMEM = "Oxygen Sensor 8 - Fuel–Air Equivalence Ratio";
const char PID_NAME_0x2c[] PROGMEM = "Commanded EGR";
const char PID_NAME_0x2d[] PROGMEM = "EGR Error";
const char PID_NAME_0x2e[] PROGMEM = "Commanded evaporative purge";
const char PID_NAME_0x2f[] PROGMEM = "Fuel Tank Level Input";
const char PID_NAME_0x30[] PROGMEM = "Warm-ups since codes cleared";
const char PID_NAME_0x31[] PROGMEM = "Distance traveled since codes cleared";
const char PID_NAME_0x32[] PROGMEM = "Evap. System Vapor Pressure";
const char PID_NAME_0x33[] PROGMEM = "Absolute Barometric Pressure";
const char PID_NAME_0x34[] PROGMEM = "Oxygen Sensor 1 - Fuel–Air Equivalence Ratio";
const char PID_NAME_0x35[] PROGMEM = "Oxygen Sensor 2 - Fuel–Air Equivalence Ratio";
const char PID_NAME_0x36[] PROGMEM = "Oxygen Sensor 3 - Fuel–Air Equivalence Ratio";
const char PID_NAME_0x37[] PROGMEM = "Oxygen Sensor 4 - Fuel–Air Equivalence Ratio";
const char PID_NAME_0x38[] PROGMEM = "Oxygen Sensor 5 - Fuel–Air Equivalence Ratio";
const char PID_NAME_0x39[] PROGMEM = "Oxygen Sensor 6 - Fuel–Air Equivalence Ratio";
const char PID_NAME_0x3a[] PROGMEM = "Oxygen Sensor 7 - Fuel–Air Equivalence Ratio";
const char PID_NAME_0x3b[] PROGMEM = "Oxygen Sensor 8 - Fuel–Air Equivalence Ratio";
const char PID_NAME_0x3c[] PROGMEM = "Catalyst Temperature: Bank 1, Sensor 1";
const char PID_NAME_0x3d[] PROGMEM = "Catalyst Temperature: Bank 2, Sensor 1";
const char PID_NAME_0x3e[] PROGMEM = "Catalyst Temperature: Bank 1, Sensor 2";
const char PID_NAME_0x3f[] PROGMEM = "Catalyst Temperature: Bank 2, Sensor 2";
const char PID_NAME_0x40[] PROGMEM = "PIDs supported [41 - 60]";
const char PID_NAME_0x41[] PROGMEM = "Monitor status this drive cycle";
const char PID_NAME_0x42[] PROGMEM = "Control module voltage";
const char PID_NAME_0x43[] PROGMEM = "Absolute load value";
const char PID_NAME_0x44[] PROGMEM = "Fuel–Air commanded equivalence ratio";
const char PID_NAME_0x45[] PROGMEM = "Relative throttle position";
const char PID_NAME_0x46[] PROGMEM = "Ambient air temperature";
const char PID_NAME_0x47[] PROGMEM = "Absolute throttle position B";
const char PID_NAME_0x48[] PROGMEM = "Absolute throttle position C";
const char PID_NAME_0x49[] PROGMEM = "Absolute throttle position D";
const char PID_NAME_0x4a[] PROGMEM = "Absolute throttle position E";
const char PID_NAME_0x4b[] PROGMEM = "Absolute throttle position F";
const char PID_NAME_0x4c[] PROGMEM = "Commanded throttle actuator";
const char PID_NAME_0x4d[] PROGMEM = "Time run with MIL on";
const char PID_NAME_0x4e[] PROGMEM = "Time since trouble codes cleared";
const char PID_NAME_0x4f[] PROGMEM = "Maximum value for Fuel–Air equivalence ratio, oxygen sensor voltage, oxygen sensor current, and intake manifold absolute pressure";
const char PID_NAME_0x50[] PROGMEM = "Maximum value for air flow rate from mass air flow sensor";
const char PID_NAME_0x51[] PROGMEM = "Fuel Type";
const char PID_NAME_0x52[] PROGMEM = "Ethanol fuel percentage";
const char PID_NAME_0x53[] PROGMEM = "Absolute Evap system Vapor Pressure";
const char PID_NAME_0x54[] PROGMEM = "Evap system vapor pressure";
const char PID_NAME_0x55[] PROGMEM = "Short term secondary oxygen sensor trim";
const char PID_NAME_0x56[] PROGMEM = "Long term secondary oxygen sensor trim";
const char PID_NAME_0x57[] PROGMEM = "Short term secondary oxygen sensor trim";
const char PID_NAME_0x58[] PROGMEM = "Long term secondary oxygen sensor trim";
const char PID_NAME_0x59[] PROGMEM = "Fuel rail absolute pressure";
const char PID_NAME_0x5a[] PROGMEM = "Relative accelerator pedal position";
const char PID_NAME_0x5b[] PROGMEM = "Hybrid battery pack remaining life";
const char PID_NAME_0x5c[] PROGMEM = "Engine oil temperature";
const char PID_NAME_0x5d[] PROGMEM = "Fuel injection timing";
const char PID_NAME_0x5e[] PROGMEM = "Engine fuel rate";
const char PID_NAME_0x5f[] PROGMEM = "Emission requirements to which vehicle is designed";
const char PID_NAME_0x60[] PROGMEM = "Carloop Battery Voltage";

const char* const PID_NAME_MAPPER[] PROGMEM = {
  PID_NAME_0x00,
  PID_NAME_0x01,
  PID_NAME_0x02,
  PID_NAME_0x03,
  PID_NAME_0x04,
  PID_NAME_0x05,
  PID_NAME_0x06,
  PID_NAME_0x07,
  PID_NAME_0x08,
  PID_NAME_0x09,
  PID_NAME_0x0a,
  PID_NAME_0x0b,
  PID_NAME_0x0c,
  PID_NAME_0x0d,
  PID_NAME_0x0e,
  PID_NAME_0x0f,
  PID_NAME_0x10,
  PID_NAME_0x11,
  PID_NAME_0x12,
  PID_NAME_0x13,
  PID_NAME_0x14,
  PID_NAME_0x15,
  PID_NAME_0x16,
  PID_NAME_0x17,
  PID_NAME_0x18,
  PID_NAME_0x19,
  PID_NAME_0x1a,
  PID_NAME_0x1b,
  PID_NAME_0x1c,
  PID_NAME_0x1d,
  PID_NAME_0x1e,
  PID_NAME_0x1f,
  PID_NAME_0x20,
  PID_NAME_0x21,
  PID_NAME_0x22,
  PID_NAME_0x23,
  PID_NAME_0x24,
  PID_NAME_0x25,
  PID_NAME_0x26,
  PID_NAME_0x27,
  PID_NAME_0x28,
  PID_NAME_0x29,
  PID_NAME_0x2a,
  PID_NAME_0x2b,
  PID_NAME_0x2c,
  PID_NAME_0x2d,
  PID_NAME_0x2e,
  PID_NAME_0x2f,
  PID_NAME_0x30,
  PID_NAME_0x31,
  PID_NAME_0x32,
  PID_NAME_0x33,
  PID_NAME_0x34,
  PID_NAME_0x35,
  PID_NAME_0x36,
  PID_NAME_0x37,
  PID_NAME_0x38,
  PID_NAME_0x39,
  PID_NAME_0x3a,
  PID_NAME_0x3b,
  PID_NAME_0x3c,
  PID_NAME_0x3d,
  PID_NAME_0x3e,
  PID_NAME_0x3f,
  PID_NAME_0x40,
  PID_NAME_0x41,
  PID_NAME_0x42,
  PID_NAME_0x43,
  PID_NAME_0x44,
  PID_NAME_0x45,
  PID_NAME_0x46,
  PID_NAME_0x47,
  PID_NAME_0x48,
  PID_NAME_0x49,
  PID_NAME_0x4a,
  PID_NAME_0x4b,
  PID_NAME_0x4c,
  PID_NAME_0x4d,
  PID_NAME_0x4e,
  PID_NAME_0x4f,
  PID_NAME_0x50,
  PID_NAME_0x51,
  PID_NAME_0x52,
  PID_NAME_0x53,
  PID_NAME_0x54,
  PID_NAME_0x55,
  PID_NAME_0x56,
  PID_NAME_0x57,
  PID_NAME_0x58,
  PID_NAME_0x59,
  PID_NAME_0x5a,
  PID_NAME_0x5b,
  PID_NAME_0x5c,
  PID_NAME_0x5d,
  PID_NAME_0x5e,
  PID_NAME_0x5f,
  PID_NAME_0x60,
};

const char PERCENTAGE[] PROGMEM = "%";
const char KPA[] PROGMEM = "kPa";
const char PA[] PROGMEM = "Pa";
const char RPM[] PROGMEM = "rpm";
const char KPH[] PROGMEM = "km/h";
const char DEGREES_BEFORE_TDC[] PROGMEM = "° before TDC";
const char GRAMS_PER_SECOND[] PROGMEM = "grams/sec";
const char SECONDS[] PROGMEM = "seconds";
const char RATIO[] PROGMEM = "ratio";
const char COUNT[] PROGMEM = "count";
const char KM[] PROGMEM = "km";
const char VOLTS[] PROGMEM = "V";
const char MINUTES[] PROGMEM = "minutes";
const char GPS[] PROGMEM = "g/s";
const char DEGREES[] PROGMEM = "°";
const char DEGREES_CELCIUS[] PROGMEM = "°C";
const char LPH[] PROGMEM = "L/h";

const char* const PID_UNIT_MAPPER[] PROGMEM = {
  NULL,
  NULL,
  NULL,
  NULL,
  PERCENTAGE,
  DEGREES_CELCIUS,
  PERCENTAGE,
  PERCENTAGE,
  PERCENTAGE,
  PERCENTAGE,
  KPA,
  KPA,
  RPM,
  KPH,
  DEGREES_BEFORE_TDC,
  DEGREES_CELCIUS,
  GRAMS_PER_SECOND,
  PERCENTAGE,
  NULL,
  NULL,
  PERCENTAGE,
  PERCENTAGE,
  PERCENTAGE,
  PERCENTAGE,
  PERCENTAGE,
  PERCENTAGE,
  PERCENTAGE,
  PERCENTAGE,
  NULL,
  NULL,
  NULL,
  SECONDS,
  NULL,
  KM,
  KPA,
  KPA,
  RATIO,
  RATIO,
  RATIO,
  RATIO,
  RATIO,
  RATIO,
  RATIO,
  RATIO,
  PERCENTAGE,
  PERCENTAGE,
  PERCENTAGE,
  PERCENTAGE,
  COUNT,
  KM,
  PA,
  KPA,
  RATIO,
  RATIO,
  RATIO,
  RATIO,
  RATIO,
  RATIO,
  RATIO,
  RATIO,
  DEGREES_CELCIUS,
  DEGREES_CELCIUS,
  DEGREES_CELCIUS,
  DEGREES_CELCIUS,
  NULL,
  NULL,
  VOLTS,
  PERCENTAGE,
  RATIO,
  PERCENTAGE,
  DEGREES_CELCIUS,
  PERCENTAGE,
  PERCENTAGE,
  PERCENTAGE,
  PERCENTAGE,
  PERCENTAGE,
  PERCENTAGE,
  MINUTES,
  MINUTES,
  NULL,
  GPS,
  NULL,
  PERCENTAGE,
  KPA,
  PA,
  PERCENTAGE,
  PERCENTAGE,
  PERCENTAGE,
  PERCENTAGE,
  KPA,
  PERCENTAGE,
  PERCENTAGE,
  DEGREES_CELCIUS,
  DEGREES,
  LPH,
  NULL,
  // carloop batt voltage
  VOLTS,
};
