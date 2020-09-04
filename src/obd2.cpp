#include "obd2.h"

// Sourced from https://github.com/sandeepmistry/arduino-OBD2/blob/master/src/OBD2.cpp

String getPidName(uint8_t pid) {
  if (pid >= PID_SIZE) {
    return "Unknown";
  }

#ifdef __AVR__
  const char* pgmName = pgm_read_ptr(&PID_NAME_MAPPER[pid]);
  String name;

  if (pgmName != NULL) {
    while (char c = pgm_read_byte(pgmName++)) {
      name += c;
    }
  }

  return name;
#else
  return PID_NAME_MAPPER[pid];
#endif
}

String getPidUnits(uint8_t pid) {
  if (pid >= PID_SIZE) {
    return "";
  }

#ifdef __AVR__
  const char* pgmUnits = pgm_read_ptr(&PID_UNIT_MAPPER[pid]);
  String units;

  if (pgmUnits != NULL) {
    while (char c = pgm_read_byte(pgmUnits++)) {
      units += c;
    }
  }

  return units;
#else
  return PID_UNIT_MAPPER[pid];
#endif
}

float getPidValue(uint8_t pid, uint8_t value[4]) {
  uint8_t A = value[0];
  uint8_t B = value[1];
  uint8_t C = value[2];
  uint8_t D = value[3];

  switch (pid) {
    default:
    case PIDS_SUPPORT_01_20: // raw
    case MONITOR_STATUS_SINCE_DTCS_CLEARED: // raw
    case FREEZE_DTC: // raw
    case PIDS_SUPPORT_21_40: // raw
    case PIDS_SUPPORT_41_60: // raw
    case MONITOR_STATUS_THIS_DRIVE_CYCLE: // raw
      // NOTE: return value can lose precision!
      return ((uint32_t)A << 24 | (uint32_t)B << 16 | (uint32_t)C << 8 | (uint32_t)D);

    case FUEL_SYSTEM_STATUS: // raw
    case RUN_TIME_SINCE_ENGINE_START:
    case DISTANCE_TRAVELED_WITH_MIL_ON:
    case DISTANCE_TRAVELED_SINCE_CODES_CLEARED:
    case TIME_RUN_WITH_MIL_ON:
    case TIME_SINCE_TROUBLE_CODES_CLEARED:
      return (A * 256.0 + B);

    case CALCULATED_ENGINE_LOAD:
    case THROTTLE_POSITION:
    case COMMANDED_EGR:
    case COMMANDED_EVAPORATIVE_PURGE:
    case FUEL_TANK_LEVEL_INPUT:
    case RELATIVE_THROTTLE_POSITION:
    case ABSOLUTE_THROTTLE_POSITION_B:
    case ABSOLUTE_THROTTLE_POSITION_C:
    case ABSOLUTE_THROTTLE_POSITION_D:
    case ABSOLUTE_THROTTLE_POSITION_E:
    case ABSOLUTE_THROTTLE_POSITION_F:
    case COMMANDED_THROTTLE_ACTUATOR:
    case ETHANOL_FUEL_PERCENTAGE:
    case RELATIVE_ACCELERATOR_PEDAL_POSITTION:
    case HYBRID_BATTERY_PACK_REMAINING_LIFE:
      return (A / 2.55);

    case COMMANDED_SECONDARY_AIR_STATUS: // raw
    case OBD_STANDARDS_THIS_VEHICLE_CONFORMS_TO: // raw
    case OXYGEN_SENSORS_PRESENT_IN_2_BANKS: // raw
    case OXYGEN_SENSORS_PRESENT_IN_4_BANKS: // raw
    case AUXILIARY_INPUT_STATUS: // raw
    case FUEL_TYPE: // raw
    case EMISSION_REQUIREMENT_TO_WHICH_VEHICLE_IS_DESIGNED: // raw
      return (A);

    case OXYGEN_SENSOR_1_SHORT_TERM_FUEL_TRIM:
    case OXYGEN_SENSOR_2_SHORT_TERM_FUEL_TRIM:
    case OXYGEN_SENSOR_3_SHORT_TERM_FUEL_TRIM:
    case OXYGEN_SENSOR_4_SHORT_TERM_FUEL_TRIM:
    case OXYGEN_SENSOR_5_SHORT_TERM_FUEL_TRIM:
    case OXYGEN_SENSOR_6_SHORT_TERM_FUEL_TRIM:
    case OXYGEN_SENSOR_7_SHORT_TERM_FUEL_TRIM:
    case OXYGEN_SENSOR_8_SHORT_TERM_FUEL_TRIM:
      return ((B / 1.28) - 100.0);
      break;

    case ENGINE_COOLANT_TEMPERATURE:
    case AIR_INTAKE_TEMPERATURE:
    case AMBIENT_AIR_TEMPERATURE:
    case ENGINE_OIL_TEMPERATURE:
      return (A - 40.0);

    case SHORT_TERM_FUEL_TRIM_BANK_1:
    case LONG_TERM_FUEL_TRIM_BANK_1:
    case SHORT_TERM_FUEL_TRIM_BANK_2:
    case LONG_TERM_FUEL_TRIM_BANK_2:
    case EGR_ERROR:
      return ((A / 1.28) - 100.0);

    case FUEL_PRESSURE:
      return (A * 3.0);

    case INTAKE_MANIFOLD_ABSOLUTE_PRESSURE:
    case VEHICLE_SPEED:
    case WARM_UPS_SINCE_CODES_CLEARED:
    case ABSOLULTE_BAROMETRIC_PRESSURE:
      return (A);

    case ENGINE_RPM:
      return ((A * 256.0 + B) / 4.0);

    case TIMING_ADVANCE:
      return ((A / 2.0) - 64.0);

    case MAF_AIR_FLOW_RATE:
      return ((A * 256.0 + B) / 100.0);

    case FUEL_RAIL_PRESSURE:
      return ((A * 256.0 + B) * 0.079);

    case FUEL_RAIL_GAUGE_PRESSURE:
    case FUEL_RAIL_ABSOLUTE_PRESSURE:
      return ((A * 256.0 + B) * 10.0);

    case OXYGEN_SENSOR_1_FUEL_AIR_EQUIVALENCE_RATIO:
    case OXYGEN_SENSOR_2_FUEL_AIR_EQUIVALENCE_RATIO:
    case OXYGEN_SENSOR_3_FUEL_AIR_EQUIVALENCE_RATIO:
    case OXYGEN_SENSOR_4_FUEL_AIR_EQUIVALENCE_RATIO:
    case OXYGEN_SENSOR_5_FUEL_AIR_EQUIVALENCE_RATIO:
    case OXYGEN_SENSOR_6_FUEL_AIR_EQUIVALENCE_RATIO:
    case OXYGEN_SENSOR_7_FUEL_AIR_EQUIVALENCE_RATIO:
    case OXYGEN_SENSOR_8_FUEL_AIR_EQUIVALENCE_RATIO:
    case 0x34:
    case 0x35:
    case 0x36:
    case 0x37:
    case 0x38:
    case 0x39:
    case 0x3a:
    case 0x3b:
      return (((A * 256.0 + B) * 2.0) / 65536.0);

    case EVAP_SYSTEM_VAPOR_PRESSURE:
      return (((int16_t)(A * 256.0 + B)) / 4.0);

    case CATALYST_TEMPERATURE_BANK_1_SENSOR_1:
    case CATALYST_TEMPERATURE_BANK_2_SENSOR_1:
    case CATALYST_TEMPERATURE_BANK_1_SENSOR_2:
    case CATALYST_TEMPERATURE_BANK_2_SENSOR_2:
      return (((A * 256.0 + B) / 10.0) - 40.0);

    case CONTROL_MODULE_VOLTAGE:
      return ((A * 256.0 + B) / 1000.0);

    case ABSOLUTE_LOAD_VALUE:
      return ((A * 256.0 + B) / 2.55);

    case FUEL_AIR_COMMANDED_EQUIVALENCE_RATE:
      return (2.0 * (A * 256.0 + B) / 65536.0);

    case ABSOLUTE_EVAP_SYSTEM_VAPOR_PRESSURE:
      return ((A * 256.0 + B) / 200.0);

    case 0x54:
      return ((A * 256.0 + B) - 32767.0);

    case FUEL_INJECTION_TIMING:
      return (((A * 256.0 + B) / 128.0) - 210.0);

    case ENGINE_FUEL_RATE:
      return ((A * 256.0 + B) / 20.0);
  }
}