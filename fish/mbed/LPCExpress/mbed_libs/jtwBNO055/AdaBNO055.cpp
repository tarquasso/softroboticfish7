#include <math.h>
#include <limits.h>

#include "AdaBNO055.h"

BNO055::BNO055(PinName SDA, PinName SCL) : _i2c(SDA,SCL){
  _i2c.frequency(400000);
  address = BNOAddress;
  accel_scale = 0.001f;
  rate_scale = 1.0f/16.0f;
  angle_scale = 1.0f/16.0f;
  temp_scale = 1;
}

bool BNO055::begin(char mode, Serial &pc)
{
  unsigned char systrig = read8(BNO055_SYS_TRIGGER_ADDR);
  systrig = systrig | 0x20;
  write8(BNO055_SYS_TRIGGER_ADDR, (char)systrig);
  pc.printf("READING ID\r\n");
  unsigned char id = read8(BNO055_CHIP_ID_ADDR);
  pc.printf("READ ID\r\n");
  if(id != BNO055_ID) {
    pc.printf("BAD ID %d\r\n", id);
    wait_ms(1000);              // hold on for boot
    pc.printf("CHECKING AGAIN\r\n");
    id = read8(BNO055_CHIP_ID_ADDR);
    pc.printf("READ AGAIN\r\n");
    if(id != BNO055_ID) {
      pc.printf("STILL BAD.  WE OUT. %d\r\n", id);
      return false;
    }
  }

  // Switch to config mod (just in case since this is the default)
  setMode(OPERATION_MODE_CONFIG);

  // Reset
  write8(BNO055_SYS_TRIGGER_ADDR, 0x20);
  while(read8(BNO055_CHIP_ID_ADDR) != BNO055_ID) {
    wait_ms(10);
  }
  wait_ms(50);

  // Set to normal power mode
  write8(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
  wait_ms(10);

  write8(BNO055_PAGE_ID_ADDR, 0);

   /* Set the output units */
  /*
  uint8_t unitsel = (0 << 7) | // Orientation = Android
                    (0 << 4) | // Temperature = Celsius
                    (0 << 2) | // Euler = Degrees
                    (1 << 1) | // Gyro = Rads
                    (0 << 0);  // Accelerometer = m/s^2
  write8(BNO055_UNIT_SEL_ADDR, unitsel);
  */

  /* Configure axis mapping (see section 3.4) */
  /*
  write8(BNO055_AXIS_MAP_CONFIG_ADDR, REMAP_CONFIG_P2); // P0-P7, Default is P1
  delay(10);
  write8(BNO055_AXIS_MAP_SIGN_ADDR, REMAP_SIGN_P2); // P0-P7, Default is P1
  delay(10);
  */

  write8(BNO055_SYS_TRIGGER_ADDR, 0x0);
  wait_ms(10);
  // Set the requested operating mode
  setMode(mode);
  wait_ms(20);

  return true;
}

void BNO055::setMode(char mode) {
  _mode = mode;
  write8(BNO055_OPR_MODE_ADDR, _mode);
  wait_ms(30);
}

void BNO055::setExtCrystalUse(bool usextal) {
  char modeback = _mode;

  setMode(OPERATION_MODE_CONFIG);
  wait_ms(25);
  write8(BNO055_PAGE_ID_ADDR, 0);
  if(usextal) {
    write8(BNO055_SYS_TRIGGER_ADDR, 0x80);
  } else {
    write8(BNO055_SYS_TRIGGER_ADDR, 0x00);
  }
  wait_ms(10);
  // Set the requested operating mode
  setMode(modeback);
  wait_ms(20);
}

void BNO055::getSystemStatus(unsigned char *system_status, unsigned char *self_test_result, unsigned char *system_error) {
  write8(BNO055_PAGE_ID_ADDR, 0);

  /* System Status (see section 4.3.58)
     ---------------------------------
     0 = Idle
     1 = System Error
     2 = Initializing Peripherals
     3 = System Iniitalization
     4 = Executing Self-Test
     5 = Sensor fusio algorithm running
     6 = System running without fusion algorithms */

  if(system_status != 0) {
    *system_status = read8(BNO055_SYS_STAT_ADDR);
  }

    /* Self Test Results (see section )
     --------------------------------
     1 = test passed, 0 = test failed

     Bit 0 = Accelerometer self test
     Bit 1 = Magnetometer self test
     Bit 2 = Gyroscope self test
     Bit 3 = MCU self test

     0x0F = all good! */

  if(self_test_result != 0) {
    *self_test_result = read8(BNO055_SELFTEST_RESULT_ADDR);
  }

    /* System Error (see section 4.3.59)
     ---------------------------------
     0 = No error
     1 = Peripheral initialization error
     2 = System initialization error
     3 = Self test result failed
     4 = Register map value out of range
     5 = Register map address out of range
     6 = Register map write error
     7 = BNO low power mode not available for selected operat ion mode
     8 = Accelerometer power mode not available
     9 = Fusion algorithm configuration error
     A = Sensor configuration error */

  if(system_error != 0) {
    *system_error = read8(BNO055_SYS_ERR_ADDR);
  }

  wait_ms(200);
}

// void BNO055::getRevInfo(char *info) {
//   unsigned char a, b;

//   memset(info, 0, sizeof(char));

//   // Check the accelerometer revision
//   info->accel_rev = read8(BNO055_ACCEL_REV_ID_ADDR);

//   // Check the magnetometer revision
//   info->mag_rev = read8(BNO055_MAG_REV_ID_ADDR);

//   // Check the gyroscope revision
//   info->gyro_rev = read8(BNO055_GYRO_REV_ID_ADDR);

//   // Check the SW revision
//   info->bl_rev = read8(BNO055_BL_REV_ID_ADDR);

//   a = read8(BNO055_SW_REV_ID_LSB_ADDR);
//   b = read8(BNO055_SW_REV_ID_MSB_ADDR);
//   info->sw_rev = (((uint16_t)b) << 8) | ((uint16_t)a);
// }

void BNO055::getCalibration(unsigned char* sys, unsigned char* gyro, unsigned char* accel, unsigned char* mag) {
  unsigned char calData= read8(BNO055_CALIB_STAT_ADDR);
  if(sys != NULL) {
    *sys = (calData >> 6) & 0x03;
  }
  if(gyro != NULL) {
    *gyro = (calData >> 4) & 0x03;
  }
  if(accel != NULL) {
    *accel = (calData >> 2) & 0x03;
  }
  if(mag != NULL) {
    *mag = calData & 0x03;
  }
}

char BNO055::getTemp(void) {
  char temp = (char)(read8(BNO055_TEMP_ADDR));
  return temp;
}

Vector BNO055::getVector(char vector_type) {
  Vector xyz;
  char buffer[6];      // TODO: maybe uint8?
  memset(buffer, 0, 6);

  int16_t x, y, z;
  x = y = z = 0;

  // Read vector data (6 bytes)
  readLen(vector_type, buffer, 6);

  x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
  y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
  z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

  // Convert the value to an appropriate range
  // and assign the value to the Vector type
  switch(vector_type) {
  case VECTOR_MAGNETOMETER:
    xyz[0] = ((double)x)/16.0;
    xyz[1] = ((double)y)/16.0;
    xyz[2] = ((double)z)/16.0;
    break;
  case VECTOR_GYROSCOPE:
    xyz[0] = ((double)x)/900.0;
    xyz[1] = ((double)y)/900.0;
    xyz[2] = ((double)z)/900.0;
    break;
  case VECTOR_EULER:
    /* 1 degree = 16 LSB */
    xyz[0] = ((double)x)/16.0;
    xyz[1] = ((double)y)/16.0;
    xyz[2] = ((double)z)/16.0;
    break;
  case VECTOR_ACCELEROMETER:
  case VECTOR_LINEARACCEL:
  case VECTOR_GRAVITY:
    xyz[0] = ((double)x)/100.0;
    xyz[1] = ((double)y)/100.0;
    xyz[2] = ((double)z)/100.0;
    break;
  }

  return xyz;
}

Quaternion BNO055::getQuat() {
  char buffer[8];
  memset(buffer, 0, 8);

  int16_t x, y, z, w;
  x = y = z = w = 0;

  readLen(BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8);
  w = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
  x = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
  y = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);
  z = (((uint16_t)buffer[7]) << 8) | ((uint16_t)buffer[6]);

  // Assign to Quaternion
  const double scale = (1.0 / (1<<14));
  Quaternion quat(scale * w, scale * x, scale * y, scale * z);
  return quat;
}

// BNO055::getSensor(sensor *sensor) {
//   // Clear the sensor_t object
//   memset(sensor, 0, sizeof(sensor_t));

//   // Insert the sensor name in the fixed length char array
//   strncpy(sensor->name, "BNO055", sizeof(sensor->name) - 1);
//   sensor->name[sizeof(sensor->name)- 1] = 0;
//   sensor->version     = 1;
//   sensor->sensor_id   = _sensorID;
//   sensor->type        = SENSOR_TYPE_ORIENTATION;
//   sensor->min_delay   = 0;
//   sensor->max_value   = 0.0F;
//   sensor->min_value   = 0.0F;
//   sensor->resolution  = 0.01F;
// }

// BNO055::getEvent(sensor_event_t *event) {
//   memset(event, 0, sizeof(sensors_event_t));

//   event->version = sizeof(sensors_event_t);
//   event->sensor_id = _sensorID;
//   event->type = SENSOR_TYPE_ORIENTATION;
//   event->timestamp = millis();

//   Vector euler = getVector(VECTOR_EULER);
//   event->orientation.x = euler.x();
//   event->orientation.y = euler.y();
//   event->orientation.z = euler.z();

//   return true;
// }

bool BNO055::getSensorOffsets(char *calibData) {
  if(isFullyCalibrated()) {
    char lastMode = _mode;
    setMode(OPERATION_MODE_CONFIG);
    char NUM_BNO055_OFFSET_REGISTERS = 22;
    readLen(ACCEL_OFFSET_X_LSB_ADDR, calibData, NUM_BNO055_OFFSET_REGISTERS);

    setMode(lastMode);
    return true;
  }
  return false;
}

// bool BNO055::getSensorOffsets(char &offsets_type) {
//   if(isFullyCalibrated()) {
//     char lastMode = mode;
//     setMode(OPERATION_MODE_CONFIG);
//     wait_ms(25);

//     offsets_type.accel_offset_x = (read8(ACCEL_OFFSET_X_MSB_ADDR) << 8) | (read8(ACCEL_OFFSET_X_LSB_ADDR));
//     offsets_type.accel_offset_y = (read8(ACCEL_OFFSET_Y_MSB_ADDR) << 8) | (read8(ACCEL_OFFSET_Y_LSB_ADDR));
//     offsets_type.accel_offset_z = (read8(ACCEL_OFFSET_Z_MSB_ADDR) << 8) | (read8(ACCEL_OFFSET_Z_LSB_ADDR));

//     offsets_type.gyro_offset_x = (read8(GYRO_OFFSET_X_MSB_ADDR) << 8) | (read8(GYRO_OFFSET_X_LSB_ADDR));
//     offsets_type.gyro_offset_y = (read8(GYRO_OFFSET_Y_MSB_ADDR) << 8) | (read8(GYRO_OFFSET_Y_LSB_ADDR));
//     offsets_type.gyro_offset_z = (read8(GYRO_OFFSET_Z_MSB_ADDR) << 8) | (read8(GYRO_OFFSET_Z_LSB_ADDR));

//     offsets_type.mag_offset_x = (read8(MAG_OFFSET_X_MSB_ADDR) << 8) | (read8(MAG_OFFSET_X_LSB_ADDR));
//     offsets_type.mag_offset_y = (read8(MAG_OFFSET_Y_MSB_ADDR) << 8) | (read8(MAG_OFFSET_Y_LSB_ADDR));
//     offsets_type.mag_offset_z = (read8(MAG_OFFSET_Z_MSB_ADDR) << 8) | (read8(MAG_OFFSET_Z_LSB_ADDR));

//     offsets_type.accel_radius = (read8(ACCEL_RADIUS_MSB_ADDR) << 8) | (read8(ACCEL_RADIUS_LSB_ADDR));
//     offsets_type.mag_radius = (read8(MAG_RADIUS_MSB_ADDR) << 8) | (read8(MAG_RADIUS_LSB_ADDR));

//     setMode(lastMode);
//     return true;
//   }
//   return false;
// }


void BNO055::setSensorOffsets(const char* calibData) {
  char lastMode = _mode;
  setMode(OPERATION_MODE_CONFIG);
  wait_ms(25);

  /* A writeLen() would make this much cleaner */
  write8(ACCEL_OFFSET_X_LSB_ADDR, calibData[0]);
  write8(ACCEL_OFFSET_X_MSB_ADDR, calibData[1]);
  write8(ACCEL_OFFSET_Y_LSB_ADDR, calibData[2]);
  write8(ACCEL_OFFSET_Y_MSB_ADDR, calibData[3]);
  write8(ACCEL_OFFSET_Z_LSB_ADDR, calibData[4]);
  write8(ACCEL_OFFSET_Z_MSB_ADDR, calibData[5]);

  write8(GYRO_OFFSET_X_LSB_ADDR, calibData[6]);
  write8(GYRO_OFFSET_X_MSB_ADDR, calibData[7]);
  write8(GYRO_OFFSET_Y_LSB_ADDR, calibData[8]);
  write8(GYRO_OFFSET_Y_MSB_ADDR, calibData[9]);
  write8(GYRO_OFFSET_Z_LSB_ADDR, calibData[10]);
  write8(GYRO_OFFSET_Z_MSB_ADDR, calibData[11]);

  write8(MAG_OFFSET_X_LSB_ADDR, calibData[12]);
  write8(MAG_OFFSET_X_MSB_ADDR, calibData[13]);
  write8(MAG_OFFSET_Y_LSB_ADDR, calibData[14]);
  write8(MAG_OFFSET_Y_MSB_ADDR, calibData[15]);
  write8(MAG_OFFSET_Z_LSB_ADDR, calibData[16]);
  write8(MAG_OFFSET_Z_MSB_ADDR, calibData[17]);

  write8(ACCEL_RADIUS_LSB_ADDR, calibData[18]);
  write8(ACCEL_RADIUS_MSB_ADDR, calibData[19]);

  write8(MAG_RADIUS_LSB_ADDR, calibData[20]);
  write8(MAG_RADIUS_MSB_ADDR, calibData[21]);

  setMode(lastMode);
}


// void BNO055::setSensorOffsets(const char &offsets_type) {
//   char lastMode = _mode;
//   setMode(OPERATION_MODE_CONFIG);
//   wait_ms(25);

//   write8(ACCEL_OFFSET_X_LSB_ADDR, (offsets_type.accel_offset_x) & 0x0FF);
//   write8(ACCEL_OFFSET_X_MSB_ADDR, (offsets_type.accel_offset_x >> 8) & 0x0FF);
//   write8(ACCEL_OFFSET_Y_LSB_ADDR, (offsets_type.accel_offset_y) & 0x0FF);
//   write8(ACCEL_OFFSET_Y_MSB_ADDR, (offsets_type.accel_offset_y >> 8) & 0x0FF);
//   write8(ACCEL_OFFSET_Z_LSB_ADDR, (offsets_type.accel_offset_z) & 0x0FF);
//   write8(ACCEL_OFFSET_Z_MSB_ADDR, (offsets_type.accel_offset_z >> 8) & 0x0FF);

//   write8(GYRO_OFFSET_X_LSB_ADDR, (offsets_type.gyro_offset_x) & 0x0FF);
//   write8(GYRO_OFFSET_X_MSB_ADDR, (offsets_type.gyro_offset_x >> 8) & 0x0FF);
//   write8(GYRO_OFFSET_Y_LSB_ADDR, (offsets_type.gyro_offset_y) & 0x0FF);
//   write8(GYRO_OFFSET_Y_MSB_ADDR, (offsets_type.gyro_offset_y >> 8) & 0x0FF);
//   write8(GYRO_OFFSET_Z_LSB_ADDR, (offsets_type.gyro_offset_z) & 0x0FF);
//   write8(GYRO_OFFSET_Z_MSB_ADDR, (offsets_type.gyro_offset_z >> 8) & 0x0FF);

//   write8(MAG_OFFSET_X_LSB_ADDR, (offsets_type.mag_offset_x) & 0x0FF);
//   write8(MAG_OFFSET_X_MSB_ADDR, (offsets_type.mag_offset_x >> 8) & 0x0FF);
//   write8(MAG_OFFSET_Y_LSB_ADDR, (offsets_type.mag_offset_y) & 0x0FF);
//   write8(MAG_OFFSET_Y_MSB_ADDR, (offsets_type.mag_offset_y >> 8) & 0x0FF);
//   write8(MAG_OFFSET_Z_LSB_ADDR, (offsets_type.mag_offset_z) & 0x0FF);
//   write8(MAG_OFFSET_Z_MSB_ADDR, (offsets_type.mag_offset_z >> 8) & 0x0FF);

//   write8(ACCEL_RADIUS_LSB_ADDR, (offsets_type.accel_radius) & 0x0FF);
//   write8(ACCEL_RADIUS_MSB_ADDR, (offsets_type.accel_radius >> 8) & 0x0FF);

//   write8(MAG_RADIUS_LSB_ADDR, (offsets_type.mag_radius) & 0x0FF);
//   write8(MAG_RADIUS_MSB_ADDR, (offsets_type.mag_radius >> 8) & 0x0FF);

//   setMode(lastMode);
// }

bool BNO055::isFullyCalibrated(void) {
  unsigned char system, gyro, accel, mag;
  getCalibration(&system, &gyro, &accel, &mag);
  // if (system < 3 || gyro < 3 || accel < 3 || mag < 3) {
  if(system<3) {
    return false;
  }
  return true;
}

unsigned char BNO055::read8(char loc) {
  char reg = loc;
  char res;
  _i2c.write(address, &reg, 1, true);
  _i2c.read(address+1, &res, 1, false);
  return (unsigned char) res;
}

bool BNO055::write8(char loc, char value) {
  char msg[2];
  msg[0] = loc;
  msg[1] = value;
  _i2c.write(address, msg, 2);
  return true;
}


bool BNO055::readLen(char loc, char * buffer, char len) {
  char reg = loc;
  _i2c.write(address, &reg, 1, true);
  _i2c.read(address, buffer, len, false);
  return true;
}
