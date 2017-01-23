#include <math.h>
#include <limits.h>
#include <error.h>

#include "AdaBNO055.h"

BNO055::BNO055(I2CManager const &managerIn) : _manager(managerIn) {
  address = BNOAddress;
  accel_scale = 0.001f;
  rate_scale = 1.0f/16.0f;
  angle_scale = 1.0f/16.0f;
  temp_scale = 1;
}

int BNO055::begin(char mode)
{
  unsigned char systrig;
  if(read8(BNO055_SYS_TRIGGER_ADDR, &systrig)){
    return -ERROR_NACK;
  }
  systrig = systrig | 0x20;
  if(write8(BNO055_SYS_TRIGGER_ADDR, (char)systrig)){
    return -ERROR_NACK;
  }
  //pc.printf("READING ID\r\n");
  unsigned char id;
  while(read8(BNO055_CHIP_ID_ADDR, &id) < 0){
    wait_ms(100);
  }
  //pc.printf("READ ID\r\n");
  if(id != BNO055_ID) {
    //pc.printf("BAD ID %d\r\n", id);
    wait_ms(1000);              // hold on for boot
    //pc.printf("CHECKING AGAIN\r\n");
    read8(BNO055_CHIP_ID_ADDR, &id);
    //pc.printf("READ AGAIN\r\n");
    if(id != BNO055_ID) {
      //pc.printf("STILL BAD.  WE OUT. %d\r\n", id);
      return -ERROR_BAD_ID;
    }
  }

  // Switch to config mod (just in case since this is the default)
  if(!setMode(OPERATION_MODE_CONFIG)){
    return -ERROR_NACK;
  }

  // Reset
  if(write8(BNO055_SYS_TRIGGER_ADDR, 0x20)){
    return -ERROR_NACK;
  }
  unsigned char chip_addr;
  int res;
  do {
    wait_ms(100);
    res = read8(BNO055_CHIP_ID_ADDR, &chip_addr);
  } while (res < 0 || chip_addr != BNO055_ID);
  wait_ms(50);

  // Set to normal power mode
  if(write8(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL)){
    return -ERROR_NACK;
  }
  wait_ms(10);

  if(write8(BNO055_PAGE_ID_ADDR, 0)){
    return -ERROR_NACK;
  }

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

  if(write8(BNO055_SYS_TRIGGER_ADDR, 0x0)){
    return -ERROR_NACK;
  }
  wait_ms(10);
  // Set the requested operating mode
  if(!setMode(mode)){
    return -ERROR_NACK;
  }
  wait_ms(20);

  return 0;
}

bool BNO055::setMode(char mode) {
  _mode = mode;
  if(write8(BNO055_OPR_MODE_ADDR, _mode)){
    return false;
  }
  wait_ms(30);
  return true;
}

int BNO055::setExtCrystalUse(bool usextal) {
  char modeback = _mode;

  if(!setMode(OPERATION_MODE_CONFIG)){
    return -ERROR_NACK;
  }
  wait_ms(25);
  if(write8(BNO055_PAGE_ID_ADDR, 0)){
    return -ERROR_NACK;
  }
  if(usextal) {
    if(write8(BNO055_SYS_TRIGGER_ADDR, 0x80)){
      return -ERROR_NACK;
    }
  } else {
    if(write8(BNO055_SYS_TRIGGER_ADDR, 0x00)){
      return -ERROR_NACK;
    }
  }
  wait_ms(10);
  // Set the requested operating mode
  if(!setMode(modeback)){
    return -ERROR_NACK;
  }
  wait_ms(20);
  return 0;
}

int BNO055::getSystemStatus(unsigned char *system_status, unsigned char *self_test_result, unsigned char *system_error) {
  if(write8(BNO055_PAGE_ID_ADDR, 0)){
    return -ERROR_NACK;
  }

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
    read8(BNO055_SYS_STAT_ADDR, system_status);
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
    read8(BNO055_SELFTEST_RESULT_ADDR, self_test_result);
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
    read8(BNO055_SYS_ERR_ADDR, system_error);
  }

  wait_ms(200);
  return 0;
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

int BNO055::getCalibration(unsigned char* sys, unsigned char* gyro, unsigned char* accel, unsigned char* mag) {
  unsigned char calData;
  if(read8(BNO055_CALIB_STAT_ADDR, &calData)) {
    return -ERROR_NACK;
  }
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
  return 0;
}

int BNO055::getTemp(char *t) {
  if(read8(BNO055_TEMP_ADDR, (unsigned char*)&t)){
    return -ERROR_NACK;
  }
  return 0;
}

int BNO055::getVector(char vector_type, Vector *vec) {
  if (vec == NULL) {
    return -ERROR_NULL_POINTER;
  }
  Vector xyz;
  char buffer[6];      // TODO: maybe uint8?
  memset(buffer, 0, 6);

  int16_t x, y, z;
  x = y = z = 0;

  // Read vector data (6 bytes)
  if(readLen(vector_type, buffer, 6)){
    return -ERROR_NACK;
  }

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
    xyz[0] = x;
    xyz[1] = y;
    xyz[2] = z;
    break;
  case VECTOR_LINEARACCEL:
    xyz[0] = ((double)x/100.0);
    xyz[1] = ((double)y/100.0);
    xyz[2] = ((double)z/100.0);
    break;
  case VECTOR_GRAVITY:
    xyz[0] = ((double)x)/100.0;
    xyz[1] = ((double)y)/100.0;
    xyz[2] = ((double)z)/100.0;
    break;
  }
  *vec = xyz;

  return 0;
}

int BNO055::getQuat(Quaternion *q) {
  if (q == NULL){
    return -ERROR_NULL_POINTER;
  }
  char buffer[8];
  memset(buffer, 0, 8);

  int16_t x, y, z, w;
  x = y = z = w = 0;

  if(readLen(BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8)){
    return -ERROR_NACK;
  }
  w = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
  x = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
  y = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);
  z = (((uint16_t)buffer[7]) << 8) | ((uint16_t)buffer[6]);

  // Assign to Quaternion
  const double scale = (1.0 / (1<<14));
  Quaternion quat(scale * w, scale * x, scale * y, scale * z);
  *q = quat;
  return 0;
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

int BNO055::getSensorOffsets(char *calibData) {
  if(isFullyCalibrated()) {
    char lastMode = _mode;
    if(!setMode(OPERATION_MODE_CONFIG)){
      return -ERROR_SETMODE;
    }
    char NUM_BNO055_OFFSET_REGISTERS = 22;
    bool success;
    success = readLen(ACCEL_OFFSET_X_LSB_ADDR, calibData, NUM_BNO055_OFFSET_REGISTERS);
    if(!setMode(lastMode)){
      return -ERROR_SETMODE;
    }
    if(success){
      return 0;
    } else {
      return -ERROR_NACK;
    }
  }
  return -ERROR_CALIBRATION;
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


int BNO055::setSensorOffsets(const char* calibData) {
  char lastMode = _mode;
  if(!setMode(OPERATION_MODE_CONFIG)){
    return -ERROR_SETMODE;
  }
  wait_ms(25);

  /* A writeLen() would make this much cleaner */
  if(write8(ACCEL_OFFSET_X_LSB_ADDR, calibData[0])){
    return -ERROR_NACK;
  }
  if(write8(ACCEL_OFFSET_X_MSB_ADDR, calibData[1])){
    return -ERROR_NACK;
  }
  if(write8(ACCEL_OFFSET_Y_LSB_ADDR, calibData[2])){
    return -ERROR_NACK;
  }
  if(write8(ACCEL_OFFSET_Y_MSB_ADDR, calibData[3])){
    return -ERROR_NACK;
  }
  if(write8(ACCEL_OFFSET_Z_LSB_ADDR, calibData[4])){
    return -ERROR_NACK;
  }
  if(write8(ACCEL_OFFSET_Z_MSB_ADDR, calibData[5])){
    return -ERROR_NACK;
  }

  if(write8(GYRO_OFFSET_X_LSB_ADDR, calibData[6])){
    return -ERROR_NACK;
  }
  if(write8(GYRO_OFFSET_X_MSB_ADDR, calibData[7])){
    return -ERROR_NACK;
  }
  if(write8(GYRO_OFFSET_Y_LSB_ADDR, calibData[8])){
    return -ERROR_NACK;
  }
  if(write8(GYRO_OFFSET_Y_MSB_ADDR, calibData[9])){
    return -ERROR_NACK;
  }
  if(write8(GYRO_OFFSET_Z_LSB_ADDR, calibData[10])){
    return -ERROR_NACK;
  }
  if(write8(GYRO_OFFSET_Z_MSB_ADDR, calibData[11])){
    return -ERROR_NACK;
  }

  if(write8(MAG_OFFSET_X_LSB_ADDR, calibData[12])){
    return -ERROR_NACK;
  }
  if(write8(MAG_OFFSET_X_MSB_ADDR, calibData[13])){
    return -ERROR_NACK;
  }
  if(write8(MAG_OFFSET_Y_LSB_ADDR, calibData[14])){
    return -ERROR_NACK;
  }
  if(write8(MAG_OFFSET_Y_MSB_ADDR, calibData[15])){
    return -ERROR_NACK;
  }
  if(write8(MAG_OFFSET_Z_LSB_ADDR, calibData[16])){
    return -ERROR_NACK;
  }
  if(write8(MAG_OFFSET_Z_MSB_ADDR, calibData[17])){
    return -ERROR_NACK;
  }

  if(write8(ACCEL_RADIUS_LSB_ADDR, calibData[18])){
    return -ERROR_NACK;
  }
  if(write8(ACCEL_RADIUS_MSB_ADDR, calibData[19])){
    return -ERROR_NACK;
  }

  if(write8(MAG_RADIUS_LSB_ADDR, calibData[20])){
    return -ERROR_NACK;
  }
  if(write8(MAG_RADIUS_MSB_ADDR, calibData[21])){
    return -ERROR_NACK;
  }

  if(!setMode(lastMode)){
    return -ERROR_SETMODE;
  }
  return 0;
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
  if(getCalibration(&system, &gyro, &accel, &mag) < 0){
    return false;
  }
  // if (system < 3 || gyro < 3 || accel < 3 || mag < 3) {
  if(system<3) {
    return false;
  }
  return true;
}

bool BNO055::read8(char loc, unsigned char * res) {
  struct i2creq *req = _manager.transaction_queue.alloc();
  req->req_type = I2CREQ_READ8;
  req->req_buf.read8.addr = address;
  req->req_buf.read8.loc = loc;
  req->req_buf.read8.res = (char*)res;
  bool success;
  req->req_buf.read8.success = &success;
  req->req_buf.read8.thread_id = Thread::gettid();
  _manager.transaction_queue.put(req);
  Thread::signal_wait(0x1);
  return success;
}

bool BNO055::write8(char loc, char value) {
  struct i2creq *req = _manager.transaction_queue.alloc();
  req->req_type = I2CREQ_WRITE8;
  req->req_buf.write8.addr = address;
  req->req_buf.write8.loc = loc;
  req->req_buf.write8.value = value;
  bool success;
  req->req_buf.write8.success = &success;
  req->req_buf.write8.thread_id = Thread::gettid();
  _manager.transaction_queue.put(req);
  Thread::signal_wait(0x1);
  return success;
}


bool BNO055::readLen(char loc, char * buffer, char len) {
  struct i2creq *req = _manager.transaction_queue.alloc();
  req->req_type = I2CREQ_READLEN;
  req->req_buf.readLen.addr = address;
  req->req_buf.readLen.loc = loc;
  req->req_buf.readLen.buffer = buffer;
  req->req_buf.readLen.len = len;
  bool success;
  req->req_buf.readLen.success = &success;
  req->req_buf.readLen.thread_id = Thread::gettid();
  _manager.transaction_queue.put(req);
  Thread::signal_wait(0x1);
  return success;
}
