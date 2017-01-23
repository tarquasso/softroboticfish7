#include "I2CManager.h"
#include "cmsis_os.h"
#include "mbed.h"
#include <SerialComm/SerialComm.h>

// Serial pc(USBTX, USBRX);

I2CManager::I2CManager(PinName SDA, PinName SCL, Serial *pc) : _i2c(SDA,SCL), _pc(pc){
  _i2c.frequency(400000);
}

void I2CManager::start_comms() {
  runner = new Thread(handler_helper, this, osPriorityHigh);
  _pc->printf("started comms\r\n");
}

void I2CManager::queue_handler() {
  while(true) {
    osEvent evt = transaction_queue.get();
    if (evt.status == osEventMail) {
      i2creq *req = (i2creq*)evt.value.p;
      switch(req->req_type) {
        case I2CREQ_READ8:
          *(req->req_buf.read8.success) = read8(req->req_buf.read8.addr, req->req_buf.read8.loc, req->req_buf.read8.res);
          osSignalSet(req->req_buf.read8.thread_id, 0x1);
          break;
        case I2CREQ_READLEN:
          *(req->req_buf.readLen.success) = readLen(req->req_buf.readLen.addr, req->req_buf.readLen.loc, req->req_buf.readLen.buffer, req->req_buf.readLen.len);
          osSignalSet(req->req_buf.readLen.thread_id, 0x1);
          break;
        case I2CREQ_WRITE8:
          *(req->req_buf.write8.success) = write8(req->req_buf.write8.addr, req->req_buf.write8.loc, req->req_buf.write8.value);
          osSignalSet(req->req_buf.write8.thread_id, 0x1);
          break;
        case I2CREQ_COMMAND:
          *(req->req_buf.command.success) = command(req->req_buf.command.addr, req->req_buf.command.loc);
          osSignalSet(req->req_buf.command.thread_id, 0x1);
          break;
        default:
          // Bad request type
          break;
      }
      transaction_queue.free(req);
    }
  }
}

void I2CManager::handler_helper(void const *args) {
  ((I2CManager*)args)->queue_handler();
}

bool I2CManager::read8(char address, char loc, char * res) {
  char reg = loc;
  if(_i2c.write(address, &reg, 1, true)){
    return true;
  }
  return _i2c.read(address+1, (char*)res, 1, false);
}

bool I2CManager::write8(char address, char loc, char value) {
  char msg[2];
  msg[0] = loc;
  msg[1] = value;
  return _i2c.write(address, msg, 2);
}

bool I2CManager::command(char address, char loc) {
  char reg = loc;
  return _i2c.write(address, &reg, 1);
}


bool I2CManager::readLen(char address, char loc, char * buffer, char len) {
  char reg = loc;
  if(_i2c.write(address, &reg, 1, true)){
    return true;
  }
  return _i2c.read(address, buffer, len, false);
}