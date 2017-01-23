#ifndef I2CMANAGER_H
#define I2CMANAGER_H

#include "mbed.h"
#include <rtos.h>

enum {
  I2CREQ_READ8,
  I2CREQ_READLEN,
  I2CREQ_WRITE8,
};

struct i2creq {
  int req_type;
  union i2creq_buf {
    struct i2creq_read8 {
      char addr;
      char loc;
      char *res;
      bool *success;
      osThreadId thread_id;
    } read8;
    struct i2creq_readLen {
      char addr;
      char loc;
      char *buffer;
      char len;
      bool *success;
      osThreadId thread_id;
    } readLen;
    struct i2creq_write8 {
      char addr;
      char loc;
      char value;
      bool *success;
      osThreadId thread_id;
    } write8;
  } req_buf;
};

class I2CManager {
  public:
    I2CManager(PinName SDA, PinName SCL, Serial *pc);
    Mail<i2creq, 16> transaction_queue;
    void start_comms(void);
    static void handler_helper(void const *args);
	private:
  	I2C _i2c;
    Serial *_pc;
    Thread *runner;
    bool read8(char addr, char loc, char* res);
    bool readLen(char addr, char loc, char* buffer, char len);
    bool write8(char addr, char loc, char value);
    void queue_handler(void);
};


#endif