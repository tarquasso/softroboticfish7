// buttons.cpp

#include "ButtonBoard.h"

extern "C" void mbed_reset();

ButtonBoard::ButtonBoard(PinName sda, PinName scl, PinName int1, PinName int2) :
    _i2c(sda, scl),
    _int1(int1),
    _int2(int2),
    _callbackFunction(NULL),
    _button_state(0)
{
    // Initialize callback table
    for (int i=0; i < BTTN_COUNT; i++)
    {
        _callback_table_valid[i] = false;
    }
    
    // Set I2C frequency to fast-mode 400KHz
    _i2c.frequency(400000L);
    
    ///// Configuration
    bool bd1_failure = false;
    bool bd2_failure = false;
    
    // Configure input latching
    out_buf[0] = 0x44;  // Input latch register
    out_buf[1] = 0x00;  // Don't need latching on output
    out_buf[1] = 0x00;  // Latch them all
    bd1_failure |= _i2c.write(ADDR_BOARD_1, out_buf, 3);
    bd2_failure |= _i2c.write(ADDR_BOARD_2, out_buf, 3);
    
    // Disable pull-ups
    out_buf[0] = 0x46;  // Pull-up/down enable register
    out_buf[1] = 0x00;  // Don't need on outputs
    out_buf[2] = 0x00;  // Don't need
    out_buf[3] = 0xFF;  // Select pull-ups
    out_buf[4] = 0xFF;
    bd1_failure |= _i2c.write(ADDR_BOARD_1, out_buf, 5);
    bd2_failure |= _i2c.write(ADDR_BOARD_2, out_buf, 5);
    
    // Configure outputs as open-drain
    out_buf[0] = 0x4F;  // Output port config register
    out_buf[1] = 0x02;  // Port 1 to open drain
    bd1_failure |= _i2c.write(ADDR_BOARD_1, out_buf, 2);
    bd2_failure |= _i2c.write(ADDR_BOARD_2, out_buf, 2);
    
    // Reset output register high
    _led_ports = 0xFFFF; // All LED's off
    out_buf[0] = 0x02; // Output register
    out_buf[1] = _led_ports & 0xFF; 
    bd1_failure |= _i2c.write(ADDR_BOARD_1, out_buf, 2);
    out_buf[2] = _led_ports>>8;
    bd2_failure |= _i2c.write(ADDR_BOARD_2, out_buf, 2);
    
    // Configure ports as input or outputs
    out_buf[0] = 0x06;  // Configuration registers
    out_buf[1] = 0x00;  // Port 1 -> Output
    out_buf[2] = 0xFF;  // Port 2 <- Input 
    bd1_failure |= _i2c.write(ADDR_BOARD_1, out_buf, 3); 
    bd2_failure |= _i2c.write(ADDR_BOARD_2, out_buf, 3); 
    
    // Read input registers to clear interrupts
    out_buf[0] = 0x00; // Input registers
    bd1_failure |= _i2c.write(ADDR_BOARD_1, out_buf, 1, true); 
    bd1_failure |= _i2c.read(ADDR_BOARD_1, out_buf, 2);  // Read registers
    bd2_failure |= _i2c.write(ADDR_BOARD_2, out_buf, 1, true); 
    bd2_failure |= _i2c.read(ADDR_BOARD_2, out_buf, 2);  // Read registers
    
    // Disable interrupt masking on inputs
    out_buf[0] = 0x4A;  // Interrupt mask register
    out_buf[1] = 0xFF;  // Mask outputs
    out_buf[2] = 0xFF;  // Don't mask inputs
    bd1_failure |= _i2c.write(ADDR_BOARD_1, out_buf, 3);
    bd2_failure |= _i2c.write(ADDR_BOARD_2, out_buf, 3);
    
    // Disable interrupt masking on inputs
    out_buf[0] = 0x4A;  // Interrupt mask register
    out_buf[1] = 0xFF;  // Mask outputs
    out_buf[2] = 0x00;  // Don't mask inputs
    bd1_failure |= _i2c.write(ADDR_BOARD_1, out_buf, 3);
    bd2_failure |= _i2c.write(ADDR_BOARD_2, out_buf, 3);
    
    if (bd1_failure)
    {
        // serial.printf("Nack recieved while configuring button board 1\n");
        bd1_failure = false;
    }
    if (bd2_failure)
    {
        // serial.printf("Nack recieved while configuring button board 2\n");
        bd2_failure = false;
    }
    
    // Enable mbed interrupt lines
    _int1.fall(this, &ButtonBoard::_int1_handler);
    _int2.fall(this, &ButtonBoard::_int2_handler);
    _int1.mode(PullUp);
    _int2.mode(PullUp);
    
    // Register callbacks
    //registerCallback(BTTN_RESET, &mbed_reset);
    
}

ButtonBoard::~ButtonBoard()
{
}

void ButtonBoard::_int1_handler()
{
    _fall_handler(ADDR_BOARD_1);
}

void ButtonBoard::_int2_handler()
{
    _fall_handler(ADDR_BOARD_2);
}

void ButtonBoard::_fall_handler(char board)
{    
    char int_status;
    char input_port;
    char masked_port;
    
    int board_offset;
    if (board == ADDR_BOARD_1)
    {
        board_offset = 0;
    }
    else
    {
        board_offset = BTTN_COUNT_BOARD_1;
    }
    
    // Poll board for interrupt source byte and input reg byte
    out_buf[0] = 0x4d; // Port 1 Interrupt status
    _i2c.write(board, out_buf, 1);
    _i2c.read(board, &int_status, 1);
    
    out_buf[0] = 0x01; // Port 1 input register
    _i2c.write(board, out_buf, 1);
    _i2c.read(board, &input_port, 1);
    
    // Use int status to mask input port (input goes low when button depressed)
    // int status will always indicate which button changed
    // masked_port will also indicate button if button is pressed, but will be 0 if button is released
    masked_port = int_status & (~input_port);
    if(masked_port)
        _button_state |= int_status;
    else
        _button_state &= ~int_status;
    
    for (int i=0; i<8; i++)
    {
        // For every high bit in masked port
        if (masked_port & (1<<i))
        {
            // Call corresponding callback
            int cb_i = i+board_offset;
            if (_callback_table_valid[cb_i] == true)
            {
                _callback_table[cb_i].call();
            }
        }
    }
    // Call master callback
    if(_callbackFunction != NULL)
        _callbackFunction(int_status, masked_port, _button_state);
}


void ButtonBoard::registerCallback(uint32_t button, FunctionPointer p)
{
    if (button < BTTN_COUNT)
    { 
        _callback_table[button] = p;
        _callback_table_valid[button] = true;
    }
}

void ButtonBoard::registerCallback(void (*p)(char buttonMask, bool pressed, char curState))
{
    _callbackFunction = p;
}

//void ButtonBoard::setLED(uint32_t led, bool val)
//{
//    if (led > BTTN_COUNT)
//    {
//        // invalid, skip
//        return;
//    }
//    
//    char board;
//    
//    if (led < BTTN_COUNT_BOARD_1)
//    {
//        // Address first board
//        board = ADDR_BOARD_1;
//    }
//    else
//    {
//        // Address second board
//        board = ADDR_BOARD_2;
//        led = led - BTTN_COUNT_BOARD_1;
//    }
//    
//    bool fail = false;
//    
//    // Read port state
//    char port_state;
//    out_buf[0] = 0x02; // Port 0 output register
//    
//    fail |= _i2c.write(board, out_buf, 1, true);
//    fail |= _i2c.read(board, &port_state, 1);
//    
//    if (val == true)
//    {
//        // Turn LED on by clearing bit
//        port_state &= ~(1<<led);
//    }
//    else
//    {
//        // Turn LED off by raising bit
//        port_state |= (1<<led);
//    }
//    
//    // Now write to board
//    out_buf[0] = 0x02; // Port 0 output register
//    out_buf[1] = port_state;
//    
//    fail |= _i2c.write(board, out_buf, 2);
//    
//    if (fail)
//    {
//        // serial.printf("Nack recieved when writing LED %d on Board %d", led, board);
//    }
//    else
//    {
//        _led_ports = port_state<<(8*board) + _led_ports & (~(0xFF<<(8*board)));
//    }
//}

void ButtonBoard::setLEDs(char mask, bool turnOn, char board /* = ADDR_BOARD_1 */)
{
    bool fail = false;
    
    // Read port state
    char port_state;
    out_buf[0] = 0x02; // Port 0 output register
    
    fail |= _i2c.write(board, out_buf, 1, true);
    fail |= _i2c.read(board, &port_state, 1);
    
    if(turnOn == true)
    {
        // Turn LEDs on by clearing bits
        port_state &= ~(mask);
    }
    else
    {
        // Turn LEDs off by raising bits
        port_state |= (mask);
    }
    
    // Now write to board
    out_buf[0] = 0x02; // Port 0 output register
    out_buf[1] = port_state;
    
    fail |= _i2c.write(board, out_buf, 2);
    
    if(fail)
    {
        // serial.printf("Nack recieved when writing LED %d on Board %d", led, board);
        //printf("button board write failed\n");
    }
    else
    {
        //_led_ports = port_state<<(8*board) + _led_ports & (~(0xFF<<(8*board)));
        _led_ports = port_state;
    }
}

char ButtonBoard::getLEDs(char ledMask, char board /* = ADDR_BOARD_1 */)
{
    return ~_led_ports & ledMask;
//    bool fail = false;
//    
//    // Read port state
//    char port_state;
//    out_buf[0] = 0x02; // Port 0 output register
//    
//    fail |= _i2c.write(board, out_buf, 1, true);
//    fail |= _i2c.read(board, &port_state, 1);
//    
//    return ~port_state & ledMask;
}

char ButtonBoard::getButtons(char buttonMask, char board /* = ADDR_BOARD_1 */)
{
    return _button_state & buttonMask;
}

//uint16_t ButtonBoard::readInputs()
//{
//    char b1_p1;
//    char b2_p1;
//    
//    out_buf[0] = 0x01;
//    _i2c.write(ADDR_BOARD_1, out_buf, 1, true);
//    _i2c.read(ADDR_BOARD_1, &b1_p1, 1);
//    
//    _i2c.write(ADDR_BOARD_2, out_buf, 1, true);
//    _i2c.read(ADDR_BOARD_2, &b2_p1, 1);
//    
//    uint16_t out = (b2_p1<<8) | b1_p1;
//    
//    return out;
//}
