#include "SO1602A.h"

SO1602A::SO1602A (PinName sda, PinName scl, char address)
    : p_i2c(new I2C(sda, scl)), i2c(*p_i2c), addr(address)
{
    init();
}
SO1602A::SO1602A (I2C &_i2c, char address)
    : p_i2c(NULL), i2c(_i2c), addr(address)
{
    init();
}
SO1602A::~SO1602A()
{
#ifndef ODE
    if(p_i2c != NULL)
    delete p_i2c;
#endif
}

 
bool SO1602A::cmd(char chr)
{
#ifndef ODE
    buf[0]= 0x00;
    buf[1]= chr;
    // if write success:0, other: err code.
    int status= i2c.write(addr, buf, 2);
//    wait_ms(3);
 
    if(status == 0)
        return true;
    else
        return false;
#else
	return true;
#endif
}
 
int SO1602A::_putc(int val)     // for printf()
{
#ifndef ODE
    if (val == '\n') {
        col = 0;
        row = (row + 1) % 2;
    } else {
        locate(col, row);
        buf[0]= 0x40;
        buf[1]= val;
        i2c.write(addr, buf, 2);
 
        col++;
        if (col >= 16) {
            col = 0;
            row = (row + 1) % 2;
        }
    }
    wait_ms(1);
    return val;
#else
	return 0;
#endif
}
 
// for "Stream"
int SO1602A::_getc()
{
    return -1;
}
 
void SO1602A::locate(int _col, int _row)
{
#ifndef ODE
    col= _col;
    row= _row;
    cmd(0x80 + row * 0x20 + col);
    return;
#endif
}
 
void SO1602A::init()
{
#ifndef ODE
    i2c.frequency(400000);
    col= row= 0;
    buf[0]= 0x00;
    buf[1]= 0x00;
    buf[2]= 0x00;
 
    wait_ms(10);
    this->clear();
    this->cmd(0x02);    //Return Home.
    this->setDispFlag(true, true, true);
    
    return;
#endif
}
 
void SO1602A::setContrast(char val)
{
#ifndef ODE
    // Cmd of Contrast-setting must be setted Ext-Func-Register RE & SD.
    this->setRE();
    this->setSD();
    // Double Byte Command. The contrast has 256 steps, and increase as the value.
    this->cmd(0x81);
    this->cmd(val);
    this->clearSD();
    this->clearRE();
    wait_ms(100);
    return;
#endif
}
 
void SO1602A::setDispFlag(bool disp, bool cursor, bool blink)
{
#ifndef ODE
    // set On/Off. b3=1, b2:Disp, b1:Cursor, b0:blink.
    char tmp=  0x08;
    if(disp)
        tmp += 0x04;
    if(cursor)
        tmp += 0x02;
    if(blink)
        tmp += 0x01;
    this->cmd(tmp);
    this->cmd(0x01);    //Clear Disp.
    wait_ms(20);
    return;
#endif
}
 
void SO1602A::clear()
{
#ifndef ODE
    this->cmd(0x01);
    locate(0, 0);
    wait_ms(5);
    return;
#endif
}
 
// ******************** FUNCTION SET **********************
// RE & IS func-set -> b7-4: 001*.
//                      b3: dispLine; 1(2&4), 0(1&3).
//                      RE: b1, IS: b0.
void SO1602A::setRE()
{
#ifndef ODE
    this->cmd(0x2a);
    return;
#endif
}
void SO1602A::clearRE()
{
#ifndef ODE
    this->cmd(0x28);
    return;
#endif
}
// Extention Register; SD.
// RE setted, 0b 0111 100F. F= Flag; 0: OLED-cmd is disable.
//                                   1:             enable.
void SO1602A::setSD()
{
#ifndef ODE
    this->cmd(0x79);
    return;
#endif
}
void SO1602A::clearSD()
{
#ifndef ODE
    this->cmd(0x78);
    return;
#endif
}
 
// EOF