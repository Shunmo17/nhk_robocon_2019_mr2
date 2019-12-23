#ifndef _SO1602A_H_
#define _SO1602A_H_
 
#pragma once

#include "Servo.h"

// SA0(Pin4); Low: i2c_addr= 0x78, High: 0x7a;
// DDRAM addr. Line1: 0x00 ~ 0x0f. Line2: 0x20 ~ 0x2f.

class SO1602A : public Stream{
public:
    SO1602A (PinName sda, PinName scl, char address= 0x78);
    SO1602A (I2C &_i2c, char address= 0x78);
    ~SO1602A();
//  ******************** printf() future of C-language. ***************************
 
    /** Initialize
     */
    void init();
 
    /** Clear Display.
     */
    void clear();
    
    /** Set Position of char.
     *  @param col: column, row: rows.
     */
    void locate(int col, int row);
    
    /** Set Contrast.
     *  @param val; 256steps, 0x00 ~ 0xff. Contrast increas as the value.
     */
    void setContrast(char val);
    
    /** Set Display flag.
     * @parm Enable Display, Cursor turned on, Cursor Blink.
     */
    void setDispFlag(bool disp= true, bool cursor= true, bool blink= true);
 
private:    
    I2C *p_i2c;
    I2C &i2c;
    char addr;
    char buf[3];
    int col, row;
    
    bool cmd(char chr);
    
    // virtual func for printf() in Stream-class.
    virtual int _putc(int val);
    virtual int _getc();
    
// Function SET
    void setRE();
    void clearRE();
    void setSD();
    void clearSD();
 
};
 
// EOF

#endif