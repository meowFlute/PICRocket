/* 
 * File:   i2c_func.h
 * Author: Scott
 *
 * Created on May 7, 2015, 12:24 PM
 */

#ifndef I2C_FUNC_H
#define	I2C_FUNC_H

#ifdef	__cplusplus
extern "C" {
#endif

//High Level Functions for Low Density Devices
unsigned char LDByteReadI2C(unsigned char, unsigned char, unsigned char*, unsigned char);
unsigned char LDByteWriteI2C(unsigned char, unsigned char, unsigned char);

#ifdef	__cplusplus
}
#endif

#endif	/* I2C_FUNC_H */

