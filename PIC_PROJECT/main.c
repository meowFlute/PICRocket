/* 
 * File:   main.c
 * Author: Scott
 *
 * Created on May 5, 2015, 12:07 PM
 */

#define USE_AND_OR /* To enable AND_OR mask setting */
#include <p24FV32KA301.h>
#include "MPU6050.h"
#include "common.h"
#include "declarations.h"
#include <stdio.h>
#include <stdlib.h>
#include <i2c.h>
#define DELAY 64000
#define timerCount 16384



#pragma config FWDTEN = OFF   // Watchdog Timer Enable bits (WDT disabled in hardware; SWDTEN bit disabled)
_FOSCSEL(FNOSC_FRC);          // 8MHz oscillator

void Setup_Timer1();
void Setup_I2C1(void);
/*
 * MAIN CODE BEGINS
 */
int main(int argc, char** argv)
{
    int error = 1;
    ANSELA=0x0000;
    TRISAbits.TRISA2 = 0;
    TRISAbits.TRISA3 = 0;
    LATAbits.LATA2 = 1;
    LATAbits.LATA3 = 1;
    //Initialize the I2C connection
    Setup_I2C1();

    //Initialize the accelerometer
    do{
        Setup_MPU6050();
        MPU6050_Test_I2C();
        error = MPU6050_Check_Registers();
    }while(error==1);

    Calibrate_Gyros();
    Zero_Sensors();
    Setup_Timer1();
    IEC0bits.T1IE = 1;
    while(1)
    {

    }
    
    return (EXIT_SUCCESS);
}

void Setup_I2C1(void)
{
    //******EXAMPLE INITIALIZATION******
        UINT config1 = 0,i=0;
        UINT config2 = 0;

        //if open previously, disable the module
        CloseI2C1();
        ConfigIntI2C1(MI2C_INT_OFF);
        config1 = (I2C_ON | I2C_7BIT_ADD );
        config2 = 39;
        OpenI2C1(config1,config2);   //configure I2C1
    //******EXAMPLE INITIALIZATION******

        //TRISBbits.TRISB9 = 1;
	//TRISBbits.TRISB8 = 1;
	//This function will initialize the I2C(1) peripheral.

	//Set the I2C(1) BRG Baud Rate.
	//((8MHz)/(2*100KHz))-1 = 39 for a 100KHz I2C bus clock speed
	//I2C1BRG = 39;

        //disable interrupt I2C1 interrupts
        //MI2C1_Clear_Intr_Status_Bit;
        

	//Now we will initialise the I2C peripheral for Master Mode, No Slew Rate
	//Control, SMbus levels, and leave the peripheral switched off.
	//I2C1CONbits.I2CEN = 0;          // Disable I2C Mode
	//I2C1CONbits.I2CSIDL = 0;
	//I2C1CONbits.SCLREL = 1;
	//I2C1CONbits.IPMIEN = 0;
	//I2C1CONbits.A10M = 0;
	//I2C1CONbits.DISSLW = 1;         // Disable slew rate control
	//I2C1CONbits.SMEN = 0;
	//I2C1CONbits.GCEN = 0;
	//I2C1CONbits.STREN = 0;
	//I2C1CONbits.ACKDT = 0;
	//I2C1CONbits.RCEN = 0;
	//I2C1CONbits.PEN = 0;
	//I2C1CONbits.ACKEN = 0;
	//I2C1CONbits.RSEN = 0;
	//I2C1CONbits.SEN = 0;

	//Clearing the recieve and transmit buffers
	//I2C1RCV = 0x0000;
	//I2C1TRN = 0x0000;

	//Now we can enable the peripheral
	//I2C1CONbits.I2CEN = 1;
}

void Setup_Timer1()
{
	IEC0bits.T1IE = 0; //Disable timer1 interrupt
	T1CONbits.TON = 0; //Disable timer
	T1CONbits.TSIDL = 0; //Continue operation in idle mode
	T1CONbits.TGATE = 0; //Timer gate accumulation disabled
	T1CONbits.TCKPS = 0b11; //Timer prescale 1:1, 1:8, 1:64, 1:256
	//T1CONbits.T32 = 0; //32 bit timer disabled
	T1CONbits.TCS = 0; //Internal clock source
	IPC0bits.T1IP = 0b100; //Priority 4

	//Frequency of 400Hz
	PR1 = timerCount; //Period register

	TMR1=0;
	T1CONbits.TON = 1; //Enable timer
}

void __attribute__((interrupt,no_auto_psv)) _MI2C1Interrupt(void)
{
  MI2C1_Clear_Intr_Status_Bit;  //Clear Interrupt status of I2C1
}

void __attribute__((interrupt,no_auto_psv)) _T1Interrupt()
{
    _T1IF = 0;  //clear interrupt flag
    _T1IE = 0;  //Disable timer1 interrupt
    LATAbits.LATA2 ^= 1;
    Get_Gyro_Rates();
    Get_Accel_Values();
    Get_Accel_Angles();
    _T1IE = 1;

}