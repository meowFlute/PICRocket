#define USE_AND_OR /* To enable AND_OR mask setting */
#include <p24FV32KA301.h>
#include "MPU6050.h"
#include "common.h"
#include "PWM_PID.h"
#include "declarations.h"
#include <stdio.h>
#include <stdlib.h>
#include <i2c.h>
#define timerCount 164

//offest and scalars for servos
#define servo1_Offest 0 
#define servo1_Scalar 0
#define servo2_Offest 0
#define servo2_Scalar 0
#define servo3_Offest 0
#define servo3_Scalar 0
#define servo4_Offest 0
#define servo4_Scalar 0 

#pragma config FWDTEN = OFF   // Watchdog Timer Enable bits (WDT disabled in hardware; SWDTEN bit disabled)
_FOSCSEL(FNOSC_FRC);          // 8MHz oscillator
_FICD(ICS_PGx3);              // Use debugging pins 9 & 10
_FOSC(OSCIOFNC_OFF);          // Disable clock output on pin 8

volatile unsigned int* PWMReg[] = {&OC1R, &OC2R, &OC3R, &OC4R};

void Setup_Timer1();
void Setup_I2C1(void);

int main()
{
    //Debug LED on RA3
    _TRISA3 = 0;
    _ANSA3 = 0;
    _LATA3 = 1;
    Actuate_Servo(0,75);
    Actuate_Servo(1,75);
    Actuate_Servo(2,75);
    Actuate_Servo(3,75);
    
    //Initialize the I2C connection
    Setup_I2C1();

    //Initialize the accelerometer
    Setup_MPU6050();

    Calibrate_Gyros();
    Zero_Sensors();
    Initialize_PID();

    Setup_Timer1();
    Setup_PWM();

    //Turn off LED when PIC Initialization is complete
    _LATA3 = 0;

    //Start the PID loop
    T1CONbits.TON = 1;

    Actuate_Servo(0,0);
    Actuate_Servo(1,0);
    Actuate_Servo(2,0);
    Actuate_Servo(3,0);
    
    while(1){}
    
    return (EXIT_SUCCESS);
}

void Setup_I2C1(void)
{
        UINT config1 = 0;
        UINT config2 = 0;

        //if open previously, disable the module
        CloseI2C1();
        ConfigIntI2C1(MI2C_INT_OFF);
        config1 = (I2C_ON | I2C_7BIT_ADD );
        config2 = 39;
        OpenI2C1(config1,config2);   //configure I2C1
}

void Setup_Timer1()
{
	T1CONbits.TON = 0; //Disable timer
	T1CONbits.TSIDL = 0; //Continue operation in idle mode
	T1CONbits.TGATE = 0; //Timer gate accumulation disabled
	T1CONbits.TCKPS = 0b11; //Timer prescale 1:1, 1:8, 1:64, 1:256
	//T1CONbits.T32 = 0; //32 bit timer disabled
	T1CONbits.TCS = 0; //Internal clock source

	_T1IP = 0b100; //Priority 4
        _T1IF = 1;
        _T1IE = 1;

	//Frequency of 400Hz
	PR1 = timerCount; //Period register

	TMR1=0;
}

void __attribute__((interrupt,no_auto_psv)) _MI2C1Interrupt(void)
{
  MI2C1_Clear_Intr_Status_Bit;  //Clear Interrupt status of I2C1
}

void __attribute__((interrupt,no_auto_psv)) _T1Interrupt()
{
    _T1IF = 0;  //clear interrupt flag
    _T1IE = 0;

    LATAbits.LATA2 ^= 1;
    Get_Gyro_Rates();
    Get_Accel_Values();

    Update_PID();
    Update_Servos();

    _T1IE = 1;
}

void __attribute__((interrupt,no_auto_psv)) _T3Interrupt()
{
    _T3IF = 0;
    _TMR3++;

    if (_TMR3 >= OC4R)
    {
        _LATA1 = 0;
    }
    else
    {
        _LATA1 = 1;
    }

    if(_TMR3>=_PR3)
    {
        _TMR3 = 0;
    }
}
