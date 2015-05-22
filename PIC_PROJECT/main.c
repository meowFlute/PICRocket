#define USE_AND_OR /* To enable AND_OR mask setting */
#include <p24FV32KA301.h>
#include "MPU6050.h"
#include "common.h"
#include "declarations.h"
#include <stdio.h>
#include <stdlib.h>
#include <i2c.h>
#define timerCount 164

#pragma config FWDTEN = OFF   // Watchdog Timer Enable bits (WDT disabled in hardware; SWDTEN bit disabled)
_FOSCSEL(FNOSC_FRC);          // 8MHz oscillator
_FICD(ICS_PGx3);              // Use debugging pins 9 & 10
_FOSC(OSCIOFNC_OFF);          // Disable clock output on pin 8

void Setup_Timer1();
void Setup_PWM();
void Setup_I2C1(void);
void Actuate_Servo(unsigned short servoNum, float angle);
void Update_Servos();
void Update_PID();
void Initialize_PID();

unsigned int OC4R, _TMR3, _PR3;
volatile unsigned int* const PWMReg[] = {&OC1R, &OC2R, &OC3R, &OC4R};

float rollKD;
float rollKP;
float pitchKD;
float pitchKP;
float yawKD;
float yawKP;

float rollDesired;
float pitchDesired;
float yawDesired;

float rollErr;
float rollDot;
float rollCommand;
float pitchErr;
float pitchDot;
float pitchCommand;
float yawErr;
float yawDot;
float yawCommand;

int main()
{
    //Debug LED on RA3
    _TRISA3 = 0;
    _ANSA3 = 0;
    _LATA3 = 1;

    //Initialize the I2C connection
    Setup_I2C1();

    //Initialize the accelerometer
    Setup_MPU6050();

    Calibrate_Gyros();
    Zero_Sensors();
    Initialize_PID();

    Setup_Timer1();
    Setup_PWM();

    _LATA3 = 0;
    
    while(1){}
    
    return (EXIT_SUCCESS);
}
void Actuate_Servo(unsigned short servoNum, float angle)
{
    /* Servo moves between 0 degrees and 180 degrees, 90 degrees is along rocket body */
    unsigned int OCR;
    switch(servoNum)
    {
        case 0:
        case 1:
        case 2:
            OCR = 315+(angle/180.0)*840;
            break;
        case 3:
            OCR = 20+(angle/180.0)*58;
    }

    *PWMReg[servoNum] = OCR;
}
void Setup_PWM()
{
    /* OC1: Pin11
     * OC2: Pin4
     * OC3: Pin5
     * All PWM pins use Timer 2
     * Simulated PWM uses Timer 3  */

    //Pin Setup
    _TRISA1 = 0;
    _TRISB0 = 0;
    _TRISB1 = 0;
    _TRISB7 = 0;
    _ANSA1 = 0;
    _ANSB0 = 0;
    _ANSB1 = 0;

    //Timer 2 Initialization
    T2CONbits.T32 = 0b0; //Use Timer2 as a 16 bit timer
    T2CONbits.TSIDL = 0b0; //Continue in Idle mode
    T2CONbits.TCS = 0b0; //Use internal clock
    T2CONbits.TGATE = 0b0; //Disable Gated Time Accumulation
    T2CONbits.TCKPS = 0b01; //Use 1:8 Prescaling mode
    PR2 = 10486;    //Period Value for a 50Hz timer with 1:8 prescaling mode and 8mhz clock

    //Timer 3 Initialization
    T3CONbits.TSIDL = 0b0; //Continue in Idle mode
    T3CONbits.TCS = 0b0; //Use internal clock
    T3CONbits.TGATE = 0b0; //Disable Gated Time Accumulation
    T3CONbits.TCKPS = 0b10; //Use 1:64 Prescaling mode
    _T3IE = 1;
    _T3IF = 0;
    _T3IP = 5;
    PR3 = 1;
    _PR3 = 655;
    
    //Edge-Aligned PWM mode
    OC1CON1bits.OCM = 0b110;
    OC2CON1bits.OCM = 0b110;
    OC3CON1bits.OCM = 0b110;

    //Select Timer 2
    OC1CON1bits.OCTSEL = 0b000;
    OC2CON1bits.OCTSEL = 0b000;
    OC3CON1bits.OCTSEL = 0b000;

    //Continue when CPU is idle
    OC1CON1bits.OCSIDL = 0b0;
    OC2CON1bits.OCSIDL = 0b0;
    OC3CON1bits.OCSIDL = 0b0;

    //Synchronize with Timer 2
    OC1CON2bits.SYNCSEL = 0b01100;
    OC2CON2bits.SYNCSEL = 0b01100;
    OC3CON2bits.SYNCSEL = 0b01100;

    //Initialize Duty Cycle for 1.5 ms pulse (90 degrees)
    OC1R = 786;
    OC2R = 786;
    OC3R = 786;
    OC4R = 49;

    //Start Timers
    T2CONbits.TON = 1;
    T3CONbits.TON = 1;
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
	T1CONbits.TON = 1; //Enable timer
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

//    Update_PID();
//    Update_Servos();

    _T1IE = 1;
}
void Initialize_PID()
{
    rollKD = .01;
    rollKP = .05;
    pitchKD = .1;
    pitchKP = .667;
    yawKD = .1;
    yawKP = .667;

    rollDesired = 0;
    pitchDesired = 0;
    yawDesired = 0;

    rollErr = 0;
    rollDot = 0;
    rollCommand = 0;
    pitchErr = 0;
    pitchDot = 0;
    pitchCommand = 0;
    yawErr = 0;
    yawDot = 0;
    yawCommand = 0;
}
void Update_PID()
{
    rollErr = rollDesired - GYRO_XANGLE;
    pitchErr = pitchDesired - GYRO_ZANGLE;
    yawErr = yawDesired - GYRO_YANGLE;

    rollDot = GYRO_XRATE;
    pitchDot = GYRO_ZRATE;
    yawDot = GYRO_YRATE;

    rollCommand = rollErr*rollKP; - rollDot*rollKD;
    pitchCommand = pitchErr*pitchKP - pitchDot*pitchKD;
    yawCommand = yawErr*yawKP - yawDot*yawKD;

    /* Make sure the roll command doesn't
     * overpower the pitch and yaw commands  */
    if (rollCommand > 2) rollCommand = 2;
    else if(rollCommand < -2) rollCommand = -2;
}
void Update_Servos()
{
    Actuate_Servo(0, pitchCommand + rollCommand);
    Actuate_Servo(1, yawCommand + rollCommand);
    Actuate_Servo(2, -pitchCommand + rollCommand);
    Actuate_Servo(3, -yawCommand + rollCommand);
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
