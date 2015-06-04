#include "common.h"
#include "PWM_PID.h"

void Actuate_Servo(unsigned short servoNum, float angle)
{
    /* Servo moves between 0 degrees and 180 degrees, 0 degrees is along rocket body */
    unsigned int OCR;
    switch(servoNum)
    {
        case 0:
            OCR = 824 + angle*4.6667;
            break;
        case 1:
            OCR = 735 - angle*4.6667;
            break;
        case 2:
            OCR = 705 + angle*4.6667;
            break;
        case 3:
            OCR = 52 - angle*0.3688;
            break;
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
    T3CONbits.TCKPS = 0b01; //Use 1:64 Prescaling mode
    _T3IE = 1;
    _T3IF = 0;
    _T3IP = 7;
    PR3 = 14;
    _PR3 = 749;
    
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

    //Start Timers
    T2CONbits.TON = 1;
    T3CONbits.TON = 1;
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

    rollCommand = rollErr*rollKP - rollDot*rollKD;
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
    Actuate_Servo(2, pitchCommand + rollCommand);
    Actuate_Servo(3, yawCommand + rollCommand);
}