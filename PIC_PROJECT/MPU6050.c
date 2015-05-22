#include <i2c.h>
#include <math.h>
#define FCY 8000000UL
#include <libpic30.h>
#include "common.h"
#include "MPU6050.h"
#define gyro_xsensitivity 16 //(2^16-1)/4000 = 16.38
#define gyro_ysensitivity 16 //16 per deg/s
#define gyro_zsensitivity 16

//**************************************
//MPU6050 init
//returns the 0x68 for success
//0 for failure
//**************************************
UINT8 Setup_MPU6050(void)
{
    UINT8 Data;
    //Reset entire device
    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x80);
    __delay_ms(1);
    //it turns out that after reset, the device is put to sleep.
    //get it out of sleep mode
    LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x00);
    //ask the device who it is
    LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_WHO_AM_I, &Data, 1);

    if (Data == MPU6050_ID) {
        //Sets sample rate to 1000/1+1 = 500Hz
        //gyro output rate(1kHz w/ DLPF, 8kHz w/o)/(this value + 1) because 0x00 isn't 0
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_SMPLRT_DIV, 0x01);
        //LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_SMPLRT_DIV, &Data, 1);
	//Disable Frame Sync pin, 44Hz Digital Low Pass Filter with a processing delay of 4.9ms
        //0x00 = 260Hz
        //0x01 = 184Hz
        //0x02 = 94Hz
        //0x03 = 44Hz
        //0x04 = 21Hz
        //0x05 = 10Hz
        //0x06 = 5Hz
        //0x07 = RESERVED
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_CONFIG, 0x03);
        //LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_CONFIG, &Data, 1);
	//Disable gyro self tests, scale of 2000 degrees/s
        //0b00000000 = +- 250 deg/s
        //0b00001000 = +- 500 deg/s
        //0b00010000 = +- 1000 deg/s
        //0b00011000 = +- 2000 deg/s
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, 0b00011000);
        //LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, &Data, 1);
	//Disable accel self tests, scale of +-16g, no DHPF
        //0b00000000 = +- 2g
        //0b00001000 = +- 4g
        //0b00010000 = +- 8g
        //0b00011000 = +- 16
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0b00011000);
        //LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, &Data, 1);
	//Freefall threshold of <|0mg|
        //not in data sheet
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_FF_THR, 0x00);
	//Freefall duration limit of 0
        //not in data sheet
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_FF_DUR, 0x00);
	//Motion threshold of >0mg
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_MOT_THR, 0x00);
	//Motion duration of >0s
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_MOT_DUR, 0x00);
	//Zero motion threshold
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_ZRMOT_THR, 0x00);
	//Zero motion duration threshold
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_ZRMOT_DUR, 0x00);
	//Disable sensor output to FIFO buffer because sample rate is low
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_FIFO_EN, 0x00);

	//AUX I2C setup
	//Sets AUX I2C to single master control, plus other config
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_MST_CTRL, 0x00);
	//Setup AUX I2C slaves
 	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_ADDR, 0x00);
 	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_REG, 0x00);
 	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_CTRL, 0x00);
 	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_ADDR, 0x00);
 	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_REG, 0x00);
 	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_CTRL, 0x00);
 	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_ADDR, 0x00);
 	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_REG, 0x00);
 	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_CTRL, 0x00);
 	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_ADDR, 0x00);
 	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_REG, 0x00);
 	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_CTRL, 0x00);
 	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_ADDR, 0x00);
 	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_REG, 0x00);
 	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_DO, 0x00);
 	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_CTRL, 0x00);
 	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_DI, 0x00);


 	//MPU6050_RA_I2C_MST_STATUS //Read-only

 	//Setup INT pin and AUX I2C pass through
        //you can either pulse or hold the interrupt pin
        //
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, 0x00);
	//Enable data ready interrupt
        //disable is 0
        //bit6 is motion interrupt
        //bit4 is the FIFO buffer overflow
        //bit0 is data ready (each time a write to every sensor register is done)
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 0x00);

	//MPU6050_RA_DMP_INT_STATUS		//Read-only
	//MPU6050_RA_INT_STATUS 3A		//Read-only
    //MPU6050_RA_ACCEL_XOUT_H 		//Read-only
    //MPU6050_RA_ACCEL_XOUT_L 		//Read-only
    //MPU6050_RA_ACCEL_YOUT_H 		//Read-only
    //MPU6050_RA_ACCEL_YOUT_L 		//Read-only
    //MPU6050_RA_ACCEL_ZOUT_H 		//Read-only
    //MPU6050_RA_ACCEL_ZOUT_L 		//Read-only
    //MPU6050_RA_TEMP_OUT_H 		//Read-only
    //MPU6050_RA_TEMP_OUT_L 		//Read-only
    //MPU6050_RA_GYRO_XOUT_H 		//Read-only
    //MPU6050_RA_GYRO_XOUT_L 		//Read-only
    //MPU6050_RA_GYRO_YOUT_H 		//Read-only
    //MPU6050_RA_GYRO_YOUT_L 		//Read-only
    //MPU6050_RA_GYRO_ZOUT_H 		//Read-only
    //MPU6050_RA_GYRO_ZOUT_L 		//Read-only
    //MPU6050_RA_EXT_SENS_DATA_00 	//Read-only
    //MPU6050_RA_EXT_SENS_DATA_01 	//Read-only
    //MPU6050_RA_EXT_SENS_DATA_02 	//Read-only
    //MPU6050_RA_EXT_SENS_DATA_03 	//Read-only
    //MPU6050_RA_EXT_SENS_DATA_04 	//Read-only
    //MPU6050_RA_EXT_SENS_DATA_05 	//Read-only
    //MPU6050_RA_EXT_SENS_DATA_06 	//Read-only
    //MPU6050_RA_EXT_SENS_DATA_07 	//Read-only
    //MPU6050_RA_EXT_SENS_DATA_08 	//Read-only
    //MPU6050_RA_EXT_SENS_DATA_09 	//Read-only
    //MPU6050_RA_EXT_SENS_DATA_10 	//Read-only
    //MPU6050_RA_EXT_SENS_DATA_11 	//Read-only
    //MPU6050_RA_EXT_SENS_DATA_12 	//Read-only
    //MPU6050_RA_EXT_SENS_DATA_13 	//Read-only
    //MPU6050_RA_EXT_SENS_DATA_14 	//Read-only
    //MPU6050_RA_EXT_SENS_DATA_15 	//Read-only
    //MPU6050_RA_EXT_SENS_DATA_16 	//Read-only
    //MPU6050_RA_EXT_SENS_DATA_17 	//Read-only
    //MPU6050_RA_EXT_SENS_DATA_18 	//Read-only
    //MPU6050_RA_EXT_SENS_DATA_19 	//Read-only
    //MPU6050_RA_EXT_SENS_DATA_20 	//Read-only
    //MPU6050_RA_EXT_SENS_DATA_21 	//Read-only
    //MPU6050_RA_EXT_SENS_DATA_22 	//Read-only
    //MPU6050_RA_EXT_SENS_DATA_23 	//Read-only
    //MPU6050_RA_MOT_DETECT_STATUS 	//Read-only

	//Slave out, dont care
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_DO, 0x00);
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_DO, 0x00);
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_DO, 0x00);
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_DO, 0x00);
	//More slave config
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_MST_DELAY_CTRL, 0x00);
	//Reset sensor signal paths
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_SIGNAL_PATH_RESET, 0x00);
	//Motion detection control
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_MOT_DETECT_CTRL, 0x00);
	//Disables FIFO, AUX I2C, FIFO and I2C reset bits to 0
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, 0x00);
	//Sets clock source to gyro reference w/ PLL
        //apparently this is more effective than the internal oscilator
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0b00000010);
        LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, &Data, 1);
	//Controls frequency of wakeups in accel low power mode plus the sensor standby modes
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 0x00);
    //MPU6050_RA_BANK_SEL			//Not in datasheet
    //MPU6050_RA_MEM_START_ADDR		//Not in datasheet
    //MPU6050_RA_MEM_R_W			//Not in datasheet
    //MPU6050_RA_DMP_CFG_1			//Not in datasheet
    //MPU6050_RA_DMP_CFG_2			//Not in datasheet
    //MPU6050_RA_FIFO_COUNTH		//Read-only
    //MPU6050_RA_FIFO_COUNTL		//Read-only
	//Data transfer to and from the FIFO buffer
	LDByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_FIFO_R_W, 0x00);
    //MPU6050_RA_WHO_AM_I 			//Read-only, I2C address
    }
    else
        Data = 0x00;

    return Data;
}

//returns 0 for success
int MPU6050_Test_I2C()
{
	unsigned char Data = 0x00;
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_WHO_AM_I, &Data, 1);

	if(Data == 0x68)
	{
            return 0;
	}
	else
	{
            return 1;
	}
}

int MPU6050_Check_Registers()
{
	unsigned char Data = 0x00;
	unsigned char Failed = 0;

        LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_WHO_AM_I, &Data, 1);
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_SMPLRT_DIV, &Data, 1);
	if(Data != 0x01) Failed = 1;
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_CONFIG, &Data, 1);
	if(Data != 0x03) Failed = 1;
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, &Data, 1);
	if(Data != 0b00011000) Failed = 1;
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, &Data, 1);
	if(Data != 0b00011000) Failed = 1;
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_FF_THR, &Data, 1);
	if(Data != 0x00) Failed = 1;
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_FF_DUR, &Data, 1);
	if(Data != 0x00) Failed = 1;
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_MOT_THR, &Data, 1);
	if(Data != 0x00) Failed = 1;
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_MOT_DUR, &Data, 1);
	if(Data != 0x00) Failed = 1;
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_ZRMOT_THR, &Data, 1);
	if(Data != 0x00) Failed = 1;
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_ZRMOT_DUR, &Data, 1);
	if(Data != 0x00) Failed = 1;
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_FIFO_EN, &Data, 1);
	if(Data != 0x00) Failed = 1;
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_MST_CTRL, &Data, 1);
	if(Data != 0x00) Failed = 1;
 	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_ADDR, &Data, 1);
	if(Data != 0x00) Failed = 1;
 	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_REG, &Data, 1);
	if(Data != 0x00) Failed = 1;
 	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_CTRL, &Data, 1);
	if(Data != 0x00) Failed = 1;
 	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_ADDR, &Data, 1);
	if(Data != 0x00) Failed = 1;
 	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_REG, &Data, 1);
	if(Data != 0x00) Failed = 1;
 	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_CTRL, &Data, 1);
	if(Data != 0x00) Failed = 1;
 	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_ADDR, &Data, 1);
	if(Data != 0x00) Failed = 1;
 	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_REG, &Data, 1);
	if(Data != 0x00) Failed = 1;
 	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_CTRL, &Data, 1);
	if(Data != 0x00) Failed = 1;
 	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_ADDR, &Data, 1);
	if(Data != 0x00) Failed = 1;
 	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_REG, &Data, 1);
	if(Data != 0x00) Failed = 1;
 	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_CTRL, &Data, 1);
	if(Data != 0x00) Failed = 1;
 	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_ADDR, &Data, 1);
	if(Data != 0x00) Failed = 1;
 	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_REG, &Data, 1);
	if(Data != 0x00) Failed = 1;
 	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_DO, &Data, 1);
	if(Data != 0x00) Failed = 1;
 	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_CTRL, &Data, 1);
	if(Data != 0x00) Failed = 1;
 	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_DI, &Data, 1);
	if(Data != 0x00) Failed = 1;
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, &Data, 1);
	if(Data != 0x00) Failed = 1;
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, &Data, 1);
	if(Data != 0x00) Failed = 1;
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_DO, &Data, 1);
	if(Data != 0x00) Failed = 1;
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_DO, &Data, 1);
	if(Data != 0x00) Failed = 1;
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_DO, &Data, 1);
	if(Data != 0x00) Failed = 1;
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_DO, &Data, 1);
	if(Data != 0x00) Failed = 1;
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_MST_DELAY_CTRL, &Data, 1);
	if(Data != 0x00) Failed = 1;
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_SIGNAL_PATH_RESET, &Data, 1);
	if(Data != 0x00) Failed = 1;
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_MOT_DETECT_CTRL, &Data, 1);
	if(Data != 0x00) Failed = 1;
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, &Data, 1);
	if(Data != 0x00) Failed = 1;
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, &Data, 1);
	if(Data != 0x02) Failed = 1;
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, &Data, 1);
	if(Data != 0x00) Failed = 1;
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_FIFO_R_W, &Data, 1);
	if(Data != 0x00) Failed = 1;
	
	return(Failed);
}

void Calibrate_Gyros()
{
	int x = 0;
	GYRO_XOUT_OFFSET_1000SUM = 0;
	GYRO_YOUT_OFFSET_1000SUM = 0;
	GYRO_ZOUT_OFFSET_1000SUM = 0;
	for(x = 0; x<1000; x++)
	{
		LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_H, &GYRO_XOUT_H, 1);
		LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_L, &GYRO_XOUT_L, 1);
		LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_GYRO_YOUT_H, &GYRO_YOUT_H, 1);
		LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_GYRO_YOUT_L, &GYRO_YOUT_L, 1);
		LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_GYRO_ZOUT_H, &GYRO_ZOUT_H, 1);
		LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_GYRO_ZOUT_L, &GYRO_ZOUT_L, 1);

		GYRO_XOUT_OFFSET_1000SUM += ((GYRO_XOUT_H<<8)|GYRO_XOUT_L);
		GYRO_YOUT_OFFSET_1000SUM += ((GYRO_YOUT_H<<8)|GYRO_YOUT_L);
		GYRO_ZOUT_OFFSET_1000SUM += ((GYRO_ZOUT_H<<8)|GYRO_ZOUT_L);

		__delay_ms(2);
	}
	GYRO_XOUT_OFFSET = GYRO_XOUT_OFFSET_1000SUM/1000;
	GYRO_YOUT_OFFSET = GYRO_YOUT_OFFSET_1000SUM/1000;
	GYRO_ZOUT_OFFSET = GYRO_ZOUT_OFFSET_1000SUM/1000;
}

//Gets raw accelerometer data, performs no processing
void Get_Accel_Values()
{
    //Read in the individual bytes
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, &ACCEL_XOUT_H, 1);
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_L, &ACCEL_XOUT_L, 1);
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_ACCEL_YOUT_H, &ACCEL_YOUT_H, 1);
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_ACCEL_YOUT_L, &ACCEL_YOUT_L, 1);
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_ACCEL_ZOUT_H, &ACCEL_ZOUT_H, 1);
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_ACCEL_ZOUT_L, &ACCEL_ZOUT_L, 1);
    //Read shift the high bits over 8 and then OR that with the low bits
	ACCEL_XOUT = ((ACCEL_XOUT_H<<8)|ACCEL_XOUT_L);
	ACCEL_YOUT = ((ACCEL_YOUT_H<<8)|ACCEL_YOUT_L);
	ACCEL_ZOUT = ((ACCEL_ZOUT_H<<8)|ACCEL_ZOUT_L);
}

//Converts the already acquired accelerometer data into 3D euler angles
void Get_Accel_Angles()
{
	//ACCEL_XANGLE = 57.295*atan((float)ACCEL_YOUT/ sqrt(pow((float)ACCEL_ZOUT,2)+pow((float)ACCEL_XOUT,2)))*a + (1-a)*ACCEL_XANGLE;
	//ACCEL_YANGLE = 57.295*atan((float)-ACCEL_XOUT/ sqrt(pow((float)ACCEL_ZOUT,2)+pow((float)ACCEL_YOUT,2)))*a + (1-a)*ACCEL_YANGLE;

	//ACCEL_XANGLE = 57.295*atan((float)ACCEL_YOUT/ sqrt(pow((float)ACCEL_ZOUT,2)+pow((float)ACCEL_XOUT,2)));
	ACCEL_XANGLE = 57.295*atan((float)ACCEL_XOUT/ sqrt(pow((float)ACCEL_ZOUT,2)+pow((float)ACCEL_YOUT,2)));
        ACCEL_YANGLE = 57.295*atan((float)ACCEL_ZOUT/ sqrt(pow((float)ACCEL_XOUT,2)+pow((float)ACCEL_YOUT,2)));
        ACCEL_ZANGLE = 57.295*atan((float)ACCEL_YOUT/ sqrt(pow((float)ACCEL_XOUT,2)+pow((float)ACCEL_ZOUT,2)));
}

//Function to read the gyroscope rate data and convert it into degrees/s
void Get_Gyro_Rates()
{
    //Read in the individual bytes
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_H, &GYRO_XOUT_H, 1);
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_L, &GYRO_XOUT_L, 1);
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_GYRO_YOUT_H, &GYRO_YOUT_H, 1);
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_GYRO_YOUT_L, &GYRO_YOUT_L, 1);
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_GYRO_ZOUT_H, &GYRO_ZOUT_H, 1);
	LDByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_GYRO_ZOUT_L, &GYRO_ZOUT_L, 1);
    //Read shift the high bits over 8 and then OR that with the low bits
	GYRO_XOUT = ((GYRO_XOUT_H<<8)|GYRO_XOUT_L) - GYRO_XOUT_OFFSET;
	GYRO_YOUT = ((GYRO_YOUT_H<<8)|GYRO_YOUT_L) - GYRO_YOUT_OFFSET;
	GYRO_ZOUT = ((GYRO_ZOUT_H<<8)|GYRO_ZOUT_L) - GYRO_ZOUT_OFFSET;


	GYRO_XRATE = (float)GYRO_XOUT/gyro_xsensitivity;
	GYRO_YRATE = (float)GYRO_YOUT/gyro_ysensitivity;
	GYRO_ZRATE = (float)GYRO_ZOUT/gyro_zsensitivity;

	GYRO_XANGLE += GYRO_XRATE*dt;
	GYRO_YANGLE += GYRO_YRATE*dt;
	GYRO_ZANGLE += GYRO_ZRATE*dt;
}

#if 0
void vTask_Gyro_MPU6050(void *pvParameters )
{
    UINT8 a,b;
    INT16 acc_x, acc_y, acc_z, temp;
    INT16 gyro_x, gyro_y, gyro_z;
    portTickType xLastWakeTime;
    int distance;

    Sonar_HCSR04_Init();
    xLastWakeTime = xTaskGetTickCount();
    for( ;; )
    {
        vTaskDelayUntil(&xLastWakeTime, 500);
        distance = Sonar_HCSR04_DetectRange();

        speed = (INT8)(distance%40);
        StepMoter_28BYJ48_Speed(speed);
    }
}
#endif

/*********************************************************************
* Function:        ACKStatus()
*
* Input:		None.
*
* Output:		Acknowledge Status.
*
* Overview:		Return the Acknowledge status on the bus
*
* Note:			None
********************************************************************/
unsigned char ACKStatus(void)
{
	return (!I2C2STATbits.ACKSTAT);		//Return Ack Status
}

/*********************************************************************
* Function:        LDByteWriteI2C()
*
* Input:		Control Byte, 8 - bit address, data.
*
* Output:		None.
*
* Overview:		Write a byte to low density device at address LowAdd
*
* Note:			None
********************************************************************/
unsigned char LDByteWriteI2C(unsigned char ControlByte, unsigned char LowAdd, unsigned char data)
{
	unsigned char ErrorCode;

        int i;
        //********************** START **********************************************************************
        IdleI2C1();                             //Ensure Module is Idle
        StartI2C1();                            //Generate Start COndition
        while(I2C1CONbits.SEN );                //wait to send start bit
        MI2C1_Clear_Intr_Status_Bit;

        //**********************WRITE THE DEVICE I2C ADDRESS*************************************************
        MasterWriteI2C1(ControlByte);           //Write Control byte (this is the physical address of the slave)
        while(I2C1STATbits.TBF);                //Wait till address is transmitted
        while(!IFS1bits.MI2C1IF);               //Wait for ninth clock cycle
        while(I2C1STATbits.ACKSTAT);            //Wait for acknowledgment
        MI2C1_Clear_Intr_Status_Bit;            //Clear interrupt flag

        //for(i=0;i<1000;i++);
        //IdleI2C1();                           //wait for an idle
        //ErrorCode = ACKStatus();              //Return ACK Status
        //**********************WRITE THE REGIST ADDRESS THE DEVICE HOLDS************************************
        MasterWriteI2C1(LowAdd);                //Write register address
        while(I2C1STATbits.TBF);                //Wait till address is transmitted
        while(!IFS1bits.MI2C1IF);               //Wait for ninth clock cycle
        while(I2C1STATbits.ACKSTAT);            //Wait for acknowledgment
        MI2C1_Clear_Intr_Status_Bit;            //Clear interrupt flag

        //for(i=0;i<1000;i++);
        //IdleI2C1();                           //wait for an idle
	//ErrorCode = ACKStatus();		//Return ACK Status
        
        //**********************WRITE THE DATA TO DEVICE*****************************************************
	MasterWriteI2C1(data);                  //Write Data
        while(I2C1STATbits.TBF);                //Wait till address is transmitted
        while(!IFS1bits.MI2C1IF);               //Wait for ninth clock cycle
        while(I2C1STATbits.ACKSTAT);            //Wait for acknowledgment
        MI2C1_Clear_Intr_Status_Bit;            //Clear interrupt flag

        //for(i=0;i<1000;i++);
        //********************** STOP ***********************************************************************
	StopI2C1();				//Initiate Stop Condition
        while(I2C1CONbits.PEN);                 //Wait till stop sequence is completed
	// Regis EEAckPolling(ControlByte);	//Perform ACK polling
	return(ErrorCode);                      //If you want to know more about the error?
                                                //currently returns nothing and just gets stuck
                                                //if there is no ACK bit returned
}


/*********************************************************************
* Function:        LDByteReadI2C()
*
* Input:		Control Byte, Address, *Data, Length.
*
* Output:		None.
*
* Overview:		Performs a low density read of Length bytes and stores in *Data array
*				starting at Address.
*
* Note:			None
********************************************************************/
void LDByteReadI2C(unsigned char ControlByte, unsigned char Address, unsigned char *Data, unsigned char Length)
{
    int i;
    //********************** START **********************************************************************
    IdleI2C1();                             //Ensure Module is Idle
    StartI2C1();                            //Generate Start COndition
    while(I2C1CONbits.SEN );                //wait to send start bit
    MI2C1_Clear_Intr_Status_Bit;
    
    //**********************WRITE THE DEVICE I2C ADDRESS*************************************************
    MasterWriteI2C1(ControlByte);           //Write Control byte (this is the physical address of the slave)
    while(I2C1STATbits.TBF);                //Wait till address is transmitted
    while(!IFS1bits.MI2C1IF);               //Wait for ninth clock cycle
    while(I2C1STATbits.ACKSTAT);            //Wait for acknowledgment
    MI2C1_Clear_Intr_Status_Bit;            //Clear interrupt flag

    //for(i=0;i<1000;i++);
    //IdleI2C1();                           //wait for an idle
    //ErrorCode = ACKStatus();              //Return ACK Status
    //**********************WRITE THE REGIST ADDRESS THE DEVICE HOLDS************************************
    MasterWriteI2C1(Address);               //Write register address
    while(I2C1STATbits.TBF);                //Wait till address is transmitted
    while(!IFS1bits.MI2C1IF);               //Wait for ninth clock cycle
    while(I2C1STATbits.ACKSTAT);            //Wait for acknowledgment
    MI2C1_Clear_Intr_Status_Bit;            //Clear interrupt flag

    //********************** RESTART ********************************************************************
    //IdleI2C1();
    RestartI2C1();
    while(I2C1CONbits.RSEN );               //wait to send start bit
    //for(i=0;i<1000;i++);

    //**********************WRITE THE DEVICE I2C ADDRESS*************************************************
    Nop();
    MI2C1_Clear_Intr_Status_Bit;
    MasterWriteI2C1(ControlByte | 0x01);    //Write slave address OR'd with read bit
    while(I2C1STATbits.TBF);                //Wait till address is transmitted
    while(!IFS1bits.MI2C1IF);               //Wait for ninth clock cycle
    while(I2C1STATbits.ACKSTAT);            //Wait for acknowledgment
    MI2C1_Clear_Intr_Status_Bit;            //Clear interrupt flag
    
    //********************** RECEIVE SLAVE DATA STRING **************************************************
    MastergetsI2C1(Length, Data, 1000);     //read Length number of bytes
                                            //I am fairly certain this function sends a NACK bit at the end
                                            //It has to wait a bit to return if the slave takes a while for some reason

    //********************** STOP ***********************************************************************
    NotAckI2C1();
    IdleI2C1();
    StopI2C1();                             //Initiate Stop Condition
    while(I2C1CONbits.PEN);                 //Wait till stop sequence is completed
}

void Zero_Sensors(void)
{
	float BUFFER_ZANGLE = 0;
	float BUFFER_YANGLE = 0;
        float BUFFER_XANGLE = 0;
	int x = 0;
	for(x=0; x<1000; x++)
	{
		Get_Accel_Values();
		BUFFER_ZANGLE += ACCEL_ZOUT;
		BUFFER_YANGLE += ACCEL_YOUT;
                BUFFER_XANGLE += ACCEL_XOUT;
		__delay_ms(1);
	}


	GYRO_ZANGLE = 57.2958*atan(BUFFER_YANGLE/BUFFER_XANGLE);
	GYRO_YANGLE = 0;
        GYRO_XANGLE = 57.2958*atan(BUFFER_ZANGLE/BUFFER_YANGLE);
}
