/* 
 * File:   PWM.h
 * Author: Scott
 *
 * Created on June 3, 2015, 5:20 PM
 */

#ifndef PWM_H
#define	PWM_H

#ifdef	__cplusplus
extern "C" {
#endif

void Actuate_Servo(unsigned short servoNum, float angle);
void Setup_PWM();
void Initialize_PID();
void Update_PID();
void Update_Servos();

#ifdef	__cplusplus
}
#endif

#endif	/* PWM_H */

