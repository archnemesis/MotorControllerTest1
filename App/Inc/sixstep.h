/*
 * sixstep.h
 *
 *  Created on: Nov 24, 2018
 *      Author: Robin
 */

#ifndef SIXSTEP_H_
#define SIXSTEP_H_

void SixStep_Init(void);
void SixStep_StartMotor(void);
void SixStep_StopMotor(void);
void SixStep_StartADC(void);
void SixStep_CommutationInterrupt(void);
void SixStep_Commutate(int step);
void SixStep_ADCInterrupt(void);
void SixStep_SetTimerSpeed(unsigned int rpm);
void SixStep_SetCurrentLimit(unsigned int current_limit);

#endif /* SIXSTEP_H_ */
