#ifndef __MOTOR_H
#define __MOTOR_H
#include "sys.h"

#define relay1 PAout(10)       // PB0输出
#define relay2 PAout(11)       // PB1输出
void motor_Init(void);       //电机驱动初始化
void motor_forward(void);    //电机正转
void motor_backward(void);   //电机反转
void motor_off(void);        //电机停

#endif



