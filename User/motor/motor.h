#ifndef __MOTOR_H
#define __MOTOR_H
#include "sys.h"

#define relay1 PAout(10)       // PB0���
#define relay2 PAout(11)       // PB1���
void motor_Init(void);       //���������ʼ��
void motor_forward(void);    //�����ת
void motor_backward(void);   //�����ת
void motor_off(void);        //���ͣ

#endif



