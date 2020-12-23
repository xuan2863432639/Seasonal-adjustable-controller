#include "motor.h"

void motor_Init(void)                                    //���������ʼ��
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //ʹ��PB�˿�ʱ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOA,GPIO_Pin_10|GPIO_Pin_11);
}

void motor_forward(void)                                  //�����ת
{
    relay1=0;
    relay2=1;
}

void motor_backward(void)                                  //�����ת
{
    relay1=1;
    relay2=0;
}

void motor_off(void)                                       //���ͣ
{
	relay1=0;
	relay2=0;
}