#include "motor.h"

void motor_Init(void)                                    //电机驱动初始化
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //使能PB端口时钟
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOA,GPIO_Pin_10|GPIO_Pin_11);
}

void motor_forward(void)                                  //电机正转
{
    relay1=0;
    relay2=1;
}

void motor_backward(void)                                  //电机反转
{
    relay1=1;
    relay2=0;
}

void motor_off(void)                                       //电机停
{
	relay1=0;
	relay2=0;
}