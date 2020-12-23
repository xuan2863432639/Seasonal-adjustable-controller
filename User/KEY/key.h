#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"

//#define KEY0 PEin(4)   	//PE4
//#define KEY1 PEin(3)	//PE3 
//#define KEY2 PEin(2)	//PE2
//#define WK_UP PAin(0)	//PA0  WK_UP

#define KEY0  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8)//读取按键0
#define KEY1  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_9)//读取按键1

#define KEY0_PRES 	1	//KEY0按下
#define KEY1_PRES	2	//KEY1按下

void KEY_Init(void);//IO初始化
u8 KEY_Scan(u8);  	//按键扫描函数					    
#endif
