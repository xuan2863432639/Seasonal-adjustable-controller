/****************************************************************************
* 硬件连接：
* LoRa模块引脚连接
*  1--VDD--3.3V
*  3--GND--GND
*  5--PE1--RESET 可以不接
*  7--PD4--DIO0
* 11--PD7--DIO2
*
* 2--PA5--SCK
* 4--PB7--NSS
* 6--PA7--MOSI
* 8--PA6--MISO
*  ------------------------
* | PA9  - USART1(Tx)      |
* | PA10 - USART1(Rx)      |
*  ------------------------
*  ST-LINK
*  PA13--DIO
*  PA14--CLK
*******************************************************************************/
#include "sys_config.h"

#define BUFFER_SIZE     5                     // Define the payload size here
//#define VOLTAGE1        9                     //继电器动作电压
//#define VOLTAGE2        15.6                  //继电器最大电压
#define VOLTAGE1        9                     //继电器动作电压
#define VOLTAGE2        11                    //继电器最大电压

uint16_t BufferSize = BUFFER_SIZE;	          // RF buffer size
uint8_t  Buffer[BUFFER_SIZE]= {0};			  // RF buffer
uint8_t EnableMaster = false; 				  // Master/Slave selection

tRadioDriver *Radio = NULL;
uint8_t i=0;                        //清空Buffer数组时用的下标

float pitch_PV=0,roll=0,yaw=0; 		//欧拉角     俯仰角（Pitch）、横滚角（Roll）和偏航角（Yaw）
float pitch_SV=0;                   //接收到的俯仰角（pitch）
float err=0;                        //err=pitch_PV-pitch_SV    err=实测-接收
float abs_err=0;                    //误差的绝对值
float adc_value=0;                   //ADC 采样值
float adc_voltage=0;                 //ADC 电压值
float V_CAP=0;                       //电容电压
u8 key=0;                            //键值

bool voltage_ok=false;               //电压是否合适
bool auto_up=false;                  //自动模式，向上
bool auto_down=false;                //自动模式，向下
bool hand_up=false;                  //手动模式，向上
bool hand_down=false;                //手动模式，向下
bool auto_mode=false;                //是否是自动模式
bool forward_to_backward=true;       //自动模式下，正转切换到反转
bool backward_to_forward=true;       //自动模式下，反转切换到正转
bool east_limit=false;               //东限位  true:到达限位  false:未到达限位
bool west_limit=false;               //西限位  true:到达限位  false:未到达限位

union                                //通过共用体实现IEEE754与float之间的转换
{
    float a;
    char b[4];
} IEEE_to_float;


int main(void)
{
    Soft_delay_ms(100);          //初始化之前必须有个延时，这样的话充放电管理才可以正常初始化，断电后重新上电，才可以升压
    sys_Configuration();         //stm32 config
    BoardInit();                 //内部包含spi的初始化
    motor_Init();                //电机驱动初始化
    Adc_Init();                  //ADC初始化
    KEY_Init();                  //按键初始化
	IWDG_Init(6,625);            //溢出时间为4s

    Radio = RadioDriverInit( );
    Radio->Init( );
    Radio->StartRx( );           //RFLR_STATE_RX_INIT

    delay_init();	             //延时初始化
    MPU_Init();				   	 //初始化MPU6050
    mpu_dmp_init();              //初始化dmp

    while(1)
    {
		IWDG_Feed();                                      //喂狗
		LED1_TOGGLE;
        mpu_dmp_get_data(&pitch_PV,&roll,&yaw);           //通过dmp获得实测的俯仰角
		if(pitch_PV<25)
		{
			east_limit=true;
			west_limit=false;
		}else if(pitch_PV>45)
		{
			west_limit=true;
			east_limit=false;
		}else
		{
			east_limit=false;
			west_limit=false;
		}
        adc_value=Get_Adc_Average(ADC_Channel_1,3);       //adc采样值             ADC采样次数过多(比如10)，pitch_PV直为0
        adc_voltage=(float)adc_value*(3.3/4096);          //adc电压值             参考电压为VDDA 3.3V
        V_CAP=11*adc_voltage;                             //电容电压
        if(V_CAP>VOLTAGE2)                                //电容电压>继电器最大电压
        {
            voltage_ok=true;
        } else if(V_CAP<VOLTAGE1)                         //电容电压<继电器动作电压
        {
            voltage_ok=false;
        }

        if(Radio->Process( ) == RF_RX_DONE)               //接收完毕
        {
            LED0_TOGGLE;                                  //接收到数据包，LED0翻转
            Radio->GetRxPacket( Buffer, ( uint16_t* )&BufferSize );    //获取接收到的数据包并存在Buffer数组中
            IEEE_to_float.b[0]=Buffer[0];           //取出收到的pitch
            IEEE_to_float.b[1]=Buffer[1];
            IEEE_to_float.b[2]=Buffer[2];
            IEEE_to_float.b[3]=Buffer[3];
            pitch_SV=IEEE_to_float.a;
            for(i=0; i<BUFFER_SIZE; i++)
            {
                Buffer[i] = 0;
            }
            Radio->StartRx( );
        }

        err=pitch_PV-pitch_SV;                  //err=实测-接收
		abs_err=fabs(err);
        if(auto_mode==true)                     //自动模式下的处理
        {
            if(err>1)
            {
                if(forward_to_backward==true)   //避免正反转的直接切换
                {
                    motor_off();
//                    Soft_delay_ms(1000);
                    Soft_delay_ms(100);
                }
                auto_up=true;
                auto_down=false;
                forward_to_backward=false;
                backward_to_forward=true;
            } else if(err<(-1))
            {
                if(backward_to_forward==true)  //避免正反转的直接切换
                {
                    motor_off();
//                    Soft_delay_ms(1000);
                    Soft_delay_ms(100);
                }
                auto_down=true;
                auto_up=false;
                backward_to_forward=false;
                forward_to_backward=true;

            } else if(abs_err<0.2)
            {
                auto_up=false;
                auto_down=false;
            }
        }

        key=KEY_Scan(1);	//得到键值
        if(key)
        {
            switch(key)
            {
            case KEY0_PRES:	            //键值为1，左边的按钮
                LED0_TOGGLE;
                if(auto_mode==true)     //避免自动模式到手动模式切换时正反转直接切换
                {
                    motor_off();
//                    Soft_delay_ms(1000);
                    Soft_delay_ms(100);
                }
                hand_up=true;
                auto_up=false;
                auto_down=false;
                auto_mode=false;
                forward_to_backward=true;
                backward_to_forward=true;
                break;
            case KEY1_PRES:              //键值为2，右边的按钮
                LED0_TOGGLE;
                if(auto_mode==true)    //避免自动模式到手动模式切换时正反转直接切换
                {
                    motor_off();
//                    Soft_delay_ms(1000);
                    Soft_delay_ms(100);
                }
                hand_down=true;
                auto_up=false;
                auto_down=false;
                auto_mode=false;
                forward_to_backward=true;
                backward_to_forward=true;
                break;
            }
        } else
        {
            hand_up=false;
            hand_down=false;
            auto_mode=true;
            delay_ms(10);
        }

        if(voltage_ok)
        {
            if(((auto_up==true)||(hand_up==true))&&(east_limit==false))
            {
                motor_forward();
            } else if(((auto_down==true)||(hand_down==true))&&(west_limit==false))
            {
                motor_backward();
            } else
            {
                motor_off();
            }

        } else
        {
            motor_off();
        }

    }
}


/*********************************************END OF FILE**********************/
