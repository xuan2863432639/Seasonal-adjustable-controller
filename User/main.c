/****************************************************************************
* Ӳ�����ӣ�
* LoRaģ����������
*  1--VDD--3.3V
*  3--GND--GND
*  5--PE1--RESET ���Բ���
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
//#define VOLTAGE1        9                     //�̵���������ѹ
//#define VOLTAGE2        15.6                  //�̵�������ѹ
#define VOLTAGE1        9                     //�̵���������ѹ
#define VOLTAGE2        11                    //�̵�������ѹ

uint16_t BufferSize = BUFFER_SIZE;	          // RF buffer size
uint8_t  Buffer[BUFFER_SIZE]= {0};			  // RF buffer
uint8_t EnableMaster = false; 				  // Master/Slave selection

tRadioDriver *Radio = NULL;
uint8_t i=0;                        //���Buffer����ʱ�õ��±�

float pitch_PV=0,roll=0,yaw=0; 		//ŷ����     �����ǣ�Pitch��������ǣ�Roll����ƫ���ǣ�Yaw��
float pitch_SV=0;                   //���յ��ĸ����ǣ�pitch��
float err=0;                        //err=pitch_PV-pitch_SV    err=ʵ��-����
float abs_err=0;                    //���ľ���ֵ
float adc_value=0;                   //ADC ����ֵ
float adc_voltage=0;                 //ADC ��ѹֵ
float V_CAP=0;                       //���ݵ�ѹ
u8 key=0;                            //��ֵ

bool voltage_ok=false;               //��ѹ�Ƿ����
bool auto_up=false;                  //�Զ�ģʽ������
bool auto_down=false;                //�Զ�ģʽ������
bool hand_up=false;                  //�ֶ�ģʽ������
bool hand_down=false;                //�ֶ�ģʽ������
bool auto_mode=false;                //�Ƿ����Զ�ģʽ
bool forward_to_backward=true;       //�Զ�ģʽ�£���ת�л�����ת
bool backward_to_forward=true;       //�Զ�ģʽ�£���ת�л�����ת
bool east_limit=false;               //����λ  true:������λ  false:δ������λ
bool west_limit=false;               //����λ  true:������λ  false:δ������λ

union                                //ͨ��������ʵ��IEEE754��float֮���ת��
{
    float a;
    char b[4];
} IEEE_to_float;


int main(void)
{
    Soft_delay_ms(100);          //��ʼ��֮ǰ�����и���ʱ�������Ļ���ŵ����ſ���������ʼ�����ϵ�������ϵ磬�ſ�����ѹ
    sys_Configuration();         //stm32 config
    BoardInit();                 //�ڲ�����spi�ĳ�ʼ��
    motor_Init();                //���������ʼ��
    Adc_Init();                  //ADC��ʼ��
    KEY_Init();                  //������ʼ��
	IWDG_Init(6,625);            //���ʱ��Ϊ4s

    Radio = RadioDriverInit( );
    Radio->Init( );
    Radio->StartRx( );           //RFLR_STATE_RX_INIT

    delay_init();	             //��ʱ��ʼ��
    MPU_Init();				   	 //��ʼ��MPU6050
    mpu_dmp_init();              //��ʼ��dmp

    while(1)
    {
		IWDG_Feed();                                      //ι��
		LED1_TOGGLE;
        mpu_dmp_get_data(&pitch_PV,&roll,&yaw);           //ͨ��dmp���ʵ��ĸ�����
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
        adc_value=Get_Adc_Average(ADC_Channel_1,3);       //adc����ֵ             ADC������������(����10)��pitch_PVֱΪ0
        adc_voltage=(float)adc_value*(3.3/4096);          //adc��ѹֵ             �ο���ѹΪVDDA 3.3V
        V_CAP=11*adc_voltage;                             //���ݵ�ѹ
        if(V_CAP>VOLTAGE2)                                //���ݵ�ѹ>�̵�������ѹ
        {
            voltage_ok=true;
        } else if(V_CAP<VOLTAGE1)                         //���ݵ�ѹ<�̵���������ѹ
        {
            voltage_ok=false;
        }

        if(Radio->Process( ) == RF_RX_DONE)               //�������
        {
            LED0_TOGGLE;                                  //���յ����ݰ���LED0��ת
            Radio->GetRxPacket( Buffer, ( uint16_t* )&BufferSize );    //��ȡ���յ������ݰ�������Buffer������
            IEEE_to_float.b[0]=Buffer[0];           //ȡ���յ���pitch
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

        err=pitch_PV-pitch_SV;                  //err=ʵ��-����
		abs_err=fabs(err);
        if(auto_mode==true)                     //�Զ�ģʽ�µĴ���
        {
            if(err>1)
            {
                if(forward_to_backward==true)   //��������ת��ֱ���л�
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
                if(backward_to_forward==true)  //��������ת��ֱ���л�
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

        key=KEY_Scan(1);	//�õ���ֵ
        if(key)
        {
            switch(key)
            {
            case KEY0_PRES:	            //��ֵΪ1����ߵİ�ť
                LED0_TOGGLE;
                if(auto_mode==true)     //�����Զ�ģʽ���ֶ�ģʽ�л�ʱ����תֱ���л�
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
            case KEY1_PRES:              //��ֵΪ2���ұߵİ�ť
                LED0_TOGGLE;
                if(auto_mode==true)    //�����Զ�ģʽ���ֶ�ģʽ�л�ʱ����תֱ���л�
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
