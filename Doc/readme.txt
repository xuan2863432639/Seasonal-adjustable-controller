SX1278�˿����ã�

ͨ�Žӿ�SPI1:
PA4: NSS
PA5: SCK
PA6: MISO
PA7: MOSI

��λ��PA1

��Ƶ����оƬ�շ��л���       CTRL  /CTRL
CTRL:  PA2                   1     0      RF_RX
/CTRL: PA3                   0     1      RF_TX

DIO0: PA0    �����շ��ж���
DIO1: PB1
DIO2: PB2
DIO3: PA8
DIO4: PA11
DIO5: PA12

LEDָʾ�ƣ� PB0



V1.01 
���̣�δ�޸�

V1.02
1 sx1276-Hal.c�ļ����޸�NSS��RST��DIO������
2 Main.c��ȥ��static�ؼ��֣�i��Ϊȫ�ֱ�����ע�͵�USART_putchar(USART1,Buffer[i])
3 �Ѽ��ٶȡ��Ƕȵȶ���Ϊȫ�ֱ���
4 Mpuiic.c�е�MPU_IIC_Init���޸�����  PB6  PB7
5 Mpuiic.h�е�IO����������IO��������
#define MPU_SDA_IN()  {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=(u32)8<<28;}
#define MPU_SDA_OUT() {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=(u32)3<<28;}
6 ע�͵� �Լ�
7 mpu_dmp_get_dataд��forѭ������
8 mpu_dmp_init()д��whileѭ������
9 Debugʱ����viewѡ���ѡ������ʾ

V1.03
1 ���ָʾ�Ʋ�������˸�����⣨�ı�LED0_TOGGLE��λ�ã�
2 ��������ʵ��
3 ADC����ʵ��
4 ��������ϵ粻����ѹ�����⣨�ս�main�������һ����ʱ��
5 ���������߶˲��룬USB�˰γ�����Ƭ���޷�����
6 ADC�����������ܹ��࣬����pitchһֱΪ0
7 stlink�̼��汾̫�͵Ļ������Ի�����⡣��һ�ο��Ե��ԣ��ڶ��ε���ʱ�򣬵�Ƭ��ֱ�����У�watch���ڵ�ȫ�ֱ���û�г�ʼ��Ϊ0�����ǹ̶�ֵ

V1.04
1 ���ӳ�������ָʾ�ƺͷ������ָʾ��
2 ��Ϊ��������

V1.05�������°��ӣ�
1 �޸�led����ΪPB14,PB15
2 �޸�sx1276-Hal.c�е�DIO4_PIN��DIO5_PINΪGPIO_Pin_13
3 �޸�motor�е�����ΪPA10,PA11
4 �޸�key�е�����ΪPA8,PA9
5 ɾ����ŵ����
6 ���ӿ��Ź�
7 ���Ӷ�����λ






