SX1278端口配置：

通信接口SPI1:
PA4: NSS
PA5: SCK
PA6: MISO
PA7: MOSI

复位：PA1

射频开关芯片收发切换：       CTRL  /CTRL
CTRL:  PA2                   1     0      RF_RX
/CTRL: PA3                   0     1      RF_TX

DIO0: PA0    可做收发中断用
DIO1: PB1
DIO2: PB2
DIO3: PA8
DIO4: PA11
DIO5: PA12

LED指示灯： PB0



V1.01 
例程，未修改

V1.02
1 sx1276-Hal.c文件中修改NSS、RST、DIO等引脚
2 Main.c中去掉static关键字，i改为全局变量，注释掉USART_putchar(USART1,Buffer[i])
3 把加速度、角度等定义为全局变量
4 Mpuiic.c中的MPU_IIC_Init中修改引脚  PB6  PB7
5 Mpuiic.h中的IO操作函数及IO方向设置
#define MPU_SDA_IN()  {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=(u32)8<<28;}
#define MPU_SDA_OUT() {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=(u32)3<<28;}
6 注释掉 自检
7 mpu_dmp_get_data写在for循环外面
8 mpu_dmp_init()写在while循环外面
9 Debug时候在view选项卡勾选周期显示

V1.03
1 解决指示灯不正常闪烁的问题（改变LED0_TOGGLE的位置）
2 按键功能实现
3 ADC功能实现
4 解决重新上电不能升压的问题（刚进main函数后加一个延时）
5 调试器排线端插入，USB端拔出，单片机无法供电
6 ADC采样次数不能过多，否则pitch一直为0
7 stlink固件版本太低的话，调试会出问题。第一次可以调试，第二次调试时候，单片机直接运行，watch窗口的全局变量没有初始化为0，都是固定值

V1.04
1 增加程序运行指示灯和发送完成指示灯
2 改为连续接收

V1.05（适配新板子）
1 修改led引脚为PB14,PB15
2 修改sx1276-Hal.c中的DIO4_PIN和DIO5_PIN为GPIO_Pin_13
3 修改motor中的引脚为PA10,PA11
4 修改key中的引脚为PA8,PA9
5 删除充放电管理
6 增加看门狗
7 增加东西限位






