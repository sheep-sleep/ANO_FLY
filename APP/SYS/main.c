#include "stm32f10x.h"
#include "BSP/BSP.H"
#include "app/uart/uart1.h"
#include "app/rc/rc.h"

#include "app/I2C_6050/I2C_MPU6050.h"		//定义6050初始化
#include "app/control/control.h"

#define CLI()      __set_PRIMASK(1)  
#define SEI()      __set_PRIMASK(0)

void SYS_INIT(void)
{
	LED_INIT();			//LED及串口IO 初始化
	LED_FLASH();		//LED闪烁
	Tim3_Init(500);	//中断初始化 //1000=1MS,500=0.5MS
	Moto_Init();	  //PWM
	
	//	Uart1_Init(115200);	//串口初始化，飞控上几乎无用
	 
	Spi1_Init();		//SPI初始化
	Nvic_Init();		//中断初始化
	Nrf24l01_Init(MODEL_TX2,40);	//2401中断初始化  主发送 通道 40
	
// 	if(Nrf24l01_Check())	Uart1_Put_String("NRF24L01 IS OK !\r\n");			//检测2401是否初始化成功
// 	else 									Uart1_Put_String("NRF24L01 IS NOT OK !\r\n");
	
	InitMPU6050();
	
	ADC1_Init();		//检测电池电压
	FLASH_Unlock();		//保存飞飞控参数
	EE_INIT();
	EE_READ_ACC_OFFSET();
	EE_READ_GYRO_OFFSET();
	EE_READ_PID();
	
	PID_ROL.P = PID_PIT.P = 5;	//用于初始化pid，如用匿名上位机写入pid，则屏蔽
	PID_ROL.D = PID_PIT.D = 0.1;			
	PID_YAW.P = 0.5;	
	PID_YAW.D = 0.05;			
}

int main(void)
{
	SYS_INIT_OK=0;	//初始化标志
	SYS_INIT();
	SYS_INIT_OK=1;

	while (1){}
}


