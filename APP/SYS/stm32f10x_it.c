#include "stm32f10x_it.h"
#include "bsp/bsp.h"
#include "app/uart/uart1.h"
#include "app/rc/rc.h"
#include "app/imu/imu.h"
#include "app/control/control.h"


#include "bsp/led.h"

void USART1_IRQHandler(void)  //串口中断函数
{
	Uart1_IRQ();
}
/*=====================================================================================================*/
/*=====================================================================================================*/
void TIM3_IRQHandler(void)		//0.5ms中断一次
{
	static u8 ms1 = 0,ms2 = 0,ms100 = 0;				//中断次数计数器
	if(TIM3->SR & TIM_IT_Update)		//if( TIM_GetITStatus(TIM3,TIM_IT_Update) != RESET ) 
	{
		TIM3->SR = ~TIM_FLAG_Update;	//TIM_ClearITPendingBit(TIM3,TIM_FLAG_Update);   //清除中断标志
		TIM3_IRQCNT ++; 
		if(!SYS_INIT_OK)	//初始化检查		
			return;
		//每次中断都执行,0.5ms一次中断
		ms1++;
		ms2++;
		ms100++;
		
		Nrf_Check_Event();//此句只能在中断中使用
		
		if(ms1==2)				//每两次中断执行一次,1ms
		{
			ms1=0;
			Prepare_Data();
		}
		
		if(ms2==4)				//每四次中断执行一次,2ms
		{
			ms2=0;
			Get_Attitude();		//姿态计算
			CONTROL(Q_ANGLE.X,Q_ANGLE.Y,Q_ANGLE.Z,RC_Target_ROL,RC_Target_PIT,RC_Target_YAW);
			NRF_Send_AF();		//发送 传感器与姿态数据+5通道数据
//		Uart1_Send_Buf(NRF24L01_TXDATA,32);		//串口给上位机返回 上述数据
			
			if(ARMED)		LED3_ON;	//解锁则亮LED3
		}
		
		if(ms100==200)	//10HZ
		{
			ms100 = 0;		
			Rc_Get.AUX5 = ADC_ConvertedValue;	//四轴电压参数
			NRF_Send_AE();		//发送遥控以及电机转速电压数据
			
			LEDALL_OFF	
		}	
			
	}
}


void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{

}