#include "stm32f10x_it.h"
#include "bsp/bsp.h"
#include "app/uart/uart1.h"
#include "app/rc/rc.h"
#include "app/imu/imu.h"
#include "app/control/control.h"


#include "bsp/led.h"

void USART1_IRQHandler(void)  //�����жϺ���
{
	Uart1_IRQ();
}
/*=====================================================================================================*/
/*=====================================================================================================*/
void TIM3_IRQHandler(void)		//0.5ms�ж�һ��
{
	static u8 ms1 = 0,ms2 = 0,ms100 = 0;				//�жϴ���������
	if(TIM3->SR & TIM_IT_Update)		//if( TIM_GetITStatus(TIM3,TIM_IT_Update) != RESET ) 
	{
		TIM3->SR = ~TIM_FLAG_Update;	//TIM_ClearITPendingBit(TIM3,TIM_FLAG_Update);   //����жϱ�־
		TIM3_IRQCNT ++; 
		if(!SYS_INIT_OK)	//��ʼ�����		
			return;
		//ÿ���ж϶�ִ��,0.5msһ���ж�
		ms1++;
		ms2++;
		ms100++;
		
		Nrf_Check_Event();//�˾�ֻ�����ж���ʹ��
		
		if(ms1==2)				//ÿ�����ж�ִ��һ��,1ms
		{
			ms1=0;
			Prepare_Data();
		}
		
		if(ms2==4)				//ÿ�Ĵ��ж�ִ��һ��,2ms
		{
			ms2=0;
			Get_Attitude();		//��̬����
			CONTROL(Q_ANGLE.X,Q_ANGLE.Y,Q_ANGLE.Z,RC_Target_ROL,RC_Target_PIT,RC_Target_YAW);
			NRF_Send_AF();		//���� ����������̬����+5ͨ������
//		Uart1_Send_Buf(NRF24L01_TXDATA,32);		//���ڸ���λ������ ��������
			
			if(ARMED)		LED3_ON;	//��������LED3
		}
		
		if(ms100==200)	//10HZ
		{
			ms100 = 0;		
			Rc_Get.AUX5 = ADC_ConvertedValue;	//�����ѹ����
			NRF_Send_AE();		//����ң���Լ����ת�ٵ�ѹ����
			
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