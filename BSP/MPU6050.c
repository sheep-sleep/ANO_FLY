///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//����ĩ�μ��˱���������ɳ����������˼�Ļ��ͨ���������ʶ�������ø��ܶ������������������Ŷӣ�һ�����о����ᣡ
//��������������˼���û�У���˳������ֲ�͵��Զ����Լ�������ʱ���������ģ�Ҳ�������ʡ�
//������EEPW�����̳�Խ��Խ��������ڷ��׳��Լ��Ĵ��룬���Ǹ��õ�����
//��Ϊֻ�в��Ͻ���������ȡ�ø���Ľ�����������Ƥ�������������~ 
//    
//�����������Ҿ������Լ���ֲ���ԵĴ��뿪Դ�����������ѧϰ��ϣ�������һ�����������������������ǡ�
//��Ϊ���ǵ��׼�����һ���ģ�Ӳ������һ�µ��ˣ��Ǿ�ֻ��Ҫ�о������ϵ��㷨�����ˡ����������᷽��Ķ࣡
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//����DIY����飺http://www.eepw.com.cn/event/action/QuadCopter_DIY/
//
//������̳��http://forum.eepw.com.cn/forum/368/1 
//
//�ҵ�����DIY��������http://forum.eepw.com.cn/thread/248747/1
//
//�Ա����̣�http://item.taobao.com/item.htm?spm=a230r.1.14.23.sYD4gY&id=35605621244
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	BY:������ɣ�2014,4,30
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "MPU6050.h"
//#include "ANO_Tech_STM32F10x_I2C.h"
#include "tim.h"
#include "BSP.h"  //�������ⲿ���� EE_SAVE_ACC_OFFSET(); EE_SAVE_GYRO_OFFSET();

u8						mpu6050_buffer[14];					//iic��ȡ��������
S_INT16_XYZ		GYRO_OFFSET,ACC_OFFSET;			//��Ư
u8						GYRO_OFFSET_OK = 0;
u8						ACC_OFFSET_OK = 0;
S_INT16_XYZ		MPU6050_ACC_LAST,MPU6050_GYRO_LAST;		//����һ�ζ�ȡֵ

void Delay_ms_mpu(u16 nms)
{	
	u8 delay_cnt = TIM3_IRQCNT;
	while((delay_cnt+(nms*2)) > TIM3_IRQCNT);
} 
/**************************ʵ�ֺ���********************************************
//��iic��ȡ�������ݷֲ�,������Ӧ�Ĵ���
*******************************************************************************/
void MPU6050_Dataanl(void)
{
	MPU6050_ACC_LAST.X=((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]) - ACC_OFFSET.X;	//��ȥ��ƫ
	MPU6050_ACC_LAST.Y=((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]) - ACC_OFFSET.Y;
	MPU6050_ACC_LAST.Z=((((int16_t)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]) - ACC_OFFSET.Z;
	//�����¶�ADC
	MPU6050_GYRO_LAST.X=((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]) - GYRO_OFFSET.X;
	MPU6050_GYRO_LAST.Y=((((int16_t)mpu6050_buffer[10]) << 8) | mpu6050_buffer[11]) - GYRO_OFFSET.Y;
	MPU6050_GYRO_LAST.Z=((((int16_t)mpu6050_buffer[12]) << 8) | mpu6050_buffer[13]) - GYRO_OFFSET.Z;
	
	if(!GYRO_OFFSET_OK)	//������ƫ����
	{
		static int32_t	tempgx=0,tempgy=0,tempgz=0;
		static uint8_t cnt_g=0;
// 		LED1_ON;
		if(cnt_g==0)
		{
			GYRO_OFFSET.X=0;
			GYRO_OFFSET.Y=0;
			GYRO_OFFSET.Z=0;
			tempgx = 0;
			tempgy = 0;
			tempgz = 0;
			cnt_g = 1;	//��ֹ�Ժ����㾲̬����
			return;
		}
		tempgx+= MPU6050_GYRO_LAST.X;
		tempgy+= MPU6050_GYRO_LAST.Y;
		tempgz+= MPU6050_GYRO_LAST.Z;
		if(cnt_g==200)
		{
			GYRO_OFFSET.X=tempgx/cnt_g;
			GYRO_OFFSET.Y=tempgy/cnt_g;
			GYRO_OFFSET.Z=tempgz/cnt_g;
			cnt_g = 0;
			GYRO_OFFSET_OK = 1;
			EE_SAVE_GYRO_OFFSET();//��������
			return;
		}
		cnt_g++;
	}
	if(!ACC_OFFSET_OK)
	{
		static int32_t	tempax=0,tempay=0,tempaz=0;
		static uint8_t cnt_a=0;
// 		LED1_ON;
		if(cnt_a==0)
		{
			ACC_OFFSET.X = 0;
			ACC_OFFSET.Y = 0;
			ACC_OFFSET.Z = 0;
			tempax = 0;
			tempay = 0;
			tempaz = 0;
			cnt_a = 1;
			return;
		}
		tempax+= MPU6050_ACC_LAST.X;
		tempay+= MPU6050_ACC_LAST.Y;
		//tempaz+= MPU6050_ACC_LAST.Z;
		if(cnt_a==200)
		{
			ACC_OFFSET.X=tempax/cnt_a;
			ACC_OFFSET.Y=tempay/cnt_a;
			ACC_OFFSET.Z=tempaz/cnt_a;
			cnt_a = 0;
			ACC_OFFSET_OK = 1;
			EE_SAVE_ACC_OFFSET();//��������
			return;
		}
		cnt_a++;		
	}
}







/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_initialize(void)
*��������:	    ��ʼ�� 	MPU6050 �Խ������״̬��
*******************************************************************************/
void MPU6050_Init(void)
{
////	MPU6050_setSleepEnabled(0); //���빤��״̬
////	Delay_ms_mpu(200);
////	MPU6050_setClockSource(MPU6050_CLOCK_PLL_XGYRO); //����ʱ��  0x6b   0x01
////	Delay_ms_mpu(50);
////	MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);//������������� +-2000��ÿ��
////	Delay_ms_mpu(50);
////	MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_4);	//���ٶȶ�������� +-4G
////	Delay_ms_mpu(50);
////	MPU6050_setDLPF(MPU6050_DLPF_BW_42);
////	Delay_ms_mpu(50);
////	MPU6050_setI2CMasterModeEnabled(0);	 //����MPU6050 ����AUXI2C
////	Delay_ms_mpu(50);
////	MPU6050_setI2CBypassEnabled(1);	 //����������I2C��	MPU6050��AUXI2C	ֱͨ������������ֱ�ӷ���HMC5883L
////	Delay_ms_mpu(50);




}
///**************************ʵ�ֺ���********************************************
////	��д����������	ANO_Tech_I2C1_Read_Int
//*******************************************************************************/
//void ANO_Tech_I2C1_Read_Int(u8 Addr,u8 MPU6050,u8 num,u8 *mpu_buffer)
//{
//}
/**************************ʵ�ֺ���********************************************
//��iic��ȡ�������ݷֲ�,������Ӧ�Ĵ���,����MPU6050_Last
*******************************************************************************/
void MPU6050_Read(void)
{
//		ANO_Tech_I2C1_Read_Int(devAddr,MPU6050_RA_ACCEL_XOUT_H,14,mpu6050_buffer);
	
	
	
	
	
	
	
	
	
	
}

///**************************ʵ�ֺ���********************************************
////	��д����������	ANO_Tech_I2C1_Read_Buf
//*******************************************************************************/

///**************************ʵ�ֺ���********************************************
//*����ԭ��:		u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
//*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �е�1��λ
//����	dev  Ŀ���豸��ַ
//reg	   �Ĵ�����ַ
//bitNum  Ҫ�޸�Ŀ���ֽڵ�bitNumλ
//data  Ϊ0 ʱ��Ŀ��λ������0 ���򽫱���λ
//����   �ɹ� Ϊ1 
//ʧ��Ϊ0
//*******************************************************************************/ 
//void IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data){
//	u8 b;
//	ANO_Tech_I2C1_Read_Buf(dev, reg, 1, &b);
//	b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
//	ANO_Tech_I2C1_Write_1Byte(dev, reg, b);
//}
///**************************ʵ�ֺ���********************************************
//*����ԭ��:		u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
//*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �еĶ��λ
//����	dev  Ŀ���豸��ַ
//reg	   �Ĵ�����ַ
//bitStart  Ŀ���ֽڵ���ʼλ
//length   λ����
//data    ��Ÿı�Ŀ���ֽ�λ��ֵ
//����   �ɹ� Ϊ1 
//ʧ��Ϊ0
//*******************************************************************************/ 
//void IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
//{
//	
//	u8 b,mask;
//	ANO_Tech_I2C1_Read_Buf(dev, reg, 1, &b);
//	mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
//	data <<= (8 - length);
//	data >>= (7 - bitStart);
//	b &= mask;
//	b |= data;
//	ANO_Tech_I2C1_Write_1Byte(dev, reg, b);
//}
///**************************ʵ�ֺ���********************************************
//*����ԭ��:		void MPU6050_setClockSource(uint8_t source)
//*��������:	    ����  MPU6050 ��ʱ��Դ
//* CLK_SEL | Clock Source
//* --------+--------------------------------------
//* 0       | Internal oscillator
//* 1       | PLL with X Gyro reference
//* 2       | PLL with Y Gyro reference
//* 3       | PLL with Z Gyro reference
//* 4       | PLL with external 32.768kHz reference
//* 5       | PLL with external 19.2MHz reference
//* 6       | Reserved
//* 7       | Stops the clock and keeps the timing generator in reset
//*******************************************************************************/
//void MPU6050_setClockSource(uint8_t source){
//	IICwriteBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
//	
//}
///** Set full-scale gyroscope range.
//* @param range New full-scale gyroscope range value
//* @see getFullScaleRange()
//* @see MPU6050_GYRO_FS_250
//* @see MPU6050_RA_GYRO_CONFIG
//* @see MPU6050_GCONFIG_FS_SEL_BIT
//* @see MPU6050_GCONFIG_FS_SEL_LENGTH
//*/
//void MPU6050_setFullScaleGyroRange(uint8_t range) {
//	IICwriteBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
//}

///**************************ʵ�ֺ���********************************************
//*����ԭ��:		void MPU6050_setFullScaleAccelRange(uint8_t range)
//*��������:	    ����  MPU6050 ���ٶȼƵ��������
//*******************************************************************************/
//void MPU6050_setFullScaleAccelRange(uint8_t range) {
//	IICwriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
//}
///**************************ʵ�ֺ���********************************************
//*����ԭ��:		void MPU6050_setSleepEnabled(uint8_t enabled)
//*��������:	    ����  MPU6050 �Ƿ����˯��ģʽ
//enabled =1   ˯��
//enabled =0   ����
//*******************************************************************************/
//void MPU6050_setSleepEnabled(uint8_t enabled) {
//	IICwriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
//}

///**************************ʵ�ֺ���********************************************
//*����ԭ��:		void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
//*��������:	    ���� MPU6050 �Ƿ�ΪAUX I2C�ߵ�����
//*******************************************************************************/
//void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
//	IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
//}

///**************************ʵ�ֺ���********************************************
//*����ԭ��:		void MPU6050_setI2CBypassEnabled(uint8_t enabled)
//*��������:	    ���� MPU6050 �Ƿ�ΪAUX I2C�ߵ�����
//*******************************************************************************/
//void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
//	IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
//}

//void MPU6050_setDLPF(uint8_t mode)
//{
//	IICwriteBits(devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
//}
