
#include "config.h"             //包含所有的驱动头文件
/*    
      ____                      _____                  +---+
     / ___\                     / __ \                 | R |
    / /                        / /_/ /                 +---+
   / /   ________  ____  ___  / ____/___  ____  __   __
  / /  / ___/ __ `/_  / / _ \/ /   / __ \/ _  \/ /  / /
 / /__/ /  / /_/ / / /_/  __/ /   / /_/ / / / / /__/ /
 \___/_/   \__,_/ /___/\___/_/    \___ /_/ /_/____  /
                                                 / /
                                            ____/ /
                                           /_____/
Tim.c file
编写者：小马  (Camel)、Nieyong
作者E-mail：375836945@qq.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2014-01-28
功能：
1.按键IO口初始化
2.这部分按键的功能是对外开放的，我没有定义 其功能，大家自由发挥吧
------------------------------------
*/

#include "config.h"

extern int  Pitch_Offest;
extern int  Roll_Offest;

/********************************************
              Key初始化函数
功能：
1.配置Key接口IO输入方向

描述：
Key接口：
Key1-->PB2
Key2-->PB1
Key3-->PB3
********************************************/
void KeyInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//EXTI_InitTypeDef EXTI_InitStructure;

	/* config the extiline(PB1,PB3,PA8) clock and AFIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,ENABLE);

	/* config the NVIC(Mode-->PB1) */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* config the NVIC(“＋”-->PB3) */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* config the NVIC(“－”-->PA8) */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Configure Mode-->PB1 as output push-pull */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Configure “＋”-->PB3 as output push-pull */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Configure “－”-->PA8 as output push-pull */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	
//	//Mode-->PB1
//	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);
//	EXTI_InitStructure.EXTI_Line = EXTI_Line1;            //设定外部中断1
//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;  //设定中断模式
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //设定下降沿触发模式
//	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//	EXTI_Init(&EXTI_InitStructure);

//	//“＋”-->PB3
//	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource3);
//	EXTI_InitStructure.EXTI_Line = EXTI_Line3;            //设定外部中断1
//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;  //设定中断模式
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //设定下降沿触发模式
//	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//	EXTI_Init(&EXTI_InitStructure);

//	//“－”-->PA8
//	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource8);
//	EXTI_InitStructure.EXTI_Line = EXTI_Line8;              //设定外部中断1
//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;     //设定中断模式
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //设定下降沿触发模式
//	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//	EXTI_Init(&EXTI_InitStructure);
	
}




extern char IMUcalibratflag;

//void EXTI1_IRQHandler(void){
//	if(EXTI_GetITStatus(EXTI_Line1) != RESET) //确保是否产生了EXTI Line中断
//	{
//    
//		ClibraFlag = FAIL;
//		 #ifdef UART_DEBUG
//	    	printf("key mode press...\r\n");
//		 #endif
//		
//		NRF24L01_SetTxAddr();

//   	EXTI_ClearITPendingBit(EXTI_Line1);     //清除中断标志位
//	}
//}

extern char Lockflag;

//“+”按键，控制电机是否待机转动
//void EXTI3_IRQHandler(void){
//	if(EXTI_GetITStatus(EXTI_Line3) != RESET) //确保是否产生了EXTI Line中断
//	{
//    Lockflag = 1;
//		 #ifdef UART_DEBUG
//	    	printf("key add press...\r\n");
//		 #endif

//		EXTI_ClearITPendingBit(EXTI_Line3);     //清除中断标志位
//		
//	}
//}

//“-”按键，控制IMU校准
//void EXTI9_5_IRQHandler(void){
//	if(EXTI_GetITStatus(EXTI_Line8) != RESET) //确保是否产生了EXTI Line中断
//	{	    
//		 IMUcalibratflag = !IMUcalibratflag;		
//		
//		 #ifdef UART_DEBUG
//	    	printf("key sub press...\r\n");
//		 #endif

//		EXTI_ClearITPendingBit(EXTI_Line8);     //清除中断标志位
//	}
//}
#define MODE_KEY GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1)
#define LOCK_KEY GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3)
#define CALIBRATION_KEY GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8)

enum
{
	RELEASE = 0,
	PRESS,
	
};
u8 modifyNrfAddressFlag;
u8 onlineModeFlag;
void keyCheck(void)
{
	static u8 modeKeyFlag, lockKeyFlag, calibrationKeyFlag;
	//mode key
	if(MODE_KEY == 0)
	{
		modeKeyFlag = PRESS;    //if press,save status
	}
	else
	{
		if(PRESS == modeKeyFlag)    //if last time is press, target once key function
		{
			if(Throttle < 1150 && Yaw < 1150 && Pitch < 1150 && Roll > 1850)    //matching Nrf address
			{
				modifyNrfAddressFlag = 1;	
			}
			else    //use online mode
			{
				modifyNrfAddressFlag = 0;
				onlineModeFlag = !onlineModeFlag;
			}
			
			//led instruction current status
			LedSet(led2,modifyNrfAddressFlag);    
			LedSet(led3,onlineModeFlag);
		}
		modeKeyFlag = RELEASE;  //if release, save state
	}
	// "+" key
	if(LOCK_KEY == 0)
	{
		lockKeyFlag = PRESS;
	}
	else
	{
		if(PRESS == lockKeyFlag)
		{
			Lockflag = 1;
		}
		lockKeyFlag = RELEASE;
	}
	//"-" key
	if(CALIBRATION_KEY == 0)
	{
		calibrationKeyFlag = PRESS;
	}
	else
	{
		if(PRESS == calibrationKeyFlag)
		{
			IMUcalibratflag = !IMUcalibratflag;
		}
		calibrationKeyFlag = RELEASE;
	}
}


