#include "sys.h"
#include "delay.h"
#include "motors.h"
#include "pm.h"



static bool isInit = false;
u32 motor_ratios[] = {0, 0, 0, 0};
static const u32 MOTORS[] = { MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4 };



static u16 ratioToCCRx(u16 val)
{
	return ((val) >> (16 - MOTORS_PWM_BITS) & ((1 << MOTORS_PWM_BITS) - 1));
}

void motorsInit(void)	/*�����ʼ��*/
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB, ENABLE);	//ʹ��PORTA PORTBʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2|RCC_APB1Periph_TIM4,ENABLE);  	//TIM2��TIM4ʱ��ʹ��    
	
	TIM_DeInit(TIM4);	//���³�ʼ��TIM4ΪĬ��״̬
	TIM_DeInit(TIM2);	//���³�ʼ��TIM2ΪĬ��״̬
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_TIM4); 	//PB7 ����ΪTIM4 CH2	MOTOR1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4); 	//PB6 ����ΪTIM4 CH1	MOTOR2
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_TIM2); 	//PB10����ΪTIM2 CH3	MOTOR3
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_TIM2); 	//PA5 ����ΪTIM2 CH1	MOTOR4
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_10;	//PB6 7 10
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;        				//���ù���
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;				//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;      				//���츴�����
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;        				//����
	GPIO_Init(GPIOB,&GPIO_InitStructure);              				//��ʼ��PB6 7 10
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5;							//PA5
	GPIO_Init(GPIOA,&GPIO_InitStructure);              				//��ʼ��PA5		
	
	TIM_TimeBaseStructure.TIM_Period=MOTORS_PWM_PERIOD;			//�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_Prescaler=MOTORS_PWM_PRESCALE;	//��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;	//���ϼ���ģʽ	
	TIM_TimeBaseStructure.TIM_ClockDivision=0; 					//ʱ�ӷ�Ƶ
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;				//�ظ���������
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);				//��ʼ��TIM4
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);				//��ʼ��TIM2
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;				//PWMģʽ1
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;	//ʹ�����
	TIM_OCInitStructure.TIM_Pulse=0;							//CCRx
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;		//�ߵ�ƽ��Ч
	TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Set;	//���иߵ�ƽ	
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);  	//��ʼ��TIM4 CH2����Ƚ�
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);  	//��ʼ��TIM4 CH1����Ƚ�
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);  	//��ʼ��TIM2 CH3����Ƚ�
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);  	//��ʼ��TIM2 CH1����Ƚ�
	
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR2�ϵ�Ԥװ�ؼĴ���
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR1�ϵ�Ԥװ�ؼĴ���
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);  //ʹ��TIM2��CCR3�ϵ�Ԥװ�ؼĴ���
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);  //ʹ��TIM2��CCR1�ϵ�Ԥװ�ؼĴ���
 
	TIM_ARRPreloadConfig(TIM4,ENABLE);	//TIM4	ARPEʹ�� 
	TIM_ARRPreloadConfig(TIM2,ENABLE);	//TIM2	ARPEʹ�� 
	
	TIM_Cmd(TIM4, ENABLE);  //ʹ��TIM4
	TIM_Cmd(TIM2, ENABLE);  //ʹ��TIM2	

	isInit = true;
}

/*�������*/
bool motorsTest(void)
{
	int i;
	
	for (i = 0; i < sizeof(MOTORS) / sizeof(*MOTORS); i++)
	{	
		motorsSetRatio(MOTORS[i], MOTORS_TEST_RATIO);
		delay_xms(MOTORS_TEST_ON_TIME_MS);
		motorsSetRatio(MOTORS[i], 0);
		delay_xms(MOTORS_TEST_DELAY_TIME_MS);
	}

	return isInit;
}

extern bool isExitFlip;

/*���õ��PWMռ�ձ�*/
void motorsSetRatio(u32 id, u16 ithrust)
{
	if (isInit) 
	{
		u16 ratio=ithrust;

	#ifdef ENABLE_THRUST_BAT_COMPENSATED		
		if(isExitFlip == true)		/*500Hz*/
		{
			float thrust = ((float)ithrust / 65536.0f) * 60;
			float volts = -0.0006239f * thrust * thrust + 0.088f * thrust;
			float supply_voltage = pmGetBatteryVoltage();
			float percentage = volts / supply_voltage;
			percentage = percentage > 1.0f ? 1.0f : percentage;
			ratio = percentage * UINT16_MAX;
			motor_ratios[id] = ratio;
		}		
	#endif
		
		switch(id)
		{
			case 0:		/*MOTOR_M1*/
				TIM_SetCompare2(TIM4,ratioToCCRx(ratio));
				break;
			case 1:		/*MOTOR_M2*/
				TIM_SetCompare1(TIM4,ratioToCCRx(ratio));
				break;
			case 2:		/*MOTOR_M3*/
				TIM_SetCompare3(TIM2,ratioToCCRx(ratio));
				break;
			case 3:		/*MOTOR_M4*/	
				TIM_SetCompare1(TIM2,ratioToCCRx(ratio));
				break;
			default: break;
		}	
	}
}

