#include "system.h"


********************************************************************************/

static bool systemTest(void);


/*�ײ�Ӳ����ʼ��*/
void systemInit(void)
{
	u8 cnt = 0;
	
	nvicInit();			/*�ж����ó�ʼ��*/
	extiInit();			/*�ⲿ�жϳ�ʼ��*/	
	delay_init(96);		/*delay��ʼ��*/
	ledInit();			/*led��ʼ��*/
	ledseqInit();		/*led�����г�ʼ��*/
	
	commInit();			/*ͨ�ų�ʼ��  STM32 & NRF51822 */
	atkpInit();			/*����Э���ʼ��*/
	consoleInit();		/*��ӡ��ʼ��*/
	printf("<--------------------->\n");
	
	configParamInit();	/*��ʼ�����ò���*/
	pmInit();			/*��Դ������ʼ��*/
	stabilizerInit();	/*��� ������ PID��ʼ��*/
	expModuleDriverInit();/*��չģ��������ʼ��*/
	
	if(systemTest() == true)
	{	
		while(cnt++ < 5)	/*��ʼ��ͨ�� �����̵ƿ���5��*/
		{
			ledFlashOne(LED_GREEN_L, 50, 50);
		}			
	}else
	{		
		while(1)		/*��ʼ������ ���Ϻ�Ƽ��1s����5��*/
		{
			if(cnt++ > 4)
			{
				cnt=0;
				delay_xms(1000);
			}
			ledFlashOne(LED_RED_R, 50, 50);		
		}
	}

	watchdogInit(WATCHDOG_RESET_MS);	/*���Ź���ʼ��*/
	
}
static bool systemTest(void)
{
	bool pass = true;
	
	pass &= ledseqTest();
	pass &= pmTest();
	pass &= configParamTest();
	pass &= commTest();
	pass &= stabilizerTest();	
	pass &= watchdogTest();
	
	return pass;
}

