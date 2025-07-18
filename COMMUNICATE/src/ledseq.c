#include <stdbool.h>
#include "led.h"
#include "ledseq.h"

/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "timers.h"
#include "semphr.h"

/********************************************************************************	 

********************************************************************************/

/* LED�������ȼ� */
static ledseq_t const * sequences[] = 
{
	seq_lowbat,
	seq_charged,
	seq_charging,
	seq_calibrated,
	seq_alive,
	seq_linkup,
};

/*Led ����*/
ledseq_t const seq_lowbat[] = 	/*��ص͵�ѹ����*/
{
	{ true, LEDSEQ_WAITMS(1000)},
	{    0, LEDSEQ_LOOP},
};
const ledseq_t seq_calibrated[] = /*������У׼�������*/
{
	{ true, LEDSEQ_WAITMS(50)},
	{false, LEDSEQ_WAITMS(450)},
	{    0, LEDSEQ_LOOP},
};
const ledseq_t seq_alive[] = 	/*��������*/
{
	{ true, LEDSEQ_WAITMS(50)},
	{false, LEDSEQ_WAITMS(1950)},
	{    0, LEDSEQ_LOOP},
};
const ledseq_t seq_linkup[] = 	/*ͨ����������*/
{
	{ true, LEDSEQ_WAITMS(1)},
	{false, LEDSEQ_WAITMS(0)},
	{    0, LEDSEQ_STOP},
};
const ledseq_t seq_charged[] = 	/*��س���������*/
{
	{ true, LEDSEQ_WAITMS(1000)},
	{    0, LEDSEQ_LOOP},
};
ledseq_t const seq_charging[] = /*��س�����������*/
{
	{ true, LEDSEQ_WAITMS(200)},
	{false, LEDSEQ_WAITMS(800)},
	{    0, LEDSEQ_LOOP},
};

#define SEQ_NUM (sizeof(sequences)/sizeof(sequences[0]))
	
static void updateActive(led_e led);		/*����led��������ȼ�����*/
static int getPrio(const ledseq_t *seq);	/*��ȡled���ȼ�*/
static void runLedseq(xTimerHandle xTimer);

static bool isInit = false;
static bool ledseqEnabled = true;
static int activeSeq[LED_NUM];		/*ÿ��LED��Ӧ�Ļ���ȼ�����*/
static int state[LED_NUM][SEQ_NUM];	/*ÿ��LED��Ӧ�����еĵ�ǰλ��*/

static xTimerHandle timer[LED_NUM];	/*��ʱ�����*/
static xSemaphoreHandle ledseqSem;	/*�ź���*/


void ledseqInit()
{
	int i,j;
	if(isInit) return;

	ledInit();
	
	/*��ʼ����������״̬*/
	for(i=0; i<LED_NUM; i++) 
	{
		activeSeq[i] = LEDSEQ_STOP;
		for(j=0; j<SEQ_NUM; j++)
			state[i][j] = LEDSEQ_STOP;
	}
	
	/*����������ʱ��*/
	for(i=0; i<LED_NUM; i++)
		timer[i] = xTimerCreate("ledseqTimer", 1000, pdFALSE, (void*)i, runLedseq);

	vSemaphoreCreateBinary(ledseqSem);	/*����һ��2ֵ�ź���*/

	isInit = true;
}

bool ledseqTest(void)
{
	bool status;

	status = isInit & ledTest();
	ledseqEnable(true);
	return status;
}

void ledseqEnable(bool enable)
{
	ledseqEnabled = enable;
}

void ledseqSetTimes(ledseq_t *sequence, int onTime, int offTime)
{
	sequence[0].action = onTime;
	sequence[1].action = offTime;
}

/*����led��sequence����*/
void ledseqRun(led_e led, const ledseq_t *sequence)
{
	int prio = getPrio(sequence);	/*��ȡled���ȼ�����*/

	if(prio<0) return;

	xSemaphoreTake(ledseqSem, portMAX_DELAY);
	state[led][prio] = 0; 
	updateActive(led);
	xSemaphoreGive(ledseqSem);

	if(activeSeq[led] == prio)	/*��ǰ�������ȼ����ڻ�������ȼ�*/
		runLedseq(timer[led]);
}

/*ֹͣled��sequence����*/
void ledseqStop(led_e led, const ledseq_t *sequence)
{
	int prio = getPrio(sequence);

	if(prio<0) return;

	xSemaphoreTake(ledseqSem, portMAX_DELAY);
	state[led][prio] = LEDSEQ_STOP;  
	updateActive(led);
	xSemaphoreGive(ledseqSem);

	runLedseq(timer[led]);
}

/*FreeRTOS ��ʱ���ص�����*/
static void runLedseq( xTimerHandle xTimer )
{
	bool leave = false;
	const ledseq_t *step;
	led_e led = (led_e)pvTimerGetTimerID(xTimer);

	if (!ledseqEnabled) return;

	while(!leave) 
	{
		int prio = activeSeq[led];

		if (prio == LEDSEQ_STOP)
			return;

		step = &sequences[prio][state[led][prio]];

		state[led][prio]++;

		xSemaphoreTake(ledseqSem, portMAX_DELAY);
		switch(step->action)
		{
			case LEDSEQ_LOOP:
				state[led][prio] = 0;
				break;
			case LEDSEQ_STOP:
				state[led][prio] = LEDSEQ_STOP;
				updateActive(led);
				break;
			default:  /*LED��ʱ*/
				ledSet(led, step->value);	/*��ʱstep->value*/
				if (step->action == 0)
					break;
				xTimerChangePeriod(xTimer, step->action, 0);
				xTimerStart(xTimer, 0);
				leave=true;
				break;
		}
		xSemaphoreGive(ledseqSem);
	}
}

/*��ȡled�������ȼ�*/
static int getPrio(const ledseq_t *seq)
{
	int prio;

	for(prio=0; prio<SEQ_NUM; prio++)
		if(sequences[prio]==seq) return prio;

	return -1; /*��Ч����*/
}

/*����led��������ȼ�����*/
static void updateActive(led_e led)
{
	int prio;

	ledSet(led, false);
	activeSeq[led]=LEDSEQ_STOP;
	
	for(prio=0;prio<SEQ_NUM;prio++)
	{
		if (state[led][prio] != LEDSEQ_STOP)
		{
			activeSeq[led]=prio;
			break;
		}
	}
}
