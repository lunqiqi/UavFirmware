#include <string.h>
#include "sys.h"
#include "config.h"
#include "uart_syslink.h"
#include "debug_assert.h"
#include "config.h"

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

/********************************************************************************	 



#define UARTSLK_DATA_TIMEOUT_MS 	1000
#define UARTSLK_DATA_TIMEOUT_TICKS (UARTSLK_DATA_TIMEOUT_MS / portTICK_RATE_MS)
#define CCR_ENABLE_SET  ((u32)0x00000001)

static bool isInit = false;

static xSemaphoreHandle waitUntilSendDone;
static xSemaphoreHandle uartBusy;
static xQueueHandle uartslkDataDelivery;

static u8 dmaBuffer[64];
static u8 *outDataIsr;
static u8 dataIndexIsr;
static u8 dataSizeIsr;
static bool    isUartDmaInitialized;
static DMA_InitTypeDef DMA_InitStructure;
static u32 initialDMACount;
static u32 remainingDMACount;
static bool     dmaIsPaused;

static void uartslkPauseDma(void);
static void uartslkResumeDma(void);

/*���ô���DMA*/
void uartslkDmaInit(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);	

	/* USART TX DMA ͨ������*/
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&UARTSLK_TYPE->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)dmaBuffer;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_BufferSize = 0;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
	DMA_InitStructure.DMA_Channel = UARTSLK_DMA_CH;

	NVIC_InitStructure.NVIC_IRQChannel = UARTSLK_DMA_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	isUartDmaInitialized = true;
}

void uartslkInit(void)	/*���ڳ�ʼ��*/
{
	waitUntilSendDone = xSemaphoreCreateBinary(); 	/*�ȴ�������� ��ֵ�ź���*/
	uartBusy = xSemaphoreCreateBinary();			/*����æ ��ֵ�ź���*/
	xSemaphoreGive(uartBusy); 
	
	uartslkDataDelivery = xQueueCreate(1024, sizeof(u8));	/*���� 1024����Ϣ*/
	ASSERT(uartslkDataDelivery);

	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	/* ʹ��GPIO �� UART ʱ��*/
	RCC_AHB1PeriphClockCmd(UARTSLK_GPIO_PERIF, ENABLE);
	ENABLE_UARTSLK_RCC(UARTSLK_PERIF, ENABLE);

	/* ����USART RxΪ��������*/
	GPIO_InitStructure.GPIO_Pin   = UARTSLK_GPIO_RX_PIN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(UARTSLK_GPIO_PORT, &GPIO_InitStructure);

	/* ����USART Tx ���ù������*/
	GPIO_InitStructure.GPIO_Pin   = UARTSLK_GPIO_TX_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_Init(UARTSLK_GPIO_PORT, &GPIO_InitStructure);

	/*�˿�ӳ��*/
	GPIO_PinAFConfig(UARTSLK_GPIO_PORT, UARTSLK_GPIO_AF_TX_PIN, UARTSLK_GPIO_AF_TX);
	GPIO_PinAFConfig(UARTSLK_GPIO_PORT, UARTSLK_GPIO_AF_RX_PIN, UARTSLK_GPIO_AF_RX);


	USART_InitStructure.USART_BaudRate            = 1000000;
	USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;
	USART_InitStructure.USART_Parity              = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(UARTSLK_TYPE, &USART_InitStructure);

	uartslkDmaInit();

	/*���ô��ڷǿ��ж�*/
	NVIC_InitStructure.NVIC_IRQChannel = UARTSLK_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/*���ڽ������ݼĴ����ǿ��ж�*/
	USART_ITConfig(UARTSLK_TYPE, USART_IT_RXNE, ENABLE);	

	/*����TXEN����(NRF������)*/
	RCC_AHB1PeriphClockCmd(UARTSLK_TXEN_PERIF, ENABLE);

	memset(&GPIO_InitStructure, 0,sizeof(GPIO_InitStructure));
	GPIO_InitStructure.GPIO_Pin = UARTSLK_TXEN_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(UARTSLK_TXEN_PORT, &GPIO_InitStructure);

	/*PA0�ⲿ�ж�����*/
	EXTI_InitStructure.EXTI_Line = UARTSLK_TXEN_EXTI;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	EXTI_ClearITPendingBit(UARTSLK_TXEN_EXTI);

	NVIC_EnableIRQ(EXTI0_IRQn);

	USART_Cmd(UARTSLK_TYPE, ENABLE);	/*ʹ�ܶ�ʱ��*/
	isInit = true;
}

bool uartslkTest(void)
{
	return isInit;
}
/*�ӽ��ն��ж�ȡ����(����ʱ����)*/
bool uartslkGetDataWithTimout(u8 *c)
{
	/*����uartslkDataDelivery(1024������)��Ϣ*/
	if (xQueueReceive(uartslkDataDelivery, c, UARTSLK_DATA_TIMEOUT_TICKS) == pdTRUE)	
	{
		return true;
	}
	*c = 0;
	return false;
}
/*����ԭʼ����*/
void uartslkSendData(u32 size, u8* data)
{
	u32 i;

	if (!isInit) return;

	for(i = 0; i < size; i++)
	{
	#ifdef UARTSLK_SPINLOOP_FLOWCTRL
		while(GPIO_ReadInputDataBit(UARTSLK_TXEN_PORT, UARTSLK_TXEN_PIN) == Bit_SET);
	#endif
		while (!(UARTSLK_TYPE->SR & USART_FLAG_TXE))
		{};
		UARTSLK_TYPE->DR = (data[i] & 0x00FF);
	}
}
/*�жϷ�ʽ����ԭʼ����*/
void uartslkSendDataIsrBlocking(u32 size, u8* data)
{
	xSemaphoreTake(uartBusy, portMAX_DELAY);
	outDataIsr = data;
	dataSizeIsr = size;
	dataIndexIsr = 1;
	uartslkSendData(1, &data[0]);
	USART_ITConfig(UARTSLK_TYPE, USART_IT_TXE, ENABLE);	/*���ڷ������ݼĴ���Ϊ���ж�*/
	xSemaphoreTake(waitUntilSendDone, portMAX_DELAY);
	outDataIsr = 0;
	xSemaphoreGive(uartBusy);
}
/*����һ���ַ�������*/
int uartslkPutchar(int ch)
{
    uartslkSendData(1, (u8 *)&ch);
    
    return (u8)ch;
}
/*ͨ��DMA����ԭʼ����*/
void uartslkSendDataDmaBlocking(u32 size, u8* data)
{
	if (isUartDmaInitialized)
	{
		xSemaphoreTake(uartBusy, portMAX_DELAY);
		while(DMA_GetCmdStatus(UARTSLK_DMA_STREAM) != DISABLE);	/*�ȴ�DMA����*/
		memcpy(dmaBuffer, data, size);		/*�������ݵ�DMA������*/
		DMA_InitStructure.DMA_BufferSize = size;
		initialDMACount = size;
		DMA_Init(UARTSLK_DMA_STREAM, &DMA_InitStructure);	/*���³�ʼ��DMA������*/
		DMA_ITConfig(UARTSLK_DMA_STREAM, DMA_IT_TC, ENABLE);/*����DMA��������ж�*/		
		USART_DMACmd(UARTSLK_TYPE, USART_DMAReq_Tx, ENABLE);/* ʹ��USART DMA TX���� */
		USART_ClearFlag(UARTSLK_TYPE, USART_FLAG_TC);		/* �����������жϱ�־λ */
		DMA_Cmd(UARTSLK_DMA_STREAM, ENABLE);	/* ʹ��DMA USART TX������ */
		xSemaphoreTake(waitUntilSendDone, portMAX_DELAY);
		xSemaphoreGive(uartBusy);
	}
}
/*��ͣDMA����*/
static void uartslkPauseDma()
{
	if (DMA_GetCmdStatus(UARTSLK_DMA_STREAM) == ENABLE)
	{
		DMA_ITConfig(UARTSLK_DMA_STREAM, DMA_IT_TC, DISABLE);	/*�ر�DMA��������ж�*/	
		DMA_Cmd(UARTSLK_DMA_STREAM, DISABLE);
		while(DMA_GetCmdStatus(UARTSLK_DMA_STREAM) != DISABLE);
		DMA_ClearITPendingBit(UARTSLK_DMA_STREAM, UARTSLK_DMA_IT_TCIF);
		remainingDMACount = DMA_GetCurrDataCounter(UARTSLK_DMA_STREAM);
		dmaIsPaused = true;
	}
}
/*�ָ�DMA����*/
static void uartslkResumeDma()
{
	if (dmaIsPaused)
	{
		DMA_SetCurrDataCounter(UARTSLK_DMA_STREAM, remainingDMACount);	/*����DMA������*/
		UARTSLK_DMA_STREAM->M0AR = (u32)&dmaBuffer[initialDMACount - remainingDMACount];/*�����ڴ��ȡ��ַ*/
		DMA_ITConfig(UARTSLK_DMA_STREAM, DMA_IT_TC, ENABLE);	/*����DMA��������ж�*/	
		USART_ClearFlag(UARTSLK_TYPE, USART_FLAG_TC);	/* �����������жϱ�־λ */
		DMA_Cmd(UARTSLK_DMA_STREAM, ENABLE);	/* ʹ��DMA USART TX������ */
		dmaIsPaused = false;
	}
}

void uartslkDmaIsr(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	DMA_ITConfig(UARTSLK_DMA_STREAM, DMA_IT_TC, DISABLE);
	DMA_ClearITPendingBit(UARTSLK_DMA_STREAM, UARTSLK_DMA_IT_TCIF);
	USART_DMACmd(UARTSLK_TYPE, USART_DMAReq_Tx, DISABLE);
	DMA_Cmd(UARTSLK_DMA_STREAM, DISABLE);

	remainingDMACount = 0;
	xSemaphoreGiveFromISR(waitUntilSendDone, &xHigherPriorityTaskWoken);
}

void uartslkIsr(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	if ((UARTSLK_TYPE->SR & (1<<5)) != 0) /*���շǿ��ж�*/
	{
		u8 rxDataInterrupt = (u8)(UARTSLK_TYPE->DR & 0xFF);
		xQueueSendFromISR(uartslkDataDelivery, &rxDataInterrupt, &xHigherPriorityTaskWoken);
	}
	else if (USART_GetITStatus(UARTSLK_TYPE, USART_IT_TXE) == SET)
	{
		if (outDataIsr && (dataIndexIsr < dataSizeIsr))
		{
			USART_SendData(UARTSLK_TYPE, outDataIsr[dataIndexIsr] & 0x00FF);
			dataIndexIsr++;
		} else
		{
			USART_ITConfig(UARTSLK_TYPE, USART_IT_TXE, DISABLE);
			xSemaphoreGiveFromISR(waitUntilSendDone, &xHigherPriorityTaskWoken);
		}
	}
}

void uartslkTxenFlowctrlIsr()
{
	EXTI_ClearFlag(UARTSLK_TXEN_EXTI);
	if (GPIO_ReadInputDataBit(UARTSLK_TXEN_PORT, UARTSLK_TXEN_PIN) == Bit_SET)
	{
		uartslkPauseDma();
		//ledSet(LED_GREEN_R, 1);
	}
	else
	{
		uartslkResumeDma();
		//ledSet(LED_GREEN_R, 0);
	}
}

void __attribute__((used)) EXTI0_Callback(void)
{
	uartslkTxenFlowctrlIsr();
}

void __attribute__((used)) USART2_IRQHandler(void)	
{
	uartslkIsr();
}

void __attribute__((used)) DMA1_Stream6_IRQHandler(void)	
{
	uartslkDmaIsr();
}

