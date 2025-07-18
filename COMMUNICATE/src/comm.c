#include "comm.h"
#include "config.h"
#include "console.h"
#include "radiolink.h"
#include "usblink.h"


********************************************************************************/

static bool isInit;

void commInit(void)
{
	if (isInit) return;
	radiolinkInit();	/*����ͨ�ų�ʼ��*/
	usblinkInit();		/*USBͨ�ų�ʼ��*/
	isInit = true;
}

bool commTest(void)
{
  bool pass=isInit;
  
  pass &= consoleTest();
  
  return pass;
}

