#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include <string.h>
#include <pthread.h>
#include <string>
#include <semaphore.h>
#include <errno.h>



#include "adcuSDK.h"

using namespace std;

int main(int argc,char **argv)
{
	int devid;
	int packageCounter=0;
	adcuDeviceType deviceType;
	int channel;
	int length=0;
	adcuPwmData pwmBuffer;
	adcuSDKInit();
	if(argc !=2)
	{
		printf("please input correct parameters<channelNumber>\n");
		return 0;
	}
	deviceType = adcuPWM_IN;
	channel = atoi(argv[1]);
	if(channel <=0 || channel >= ADCU_CHANNEL_MAX)
	{
		printf("please input correct channel number<%d ~ %d>\n",ADCU_CHANNEL_1,ADCU_CHANNEL_MAX-1);
		return 0;
	}

	devid = adcuDevOpen(deviceType,(adcuDeviceChannel)channel);
	int status=0;


	while(1)
	{
		if(adcuDevStatus(devid) == ADCU_DEV_STATUS_ABNORMAL)
		{
			break;
		}

		length = adcuDevRead(devid,(uint8_t *)&pwmBuffer);
		if(length > 0)
		{
			packageCounter++;
			printf("RX %9d:",packageCounter);
			printf("pwm_frequency = %d,pwm_dutyCycle = %d\n",pwmBuffer.frequency,pwmBuffer.dutyCycle);
		}
	}
	printf("dinit all \n");
	adcuDevClose(devid);
	adcuSDKDeinit();
}
