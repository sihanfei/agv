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

pthread_t ReadThread;

void *readLoop(void *pdata)
{
	int deviceid = *(int *)pdata;
	int packageCounter=0;
	while(1)
	{
		uint8_t buffer[1024];
		int length=0;
		length = adcuDevRead(deviceid,buffer);
		if(length > 0)
		{
			packageCounter++;
			printf("RX %9d:",packageCounter);
			for(int i=0;i<length;i++)
			{
				printf("%02X ",buffer[i]);  
			}
			printf("\n");
		}
	}
}

int main(int argc,char **argv)
{
	int devid;

	uint8_t data[100];
	int len = 0;
	uint32_t delayTime = 0x0FFFFF;

	adcuDeviceType deviceType;
	int channel;
	adcuSDKInit();
	if(argc !=4)
	{
		printf("please input correct parameters<channelNumber,frequency,dutyCycle>\n");
		return 0;
	}

	channel = atoi(argv[1]);
	if(channel <=0 || channel >= ADCU_CHANNEL_MAX)
	{
		printf("please input correct channel number<%d ~ %d>\n",ADCU_CHANNEL_1,ADCU_CHANNEL_MAX-1);
		return 0;
	}

	bzero(data,100);	
	adcuPwmData pwmOutData;
	puts(argv[2]);
	puts(argv[3]);
	pwmOutData.frequency = atoi(argv[2]);
	pwmOutData.dutyCycle = atoi(argv[3]);
	if(pwmOutData.dutyCycle > 100 || pwmOutData.dutyCycle < 0)
	{
		printf("please input correct duty Cycle<0 ~ 100>\n");
		return 0;
	}

	deviceType = adcuPWM_OUT;
	len = sizeof(adcuPwmData);
	memcpy(data,(uint8_t *)&pwmOutData,len);

	devid = adcuDevOpen(deviceType,(adcuDeviceChannel)channel);
	pthread_create(&ReadThread, NULL, readLoop, &devid);

	while(1)
	{

		if(adcuDevStatus(devid) == ADCU_DEV_STATUS_ABNORMAL)
		{
			break;
		}
		if(adcuDevWrite(devid,data,len) <= 0)
		{
			printf("main write error\n");
			break;
		}
		sleep(delayTime);
	}

	printf("dinit all \n");
	adcuDevClose(devid);
	adcuSDKDeinit();

}
