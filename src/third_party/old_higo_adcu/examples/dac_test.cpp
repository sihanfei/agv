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


	adcuDeviceType deviceType = adcuDAC;
	int channel;
	adcuSDKInit();
	if(argc !=3)
	{
		printf("please input correct parameters<channelNumber,voltage(mV)>\n");
		return 0;
	}
	channel = atoi(argv[1]);
	if(channel <=0 || channel >= ADCU_CHANNEL_MAX)
	{
		printf("please input correct channel number<%d ~ %d>\n",ADCU_CHANNEL_1,ADCU_CHANNEL_MAX-1);
		return 0;
	}
	adcuDacData dacData;
	dacData.voltage_mv = atoi(argv[2]);
	if(dacData.voltage_mv < 0 || dacData.voltage_mv>5000)
	{
		printf("please input correct dac output voltage(mV)<0 ~ 5000>\n");
		return 0;
	}
	len = sizeof(adcuDacData);
	memcpy(data,(uint8_t *)&dacData,len);
	delayTime = 0x0FFFFF;

	devid = adcuDevOpen(deviceType,(adcuDeviceChannel)channel);
	pthread_create(&ReadThread, NULL, readLoop, &devid);
	int status=0;


	while(1)
	{
		if(adcuDevStatus(devid) == ADCU_DEV_STATUS_ABNORMAL)
		{
			break;
		}
		if(adcuDevWrite(devid,data,len) <= 0)
		{
			printf("main write error\n");
			goto __end;
		}
		sleep(delayTime);
	}

__end:
	printf("dinit all \n");
	adcuDevClose(devid);
	adcuSDKDeinit();
}
