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
	adcuCanData canBuffer;
	while(1)
	{

		int length=0;
		length = adcuDevRead(deviceid,(uint8_t *)&canBuffer);
		if(length > 0)
		{
			packageCounter++;
			printf("RX %9d:",packageCounter);
			printf("id = %04X,dlc = %d,ide = %d,rtr = %d,prio = %d,data=[ ",
					canBuffer.id,canBuffer.dlc,canBuffer.ide,canBuffer.rtr,canBuffer.prio
					);
			for(int i=0;i<canBuffer.dlc;i++)
			{
				 printf("%02X ",canBuffer.can_data[i]);  
			}
			printf("]\n");
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
	if(argc !=2)
	{
		printf("please input correct parameters<channelNumber>\n");
		return 0;
	}
	channel = atoi(argv[1]);
	if(channel <=0 || channel >= ADCU_CHANNEL_MAX)
	{
		printf("please input correct channel number<%d ~ %d>\n",ADCU_CHANNEL_1,ADCU_CHANNEL_MAX-1);
		return 0;
	}

	bzero(data,100);
	adcuCanData canbuf;
	canbuf.ide = 0x00;
	canbuf.dlc = 0x08;
	canbuf.rtr = 0x00;
	canbuf.prio=0x00;
	canbuf.id = 0x123;
	canbuf.can_data[0] = 0x09;canbuf.can_data[1] = 0x09;canbuf.can_data[2] = 0x09;canbuf.can_data[3] = 0x09;
	canbuf.can_data[4] = 0x09;canbuf.can_data[5] = 0x09;canbuf.can_data[6] = 0x09;canbuf.can_data[7] = 0x09;

	deviceType = adcuCAN;
	len = sizeof(adcuCanData);
	memcpy(data,(uint8_t *)&canbuf,len);
	delayTime = 1000/60;

	devid = adcuDevOpen(deviceType,(adcuDeviceChannel)channel);
	pthread_create(&ReadThread, NULL, readLoop, &devid);

	int status=0;	

	while(1)
	{
		if(adcuDevStatus(devid) == ADCU_DEV_STATUS_ABNORMAL)
		{
			break;
		}
		for(int i = 0; i<3; i++)
		{
			canbuf.id = 0x101+ (channel -1) * 3 + i;
			bzero(data,100);
			memcpy(data,(uint8_t *)&canbuf,len);
			if(adcuDevWrite(devid,data,len) <= 0)
			{
				printf("main write error\n");
				goto __end;
			}
		}
		usleep(1000* delayTime);
	}

__end:
	printf("dinit all \n");
	adcuDevClose(devid);
	adcuSDKDeinit();   
}
