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
	adcuLinData linBuffer;
	while(1)
	{
		int length=0;
		length = adcuDevRead(deviceid,(uint8_t *)&linBuffer);
		if(length > 0)
		{
			packageCounter++;
			printf("RX %9d:",packageCounter);
			printf("lin:len = %d,dir = %d,ccs = %d,id = %02X(HEX),data = [ ",
					linBuffer.len,linBuffer.dir,linBuffer.ccs,linBuffer.id);
			for(int i=0;i<linBuffer.len;i++)
			{
				printf("%02X ",linBuffer.data[i]);  
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
	uint32_t delayTime;

	adcuDeviceType deviceType = adcuLIN;
	int channel;
	adcuLinData linData;
	adcuSDKInit();
	if(argc !=3)
	{
		printf("please input correct parameters<channelNumber,read/write>\n");
		return 0;
	}
	channel = atoi(argv[1]);
	if(channel <=0 || channel >= ADCU_CHANNEL_MAX)
	{
		printf("please input correct channel number<%d ~ %d>\n",ADCU_CHANNEL_1,ADCU_CHANNEL_MAX-1);
		return 0;
	}
	linData.dir = atoi(argv[2]);
	if(linData.dir !=0 && linData.dir != 1)
	{
		printf("please input correct lin data direction number<0:receive ,1:send>\n");
		return 0;
	}
	linData.len = 0x08; 
	linData.ccs = 0x00;
	linData.id = 0x07;
	linData.data[0] = 0x09;linData.data[1] = 0x09;linData.data[2] = 0x09;linData.data[3] = 0x09;
	linData.data[4] = 0x09;linData.data[5] = 0x09;linData.data[6] = 0x09;linData.data[7] = 0x09;

	len = sizeof(adcuLinData);
	memcpy(data,(uint8_t *)&linData,len);
	delayTime = 1;

	devid = adcuDevOpen(deviceType,(adcuDeviceChannel)channel);
	pthread_create(&ReadThread, NULL, readLoop, &devid);

	int status=0;

	while(1)
	{
		if(adcuDevStatus(devid) == ADCU_DEV_STATUS_ABNORMAL)
		{
			break;
		}
		for(int i = 0; i<1; i++)
		{
			if(adcuDevWrite(devid,data,len) <= 0)
			{
				printf("main write error\n");
				goto __end;
			}
		}
		sleep(delayTime);
	}

__end:
	printf("dinit all \n");
	adcuDevClose(devid);
	adcuSDKDeinit();
}
