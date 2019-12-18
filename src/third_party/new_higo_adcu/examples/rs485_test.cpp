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
	int channel;
	int len=16;
	int delayTime = 1000;
	uint8_t data[100]="12345678ABCDEFGH";
	uint8_t opt_buf[16];
	adcuSDKInit();
	adcuDeviceType deviceType;
	deviceType = adcuRS485;
	channel = ADCU_CHANNEL_1;

	devid = adcuDevOpen(deviceType,(adcuDeviceChannel)channel);

	pthread_create(&ReadThread, NULL, readLoop, &devid);

	int status=0;

	adcuUartOpt opt = 
	{
		.baudrate = 9600,
		.wordlength = 1,
		.parity = 4,
		.stopbit = 0
	};
	memcpy(opt_buf,&opt,sizeof(adcuUartOpt));
	adcuDevSetOpt(devid,opt_buf,sizeof(adcuUartOpt));
	sleep(1);

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
		usleep(1000* delayTime);
	}

__end:
	printf("dinit all \n");
	adcuDevClose(devid);
	adcuSDKDeinit();
}
