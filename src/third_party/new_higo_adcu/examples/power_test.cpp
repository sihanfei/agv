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
	uint8_t buffer[1024];
	adcuSDKInit();

	deviceType = adcuPOWER;
	channel = 1;

	printf("power_test use default channel 1\n");
	devid = adcuDevOpen(deviceType,(adcuDeviceChannel)channel);
	int status=0;


	while(1)
	{
		if(adcuDevStatus(devid) == ADCU_DEV_STATUS_ABNORMAL)
		{
			break;
		}

		length = adcuDevRead(devid,buffer);
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

__end:
	printf("dinit all \n");
	adcuDevClose(devid);
	adcuSDKDeinit();
}
