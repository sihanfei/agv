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


int main(int argc,char **argv)
{
    uint8_t buffer[1024];
    int devid;
	
    adcuDeviceType deviceType;
    int channel;
	int length=0;
	int packageCounter;
    adcuSDKInit();
    if(argc !=2)
    {
	    printf("please input correct parameters <channelNumber>\n");
	    return 0;
    }
	channel = atoi(argv[1]);
	if(channel <=0 || channel >= ADCU_CHANNEL_MAX)
	{
		printf("please input correct channel number<%d ~ %d>\n",ADCU_CHANNEL_1,ADCU_CHANNEL_MAX-1);
		return 0;
	}

	deviceType = adcuADC;
	
    devid = adcuDevOpen(deviceType,(adcuDeviceChannel)channel);
   
    int status=0;

    while(1)
    {
        if(adcuDevStatus(devid) == ADCU_DEV_STATUS_ABNORMAL)
        {
	        break;
        }
        
        length = adcuDevRead(devid,buffer,1024);
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

    printf("dinit all \n"); 
	adcuDevClose(devid);
    adcuSDKDeinit();   
}
