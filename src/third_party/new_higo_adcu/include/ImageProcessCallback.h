#ifndef __INAGE_PROCSS_CALLBACK_H__
#define __INAGE_PROCSS_CALLBACK_H__
/**
*! "brief image process call back function.
* this function will be called immediately when a image is caputured. 
* @param devId is the device ID which should be in range of 0~11
* @param buf is the image data captured by camera the format is RGB. the pix width is 4byte
* @see adcuAttacthCammeraCallback for more information.
* @retval nothing
*/
typedef void (*imageProcessCallback)(int devId,char *buf);

#endif
