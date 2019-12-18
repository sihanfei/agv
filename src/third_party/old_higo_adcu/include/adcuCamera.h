#ifndef __ADCU_CAMERA_H__
#define __ADCU_CAMERA_H__

#include "ImageProcessCallback.h"

/**
*! "brief this function is to initialize the runtime enviorenment if the sdk.
*! @param cameraMask is the bit map of camera. 
*   this mask should like this:"001100110011" which means the device which id is 2,3,6,7,10,11
*   will open for operation . plsease make sure all the required camaras are connected to the hardware correctly.
*   the cameras are divided into 3 group, one camera group containe 4 cameras,if one camera of the required cameras 
*   in a group is not connected correctly, all the camera in the group will fail to capure images.
*  @retval  -1 means failed 
*  @retval   0 means success.
*/
int adcuCameraInit(char *cameraMask);

/**
* ! "brief recycle the resource of the runtime enviorenment
*/
void adcuCameraDeinit();

/**
* ! "brief get a specific camera basic infomation.
* @param index is the device id
* @param width is the width of the image captured by this camera.
* @param height is the height of the image captured by this camera.
* @param pixlwidth is the width of a pixel.
* @retval  0 success
* @retval -1 something wrong happend.  
*/
int adcuGetCameraImageProp(int index,int *width,int *height,int *pixWidth);

/**
* ! "brief query the status of  a specific camera .
* @param index is the device id
* @retval 0 work normal
* @retval-1 something wrong happend.  
*/
int adcuGetCameraStatus(int index);

/**
* ! "brief attach the callback function for a specific camera .
* @param index is the device id
* @param callback is the way that camera return image to app.
*        this callback should process simple work. if you want to do some complicate works,please use another thread.
* @retval 0 success
* @retval -1 something wrong happend.  
*/
int adcuAttacthCammerCallback(int index,imageProcessCallback callback);

#endif
