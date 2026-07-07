#ifndef CAMERA_HPP
#define CAMERA_HPP 

#include <stdio.h>
#include <string.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "MvCameraControl.h"

class Camera
{
public:
    void *handle;                        //!< 创建句柄，用于指向设备
    int nRet;                            //!< 函数返回码，成功为MV_OK(0x00000000)
    int color;                           //!< 当前颜色，此处为需要击打的敌方装甲板颜色
    int attack_mode;                     //!< 当前击打模式
    unsigned char *pDataForRGB;          //!< 指向取流成功的未经过cvtColor的RGB图片的指针
    MV_CC_DEVICE_INFO_LIST stDeviceList; //!< 设备列表，一般来说只有一个设备
    MV_FRAME_OUT stOutFrame;             //!< 输出的图像，从getImageBuffer中读取到的内容会被存放在这里
    MV_CC_PIXEL_CONVERT_PARAM CvtParam;  //!< 一个参数列表，用于储存ConvertPixelType时使用的全部参数
    MVCC_FLOATVALUE frame_rate;          //!< 获取的帧率信息，用于测试时了解当前帧率

    Camera();
    ~Camera();

};
#endif // CAMERA_HPP
