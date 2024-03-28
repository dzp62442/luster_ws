#pragma once

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include "luster_camera/MvCameraControl.h"
#include <opencv2/opencv.hpp>

class LusterCamera
{
private:
    static bool g_bExit;
    static cv::Mat rgbImage;
    static std::mutex luster_camera_buffer_mutex_;

public:
    LusterCamera() {}
    ~LusterCamera() {}

public:

    int grabImage();
    bool returnImage(cv::Mat &image);
    bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo);
    
    static void* WorkThread(void* pUser)
    {
        int nRet = MV_OK;

        // ch:获取数据包大小 | en:Get payload size
        MVCC_INTVALUE stParam;
        memset(&stParam, 0, sizeof(MVCC_INTVALUE));
        nRet = MV_CC_GetIntValue(pUser, "PayloadSize", &stParam);
        if (MV_OK != nRet)
        {
            printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
            return NULL;
        }

        MV_FRAME_OUT_INFO_EX stImageInfo = {0};
        memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
        unsigned char * pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
        if (NULL == pData)
        {
            return NULL;
        }
        unsigned int nDataSize = stParam.nCurValue;

        while(ros::ok() && !g_bExit)
        {
            nRet = MV_CC_GetOneFrameTimeout(pUser, pData, nDataSize, &stImageInfo, 1000);
            if (nRet == MV_OK)
            {
                printf("GetOneFrame, Width[%d], Height[%d], nFrameNum[%d]\n", 
                    stImageInfo.nWidth, stImageInfo.nHeight, stImageInfo.nFrameNum);
                std::cout << stImageInfo.enPixelType << "\n";
                // 对于BayerRG8格式的图像
                if (stImageInfo.enPixelType == PixelType_Gvsp_BayerRG8) {
                    cv::Mat bayerImage(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC1, pData);
                    std::unique_lock<std::mutex> lock(luster_camera_buffer_mutex_);  // 线程锁
                    cv::cvtColor(bayerImage, rgbImage, cv::COLOR_BayerRG2RGB);  // 转换Bayer图像为RGB图像
                    lock.unlock();  // 解锁
                }
            }
            else{
                printf("No data[%x]\n", nRet);
            }
        }

        free(pData);
        return 0;
    }

};