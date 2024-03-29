#include "luster_camera/luster_camera.hpp"

bool LusterCamera::g_bExit = false;  // 为静态成员变量提供初始值
cv::Mat LusterCamera::rgbImage;
std::mutex LusterCamera::luster_camera_buffer_mutex_;

LusterCamera::LusterCamera() : nh_("~")
{
    nh_.param("luster_camera_width", luster_camera_width, 1920);
    nh_.param("luster_camera_height", luster_camera_height, 1200);
    nh_.param("luster_camera_fps", luster_camera_fps, 30.0);
    nh_.param("luster_camera_gain", luster_camera_gain, 15.0);
}


int LusterCamera::grabImage()
{
    int nRet = MV_OK;

    void* handle = NULL;

    do {
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

        // 枚举设备
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
            break;
        }

        if (stDeviceList.nDeviceNum > 0)
        {
            for (int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                printf("[device %d]:\n", i);
                MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (NULL == pDeviceInfo)
                {
                    break;
                } 
                PrintDeviceInfo(pDeviceInfo);            
            }  
        } 
        else
        {
            printf("Find No Devices!\n");
            break;
        }

        // 选择设备并创建句柄
        unsigned int nIndex = 0;
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
            break;
        }

        // 打开设备
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
            break;
        }
		
        // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
        if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE)
        {
            int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
            if (nPacketSize > 0)
            {
                nRet = MV_CC_SetIntValue(handle,"GevSCPSPacketSize",nPacketSize);
                if(nRet != MV_OK)
                {
                    printf("Warning: Set Packet Size fail nRet [0x%x]!\n", nRet);
                }
            }
            else
            {
                printf("Warning: Get Packet Size fail nRet [0x%x]!\n", nPacketSize);
            }
        }

        // 设置相机图像高度
        unsigned int nHeightValue = luster_camera_height;  // 宽高设置时需考虑步进(16)，即设置宽高需16的倍数
        nRet = MV_CC_SetIntValue(handle, "Height", nHeightValue);    
        if (MV_OK == nRet)
            printf("set height %d OK!\n", nHeightValue);
        else
            printf("set height failed! nRet [%x]\n\n", nRet);

        // 设置相机图像宽度
        unsigned int nWidthValue = luster_camera_width;  // 宽高设置时需考虑步进(16)，即设置宽高需16的倍数
        nRet = MV_CC_SetIntValue(handle, "Width", nWidthValue);    
        if (MV_OK == nRet)
            printf("set width %d OK!\n", nWidthValue);
        else
            printf("set width failed! nRet [%x]\n\n", nRet);

        // 设置相机增益
        float fGainValue = luster_camera_gain;
        nRet = MV_CC_SetFloatValue(handle, "Gain", fGainValue);
        if (MV_OK == nRet)
            printf("set Gain %f OK!\n", fGainValue);
        else
            printf("set Gain failed! nRet [%x]\n\n", nRet);

        // 设置相机采集帧率
        float fFrameRateValue = luster_camera_fps;
        nRet = MV_CC_SetFrameRate(handle, fFrameRateValue);
        if (MV_OK == nRet)
            printf("set Frame Rate %f OK!\n", fFrameRateValue);
        else
            printf("set Frame Rate failed! nRet [%x]\n\n", nRet);

        // 获取相机实际采集帧率
        MVCC_FLOATVALUE stResultingFrameRate = {0};
        nRet = MV_CC_GetFloatValue(handle, "ResultingFrameRate", &stResultingFrameRate);
        if (MV_OK == nRet)
            printf("resulting frame rate current value:%f\n", stResultingFrameRate.fCurValue);
        else
            printf("get resulting frame rate failed! nRet [%x]\n\n", nRet);      
		
        // 设置触发模式为off
        nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
        if (MV_OK != nRet)
        {
            printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
            break;
        }

        // 开始取流
        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
            break;
        }

        pthread_t nThreadID;
        nRet = pthread_create(&nThreadID, NULL, WorkThread, handle);
        if (nRet != 0)
        {
            printf("thread create failed.ret = %d\n",nRet);
            break;
        }

        while(ros::ok() && !g_bExit) {
            sleep(1);
        }

        // 停止取流
        printf("exiting ...\n");
        nRet = MV_CC_StopGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
            break;
        }

        // 关闭设备
        nRet = MV_CC_CloseDevice(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
            break;
        }

        // 销毁句柄
        nRet = MV_CC_DestroyHandle(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
            break;
        }
    } while (0);

    if (nRet != MV_OK)
    {
        if (handle != NULL)
        {
            MV_CC_DestroyHandle(handle);
            handle = NULL;
        }
    }

    printf("exit.\n");
    return 0;
}


bool LusterCamera::PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("The Pointer of pstMVDevInfo is NULL!\n");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

        // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
        printf("CurrentIp: %d.%d.%d.%d\n" , nIp1, nIp2, nIp3, nIp4);
        printf("UserDefinedName: %s\n\n" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
        printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    }
    else
    {
        printf("Not support.\n");
    }

    return true;
}

// 用户获取当前时刻相机图像，进行后续处理
bool LusterCamera::returnImage(cv::Mat &img)
{
    // 线程锁
    std::unique_lock<std::mutex> lock(luster_camera_buffer_mutex_);
    if (! rgbImage.empty()) {
        img = rgbImage.clone();
    }
    lock.unlock();  // 获取完图像立刻释放锁，再进行后续处理

    return true;
}