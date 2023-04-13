#ifndef CAMERA_HPP
#define CAMERA_HPP
#include "ros/ros.h"
#include <stdio.h>
#include <pthread.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include "MvErrorDefine.h"
#include "CameraParams.h"
#include "MvCameraControl.h"

namespace camera {
//********** define ************************************/
#define MAX_IMAGE_DATA_SIZE (4 * 2048 * 3072)
//********** frame ************************************/

//********** frame_empty ******************************/
bool frame_empty = 0;
//********** mutex ************************************/
pthread_mutex_t mutex;
//********** CameraProperties config ************************************/
enum CamerProperties {
  CAP_PROP_FRAMERATE_ENABLE,  //帧数可调
  CAP_PROP_FRAMERATE,         //帧数
  CAP_PROP_BURSTFRAMECOUNT,   //外部一次触发帧数
  CAP_PROP_HEIGHT,            //图像高度
  CAP_PROP_WIDTH,             //图像宽度
  CAP_PROP_EXPOSURE_TIME,     //曝光时间
  CAP_PROP_GAMMA_ENABLE,      //伽马因子可调
  CAP_PROP_GAMMA,             //伽马因子
  CAP_PROP_GAINAUTO,          //亮度
  CAP_PROP_EXPOSUREAUTO,      //自动曝光
  CAP_PROP_OFFSETX,           //X偏置
  CAP_PROP_OFFSETY,           //Y偏置
  CAP_PROP_TRIGGER_MODE,      //外部触发
  CAP_PROP_TRIGGER_SOURCE,    //触发源
  CAP_PROP_TRIGGER_ACTIVATION,//触发极性
  CAP_PROP_LINE_SELECTOR,      //触发线

};

//^ *********************************************************************************** //
//^ ********************************** Camera Class************************************ //
//^ *********************************************************************************** //
class Camera {
 public:
  //********** 构造函数  ****************************/
  Camera(ros::NodeHandle &node);
  //********** 析构函数  ****************************/
  ~Camera();
  //********** 原始信息转换线程 **********************/
  void *HKWorkThread(void *p_handle);

  //********** 输出摄像头信息 ***********************/
  bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo);
  //********** 设置摄像头参数 ***********************/
  bool set(camera::CamerProperties type, float value);
  //********** 恢复默认参数 *************************/
  bool reset();
  //********** 读图10个相机的原始图像 ********************************/
  void ReadImg(cv::Mat &image);

  image_transport::CameraPublisher image_pub;

 private:
  //********** handle ******************************/
  void *handle;
  //********** nThreadID ******************************/
  std::thread nThreadID;
  //********** yaml config ******************************/
  int nRet;
  int width;
  int height;
  int Offset_x;
  int Offset_y;
  bool FrameRateEnable;
  int FrameRate;
  int BurstFrameCount;
  int ExposureTime;
  bool GammaEnable;
  float Gamma;
  int GainAuto;
  int ExposureAuto;

  std::string SerialNumber;
  bool FlipHorizontal;
  bool FlipVertical;
  int TriggerMode;
  int TriggerSource;
  int TriggerActivation;
  int LineSelector;
};
//^ *********************************************************************************** //

//^ ********************************** Camera constructor************************************ //
Camera::Camera(ros::NodeHandle &node) {
  //********** rosnode init **********/
  image_transport::ImageTransport main_cam_image(node);
  // image_pub = new image_transport::CameraPublisher;
  image_pub = main_cam_image.advertiseCamera("image_raw", 1000);

  handle = NULL;

  //********** 读取待设置的摄像头参数 第三个参数是默认值 yaml文件未给出该值时生效 ********************************/
  node.param("width", width, 3072);
  node.param("height", height, 2048);
  node.param("FrameRateEnable", FrameRateEnable, false);
  node.param("FrameRate", FrameRate, 10);
  node.param("BurstFrameCount", BurstFrameCount, 10); // 一次触发采集的次数
  node.param("ExposureTime", ExposureTime, 50000);
  node.param("GammaEnable", GammaEnable, false);
  node.param("Gamma", Gamma, (float) 0.7);
  node.param("GainAuto", GainAuto, 2);
  node.param("ExposureAuto", ExposureAuto, 2);

  node.param("Offset_x", Offset_x, 0);
  node.param("Offset_y", Offset_y, 0);

  node.param<std::string>("SerialNumber", SerialNumber, "K66988523");
  node.param("FlipHorizontal", FlipHorizontal, false);
  node.param("FlipVertical", FlipVertical, false);
  node.param("TriggerMode", TriggerMode, 0);
  node.param("TriggerSource", TriggerSource, 2);
  node.param("TriggerActivation", TriggerActivation, 2);
  node.param("LineSelector", LineSelector, 2);

  //********** 枚举设备 ********************************/
  MV_CC_DEVICE_INFO_LIST stDeviceList;
  memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
  nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
  if (MV_OK != nRet) {
    printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
    exit(-1);
  }
  unsigned int nIndex = 0;
  if (stDeviceList.nDeviceNum > 0) {
    for (int i = 0; i < stDeviceList.nDeviceNum; i++) {
      printf("[device %d]:\n", i);
      MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];

      if (NULL == pDeviceInfo) {
        break;
      }
      // int serial = pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber;
      std::string serial(reinterpret_cast<const char *>(pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber));
      if (serial == SerialNumber) {
        //********** 选择设备并创建句柄 *************************/
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[i]);
        PrintDeviceInfo(pDeviceInfo);
      }
    }
  } else {
    printf("Find No Devices!\n");
    exit(-1);
  }

  if (MV_OK != nRet) {
    printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
    exit(-1);
  }

  // 打开设备
  //********** frame **********/

  nRet = MV_CC_OpenDevice(handle);

  if (MV_OK != nRet) {
    printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
    exit(-1);
  }

  //设置 yaml 文件里面的配置
  this->set(CAP_PROP_FRAMERATE_ENABLE, FrameRateEnable);
  if (FrameRateEnable)
    this->set(CAP_PROP_FRAMERATE, FrameRate);
  // this->set(CAP_PROP_BURSTFRAMECOUNT, BurstFrameCount);
  this->set(CAP_PROP_HEIGHT, height);
  this->set(CAP_PROP_WIDTH, width);
  this->set(CAP_PROP_OFFSETX, Offset_x);
  this->set(CAP_PROP_OFFSETY, Offset_y);
  this->set(CAP_PROP_EXPOSUREAUTO, ExposureAuto);
  if(!ExposureAuto){
    this->set(CAP_PROP_EXPOSURE_TIME, ExposureTime);
  }
  this->set(CAP_PROP_GAMMA_ENABLE, GammaEnable);
  if (GammaEnable)
    this->set(CAP_PROP_GAMMA, Gamma);
  this->set(CAP_PROP_GAINAUTO, GainAuto);
  this->set(CAP_PROP_TRIGGER_MODE, TriggerMode);
  this->set(CAP_PROP_TRIGGER_SOURCE, TriggerSource);
  this->set(CAP_PROP_TRIGGER_ACTIVATION, TriggerActivation);
  this->set(CAP_PROP_LINE_SELECTOR, LineSelector);

  //********** 图像格式 **********/
  // 0x01080001:Mono8
  // 0x01100003:Mono10
  // 0x010C0004:Mono10Packed
  // 0x01100005:Mono12
  // 0x010C0006:Mono12Packed
  // 0x01100007:Mono16
  // 0x02180014:RGB8Packed
  // 0x02100032:YUV422_8
  // 0x0210001F:YUV422_8_UYVY
  // 0x01080008:BayerGR8
  // 0x01080009:BayerRG8
  // 0x0108000A:BayerGB8
  // 0x0108000B:BayerBG8
  // 0x0110000e:BayerGB10
  // 0x01100012:BayerGB12
  // 0x010C002C:BayerGB12Packed
  nRet = MV_CC_SetEnumValue(handle, "PixelFormat", 0x01100005);

  if (MV_OK == nRet) {
    printf("set PixelFormat OK !\n");
  } else {
    printf("MV_CC_SetPixelFormat fail! nRet [%x]\n", nRet);
  }
  MVCC_ENUMVALUE t = {0};
  //********** frame **********/

  nRet = MV_CC_GetEnumValue(handle, "PixelFormat", &t);

  if (MV_OK == nRet) {
    printf("PixelFormat :%d!\n", t.nCurValue); // 35127316
  } else {
    printf("get PixelFormat fail! nRet [%x]\n", nRet);
  }
  // 开始取流
  //********** frame **********/

  nRet = MV_CC_StartGrabbing(handle);

  if (MV_OK != nRet) {
    printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
    exit(-1);
  }
  //初始化互斥量
  nRet = pthread_mutex_init(&mutex, NULL);
  if (nRet != 0) {
    perror("pthread_create failed\n");
    exit(-1);
  }
  //********** frame **********/

  nThreadID = std::thread(&Camera::HKWorkThread, this, handle);
  // nRet = pthread_create(&nThreadID, NULL, &Camera::HKWorkThread, this);

  if (nRet != 0) {
    printf("thread create failed.ret = %d\n", nRet);
    exit(-1);
  }
}

//^ ********************************** Camera constructor************************************ //
Camera::~Camera() {
  int nRet;
  //********** frame **********/

  // pthread_join(nThreadID, NULL);

  nThreadID.join();
  //********** frame **********/

  nRet = MV_CC_StopGrabbing(handle);

  if (MV_OK != nRet) {
    printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
    exit(-1);
  }
  printf("MV_CC_StopGrabbing succeed.\n");
  // 关闭设备
  //********** frame **********/

  nRet = MV_CC_CloseDevice(handle);

  if (MV_OK != nRet) {
    printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
    exit(-1);
  }
  printf("MV_CC_CloseDevice succeed.\n");
  // 销毁句柄
  //********** frame **********/

  nRet = MV_CC_DestroyHandle(handle);

  if (MV_OK != nRet) {
    printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
    exit(-1);
  }
  printf("MV_CC_DestroyHandle succeed.\n");
  // 销毁互斥量
  pthread_mutex_destroy(&mutex);
}

//^ ********************************** Camera constructor************************************ //
bool Camera::set(CamerProperties type, float value) {
  switch (type) {
    case CAP_PROP_FRAMERATE_ENABLE: {
      //********** frame **********/

      nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", value);

      if (MV_OK == nRet) {
        printf("set AcquisitionFrameRateEnable OK! value=%f\n", value);
      } else {
        printf("Set AcquisitionFrameRateEnable Failed! nRet = [%x]\n\n", nRet);
      }
      break;
    }
    case CAP_PROP_FRAMERATE: {
      //********** frame **********/

      nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", value);

      if (MV_OK == nRet) {
        printf("set AcquisitionFrameRate OK! value=%f\n", value);
      } else {
        printf("Set AcquisitionFrameRate Failed! nRet = [%x]\n\n", nRet);
      }
      break;
    }
    case CAP_PROP_BURSTFRAMECOUNT: {
      //********** frame **********/

      nRet = MV_CC_SetIntValue(handle, "AcquisitionBurstFrameCount", value);

      if (MV_OK == nRet) {
        printf("set AcquisitionBurstFrameCount OK!\n");
      } else {
        printf("Set AcquisitionBurstFrameCount Failed! nRet = [%x]\n\n", nRet);
      }
      break;
    }
    case CAP_PROP_HEIGHT: {
      //********** frame **********/

      nRet = MV_CC_SetIntValue(handle, "Height", value); //图像高度

      if (MV_OK == nRet) {
        printf("set Height OK!\n");
      } else {
        printf("Set Height Failed! nRet = [%x]\n\n", nRet);
      }
      break;
    }
    case CAP_PROP_WIDTH: {
      //********** frame **********/

      nRet = MV_CC_SetIntValue(handle, "Width", value); //图像宽度

      if (MV_OK == nRet) {
        printf("set Width OK!\n");
      } else {
        printf("Set Width Failed! nRet = [%x]\n\n", nRet);
      }
      break;
    }
    case CAP_PROP_OFFSETX: {
      //********** frame **********/

      nRet = MV_CC_SetIntValue(handle, "OffsetX", value); //图像宽度

      if (MV_OK == nRet) {
        printf("set Offset X OK!\n");
      } else {
        printf("Set Offset X Failed! nRet = [%x]\n\n", nRet);
      }
      break;
    }
    case CAP_PROP_OFFSETY: {
      //********** frame **********/

      nRet = MV_CC_SetIntValue(handle, "OffsetY", value); //图像宽度

      if (MV_OK == nRet) {
        printf("set Offset Y OK!\n");
      } else {
        printf("Set Offset Y Failed! nRet = [%x]\n\n", nRet);
      }
      break;
    }
    case CAP_PROP_EXPOSURE_TIME: {
      //********** frame **********/

      nRet = MV_CC_SetFloatValue(handle, "ExposureTime", value); //曝光时间

      if (MV_OK == nRet) {
        printf("set ExposureTime OK! value=%f\n", value);
      } else {
        printf("Set ExposureTime Failed! nRet = [%x]\n\n", nRet);
      }
      break;
    }
    case CAP_PROP_GAMMA_ENABLE: {
      //********** frame **********/

      nRet = MV_CC_SetBoolValue(handle, "GammaEnable", value); //伽马因子是否可调  默认不可调（false）

      if (MV_OK == nRet) {
        printf("set GammaEnable OK! value=%f\n", value);
      } else {
        printf("Set GammaEnable Failed! nRet = [%x]\n\n", nRet);
      }
      break;
    }
    case CAP_PROP_GAMMA: {
      //********** frame **********/

      nRet = MV_CC_SetFloatValue(handle, "Gamma", value); //伽马越小 亮度越大

      if (MV_OK == nRet) {
        printf("set Gamma OK! value=%f\n", value);
      } else {
        printf("Set Gamma Failed! nRet = [%x]\n\n", nRet);
      }
      break;
    }
    case CAP_PROP_GAINAUTO: {
      //********** frame **********/

      nRet = MV_CC_SetEnumValue(handle, "GainAuto", value); //亮度 越大越亮

      if (MV_OK == nRet) {
        printf("set GainAuto OK! value=%f\n", value);
      } else {
        printf("Set GainAuto Failed! nRet = [%x]\n\n", nRet);
      }
      break;
    }
    case CAP_PROP_EXPOSUREAUTO: {
      //********** frame **********/

      nRet = MV_CC_SetEnumValue(handle, "ExposureAuto", value); //亮度 越大越亮

      if (MV_OK == nRet) {
        printf("set ExposureAuto OK! value=%f\n", value);
      } else {
        printf("Set ExposureAuto Failed! nRet = [%x]\n\n", nRet);
      }
      break;
    }

    case CAP_PROP_TRIGGER_MODE: {

      nRet = MV_CC_SetEnumValue(handle, "TriggerMode", value); //触发模式

      if (MV_OK == nRet) {
        printf("set TriggerMode OK!\n");
      } else {
        printf("Set TriggerMode Failed! nRet = [%x]\n\n", nRet);
      }
      break;
    }
    case CAP_PROP_TRIGGER_SOURCE: {

      nRet = MV_CC_SetEnumValue(handle, "TriggerSource", value); //触发源

      if (MV_OK == nRet) {
        printf("set TriggerSource OK!\n");
      } else {
        printf("Set TriggerSource Failed! nRet = [%x]\n\n", nRet);
      }
      break;
    }
    case CAP_PROP_TRIGGER_ACTIVATION: {

      nRet = MV_CC_SetEnumValue(handle, "TriggerActivation", value); //触发源

      if (MV_OK == nRet) {
        printf("set TriggerActivation OK!\n");
      } else {
        printf("Set TriggerActivation Failed! nRet = [%x]\n\n", nRet);
      }
      break;
    }
    case CAP_PROP_LINE_SELECTOR: {

      nRet = MV_CC_SetEnumValue(handle, "LineSelector", value); //IO选择

      if (MV_OK == nRet) {
        printf("set LineSelector OK!\n");
      } else {
        printf("Set LineSelector Failed! nRet = [%x]\n\n", nRet);
      }
      break;
    }

    default:return 0;
  }
  return nRet;
}

//^ ********************************** Camera constructor************************************ //
bool Camera::reset() {
  nRet = this->set(CAP_PROP_FRAMERATE_ENABLE, FrameRateEnable);
  nRet = this->set(CAP_PROP_FRAMERATE, FrameRate) || nRet;
  // nRet = this->set(CAP_PROP_BURSTFRAMECOUNT, BurstFrameCount) || nRet;
  nRet = this->set(CAP_PROP_HEIGHT, height) || nRet;
  nRet = this->set(CAP_PROP_WIDTH, width) || nRet;
  nRet = this->set(CAP_PROP_OFFSETX, Offset_x) || nRet;
  nRet = this->set(CAP_PROP_OFFSETY, Offset_y) || nRet;
  nRet = this->set(CAP_PROP_EXPOSURE_TIME, ExposureTime) || nRet;
  nRet = this->set(CAP_PROP_GAMMA_ENABLE, GammaEnable) || nRet;
  nRet = this->set(CAP_PROP_GAMMA, Gamma) || nRet;
  nRet = this->set(CAP_PROP_GAINAUTO, GainAuto) || nRet;
  nRet = this->set(CAP_PROP_TRIGGER_MODE, TriggerMode) || nRet;
  nRet = this->set(CAP_PROP_TRIGGER_SOURCE, TriggerSource) || nRet;
  nRet = this->set(CAP_PROP_TRIGGER_ACTIVATION, TriggerActivation) || nRet;
  nRet = this->set(CAP_PROP_LINE_SELECTOR, LineSelector) || nRet;
  return nRet;
}

//^ ********************************** PrintDeviceInfo ************************************ //
bool Camera::PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo) {
  if (NULL == pstMVDevInfo) {
    printf("%s\n", "The Pointer of pstMVDevInfoList is NULL!");
    return false;
  }
  if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE) {
    printf("%s %x\n", "nCurrentIp:", pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp);                 //当前IP
    printf("%s %s\n\n", "chUserDefinedName:", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName); //用户定义名
  } else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE) {
    printf("SerialNumber:%s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
  } else {
    printf("Not support.\n");
  }
  return true;
}

//^ ********************************** HKWorkThread1 ************************************ //
void *Camera::HKWorkThread(void *p_handle) {
  cv::Mat frame;
  double start;
  int nRet;
  unsigned char *m_pBufForDriver = (unsigned char *) malloc(sizeof(unsigned char) * MAX_IMAGE_DATA_SIZE);
  unsigned char *m_pBufForSaveImage = (unsigned char *) malloc(MAX_IMAGE_DATA_SIZE);
  MV_FRAME_OUT_INFO_EX stImageInfo = {0};
  MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
  cv::Mat tmp;
  cv_bridge::CvImagePtr cv_ptr = boost::make_shared<cv_bridge::CvImage>();
  sensor_msgs::Image image_msg;
  sensor_msgs::CameraInfo camera_info_msg;

  int image_empty_count = 0; //空图帧数
  while (ros::ok()) {
    start = static_cast<double>(cv::getTickCount());
    nRet = MV_CC_GetOneFrameTimeout(p_handle, m_pBufForDriver, MAX_IMAGE_DATA_SIZE, &stImageInfo, 100);
    if (nRet != MV_OK) {
      if (++image_empty_count > 100) {
        ROS_INFO("The Number of Failed Reading Exceed The Set Value!\n");
        exit(-1);
      }
      continue;
    }
    image_empty_count = 0; //空图帧数

    stConvertParam.nWidth = stImageInfo.nWidth;                               //ch:图像宽 | en:image width
    stConvertParam.nHeight = stImageInfo.nHeight;                              //ch:图像高 | en:image height
    stConvertParam.pSrcData = m_pBufForDriver;                  //ch:输入数据缓存 | en:input data buffer
    stConvertParam.nSrcDataLen = stImageInfo.nFrameLen;           //ch:输入数据大小 | en:input data size
    stConvertParam.enSrcPixelType = stImageInfo.enPixelType;    //ch:输入像素格式 | en:input pixel format
    stConvertParam.enDstPixelType = PixelType_Gvsp_Mono8;      //ch:输出像素格式 | en:output pixel format
    stConvertParam.pDstBuffer = m_pBufForSaveImage;             //ch:输出数据缓存 | en:output data buffer
    stConvertParam.nDstBufferSize = MAX_IMAGE_DATA_SIZE;        //ch:输出缓存大小 | en:output buffer size
    MV_CC_ConvertPixelType(p_handle, &stConvertParam);

    frame = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_16UC1, m_pBufForDriver).clone();
    frame *= 1 << 4;
    if(FlipHorizontal){
      cv::flip(frame, frame, 0); //水平翻转
    }
    if(FlipVertical){
      cv::flip(frame, frame, 1); //垂直翻转
    }
    frame_empty = 0;
    double time = ((double) cv::getTickCount() - start) / cv::getTickFrequency();
    //*************************************testing img********************************//
    // std::cout << "HK_camera,Time:" << time << "\tFPS:" << 1 / time << std::endl;

    cv_ptr->encoding = sensor_msgs::image_encodings::MONO16;
    cv_ptr->image = frame;
    image_msg = *(cv_ptr->toImageMsg());
    image_msg.header.stamp = ros::Time::now();  // ros发出的时间不是快门时间
    image_msg.header.frame_id = "hikrobot_camera";

    camera_info_msg.header.frame_id = image_msg.header.frame_id;
    camera_info_msg.header.stamp = image_msg.header.stamp;
    image_pub.publish(image_msg, camera_info_msg);
  }
  free(m_pBufForDriver);
  free(m_pBufForSaveImage);
  return 0;
}
} // namespace camera
#endif
