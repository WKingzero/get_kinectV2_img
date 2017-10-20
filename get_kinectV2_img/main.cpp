// last month,I bought a kinect V2, I want to do a lot of things with it,
// I am too speechless ,because I searched for a lot of information on the web, but I didn't find it
// so I decided to write my own code to complete my development
// I hope this can help you when you use kinect V2 as well as me 
#include <iostream>
#include <cstdlib>
#include <signal.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <fstream>

#include <pcl/pcl_config.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <map>
//自定义状况记录器  继承
class MyFileLogger: public libfreenect2::Logger
{
private:
  std::ofstream logfile_;
public:
  MyFileLogger(const char *filename)
  {
    if (filename)
      logfile_.open(filename);
    level_ = Debug;
  }
  bool good()
  {
    return logfile_.is_open() && logfile_.good();
  }
  virtual void log(Level level, const std::string &message)
  {
    logfile_ << "[" << libfreenect2::Logger::level2str(level) << "] " << message << std::endl;
  }
};

//mian
int main(int argc, char *argv[])
{
  std::string program_path(argv[0]);
  //从最后开始，向前查找"main" 返回下标
  size_t executable_name_idx = program_path.rfind("main");

  //std::cout<<"program_path="<<program_path<<std::endl;
  //std::cout<<"executable_name_idx="<<executable_name_idx<<std::endl;

  std::string binpath = "/";
  //找到了
  if(executable_name_idx != std::string::npos)
  {
      //复制 从开始到executable_name_idx
     binpath = program_path.substr(0, executable_name_idx);
     //std::cout<<"binpath="<<binpath<<std::endl;//  ./
  }

  libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Debug));
  MyFileLogger *filelogger = new MyFileLogger(getenv("LOGFILE"));//取得环境变量内容  指针
  if (filelogger->good())
     libfreenect2::setGlobalLogger(filelogger);
  else
     delete filelogger;

  //Find and Init kinect
  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *dev = 0;
  libfreenect2::PacketPipeline *pipeline = 0;

  std::string serial = "";

  //是否显示 图像框，rgb图，depth图  后期可删除 换成opencv
  /*
  bool viewer_enabled = true;
  bool enable_rgb = true;
  bool enable_depth = true;*/
  size_t framemax = -1;

  //枚举所有的kinect设备
  if(freenect2.enumerateDevices() == 0)
   {
     std::cout << "no device connected!" << std::endl;
     return -1;
   }
  if (serial == "")
   {
     serial = freenect2.getDefaultDeviceSerialNumber();
   }
  //打开
  if(pipeline)
   {
  //打开kinect
     dev = freenect2.openDevice(serial, pipeline);
   }
   else
   {
     dev = freenect2.openDevice(serial);
   }
   if(dev == 0)
   {
     std::cout << "failure opening device!" << std::endl;
     return -1;
   }
  int types = 0;
  types |= libfreenect2::Frame::Color| libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
   //等待所有指定类型的帧被接收一次
  libfreenect2::SyncMultiFrameListener listener(types);
  libfreenect2::FrameMap frames;
  dev->setColorFrameListener(&listener);
  dev->setIrAndDepthFrameListener(&listener);
  if (!dev->start())
        return -1;
  std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
  std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

  //start
  //可用setIrCameraParams（）替换成自己的矫正参数   这里获取的是默认的
  libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
  libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920, 1080 + 2, 4);
  size_t framecount = 0;
   cv::Mat rgbmat, depthmat, depthmatUndistorted, irmat, rgbd, rgbd2;
  // [loop start]  循环接受图像
  while(  (framemax == (size_t)-1 || framecount < framemax))
  {
      if (!listener.waitForNewFrame(frames, 10*1000)) // 10 sconds  这里将阻塞，直到所有帧都被接收，然后可以Frame根据类型提取。
      {
        std::cout << "timeout!" << std::endl;
        return -1;
      }
      libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
      libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
      libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
  /// [loop start]

      cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
      cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(irmat);
      cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);
      cv::imshow("rgb", rgbmat);
      cv::imshow("ir", irmat / 4500.0f);
      cv::imshow("depth", depthmat / 4500.0f);



      //彩图映射到深度图上 创建点云图
      registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);

      cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data).copyTo(depthmatUndistorted);
      cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(rgbd);
      cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(rgbd2);

      cv::imshow("undistorted", depthmatUndistorted / 4500.0f);
      cv::imshow("registered", rgbd);
      cv::imshow("depth2RGB", rgbd2 / 4500.0f);

       cv::waitKey(1);
      framecount++;

  /// [loop end]
      listener.release(frames);

      //cv::imwrite("rgb.jpg",rgb);
  }
  dev->stop();
  dev->close();
  delete registration;

  return 0;
}






