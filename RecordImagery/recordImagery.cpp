#include <Windows.h>
#include <fstream>
#include <opencv/cv.h>
#include <opencv/cxmisc.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <CLEyeInclude/CLEyeMulticam.h>
#include <opencv/cvaux.hpp>
#include <vector>
#include <string>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <sys/stat.h>

bool captureImgs(bool useUndistortMaps, cv::Mat* mx1 = NULL, 
  cv::Mat* my1 = NULL, cv::Mat* mx2 = NULL, cv::Mat* my2 = NULL)
{  
  int imgCount = 0;

  std::cout<<"Press [s] to swap left-right"<<std::endl<<
    "Press [c] to save the images to disk"<<std::endl<<
    "Press [x] to stop recording images and quit"<<std::endl<<std::endl;

  std::string baseFilename;
  std::cout<<"Enter a base file name for this img set"<<std::endl;
  std::cin>>baseFilename;

  int numCams = CLEyeGetCameraCount();
  if(numCams != 2)
  {
    std::cerr<<"Wrong number of cameras.  Quitting"<<std::endl;
    return false;
  }

  std::vector<CLEyeCameraInstance> cams;

  cams.push_back(CLEyeCreateCamera(CLEyeGetCameraUUID(0), CLEYE_MONO_PROCESSED, CLEYE_VGA, 60));
  cams.push_back(CLEyeCreateCamera(CLEyeGetCameraUUID(1), CLEYE_MONO_PROCESSED, CLEYE_VGA, 60));

  if(cams[0] == NULL)
  {
    std::cout<<"Camera 0 failed to open"<<std::endl;
    return false;
  }
  if(cams[1] == NULL)
  {
    std::cout<<"Camera 1 failed to open"<<std::endl;
    return false;
  }

  CLEyeCameraStart(cams[0]);
  CLEyeCameraStart(cams[1]);

  unsigned char* bufL, * bufR;
  IplImage* bufIplL = NULL, * bufIplR = NULL;
  CvSize res;
  res.height = 480;
  res.width = 640;
  bufIplL = cvCreateImage(res, IPL_DEPTH_8U, 1);
  bufIplR = cvCreateImage(res, IPL_DEPTH_8U, 1);
  cvGetImageRawData(bufIplL, &bufL);
  cvGetImageRawData(bufIplR, &bufR);

  cv::namedWindow("Camera", 1);
  
  bool swap = false;

  while(true)
  {
    cv::Mat imgL, imgR;

    if(!swap)
    {
      CLEyeCameraGetFrame(cams[0], bufL);
      CLEyeCameraGetFrame(cams[1], bufR);
    }
    else
    {
      CLEyeCameraGetFrame(cams[1], bufL);
      CLEyeCameraGetFrame(cams[0], bufR);
    }

    imgL = cv::Mat(bufIplL);
    imgR = cv::Mat(bufIplR);

    cv::Mat sideBySide(imgL.rows, imgL.cols * 2, CV_8UC1);

    if(useUndistortMaps)
    {
      cv::remap(imgL, sideBySide(cv::Rect(0, 0, imgL.cols, imgL.rows)), *mx1, *my1, cv::INTER_LINEAR);
      cv::remap(imgR, sideBySide(cv::Rect(imgL.cols, 0, imgL.cols, imgL.rows)), *mx2, *my2, cv::INTER_LINEAR);
    }
    else
    {
      imgL.copyTo(sideBySide(cv::Rect(0, 0, imgL.cols, imgL.rows)));
      imgR.copyTo(sideBySide(cv::Rect(imgL.cols, 0, imgL.cols, imgL.rows)));
    }

    cv::imshow("Camera", sideBySide);

    int keyVal = cv::waitKey(1);
    if(keyVal == 83 || keyVal == 115)//swap imgs
      swap = !swap;
    else if(keyVal == 67 || keyVal == 99)//record imgs
    {
      std::cout<<"Keep image(s)? [y]es to keep, or any other key to reject"<<std::endl;
      keyVal = cv::waitKey(0);
      
      if(keyVal == 121 || keyVal == 89)
      {
        std::ostringstream filenameL, filenameR;
        filenameL << baseFilename << "L_" << imgCount << ".png";
        cv::imwrite(filenameL.str(), imgL);

        filenameR << baseFilename << "R_" << imgCount << ".png";
        cv::imwrite(filenameR.str(), imgR);

        std::cout<<"Saved image(s) # "<<imgCount<<std::endl;
        ++imgCount;
      }
      else if(keyVal == 120 || keyVal == 88)
        break;
    }
    else if(keyVal == 120 || keyVal == 88)
      break;
  }

  cv::destroyWindow("Cameras");

  CLEyeCameraStop(cams[0]);
  CLEyeCameraStop(cams[1]);
  CLEyeDestroyCamera(cams[0]);
  CLEyeDestroyCamera(cams[1]);

  return true;
}

int main(int argc, char* argv[])
{
  bool useUndistortMaps = true;

  cv::Mat* mx1 = NULL,* mx2 = NULL,* my1 = NULL,* my2 = NULL;

  if(argc != 5)
  {
    std::cout<<"No maps supplied, will not remap imagery"<<std::endl
      <<"Remap imagery by supplying full-res of:"<<std::endl<<
      "[mx1] [my1] [mx2] [my2]"<<std::endl;
    useUndistortMaps = false;
  }
  else
  {
    bool success = true;

    mx1 = new cv::Mat(480, 640, CV_8UC1);
    my1 = new cv::Mat(480, 640, CV_8UC1);
    mx2 = new cv::Mat(480, 640, CV_8UC1);
    my2 = new cv::Mat(480, 640, CV_8UC1);

    for(int i = 1; i < 5; ++i)
    {
      cv::Mat tmp;
      cv::FileStorage fs(argv[i], cv::FileStorage::READ);
      if(i == 1)
        fs["mx1"]>>tmp;
      else if(i == 2)
        fs["my1"]>>tmp;
      else if(i == 3)
        fs["mx2"]>>tmp;
      else if(i == 4)
        fs["my2"]>>tmp;

      if(tmp.rows != 480 || tmp.cols != 640)
      {
        success = false;
        break;
      }

      if(i == 1)
        tmp.copyTo(*mx1);
      else if(i == 2)
        tmp.copyTo(*my1);
      else if(i == 3)
        tmp.copyTo(*mx2);
      else if(i == 4)
        tmp.copyTo(*my2);

    }
    if(!success)
    {
      std::cerr<<"Map does not match VGA resolution of the camera";

      delete mx1;
      delete my1;
      delete mx2;
      delete my2;

      return 1;
    }
  }

  captureImgs(useUndistortMaps, mx1, my1, mx2, my2);

  delete mx1;
  delete my1;
  delete mx2;
  delete my2;

  return 0;
}