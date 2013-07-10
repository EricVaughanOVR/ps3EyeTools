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

bool captureImgs(int numImgs, std::vector<cv::Mat>& lImgs, std::vector<cv::Mat>& rImgs, 
  std::vector<std::vector<cv::Point2f> >& vect_cornersL, std::vector<std::vector<cv::Point2f> >& vect_cornersR, cv::Size pattern,
  float squareSize, cv::Size& resolution)
{
  lImgs.clear();
  rImgs.clear();

  int imgCount = 0;

  std::cout<<"Press [s] to swap left-right"<<std::endl<<
    "Press [c] to capture the stereo-pair for calibration"<<std::endl<<
    "Press [Enter] to start"<<std::endl;
  std::cin.clear();
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  int numCams = CLEyeGetCameraCount();
  if(numCams != 2)
  {
    std::cerr<<"Incorrect number of cameras: "<<numCams<<std::endl;
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
  resolution = res;
  bufIplL = cvCreateImage(res, IPL_DEPTH_8U, 1);
  bufIplR = cvCreateImage(res, IPL_DEPTH_8U, 1);
  cvGetImageRawData(bufIplL, &bufL);
  cvGetImageRawData(bufIplR, &bufR);

  cv::namedWindow("Cameras", 1);
  
  bool swap = false;

  while(imgCount < numImgs)
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
    imgL.copyTo(sideBySide(cv::Rect(0, 0, imgL.cols, imgL.rows)));
    imgR.copyTo(sideBySide(cv::Rect(imgL.cols, 0, imgL.cols, imgL.rows)));

    cv::Mat colorImg;
    cv::cvtColor(sideBySide, colorImg, CV_GRAY2RGB);

    cv::imshow("Cameras", colorImg);
    int keyVal = cv::waitKey(1);
    if(keyVal == 83 || keyVal == 115)//swap imgs
      swap = !swap;
    else if(keyVal == 67 || keyVal == 99)//record imgs
    {
      std::vector<cv::Point2f> cornersL, cornersR;
      bool leftChess = cv::findChessboardCorners(imgL, pattern, cornersL, 
        CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
      bool rightChess = cv::findChessboardCorners(imgR, pattern, cornersR, 
        CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

      if(leftChess & rightChess)
      {
        cv::drawChessboardCorners(colorImg(cv::Rect(0, 0, imgL.cols, imgL.rows)), pattern, cornersL, leftChess);
        cv::drawChessboardCorners(colorImg(cv::Rect(imgL.cols, 0, imgL.cols, imgL.rows)), pattern, cornersR, rightChess);
        cv::imshow("Cameras", colorImg);
        std::cout<<"Keep images? [y]es to keep, or any other key to reject"<<std::endl;
        keyVal = cv::waitKey(0);
      
        if(keyVal == 121 || keyVal == 89)
        {
                /*std::ostringstream filenameL, filenameR;
      filenameL << "calL" << imgCount << ".png";
      cv::imwrite(filenameL.str(), imgL);

      filenameR << "calR" << imgCount << ".png";
      cv::imwrite(filenameR.str(), imgR);*/

          std::cout<<"Saved image # "<<imgCount<<std::endl;
          ++imgCount;
          lImgs.push_back(imgL.clone());
          rImgs.push_back(imgR.clone());
          vect_cornersL.push_back(cornersL);
          vect_cornersR.push_back(cornersR);
        }
      }
    }
  }

  cv::destroyWindow("Cameras");

  CLEyeCameraStop(cams[0]);
  CLEyeCameraStop(cams[1]);
  CLEyeDestroyCamera(cams[0]);
  CLEyeDestroyCamera(cams[1]);

  return true;
}

void calibCameras(std::vector<cv::Mat>& lImgs, std::vector<cv::Mat>& rImgs,
  std::vector<std::vector<cv::Point2f> >& cornersL, std::vector<std::vector<cv::Point2f> >& cornersR,
  cv::Size pattern, float squareSize, cv::Size imgRes)
{
  double M1[3][3], M2[3][3], D1[5], D2[5];
  double R[3][3], T[3], E[3][3], F[3][3];
  double Q[4][4];
  cv::Mat _M1(3, 3, CV_64F, M1 );
  cv::Mat _M2(3, 3, CV_64F, M2 );
  cv::Mat _D1(1, 5, CV_64F, D1 );
  cv::Mat _D2(1, 5, CV_64F, D2 );
  cv::Mat _R(3, 3, CV_64F, R );
  cv::Mat _T(3, 1, CV_64F, T );
  cv::Mat _E(3, 3, CV_64F, E );
  cv::Mat _F(3, 3, CV_64F, F );
  cv::Mat _Q(4,4, CV_64F, Q);

  cv::TermCriteria criteria = cv::TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.01 );//TODO tune for efficacy and speed

  for(int i = 0; i < lImgs.size(); ++i)
  {
    cv::cornerSubPix(lImgs[i], cornersL[i], cv::Size(3,3), cv::Size(-1, -1), criteria);
    cv::cornerSubPix(rImgs[i], cornersR[i], cv::Size(3,3), cv::Size(-1, -1), criteria);
  }

  int numObjPts = pattern.area();
  std::vector<cv::Point3f> chessCorners(numObjPts);
  std::vector< std::vector<cv::Point3f> > objectPoints;

  //Generate object-space coordinate for each corner of the pattern.  This generates a chessboard pattern
  for(int i = 0; i < pattern.height; ++i)
  {
    for(int j = 0; j <pattern.width; ++j)
    {
      chessCorners[i * pattern.width + j] = cv::Point3f(j * squareSize, i * squareSize, 0);
    }
  }

  //Copy the pattern for each pair of images
  for(int i = 0; i < lImgs.size(); ++i)
  {
    objectPoints.push_back(chessCorners);
  }

  std::cout<<"Doing stereo calibration..."<<std::endl;

  //Do stereo calibration
  double error = cv::stereoCalibrate(objectPoints, cornersL, cornersR, _M1, _D1, _M2, _D2, imgRes, _R, _T, _E, _F,
    cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5),
    /*CV_CALIB_FIX_ASPECT_RATIO + */CV_CALIB_ZERO_TANGENT_DIST + CV_CALIB_SAME_FOCAL_LENGTH );

  std::cout<<"Done."<<std::endl
           <<"Calibration Error = "<<error<<std::endl
           <<"Doing stereo rectification..."<<std::endl;

  int totPoints = numObjPts * lImgs.size();

  cv::Mat _R1, _R2, _P1, _P2, mx1_large, my1_large, mx2_large, my2_large,
    mx1_small, my1_small, mx2_small, my2_small, _Q_large, _Q_small;
  cv::Rect* roi1_large = new cv::Rect(), * roi2_large = new cv::Rect(), * roi1_small = new cv::Rect(), * roi2_small = new cv::Rect();

  cv::stereoRectify(_M1, _D1, _M2, _D2, imgRes, _R, _T, _R1, _R2, _P1, _P2, _Q_large, 0, -1.0, imgRes, roi1_large, roi2_large);
  cv::initUndistortRectifyMap(_M1, _D1, _R1, _P1, imgRes, CV_16SC2, mx1_large, my1_large);
  cv::initUndistortRectifyMap(_M2, _D2, _R2, _P2, imgRes, CV_16SC2, mx2_large, my2_large);
  
  cv::FileStorage fs("M1.xml", cv::FileStorage::WRITE);
  fs<<"M1"<<_M1;
  fs.release();

  fs = cv::FileStorage("R1.xml", cv::FileStorage::WRITE);
  fs<<"R1"<<_R1;
  fs.release();

  fs = cv::FileStorage("P1.xml", cv::FileStorage::WRITE);
  fs<<"P1"<<_P1;
  fs.release();

  fs = cv::FileStorage("M2.xml", cv::FileStorage::WRITE);
  fs<<"M2"<<_M2;
  fs.release();

  fs = cv::FileStorage("D2.xml", cv::FileStorage::WRITE);
  fs<<"D2"<<_D2;
  fs.release();

  fs = cv::FileStorage("R2.xml", cv::FileStorage::WRITE);
  fs<<"R2"<<_R2;
  fs.release();

  fs = cv::FileStorage("P2.xml", cv::FileStorage::WRITE);
  fs<<"P2"<<_P2;
  fs.release();

  fs = cv::FileStorage("Q_hiRes.xml", cv::FileStorage::WRITE);
  fs<<"Q"<<_Q_large;
  fs.release();

  fs = cv::FileStorage("mxl_hiRes.xml", cv::FileStorage::WRITE);
  fs<<"mx1"<<mx1_large;
  fs.release();

  fs = cv::FileStorage("myl_hiRes.xml", cv::FileStorage::WRITE);
  fs<<"my1"<<my1_large;
  fs.release();

  fs = cv::FileStorage("mxr_hiRes.xml", cv::FileStorage::WRITE);
  fs<<"mx2"<<mx2_large;
  fs.release();

  fs = cv::FileStorage("myr_hiRes.xml", cv::FileStorage::WRITE);
  fs<<"my2"<<my2_large;
  fs.release();

  std::ofstream roiFs("roi_hiRes.txt", std::ofstream::out);
  roiFs<<"ROI 1"<<std::endl<<"X: "<<roi1_large->x<<std::endl;
  roiFs<<"Y: "<<roi1_large->y<<std::endl;
  roiFs<<"Width: "<<roi1_large->width<<std::endl;
  roiFs<<"Height: "<<roi1_large->height<<std::endl<<std::endl;

  roiFs<<"ROI 2"<<std::endl<<"X: "<<roi2_large->x<<std::endl;
  roiFs<<"Y: "<<roi2_large->y<<std::endl;
  roiFs<<"Width: "<<roi2_large->width<<std::endl;
  roiFs<<"Height: "<<roi2_large->height<<std::endl;

  delete roi1_large;
  delete roi2_large;

  M1[0][0] /= 2;
  M2[0][0] /= 2;
  M1[0][2] /= 2;
  M2[0][2] /= 2;
  M1[1][1] /= 2;
  M2[1][1] /= 2;
  M1[1][2] /= 2;
  M2[1][2] /= 2;

  imgRes.height /= 2;
  imgRes.width /= 2;
  
  cv::stereoRectify(_M1, _D1, _M2, _D2, imgRes, _R, _T, _R1, _R2, _P1, _P2, _Q_small, 0, -1.0, cv::Size(imgRes.width / 2, imgRes.height / 2), roi1_small, roi2_small);
  cv::initUndistortRectifyMap(_M1, _D1, _R1, _P1, imgRes, CV_16SC2, mx1_small, my1_small);
  cv::initUndistortRectifyMap(_M2, _D2, _R2, _P2, imgRes, CV_16SC2, mx2_small, my2_small);

  fs = cv::FileStorage("Q_lowRes.xml", cv::FileStorage::WRITE);
  fs<<"Q"<<_Q_small;
  fs.release();

  fs = cv::FileStorage("mxl_lowRes.xml", cv::FileStorage::WRITE);
  fs<<"mx1"<<mx1_small;
  fs.release();

  fs = cv::FileStorage("myl_lowRes.xml", cv::FileStorage::WRITE);
  fs<<"my1"<<my1_small;
  fs.release();

  fs = cv::FileStorage("mxr_lowRes.xml", cv::FileStorage::WRITE);
  fs<<"mx2"<<mx2_small;
  fs.release();

  fs = cv::FileStorage("myr_lowRes.xml", cv::FileStorage::WRITE);
  fs<<"my2"<<my2_small;
  fs.release();

  roiFs = std::ofstream("roi_lowRes.txt", std::ofstream::out);
  roiFs<<"ROI 1"<<std::endl<<"X: "<<roi1_small->x<<std::endl;
  roiFs<<"Y: "<<roi1_small->y<<std::endl;
  roiFs<<"Width: "<<roi1_small->width<<std::endl;
  roiFs<<"Height: "<<roi1_small->height<<std::endl<<std::endl;

  roiFs<<"ROI 2"<<std::endl<<"X: "<<roi2_small->x<<std::endl;
  roiFs<<"Y: "<<roi2_small->y<<std::endl;
  roiFs<<"Width: "<<roi2_small->width<<std::endl;
  roiFs<<"Height: "<<roi2_small->height<<std::endl;

  delete roi2_small;
  delete roi1_small;

  cv::namedWindow("Rectified Pair", 1);

  for(int i = 0; i < lImgs.size(); ++i)
  {
    cv::Mat rectifiedL, rectifiedR;
    cv::remap(lImgs[i], rectifiedL, mx1_large, my1_large, cv::INTER_LINEAR);
    cv::remap(rImgs[i], rectifiedR, mx2_large, my2_large, cv::INTER_LINEAR);

    cv::Mat pair(rectifiedL.rows, rectifiedL.cols * 2, CV_8UC1);
    rectifiedL.copyTo(pair(cv::Rect(0, 0, rectifiedL.cols, rectifiedL.rows)));
    rectifiedR.copyTo(pair(cv::Rect(rectifiedL.cols, 0, rectifiedL.cols, rectifiedL.rows)));

    cv::Mat colorPair;
    cv::cvtColor(pair, colorPair, CV_GRAY2RGB);

    for(int j = 0; j < colorPair.rows; j += 16)
    {
      cv::line(colorPair, cv::Point(0, j), cv::Point(colorPair.cols, j), CV_RGB(0, 255, 0), 1);
    }

    cv::imshow("Rectified Pair", colorPair);
    cv::waitKey(0);
  }
}

int main(int argc, char* argv[])
{
  bool fail = false;
  int nx, ny;
  float squareSize;
  int numImgs;

  if(argc != 5)
  {
    std::cerr<<"Usage: [# of horizontal squares] [# of vertical squares] [size of squares] [number of calibration images to capture]"<<std::endl;
    return 1;
  }
  nx = atoi(argv[1]);
  ny = atoi(argv[2]);
  cv::Size chessboardPattern(nx, ny);
  squareSize = static_cast<float>(atof(argv[3]));
  numImgs = atoi(argv[4]);


  if (nx <= 0)
    {
        fail = true;
        std::cerr<<"[# of horizontal squares must be > 0]"<<std::endl;
    }
    if (ny <= 0)
    {
        fail = true;
        std::cerr<<"[# of vertical squares must be > 0]"<<std::endl;
    }   
    if (squareSize <= 0.0)
    {
        fail = true;
        std::cerr<<"[squareSize must be > 0]"<<std::endl;
    } 
    if (numImgs <= 0)
    {
        fail = true;
        std::cerr<<"[# of calibration images must be > 0]"<<std::endl;
    } 

    if(fail)
      return 1;

    std::vector<cv::Mat> lImgs, rImgs;
    std::vector<std::vector<cv::Point2f> > cornersL, cornersR;
    cv::Size resolution;
    if(!captureImgs(numImgs, lImgs, rImgs, cornersL, cornersR, chessboardPattern, squareSize, resolution))
      return 1;
    calibCameras(lImgs, rImgs, cornersL, cornersR, chessboardPattern, squareSize, resolution);
}