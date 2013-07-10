include_directories(${OpenCV_INCLUDE} ${CLEye_INCLUDE})
link_directories(${OpenCV_LIB} ${CLEye_LIB})

add_executable(recordImagery recordImagery.cpp)

target_link_libraries(recordImagery optimized opencv_core243 optimized opencv_highgui243 optimized opencv_calib3d243 optimized opencv_imgproc243 CLEyeMultiCam)

target_link_libraries(recordImagery debug opencv_core243d debug opencv_highgui243d debug opencv_calib3d243d debug opencv_imgproc243d CLEyeMultiCam)
