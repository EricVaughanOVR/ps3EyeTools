cmake_minimum_required(VERSION 2.8)

project(PS3EyeTools)

set(OpenCV_INCLUDE c:/opencv/build/include)
set(OpenCV_LIB c:/opencv/build/x86/vc10/lib)
set(CLEye_INCLUDE c:/CL-Eye\ Platform\ SDK)
set(CLEye_LIB c:/CL-Eye\ Platform\ SDK/CLEyeLib)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_subdirectory(RecordImagery)
add_subdirectory(StereoCalibration)
