ps3EyeTools
===========

Set of useful tools for working with the PS3 Eye camera.  They were made for my personal use, so I will only add more functionality as I require new tools. 
Pull requests are welcome, though!


Windows-only right now, and they all depend on the Code Laboratories SDK, which you can download from their website:

http://codelaboratories.com/

  NOTE: It would have been cool to make it camera agnostic, but OpenCV has a lot of trouble when two cameras are connected via the same USB hub.

OpenCV lib dependencies:

opencv_core
opencv_highgui
opencv_imgproc

To create out-of-source build for VS2010:

1. Download and install the Code Laboratories Multicam SDK
2. Install OpenCV, if you don't already have it.

3. Modify the root CMakeLists.txt so that the _LIB and _INCLUDE directories point at their respective installation directories.
    NOTE: I renamed the CLEye include dir so that I can include it specifically, like so:
      #include <ClEyeInclude/opencv/...>
  
    This is to avoid conflicts between the user's opencv installation and the opencv files the are distributed with the CLEye SDK.
4. Create a bld directory
5. cmake -G "Visual Studio 10" [your src directory]
