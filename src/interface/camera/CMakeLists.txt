INCLUDE_DIRECTORIES(.
  ${GLIB2_MAIN_INCLUDE_DIR}
  ${GLIB2_INTERNAL_INCLUDE_DIR}
  ${GLIBMM2_MAIN_INCLUDE_DIR}
  ${GLIBMM2_INTERNAL_INCLUDE_DIR}
  ${GTHREAD2_MAIN_INCLUDE_DIR}
  ${SIGC++_INCLUDE_DIR}
  ${DC_INCLUDE_DIR}
  ../../external/mvimpact
)

LINK_DIRECTORIES(
  ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
)

PIXHAWK_EXECUTABLE(px_imagecapture px_imagecapture.cc)
PIXHAWK_LINK_LIBRARIES(px_imagecapture
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
  ${OPENCV_CORE_LIBRARY_OPTIMIZED}
  ${OPENCV_HIGHGUI_LIBRARY_OPTIMIZED}
  ${GLIB2_LIBRARY}
  ${GTHREAD2_LIBRARY}
  mavconn_lcm
  mavconn_shm
  lcm
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
)

PIXHAWK_EXECUTABLE(px_replay px_replay.cc)
PIXHAWK_LINK_LIBRARIES(px_replay
  ${OPENCV_CORE_LIBRARY_OPTIMIZED}
  ${OPENCV_HIGHGUI_LIBRARY_OPTIMIZED}
  ${GLIB2_LIBRARY}
  ${GLIBMM2_LIBRARY}
  mavconn_lcm
  mavconn_shm
  lcm
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
)

SET_SOURCE_FILES(CAMERA_SRC_FILES
  PxCamera.cc
  PxCameraCapture.cc
  PxCameraManager.cc
  PxCameraManagerFactory.cc
  PxBluefoxCamera.cc
  PxBluefoxCameraManager.cc
  PxBluefoxStereoCamera.cc
  PxFireflyCamera.cc
  PxFireflyCameraManager.cc
  PxFireflyStereoCamera.cc
  PxStereoCamera.cc
)

# WARNING: Do not link mvDeviceManager and OPENCV_HIGHGUI_LIBRARY_OPTIMIZED
# to the same executable/library as there is some unknown cross-interaction
# between these 2 libraries which causes the camera API to work incorrectly.
PIXHAWK_LIBRARY(pxcam SHARED ${CAMERA_SRC_FILES})
PIXHAWK_LINK_LIBRARIES(pxcam
  ${DC_LIBRARY_OPTIMIZED}
  ${GLIBMM2_LIBRARY}
  ${OPENCV_CORE_LIBRARY_OPTIMIZED}
  ${SIGC++_LIBRARY}
  lcm
  mavconn_lcm
  mavconn_shm
  mvDeviceManager
)
SET_TARGET_PROPERTIES(pxcam PROPERTIES COMPILE_FLAGS "-D_REENTRANT")

PIXHAWK_EXECUTABLE(px_camera
  px_camera.cc
)
PIXHAWK_LINK_LIBRARIES(px_camera
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  ${OPENCV_IMGPROC_LIBRARY_OPTIMIZED}
  pxcam
)

PIXHAWK_EXECUTABLE(px_view px_view.cc)
PIXHAWK_LINK_LIBRARIES(px_view
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
  ${OPENCV_CORE_LIBRARY_OPTIMIZED}
  ${OPENCV_IMGPROC_LIBRARY_OPTIMIZED}
  ${OPENCV_HIGHGUI_LIBRARY_OPTIMIZED}
  mavconn_lcm
  lcm
  mavconn_shm
)
