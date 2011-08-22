PIXHAWK_LIBRARY(mavconn_shm SHARED
  PxSHM.cc
  PxSHMImageClient.cc
  PxSHMImageServer.cc
)
PIXHAWK_LINK_LIBRARIES(mavconn_shm
  ${OPENCV_CORE_LIBRARY_OPTIMIZED}
)

SET_TARGET_PROPERTIES(mavconn_shm PROPERTIES COMPILE_FLAGS "-D_REENTRANT")
target_link_libraries(mavconn_shm
  ${OPENCV_CORE_LIBRARY_OPTIMIZED}
  lcm
  mavconn_lcm
)
