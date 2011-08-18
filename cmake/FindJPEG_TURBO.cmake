INCLUDE(FindPackageHandleStandardArgs)
INCLUDE(HandleLibraryTypes)

FIND_PATH(JPEG_TURBO_INCLUDE_DIR turbojpeg.h /opt/libjpeg-turbo/include NO_DEFAULT_PATH)

FIND_LIBRARY(JPEG_TURBO_LIBRARY turbojpeg /opt/libjpeg-turbo/lib NO_DEFAULT_PATH)

# Handle the REQUIRED argument and set the <UPPERCASED_NAME>_FOUND variable
# The package is found if all variables listed are TRUE
FIND_PACKAGE_HANDLE_STANDARD_ARGS(JPEG_TURBO "Could NOT find JPEG_TURBO library"
  JPEG_TURBO_LIBRARY
  JPEG_TURBO_INCLUDE_DIR
)

