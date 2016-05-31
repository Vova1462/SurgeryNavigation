cmake_minimum_required(VERSION 3.3)

#This is needed to set OpenCV libs properly
#message (STATUS "CMAKE_MODULE_PATH=${CMAKE_MODULE_PATH}")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR})

message (STATUS "CMAKE_MODULE_PATH=${CMAKE_MODULE_PATH}")

project (SurgeryNavigation)

set (CMAKE_BUILD_TYPE Debug)
message (STATUS "Provided build type: ${CMAKE_BUILD_TYPE}")

if(NOT DEFINED CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE CACHE STRING "Build Type" FORCE)
  message (FATAL_ERROR "No build type provided! (${CMAKE_BUILD_TYPE})")
endif()
  
# using CMAKE_BUILD_TYPE variable passed through the command line
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  set (OpenCV_DIR "C:/dev/OpenCV/install.debug")
  set (CMAKE_CONFIGURATION_TYPES "Debug" CACHE STRING "Build Type" FORCE)
endif()
  
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  set (OpenCV_DIR "C:/dev/OpenCV/install.release" CACHE STRING "Build Type" FORCE)
  set (CMAKE_CONFIGURATION_TYPES "Release" CACHE STRING "Build Type" FORCE)
endif ()

message (STATUS "Configuring for: ${CMAKE_BUILD_TYPE}")
message (STATUS "Looking for OpenCV at: ${OpenCV_DIR}")

message(STATUS "opencv_PATH=${opencv_PATH}")

set(OpenCV_LIBS_USED core calib3d video highgui features2d flann)

find_package(OpenCV REQUIRED NO_MODULE COMPONENTS ${OpenCV_LIBS_USED} PATHS ${OPENCV_DIR} NO_DEFAULT_PATH)


foreach (MODULE ${OpenCV_LIBS})
	get_target_property(PROP ${MODULE} LOCATION_${CMAKE_BUILD_TYPE})	
	get_filename_component(FILENAME ${PROP} NAME_WE)
	set (NEW_NAME "${OpenCV_LIB_PATH}/${FILENAME}.lib")
	
	list(APPEND OPENCV_LIBS_WITH_PATHS ${NEW_NAME})
	
	message ("FILENAME="${FILENAME})
	message ("NEW_NAME="${NEW_NAME})
endforeach()


message(STATUS "OpenCV_FOUND=${OpenCV_FOUND}")
message(STATUS "OpenCV_LIB_COMPONENTS=${OpenCV_LIB_COMPONENTS}")
message(STATUS "OpenCV_LIBS=${OpenCV_LIBS}")
message(STATUS "_OpenCV_LIB_PATH=${_OpenCV_LIB_PATH}")

include_directories(${OpenCV_INCLUDE_DIRS})
link_libraries(${OpenCV_LIBS})

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

add_executable(Capture_Video main2.cpp)

message (STATUS "List of files for linker input: ${OPENCV_LIBS_WITH_PATHS}")
target_link_libraries(Capture_Video ${OPENCV_LIBS})