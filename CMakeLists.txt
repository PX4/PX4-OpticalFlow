cmake_minimum_required( VERSION 2.8.3 )
if( POLICY CMP0048 )
  # Silence CMP0048 warning about missing project VERSION.
  cmake_policy(SET CMP0048 NEW)
endif()
project( OpticalFlow )
set(version_major 0)
set(version_minor 1)
set(version_patch 0)
set(version "${version_major}.${version_minor}.${version_patch}")
set(package-contact "christoph@px4.io")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpic")
set(CMAKE_BUILD_TYPE Release)

set(${PROJECT_NAME}_INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/include
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)

find_package( OpenCV REQUIRED )

add_subdirectory( external/klt_feature_tracker klt_feature_tracker )
set( klt_feature_tracker_LIBS "klt_feature_tracker" )

include_directories(
	include
  ${klt_feature_tracker_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

add_library( OpticalFlow SHARED
  src/optical_flow.cpp
  src/px4flow.cpp
 	src/flow_px4.cpp
	src/flow_opencv.cpp
)

target_link_libraries( OpticalFlow
  ${OpenCV_LIBS}
  ${klt_feature_tracker_LIBS}
)

target_include_directories(OpticalFlow PUBLIC include)
