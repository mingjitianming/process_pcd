cmake_minimum_required(VERSION 2.8.3)
project(to_octomap)

# -mavx causes a lot of errors!!
add_definitions(-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_CXX_FLAGS "-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")

# pcl 1.7 causes a segfault when it is built with debug mode
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(PCL 1.7 REQUIRED)
find_package(octomap REQUIRED)
include_directories(${PCL_INCLUDE_DIRS},${OCTOMAP_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Need to find both Qt4 and QGLViewer if the QQL support is to be built
FIND_PACKAGE(Qt4 COMPONENTS QtCore QtXml QtOpenGL QtGui)

FIND_PATH(QGLVIEWER_INCLUDE_DIR qglviewer.h
    /usr/include/QGLViewer
    /opt/local/include/QGLViewer
    /usr/local/include/QGLViewer
    /sw/include/QGLViewer
    ENV QGLVIEWERROOT
  )

find_library(QGLVIEWER_LIBRARY_RELEASE
  NAMES qglviewer-qt4 qglviewer QGLViewer QGLViewer2
  PATHS /usr/lib
        /usr/local/lib
        /opt/local/lib
        /sw/lib
        ENV QGLVIEWERROOT
        ENV LD_LIBRARY_PATH
        ENV LIBRARY_PATH
  PATH_SUFFIXES QGLViewer QGLViewer/release
)
find_library(QGLVIEWER_LIBRARY_DEBUG
  NAMES dqglviewer dQGLViewer dQGLViewer2 QGLViewerd2
  PATHS /usr/lib
        /usr/local/lib
        /opt/local/lib
        /sw/lib
        ENV QGLVIEWERROOT
        ENV LD_LIBRARY_PATH
        ENV LIBRARY_PATH
  PATH_SUFFIXES QGLViewer QGLViewer/release
)

if(QGLVIEWER_LIBRARY_RELEASE)
  if(QGLVIEWER_LIBRARY_DEBUG)
    set(QGLVIEWER_LIBRARY optimized ${QGLVIEWER_LIBRARY_RELEASE} debug ${QGLVIEWER_LIBRARY_DEBUG})
  else()
    set(QGLVIEWER_LIBRARY ${QGLVIEWER_LIBRARY_RELEASE})
  endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(QGLVIEWER DEFAULT_MSG
  QGLVIEWER_INCLUDE_DIR QGLVIEWER_LIBRARY)

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})
message(STATUS "OCTOMAP_INCLUDE_DIRS:" ${OCTOMAP_INLCUDE_DIRS})
message(STATUS "OCTOMAP_INCLUDES" ${OCTOMAP_INLCUDES})
message(STATUS "OCTOMAP_LIBRARIES" ${OCTOMAP_LIBRARIES})
message(STATUS "OCTOMAP_LIBS" ${OCTOMAP_LIBS})
message(STATUS "-- OCTOMAP_VERSION" ${OCTOMAP_VERSION})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

###########
## Build ##
###########
include_directories(include)

add_executable(to_octo
  apps/to_octo.cpp
)

target_link_libraries(to_octo
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)
