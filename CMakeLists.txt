SET(PROJECT_NAME FivePoint)

PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CMAKE_VERBOSE_MAKEFILE OFF)

IF( NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -std=c++0x")

FIND_PACKAGE(OpenCV REQUIRED)

SET(SOURCE_DIR "src")

SET(SOURCES 
${SOURCE_DIR}/ImageGrabber.cpp
${SOURCE_DIR}/GLWindow2.cpp
${SOURCE_DIR}/HarrisScores.cpp
${SOURCE_DIR}/QuadTree.cpp
${SOURCE_DIR}/ImageGrid.cpp
${SOURCE_DIR}/CvOrb.cpp
${SOURCE_DIR}/FivePoint.cpp
)

SET(LIBS opencv_core opencv_features2d opencv_flann cvd GVars3 GL GLU toontag lapack boost_system boost_thread pthread)

# Apps
ADD_EXECUTABLE(main ${SOURCE_DIR}/main.cpp ${SOURCES})
TARGET_LINK_LIBRARIES(main ${LIBS})
